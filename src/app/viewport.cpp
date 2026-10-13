#include "viewport.h"
#include "core/geometry.h"
#include "core/stress.h"
#include "core/motion_preview.h"
#include "core/profile.h"
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QPainter>
#include <QVector3D>
#include <QWheelEvent>
#include <cmath>
#include <limits>

using namespace fiveaxis;
QColor toolpath_color(Role role) {
    switch(role) {
    case Role::OuterWall:return QColor("#ffb454"); case Role::Wall:return QColor("#b5a72b");
    case Role::Infill:return QColor("#d44738"); case Role::SolidInfill:return QColor("#9b61d0");
    case Role::TopSurface:return QColor("#55abdb"); case Role::BottomSurface:return QColor("#5956bb");
    case Role::Support:return QColor("#8b63b6"); case Role::Brim:return QColor("#54bcb1");
    case Role::Skirt:return QColor("#55a476"); case Role::Bridge:return QColor("#da638c");
    case Role::Retract:return QColor("#c552dc"); default:return QColor("#72aafa");
    }
}
static QMatrix4x4 qt_matrix(const Transform &tr) {
    QMatrix4x4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            out(i, j) = float(tr.matrix()(i, j));
    return out;
}
Viewport::Viewport(QWidget *parent) : QOpenGLWidget(parent) {
    setMinimumSize(500, 400);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    visibleRoles.fill(true);
    hud = new QLabel(this);
    hud->setObjectName("simulationHud");
    hud->setAttribute(Qt::WA_TransparentForMouseEvents);
    hud->setStyleSheet("background:rgba(255,255,255,220);color:#394345;font-size:12px;padding:7px;border-radius:3px;");
    hud->move(24, 20);
    pathLegend = new QLabel(this);
    pathLegend->setAttribute(Qt::WA_TransparentForMouseEvents);
    pathLegend->setStyleSheet("background:rgba(255,255,255,220);font-size:12px;padding:5px;border-radius:3px;");
    pathLegend->setTextFormat(Qt::RichText);
    pathLegend->setWordWrap(true);
    axesLegend=new QLabel(this); axesLegend->setAttribute(Qt::WA_TransparentForMouseEvents); axesLegend->setStyleSheet("background:transparent;");
    QPixmap axes(80,85); axes.fill(Qt::transparent); QPainter painter(&axes); painter.setRenderHint(QPainter::Antialiasing);
    QPoint origin(30,65);
    const QList<QPair<QPoint,QColor>> vectors={{QPoint(35,0),QColor("#ca3a3a")},{QPoint(-15,-20),QColor("#159941")},{QPoint(0,-40),QColor("#345aca")}};
    for(const auto &axis:vectors) {painter.setPen(QPen(axis.second,2));painter.drawLine(origin,origin+axis.first);}
    painter.setPen(vectors[0].second);painter.drawText(68,70,"X");painter.setPen(vectors[1].second);painter.drawText(5,43,"Y");painter.setPen(vectors[2].second);painter.drawText(25,20,"Z");
    axesLegend->setPixmap(axes); axesLegend->resize(80,85);
    update_path_legend();
    unitBox = box_mesh(Vec3::Zero(), Vec3::Ones());
    marker = box_mesh(Vec3::Zero(), Vec3(.7, .7, .7));
    for(int i=0;i<64;++i){
        double angle=(i+.5)*2*pi/64;Transform tr=translation(Vec3(std::cos(angle),std::sin(angle),0));
        tr.linear()=Eigen::AngleAxisd(angle+pi/2,Vec3::UnitZ()).toRotationMatrix();
        auto segment=box_mesh(Vec3::Zero(),Vec3(2*pi/64,.018,.018));int offset=int(rotaryRing.vertices.size());
        for(auto v:segment.vertices)rotaryRing.vertices.push_back(tr*v);
        for(auto face:segment.triangles)rotaryRing.triangles.push_back(face+Eigen::Vector3i::Constant(offset));
    }
    for(int i=0;i<2;++i){rotaryLabels[i]=new QLabel(this);rotaryLabels[i]->setAttribute(Qt::WA_TransparentForMouseEvents);rotaryLabels[i]->setStyleSheet("background:rgba(255,255,255,220);font-size:9pt;padding:3px;");rotaryLabels[i]->hide();}
}
Viewport::~Viewport() {
    if (isValid())
        makeCurrent();
    cache.clear();
    pathBuffer.destroy();
    if (isValid())
        doneCurrent();
}
void Viewport::set_project(const Project *p) {
    if (isValid())
        makeCurrent();
    cache.clear();
    pathBuffer.destroy();
    pathsUploaded=false;
    if (isValid())
        doneCurrent();
    project = p;
    axisOverride=false;
    pathSegments = p ? simulation_toolpaths(*p) : std::vector<ToolpathSegment>{};
    layerRanges.clear();
    for(size_t i=0;i<pathSegments.size();) {
        size_t end=i+1;
        while(end<pathSegments.size() && pathSegments[end].layer==pathSegments[i].layer) ++end;
        layerRanges[pathSegments[i].layer].push_back({int(i),int(end)}); i=end;
    }
    validationCurrent = p && p->report.exportable() && p->report.fingerprint == p->fingerprint();
    fieldMin = 1e100;
    fieldMax = -1e100;
    if (p) {
        if (p->stress.directional)
            for (const auto &tensor : p->stress.tensors) {
                auto d = tensor - Mat3::Identity() * tensor.trace() / 3;
                double v = std::sqrt(1.5 * d.squaredNorm());
                fieldMin = std::min(fieldMin, v);
                fieldMax = std::max(fieldMax, v);
            }
        else
            for (double v : p->stress.scalar) {
                fieldMin = std::min(fieldMin, v);
                fieldMax = std::max(fieldMax, v);
            }
    }
    if (fieldMax < fieldMin) {
        fieldMin = 0;
        fieldMax = 1;
    }
    beads = {};
    beadRanges.clear(); beadColors.clear();
    beadEnds.clear();
    if (p) {
        for (size_t i = 0; i < p->path.size(); ++i) {
            if (i && p->path[i].deltaE > 0) {
                auto &a = p->path[i - 1];
                auto &b = p->path[i];
                Vec3 delta = b.position - a.position;
                if (delta.norm() > 1e-8) {
                    int first=int(beads.triangles.size());
                    Vec3 n = (a.direction + b.direction).normalized(), tangent = delta.normalized(),
                         side = n.cross(tangent);
                    if (side.norm() < 1e-6)
                        side = tangent.unitOrthogonal();
                    side.normalize();
                    Transform tr = translation((a.position + b.position) / 2 - n * b.height / 2);
                    tr.linear().col(0) = tangent * delta.norm();
                    tr.linear().col(1) = side * b.width;
                    tr.linear().col(2) = tangent.cross(side).normalized() * b.height;
                    int offset = int(beads.vertices.size());
                    for (auto &v : unitBox.vertices)
                        beads.vertices.push_back(tr * v);
                    for (auto face : unitBox.triangles)
                        beads.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
                    auto color=toolpath_color(b.role); QVector3D rgb(color.redF(),color.greenF(),color.blueF());
                    for(size_t j=0;j<unitBox.triangles.size();++j) beadColors.push_back(rgb);
                    if(!beadRanges.empty() && beadRanges.back().layer==b.layer && beadRanges.back().role==b.role)
                        beadRanges.back().end=int(beads.triangles.size());
                    else beadRanges.push_back({first,int(beads.triangles.size()),b.layer,b.role});
                }
            }
            beadEnds.push_back(int(beads.triangles.size()));
        }
    }
    set_selected(fixed, loaded);
    set_time(time);
}
void Viewport::set_time(double t) {
    time = t;
    if (project)
        state = simulation_at(*project, t, false);
    if(axisOverride)state.joint=axisJoint;
    if (project) {
        auto axes = controller_axes(project->machine, project->active_toolhead(), state.joint);
        double flow = 0;
        if (state.move > 0 && state.move < int(project->path.size())) {
            auto &b = project->path[state.move];
            auto &a = project->path[state.move - 1];
            flow = std::max(0.0, b.deltaE) * pi * std::pow(project->machine.filamentDiameter / 2, 2) /
                   std::max(.001, b.time - a.time);
        }
        hud->setText(QString("SIMULATION · %1\nMove %2 / %3 · %4 s\nCarriage XYZ %5, %6, %7 mm · A %8° · %9 "
                             "%10°\nFlow %11 mm³/s · Minimum clearance %12 mm\n%13")
                         .arg(project->machine.layout == Layout::RotaryBed ? "Rotary bed" : "Tilting head")
                         .arg(state.move)
                         .arg(project->path.size())
                         .arg(state.time, 0, 'f', 1)
                         .arg(axes.xyz.x(), 0, 'f', 2)
                         .arg(axes.xyz.y(), 0, 'f', 2)
                         .arg(axes.xyz.z(), 0, 'f', 2)
                         .arg(state.joint.a * 180 / pi, 0, 'f', 1)
                         .arg(project->machine.axis2())
                         .arg(state.joint.b * 180 / pi, 0, 'f', 1)
                         .arg(flow, 0, 'f', 2)
                         .arg(project->report.minimumClearance < 1e90
                                  ? QString::number(project->report.minimumClearance, 'f', 3)
                                  : "pending")
                         .arg(validationCurrent ? "Geometric motion checked"
                              : project->report.status == Status::Collision
                                  ? "Collision detected · export blocked"
                                  : "Validation pending / unresolved"));
        if(project->useDummyToolhead) hud->setText(hud->text()+"\nDUMMY TOOLHEAD · synthetic geometry only");
        if(project->process.profileNormal && !project->profileGuide.empty() && !project->path.empty()) {
            const auto &sample=project->path[std::clamp(state.move,0,int(project->path.size())-1)];
            auto normal=profile_layer(*project,sample.layer).normal;
            double error=std::acos(std::clamp(part_direction(project->machine,project->active_toolhead(),state.joint).dot(normal),-1.0,1.0))*180/pi;
            hud->setText(hud->text()+(sample.deltaE>0?QString("\nPROFILE NORMAL · error %1°").arg(error,0,'f',4):"\nPROFILE NORMAL · travel orientation transition"));
        }
        if(project->deposition.checked && !project->deposition.unsupportedMoves.empty())
            hud->setText(hud->text()+QString("\n%1 deposition violations · marked red").arg(project->deposition.unsupportedMoves.size()));
        if(prepare) hud->setText("PREPARE\nImport a part, choose your machine and process, then slice.\nGrid spacing: 10 mm · Physical bed defined in Machine setup");
        else if(axisOverride)hud->setText(QString("AXIS POSE PREVIEW · %1\nA %2° · %3 %4°\nCarriage XYZ %5, %6, %7 mm\nNo extrusion · Collision validation not run for this pose")
            .arg(project->machine.layout==Layout::RotaryBed?"Tilting / rotating bed":"Tilting / rolling toolhead")
            .arg(state.joint.a*180/pi,0,'f',1).arg(project->machine.axis2()).arg(state.joint.b*180/pi,0,'f',1)
            .arg(axes.xyz.x(),0,'f',2).arg(axes.xyz.y(),0,'f',2).arg(axes.xyz.z(),0,'f',2));
        hud->adjustSize();
        hud->raise();
    }
    update();
}
void Viewport::set_axis_pose(const JointState &q,bool enabled){axisJoint=q;axisOverride=enabled;set_time(time);}
void Viewport::set_stress(bool v) {
    stressVisible = v;
    if (isValid())
        makeCurrent();
    if (project)
        cache.erase(&project->part);
    if (isValid())
        doneCurrent();
    update();
}
void Viewport::set_selected(const std::set<int> &a, const std::set<int> &b) {
    fixed = a;
    loaded = b;
    if (isValid())
        makeCurrent();
    cache.erase(&fixedMesh);
    cache.erase(&loadedMesh);
    if (isValid())
        doneCurrent();
    fixedMesh = {};
    loadedMesh = {};
    if (project) {
        fixedMesh.vertices = loadedMesh.vertices = project->part.vertices;
        for (int i : fixed)
            if (i >= 0 && i < int(project->part.triangles.size()))
                fixedMesh.triangles.push_back(project->part.triangles[i]);
        for (int i : loaded)
            if (i >= 0 && i < int(project->part.triangles.size()))
                loadedMesh.triangles.push_back(project->part.triangles[i]);
    }
    update();
}
void Viewport::fit() {
    if (!project)
        return;
    Eigen::AlignedBox3d b;
    auto bed=workpiece_pose(project->machine,prepare?JointState{}:state.joint);
    for(const auto &v:project->part.vertices)b.extend(bed*v);
    for (auto &c : project->active_toolhead().components) {
        auto pose=component_pose(project->machine,project->active_toolhead(),c,state.joint);
        for(auto& v:c.mesh.vertices)b.extend(pose*v);
    }
    if(!prepare && rotaryAxesVisible) {
        double extent=(project->machine.layout==Layout::RotaryBed?30:12)*1.4;
        for(const auto &frame:rotary_axis_frames(project->machine,project->active_toolhead(),state.joint)) {
            b.extend(frame.translation()-Vec3::Constant(extent));
            b.extend(frame.translation()+Vec3::Constant(extent));
        }
    }
    if (!b.isEmpty()) {
        target = b.center();
        distance = std::max(60.0, b.sizes().norm() * 2.2);
    }
    update();
}
QMatrix4x4 Viewport::view_projection() const {
    Vec3 aim = follow ? state.joint.xyz : target;
    double a = yaw * pi / 180, b = pitch * pi / 180;
    Vec3 eye = aim + distance * Vec3(std::cos(b) * std::cos(a), std::cos(b) * std::sin(a), std::sin(b));
    QMatrix4x4 v;
    v.lookAt(QVector3D(eye.x(), eye.y(), eye.z()), QVector3D(aim.x(), aim.y(), aim.z()), QVector3D(0, 0, 1));
    QMatrix4x4 p;
    p.perspective(42, float(width()) / std::max(1, height()), .05, 5000);
    return p * v;
}
void Viewport::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    shader.addShaderFromSourceCode(
        QOpenGLShader::Vertex,
        "attribute vec3 position;attribute vec3 normal;attribute vec3 fieldColor;uniform mat4 mvp;uniform "
        "mat4 model;varying vec3 "
        "N;varying vec3 C;void "
        "main(){gl_Position=mvp*vec4(position,1.0);N=mat3(model)*normal;C=fieldColor;}");
    shader.addShaderFromSourceCode(
        QOpenGLShader::Fragment,
        "uniform vec4 color;uniform float useField;uniform float unlit;varying vec3 N;varying vec3 C;void main(){float "
        "light=0.38+0.62*abs(dot(normalize(N),normalize(vec3(-0.5,-0.3,1.0))));gl_"
        "FragColor=vec4(mix(color.rgb,C,useField)*mix(light,1.0,unlit),color.a);}");
    shader.link();
}
void Viewport::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    update_path_legend();
    axesLegend->move(std::max(0,w-95),std::max(0,h-100)); axesLegend->raise();
}
void Viewport::update_path_legend() {
    pathLegend->setVisible(!prepare && pathMode!=3 && !pathSegments.empty());
    QString text;
    for(Role role:{Role::OuterWall,Role::Wall,Role::Infill,Role::SolidInfill,Role::Support,Role::Travel}) {
        if(role==Role::Travel && !travelPaths) continue;
        text+=QString("<span style='color:%1'>%2</span>&nbsp;&nbsp; ").arg(toolpath_color(role).name(),role_name(role));
    }
    pathLegend->setText(text+"&nbsp;&nbsp; <span style='color:#e63950'>Conflict</span><br/><span style='color:#535e64'>Future motion dimmed · Current segment white</span>");
    pathLegend->setFixedWidth(std::max(100,width()-148)); pathLegend->adjustSize();
    pathLegend->move(24,std::max(0,height()-pathLegend->height()-16)); pathLegend->raise();
}
void Viewport::draw_mesh(const Mesh &m, const Transform &t, QVector4D color, int triangles,int firstTriangle) {
    if (m.empty() || triangles == 0)
        return;
    auto found = cache.find(&m);
    if (found == cache.end()) {
        std::vector<float> data;
        data.reserve(m.triangles.size() * 27);
        int faceIndex=0;
        for (auto face : m.triangles) {
            Vec3 n =
                (m.vertices[face[1]] - m.vertices[face[0]]).cross(m.vertices[face[2]] - m.vertices[face[0]]);
            if (n.norm() < 1e-10)
                n = Vec3::UnitZ();
            else
                n.normalize();
            Vec3 rgb(.32, .52, .74);
            if(&m==&beads && faceIndex<int(beadColors.size())) { auto c=beadColors[faceIndex]; rgb=Vec3(c.x(),c.y(),c.z()); }
            ++faceIndex;
            if (project && &m == &project->part && stressVisible &&
                (!project->stress.scalar.empty() || project->stress.directional)) {
                bool covered;
                double value = project->stress.scalar_at(
                    (m.vertices[face[0]] + m.vertices[face[1]] + m.vertices[face[2]]) / 3, &covered);
                if (covered) {
                    double u = std::clamp((value - fieldMin) / std::max(1e-9, fieldMax - fieldMin), 0.0, 1.0);
                    rgb = Vec3(std::clamp(2 * u, 0.0, 1.0), std::clamp(2 - std::abs(4 * u - 2), 0.0, 1.0),
                               std::clamp(2 - 2 * u, 0.0, 1.0));
                }
            }
            for (int index : face)
                for (int j = 0; j < 9; ++j)
                    data.push_back(float(j < 3 ? m.vertices[index][j] : j < 6 ? n[j - 3] : rgb[j - 6]));
        }
        auto render = std::make_unique<RenderMesh>();
        render->buffer.create();
        render->buffer.bind();
        render->buffer.allocate(data.data(), int(data.size() * sizeof(float)));
        render->count = int(data.size() / 9);
        render->buffer.release();
        found = cache.emplace(&m, std::move(render)).first;
    }
    auto &buffer = found->second->buffer;
    buffer.bind();
    shader.bind();
    shader.setUniformValue("mvp", view_projection() * qt_matrix(t));
    shader.setUniformValue("model", qt_matrix(t));
    shader.setUniformValue("color", color);
    shader.setUniformValue("unlit", 0.0f);
    shader.setUniformValue("useField", &m==&beads || (project && &m == &project->part && stressVisible) ? 1.0f : 0.0f);
    int pos = shader.attributeLocation("position"), normal = shader.attributeLocation("normal"),
        field = shader.attributeLocation("fieldColor");
    shader.enableAttributeArray(pos);
    shader.enableAttributeArray(normal);
    shader.enableAttributeArray(field);
    shader.setAttributeBuffer(pos, GL_FLOAT, 0, 3, 9 * sizeof(float));
    shader.setAttributeBuffer(normal, GL_FLOAT, 3 * sizeof(float), 3, 9 * sizeof(float));
    shader.setAttributeBuffer(field, GL_FLOAT, 6 * sizeof(float), 3, 9 * sizeof(float));
    int first=std::clamp(firstTriangle*3,0,found->second->count);
    glDrawArrays(GL_TRIANGLES, first,
                 triangles < 0 ? found->second->count-first : std::min(found->second->count-first, 3 * triangles));
    shader.disableAttributeArray(pos);
    shader.disableAttributeArray(normal);
    shader.disableAttributeArray(field);
    buffer.release();
    shader.release();
}
void Viewport::paintGL() {
    QPainter painter(this);
    painter.beginNativePainting();
    // QPainter may leave premultiplied-alpha state from its preceding overlay pass.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_TRUE);
    glClearColor(.89f, .90f, .91f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!project) {
        painter.endNativePainting();
        return;
    }
    Transform bed = workpiece_pose(project->machine, prepare?JointState{}:state.joint);
    for (auto &f : project->fixtures)
        draw_mesh(f.mesh, f.movesWithBed ? bed : Transform::Identity(), QVector4D(.18, .23, .29, 1));
    // A metric grid is a visual reference. Fixtures in the scene define the physical bed.
    for(int i=-80;i<=80;i+=10) for(int axis=0;axis<2;++axis) {
        Transform tr=translation(axis==0?Vec3(i,0,.015):Vec3(0,i,.015)); tr.linear().diagonal()=axis==0?Vec3(.08,160,.015):Vec3(160,.08,.015);
        draw_mesh(unitBox,bed*tr,QVector4D(.57,.62,.63,1));
    }
    if (!prepare && depositedVisible && !beadEnds.empty()) {
        int completed = std::clamp(state.move - 1, 0, int(beadEnds.size()) - 1);
        for(const auto &range:beadRanges) if(range.layer>=firstLayer && range.layer<=lastLayer && visibleRoles[int(range.role)]) {
            int end=std::min(range.end,beadEnds[completed]);
            if(end>range.begin) draw_mesh(beads,bed,QVector4D(1,1,1,1),end-range.begin,range.begin);
        }
    }
    if (!prepare && depositedVisible && state.move > 0 && state.move < int(project->path.size()) && project->path[state.move].deltaE > 0 &&
        project->path[state.move].layer>=firstLayer && project->path[state.move].layer<=lastLayer && visibleRoles[int(project->path[state.move].role)]) {
        auto &a = project->path[state.move - 1];
        auto &b = project->path[state.move];
        Vec3 endpoint = part_position(project->machine, state.joint), delta = endpoint - a.position;
        if (delta.norm() > 1e-8) {
            Vec3 n = (a.direction + b.direction).normalized(), tangent = delta.normalized(),
                 side = n.cross(tangent);
            if (side.norm() < 1e-6)
                side = tangent.unitOrthogonal();
            side.normalize();
            Transform tr = translation((a.position + endpoint) / 2 - n * b.height / 2);
            tr.linear().col(0) = tangent * delta.norm();
            tr.linear().col(1) = side * b.width;
            tr.linear().col(2) = tangent.cross(side).normalized() * b.height;
            auto color=toolpath_color(b.role); draw_mesh(unitBox, bed * tr, QVector4D(color.redF(),color.greenF(),color.blueF(),1));
        }
    }
    if(!prepare && toolheadVisible) for (auto &c : project->active_toolhead().components) {
        QVector4D color = c.nozzle ? QVector4D(.95, .70, .23, 1) : QVector4D(.55, .61, .72, 1);
        if(!axisOverride)for (auto &e : project->report.events)
            if (e.move == state.move &&
                (e.first == c.name || e.second == c.name || e.first == c.name + " [" + c.id + "]" ||
                 e.second == c.name + " [" + c.id + "]"))
                color = QVector4D(1, .25, .23, 1);
        draw_mesh(c.renderMesh.empty() ? c.mesh : c.renderMesh,
                  component_pose(project->machine, project->active_toolhead(), c, state.joint), color);
    }
    glDepthMask(GL_FALSE);
    draw_mesh(project->part, bed, prepare?QVector4D(.66,.69,.70,1):QVector4D(.5,.55,.57, stressVisible ? .75f : transparent ? .16f : .65f));
    glDepthMask(GL_TRUE);
    draw_mesh(fixedMesh, bed, QVector4D(.23, .55, 1, .85));
    draw_mesh(loadedMesh, bed, QVector4D(1, .40, .24, .85));
    if(!prepare) draw_paths(bed);
    if(!axisOverride)for (auto &event : project->report.events)
        if (event.move == state.move)
            draw_mesh(marker, translation(event.contact), QVector4D(1, .16, .30, 1));
    draw_rotary_axes();
    if(project->process.profileNormal && !project->profileGuide.empty() && !prepare && !project->path.empty()) {
        const auto &sample=project->path[std::clamp(state.move,0,int(project->path.size())-1)];
        auto section=profile_layer(*project,sample.layer);double radius=0;
        for(const auto &s:project->path)if(s.layer==sample.layer && s.deltaE>0 && s.role!=Role::Support)radius=std::max(radius,(s.position-section.center).norm());
        Transform plane=translation(section.center);plane.linear()=Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(),section.normal).toRotationMatrix();
        Transform scale=Transform::Identity();scale.linear()*=std::max(1.0,radius+.3);
        draw_mesh(rotaryRing,bed*plane*scale,QVector4D(.62,.23,.8,1));
        Vec3 axis=part_direction(project->machine,project->active_toolhead(),state.joint);
        Transform pointer=translation(part_position(project->machine,state.joint)+axis*3);
        pointer.linear()=Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(),axis).toRotationMatrix()*Eigen::DiagonalMatrix<double,3>(.18,.18,6);
        draw_mesh(unitBox,bed*pointer,QVector4D(.05,.8,.25,1));
    }
    painter.endNativePainting();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QColor("#535e64"));
    if(prepare) painter.drawText(24,30,"PREPARE · Import a part, select your machine, then slice");
    QPoint origin(width()-65,height()-60);
    for(const auto &axis:QList<QPair<QPoint,QColor>>{{QPoint(35,0),QColor("#ca3a3a")},{QPoint(-15,-20),QColor("#159941")},{QPoint(0,-40),QColor("#345aca")}}) {
        painter.setPen(QPen(axis.second,2)); painter.drawLine(origin,origin+axis.first);
    }
    painter.setPen(QColor("#ca3a3a")); painter.drawText(origin+QPoint(38,4),"X");
    painter.setPen(QColor("#159941")); painter.drawText(origin+QPoint(-23,-20),"Y");
    painter.setPen(QColor("#345aca")); painter.drawText(origin+QPoint(-4,-44),"Z");

}
void Viewport::draw_rotary_axes(){
    for(auto label:rotaryLabels)label->hide();
    if(!project || prepare || !rotaryAxesVisible)return;
    auto frames=rotary_axis_frames(project->machine,project->active_toolhead(),state.joint);
    double radius=project->machine.layout==Layout::RotaryBed?30:12;
    const QVector4D colors[]={QVector4D(.9,.2,.3,1),QVector4D(.08,.55,.82,1)};
    for(int i=0;i<2;++i){
        Transform scale=Transform::Identity();scale.linear()*=radius;
        draw_mesh(rotaryRing,frames[i]*scale,colors[i]);
        Transform stem=translation(Vec3(0,0,0));stem.linear().diagonal()=Vec3(.4,.4,radius*2.4);
        draw_mesh(unitBox,frames[i]*stem,colors[i]);
        double angle=i==0?state.joint.a:state.joint.b;
        Transform pointer=rotation_z(angle)*translation(Vec3(radius/2,0,0));pointer.linear()*=Eigen::DiagonalMatrix<double,3>(radius,.6,.6);
        draw_mesh(unitBox,frames[i]*pointer,colors[i]);
        auto point=frames[i]*Vec3(0,0,radius*1.3);auto clip=view_projection()*QVector4D(point.x(),point.y(),point.z(),1);
        if(clip.w()<=0)continue;auto ndc=clip/clip.w();int x=int((ndc.x()+1)*width()/2),y=int((1-ndc.y())*height()/2);
        auto label=rotaryLabels[i];label->setText(QString("%1 · %2°").arg(i==0?"A":project->machine.axis2()).arg(angle*180/pi,0,'f',1));label->adjustSize();
        if(x>=0 && x+label->width()<width() && y>=0 && y+label->height()<height()){label->move(x,y);label->show();label->raise();}
    }
}
static Vec3 path_color(Role role) {
    auto c=toolpath_color(role); return Vec3(c.redF(),c.greenF(),c.blueF());
}
void Viewport::draw_paths(const Transform &bed) {
    if(pathMode==3 || pathSegments.empty()) return;
    if(!pathsUploaded) {
        std::vector<float> data; data.reserve(pathSegments.size()*18);
        std::set<int> conflicts(project->deposition.unsupportedMoves.begin(),project->deposition.unsupportedMoves.end());
        for(const auto &event:project->report.events) if(event.status==Status::Collision) conflicts.insert(event.move);
        for(const auto &segment:pathSegments) {
            Vec3 color=conflicts.count(segment.move)?Vec3(1,.345,.42):path_color(segment.role);
            for(const auto &v:{segment.start,segment.end}) {
                for(int j=0;j<3;++j) data.push_back(float(v[j]));
                data.insert(data.end(),{0,0,1});
                for(int j=0;j<3;++j) data.push_back(float(color[j]));
            }
        }
        pathBuffer.create(); pathBuffer.bind(); pathBuffer.allocate(data.data(),int(data.size()*sizeof(float))); pathBuffer.release(); pathsUploaded=true;
    }
    shader.bind(); pathBuffer.bind();
    shader.setUniformValue("mvp",view_projection()*qt_matrix(bed)); shader.setUniformValue("model",qt_matrix(bed));
    shader.setUniformValue("useField",1.0f); shader.setUniformValue("unlit",1.0f);
    int pos=shader.attributeLocation("position"),normal=shader.attributeLocation("normal"),field=shader.attributeLocation("fieldColor");
    for(int attribute:{pos,normal,field}) shader.enableAttributeArray(attribute);
    shader.setAttributeBuffer(pos,GL_FLOAT,0,3,9*sizeof(float)); shader.setAttributeBuffer(normal,GL_FLOAT,3*sizeof(float),3,9*sizeof(float));
    shader.setAttributeBuffer(field,GL_FLOAT,6*sizeof(float),3,9*sizeof(float));
    glDepthMask(GL_FALSE); glDepthFunc(GL_LEQUAL); glLineWidth(2);
    auto draw=[&](int begin,int end,float alpha) {
        shader.setUniformValue("color",QVector4D(1,1,1,alpha));
        auto visible=[&](int i){const auto &s=pathSegments[i]; return s.layer>=firstLayer && s.layer<=lastLayer && visibleRoles[int(s.role)] && (travelPaths || s.role!=Role::Travel);};
        for(int i=begin;i<end;) {
            while(i<end && !visible(i)) ++i;
            int start=i; while(i<end && visible(i)) ++i;
            if(i>start) glDrawArrays(GL_LINES,start*2,(i-start)*2);
        }
    };
    int complete=int(std::lower_bound(pathSegments.begin(),pathSegments.end(),state.move,
                    [](const ToolpathSegment &s,int move){return s.move<move;})-pathSegments.begin());
    int prefix=complete;
    while(prefix<int(pathSegments.size()) && pathSegments[prefix].move==state.move && pathSegments[prefix].finish<=state.fraction+1e-10) ++prefix;
    if(pathMode==0) { draw(0,int(pathSegments.size()),.3f); draw(0,prefix,1); }
    else if(pathMode==1) draw(0,prefix,1);
    else if(!project->path.empty()) {
        int layer=project->path[std::clamp(state.move,0,int(project->path.size())-1)].layer;
        for(auto range:layerRanges[layer]) { draw(range.first,range.second,.35f); draw(range.first,std::max(range.first,std::min(range.second,prefix)),1); }
    }
    for(int attribute:{pos,normal,field}) shader.disableAttributeArray(attribute);
    pathBuffer.release(); shader.release();
    if(prefix<int(pathSegments.size())) {
        const auto &s=pathSegments[prefix];
        if(s.move==state.move && s.begin<state.fraction && s.layer>=firstLayer && s.layer<=lastLayer && visibleRoles[int(s.role)] && (travelPaths || s.role!=Role::Travel)) {
            Vec3 end=part_position(project->machine,state.joint);
            float vertices[]={float(s.start.x()),float(s.start.y()),float(s.start.z()),float(end.x()),float(end.y()),float(end.z())};
            QOpenGLBuffer active; active.create(); active.bind(); active.allocate(vertices,sizeof(vertices));
            shader.bind(); shader.setUniformValue("color",QVector4D(1,1,1,1)); shader.setUniformValue("useField",0.0f);
            shader.enableAttributeArray(pos); shader.setAttributeBuffer(pos,GL_FLOAT,0,3); shader.setAttributeValue(normal,QVector3D(0,0,1));
            glDrawArrays(GL_LINES,0,2); shader.disableAttributeArray(pos); shader.release(); active.release(); active.destroy();
        }
    }
    glLineWidth(1); glDepthMask(GL_TRUE); glDepthFunc(GL_LESS);
}
void Viewport::mousePressEvent(QMouseEvent *e) {
    last = pressed = e->pos();
}
void Viewport::mouseMoveEvent(QMouseEvent *e) {
    if (e->buttons() & Qt::LeftButton) {
        auto d = e->pos() - last;
        yaw += d.x() * .4;
        pitch = std::clamp(pitch + d.y() * .4, -85.0, 85.0);
        last = e->pos();
        update();
    }
}
void Viewport::mouseReleaseEvent(QMouseEvent *e) {
    if ((e->pos() - pressed).manhattanLength() < 5)
        pick(e->pos());
}
void Viewport::wheelEvent(QWheelEvent *e) {
    distance = std::clamp(distance * std::exp(-e->angleDelta().y() * .001), 5.0, 3000.0);
    update();
}
void Viewport::pick(QPoint position) {
    if (!project)
        return;
    bool ok;
    auto inv = view_projection().inverted(&ok);
    if (!ok)
        return;
    float x = 2.f * position.x() / width() - 1, y = 1 - 2.f * position.y() / height();
    auto a = inv * QVector4D(x, y, -1, 1), b = inv * QVector4D(x, y, 1, 1);
    a /= a.w();
    b /= b.w();
    Vec3 origin(a.x(), a.y(), a.z()), ray = Vec3(b.x() - a.x(), b.y() - a.y(), b.z() - a.z()).normalized();
    double best = 1e100;
    int selected = -1;
    Vec3 hit;
    auto check = [&](const Mesh &mesh, const Transform &tr, int id) {
        for (size_t i = 0; i < mesh.triangles.size(); ++i) {
            auto t = mesh.triangles[i];
            Vec3 v = tr * mesh.vertices[t[0]], u = tr * mesh.vertices[t[1]] - v,
                 w = tr * mesh.vertices[t[2]] - v, h = ray.cross(w);
            double det = u.dot(h);
            if (std::abs(det) < 1e-12)
                continue;
            Vec3 s = origin - v;
            double p = s.dot(h) / det;
            if (p < 0 || p > 1)
                continue;
            Vec3 q = s.cross(u);
            double r = ray.dot(q) / det;
            if (r < 0 || p + r > 1)
                continue;
            double d = w.dot(q) / det;
            if (d > 0 && d < best) {
                best = d;
                selected = pickTool ? id : int(i);
                hit = tr.inverse() * (origin + ray * d);
            }
        }
    };
    if (pickTool) {
        for (size_t i = 0; i < project->active_toolhead().components.size(); ++i) {
            auto &c = project->active_toolhead().components[i];
            check(c.mesh, component_pose(project->machine, project->active_toolhead(), c, state.joint), int(i));
        }
        if (selected >= 0)
            Q_EMIT tool_picked(selected, hit);
    } else {
        check(project->part, workpiece_pose(project->machine, state.joint), 0);
        if (selected >= 0)
            Q_EMIT face_picked(selected);
    }
}
