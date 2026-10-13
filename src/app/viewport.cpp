#include "viewport.h"
#include "core/geometry.h"
#include "core/stress.h"
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QPainter>
#include <QVector3D>
#include <QWheelEvent>
#include <cmath>
#include <limits>

using namespace fiveaxis;
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
    hud = new QLabel(this);
    hud->setAttribute(Qt::WA_TransparentForMouseEvents);
    hud->setStyleSheet("background:transparent;color:#c6d9eb;font-size:12px;padding:0;");
    hud->move(24, 20);
    pathLegend = new QLabel(this);
    pathLegend->setAttribute(Qt::WA_TransparentForMouseEvents);
    pathLegend->setStyleSheet("background:transparent;font-size:12px;");
    pathLegend->setTextFormat(Qt::RichText);
    pathLegend->setWordWrap(true);
    update_path_legend();
    unitBox = box_mesh(Vec3::Zero(), Vec3::Ones());
    marker = box_mesh(Vec3::Zero(), Vec3(.7, .7, .7));
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
    beadEnds.clear();
    if (p) {
        for (size_t i = 0; i < p->path.size(); ++i) {
            if (i && p->path[i].deltaE > 0) {
                auto &a = p->path[i - 1];
                auto &b = p->path[i];
                Vec3 delta = b.position - a.position;
                if (delta.norm() > 1e-8) {
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
        if(project->deposition.checked && !project->deposition.unsupportedMoves.empty())
            hud->setText(hud->text()+QString("\n%1 deposition violations · marked red").arg(project->deposition.unsupportedMoves.size()));
        hud->adjustSize();
        hud->raise();
    }
    update();
}
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
    Eigen::AlignedBox3d b = project->part.bounds();
    for (auto &c : project->active_toolhead().components) {
        auto pose=component_pose(project->machine,project->active_toolhead(),c,state.joint);
        for(auto& v:c.mesh.vertices)b.extend(pose*v);
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
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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
}
void Viewport::update_path_legend() {
    pathLegend->setVisible(pathMode!=3);
    QString text="<span style='color:#ffb454'>Outer wall</span>&nbsp;&nbsp; "
                 "<span style='color:#efdf70'>Inner wall</span>&nbsp;&nbsp; "
                 "<span style='color:#35d4b0'>Infill</span>&nbsp;&nbsp; "
                 "<span style='color:#70e0ee'>Solid</span>&nbsp;&nbsp; "
                 "<span style='color:#d391f0'>Support</span>";
    if(travelPaths) text+="&nbsp;&nbsp; <span style='color:#72aafa'>Travel</span>";
    pathLegend->setText(text+"&nbsp;&nbsp; <span style='color:#ff586b'>Conflict</span><br/><span style='color:#a7b8ca'>Future motion dimmed · Current segment white</span>");
    pathLegend->setFixedWidth(std::max(100,width()-48)); pathLegend->adjustSize();
    pathLegend->move(24,std::max(0,height()-pathLegend->height()-16)); pathLegend->raise();
}
void Viewport::draw_mesh(const Mesh &m, const Transform &t, QVector4D color, int triangles) {
    if (m.empty() || triangles == 0)
        return;
    auto found = cache.find(&m);
    if (found == cache.end()) {
        std::vector<float> data;
        data.reserve(m.triangles.size() * 27);
        for (auto face : m.triangles) {
            Vec3 n =
                (m.vertices[face[1]] - m.vertices[face[0]]).cross(m.vertices[face[2]] - m.vertices[face[0]]);
            if (n.norm() < 1e-10)
                n = Vec3::UnitZ();
            else
                n.normalize();
            Vec3 rgb(.32, .52, .74);
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
    shader.setUniformValue("useField", project && &m == &project->part && stressVisible ? 1.0f : 0.0f);
    int pos = shader.attributeLocation("position"), normal = shader.attributeLocation("normal"),
        field = shader.attributeLocation("fieldColor");
    shader.enableAttributeArray(pos);
    shader.enableAttributeArray(normal);
    shader.enableAttributeArray(field);
    shader.setAttributeBuffer(pos, GL_FLOAT, 0, 3, 9 * sizeof(float));
    shader.setAttributeBuffer(normal, GL_FLOAT, 3 * sizeof(float), 3, 9 * sizeof(float));
    shader.setAttributeBuffer(field, GL_FLOAT, 6 * sizeof(float), 3, 9 * sizeof(float));
    glDrawArrays(GL_TRIANGLES, 0,
                 triangles < 0 ? found->second->count : std::min(found->second->count, 3 * triangles));
    shader.disableAttributeArray(pos);
    shader.disableAttributeArray(normal);
    shader.disableAttributeArray(field);
    buffer.release();
    shader.release();
}
void Viewport::paintGL() {
    QPainter painter(this);
    painter.beginNativePainting();
    glClearColor(.055f, .07f, .095f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!project) {
        painter.endNativePainting();
        return;
    }
    Transform bed = workpiece_pose(project->machine, state.joint);
    for (auto &f : project->fixtures)
        draw_mesh(f.mesh, f.movesWithBed ? bed : Transform::Identity(), QVector4D(.18, .23, .29, 1));
    if (depositedVisible && !beadEnds.empty()) {
        int completed = std::clamp(state.move - 1, 0, int(beadEnds.size()) - 1);
        draw_mesh(beads, bed, QVector4D(.20, .76, .65, 1), beadEnds[completed]);
    }
    if (depositedVisible && state.move > 0 && state.move < int(project->path.size()) && project->path[state.move].deltaE > 0) {
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
            draw_mesh(unitBox, bed * tr, QVector4D(.25, .87, .72, 1));
        }
    }
    for (auto &c : project->active_toolhead().components) {
        QVector4D color = c.nozzle ? QVector4D(.95, .70, .23, 1) : QVector4D(.55, .61, .72, 1);
        for (auto &e : project->report.events)
            if (e.move == state.move &&
                (e.first == c.name || e.second == c.name || e.first == c.name + " [" + c.id + "]" ||
                 e.second == c.name + " [" + c.id + "]"))
                color = QVector4D(1, .25, .23, 1);
        draw_mesh(c.renderMesh.empty() ? c.mesh : c.renderMesh,
                  component_pose(project->machine, project->active_toolhead(), c, state.joint), color);
    }
    glDepthMask(GL_FALSE);
    draw_mesh(project->part, bed, QVector4D(.32, .52, .74, stressVisible ? .75f : transparent ? .16f : .65f));
    glDepthMask(GL_TRUE);
    draw_mesh(fixedMesh, bed, QVector4D(.23, .55, 1, .85));
    draw_mesh(loadedMesh, bed, QVector4D(1, .40, .24, .85));
    draw_paths(bed);
    for (auto &event : project->report.events)
        if (event.move == state.move)
            draw_mesh(marker, translation(event.contact), QVector4D(1, .16, .30, 1));
    painter.endNativePainting();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QColor("#e5edf6"));
    QFont f = painter.font();
    f.setPointSize(12);
    f.setBold(true);
    painter.setFont(f);
    painter.drawText(24, 34, "SIMULATION");
    f.setPointSize(9);
    f.setBold(false);
    painter.setFont(f);
    painter.setPen(QColor("#a7b8ca"));
    painter.drawText(
        24, 56,
        QString("%1  ·  Move %2 / %3  ·  %4 s")
            .arg(project->machine.layout == Layout::RotaryBed ? "Rotary bed" : "Tilting toolhead")
            .arg(state.move)
            .arg(project->path.size())
            .arg(state.time, 0, 'f', 1));
    painter.drawText(24, 78,
                     QString("XYZ  %1  %2  %3 mm    A %4°  %5 %6°")
                         .arg(state.joint.xyz.x(), 0, 'f', 2)
                         .arg(state.joint.xyz.y(), 0, 'f', 2)
                         .arg(state.joint.xyz.z(), 0, 'f', 2)
                         .arg(state.joint.a * 180 / pi, 0, 'f', 1)
                         .arg(project->machine.axis2())
                         .arg(state.joint.b * 180 / pi, 0, 'f', 1));
    double flow = 0;
    if (state.move > 0 && state.move < int(project->path.size())) {
        auto &b = project->path[state.move];
        auto &a = project->path[state.move - 1];
        flow = std::max(0.0, b.deltaE) * pi * std::pow(project->machine.filamentDiameter / 2, 2) /
               std::max(.001, b.time - a.time);
    }
    painter.drawText(24, 100,
                     QString("Flow %1 mm³/s · Validated minimum clearance %2 mm")
                         .arg(flow, 0, 'f', 2)
                         .arg(project->report.minimumClearance < 1e90
                                  ? QString::number(project->report.minimumClearance, 'f', 3)
                                  : "pending"));
    if (stressVisible)
        painter.drawText(24, 122,
                         QString("%1 · %2 to %3 MPa")
                             .arg(project->stress.directional ? "Von Mises stress" : "Imported scalar")
                             .arg(fieldMin, 0, 'g', 4)
                             .arg(fieldMax, 0, 'g', 4));
    painter.setPen(QColor("#7e93aa"));
    if(pathMode!=3) {
        const std::pair<QString,QColor> legend[]={{"Outer wall",QColor("#ffb454")},{"Inner wall",QColor("#efdf70")},
            {"Infill",QColor("#35d4b0")},{"Solid",QColor("#70e0ee")},{"Support",QColor("#d391f0")},{"Travel",QColor("#72aafa")}};
        int x=24;
        for(auto &item:legend) { if(item.first=="Travel" && !travelPaths) continue; painter.setPen(item.second); painter.drawText(x,height()-45,item.first); x+=painter.fontMetrics().horizontalAdvance(item.first)+20; }
        painter.setPen(QColor("#7e93aa"));
    }
    painter.drawText(24, height() - 22,
                     pickTool ? "Click the nozzle tip to set its CAD coordinates"
                              : "Drag to orbit · Wheel to zoom · Click a part face to select it");
    if (!project->report.exportable()) {
        painter.setPen(QColor("#f5bd66"));
        painter.drawText(width() - 260, 34,
                         project->report.status == Status::Collision ? "Collision detected · export blocked"
                                                                     : "Motion awaits validation");
    }
}
static Vec3 path_color(Role role) {
    switch(role) {
    case Role::OuterWall: return Vec3(1,.706,.33);
    case Role::Wall: return Vec3(.94,.875,.44);
    case Role::Infill: return Vec3(.208,.831,.69);
    case Role::SolidInfill: return Vec3(.44,.878,.933);
    case Role::Support: return Vec3(.827,.569,.941);
    default: return Vec3(.447,.667,.98);
    }
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
    glDepthMask(GL_FALSE); glLineWidth(2);
    auto draw=[&](int begin,int end,float alpha) {
        shader.setUniformValue("color",QVector4D(1,1,1,alpha));
        if(travelPaths) glDrawArrays(GL_LINES,begin*2,(end-begin)*2);
        else for(int i=begin;i<end;) {
            while(i<end && pathSegments[i].role==Role::Travel) ++i;
            int start=i; while(i<end && pathSegments[i].role!=Role::Travel) ++i;
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
        if(s.move==state.move && s.begin<state.fraction && (travelPaths || s.role!=Role::Travel)) {
            Vec3 end=part_position(project->machine,state.joint);
            float vertices[]={float(s.start.x()),float(s.start.y()),float(s.start.z()),float(end.x()),float(end.y()),float(end.z())};
            QOpenGLBuffer active; active.create(); active.bind(); active.allocate(vertices,sizeof(vertices));
            shader.bind(); shader.setUniformValue("color",QVector4D(1,1,1,1)); shader.setUniformValue("useField",0.0f);
            shader.enableAttributeArray(pos); shader.setAttributeBuffer(pos,GL_FLOAT,0,3); shader.setAttributeValue(normal,QVector3D(0,0,1));
            glDrawArrays(GL_LINES,0,2); shader.disableAttributeArray(pos); shader.release(); active.release(); active.destroy();
        }
    }
    glLineWidth(1); glDepthMask(GL_TRUE);
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
