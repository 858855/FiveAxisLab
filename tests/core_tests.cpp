#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include <QFile>
#include <QTemporaryDir>
#include <QtTest/QtTest>
#include <cmath>
using namespace fiveaxis;
class CoreTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
    void kinematics();
    void thin_obstacle();
    void rotary_sweep();
    void containment();
    void deposited_only();
    void tip_contact_and_heater();
    void fea_patch();
    void vtu_tensors();
    void vtu_components_and_mismatch();
    void step_roundtrip();
    void malformed_step();
    void project_roundtrip_and_stale_export();
    void simulation_backward();
    void controller_replay();
    void scalar_field_slice();
    void blocked_repair();
    void stale_kinematics();
    void clearance_threshold();
    void successful_travel_repair();
    void quadratic_tetra_mapping();
    void step_units();
    void curved_strength_layers();
    void playback_invariance();
    void unsupported_deposition();
    void nozzle_lift_and_penetration();
    void successful_orientation_repair();
    void cavity_meshing();
    void rotary_lever_bound();
    void unresolved_budget();
    void head_motor_interpolation();
    void conical_nozzle_contact();
    void bounded_interface();
    void fem_step_metadata();
};
static Project small() {
    Project p;
    p.machine.clearance = .05;
    Component c;
    c.id = "probe";
    c.name = "Probe";
    c.mesh = box_mesh(Vec3::Zero(), Vec3(.2, .2, .2));
    p.toolhead.components.push_back(c);
    p.depositionValid = true;
    return p;
}
void CoreTests::kinematics() {
    for (Layout layout : {Layout::RotaryBed, Layout::TiltingHead}) {
        MachineProfile m;
        m.layout = layout;
        ToolheadAssembly t;
        for (int i = 0; i < 100; ++i) {
            JointState q;
            q.a = .5 * std::sin(i);
            q.b = .5 * std::cos(i);
            q.xyz = Vec3(10 + std::sin(i) * 5, 20, 30);
            Vec3 p = part_position(m, q), n = part_direction(m, t, q);
            auto solutions = inverse_kinematics(m, p, n, q);
            QVERIFY(!solutions.empty());
            QVERIFY((part_position(m, solutions[0]) - p).norm() < 1e-8);
            QVERIFY((part_direction(m, t, solutions[0]) - n).norm() < 1e-8);
            Component c;
            c.link = Link::Nozzle;
            Transform tr = component_pose(m, t, c, q);
            QVERIFY((tr * t.tcp - q.xyz).norm() < 1e-8);
        }
    }
}
void CoreTests::thin_obstacle() {
    auto p = small();
    p.fixtures.push_back({"Thin plate", box_mesh(Vec3(0, 0, 5), Vec3(.02, 1, 1)), false});
    CollisionScene scene(p);
    JointState a, b;
    a.xyz = Vec3(-5, 0, 5);
    b.xyz = Vec3(5, 0, 5);
    QCOMPARE(scene.check_pose(a, false, a.xyz).status, Status::Clear);
    QCOMPARE(scene.check_pose(b, false, b.xyz).status, Status::Clear);
    CollisionReport report;
    report.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, report));
    QVERIFY(!report.events.empty());
}
void CoreTests::rotary_sweep() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    p.machine.headPivotA = p.machine.headPivotB = Vec3::Zero();
    p.toolhead.components[0].mesh = box_mesh(Vec3(10, 0, 0), Vec3(.2, .2, .2));
    p.fixtures.push_back({"Rotary obstacle", box_mesh(Vec3(10, 0, 0), Vec3(.1, .5, .5)), false});
    CollisionScene scene(p);
    JointState a, b;
    a.xyz = b.xyz = Vec3::Zero();
    a.b = -.8;
    b.b = .8;
    QCOMPARE(scene.check_pose(a, false, Vec3::Zero()).status, Status::Clear);
    QCOMPARE(scene.check_pose(b, false, Vec3::Zero()).status, Status::Clear);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, Vec3::Zero(), Vec3::Zero(), 1, 0, 1, r));
}
void CoreTests::containment() {
    auto p = small();
    p.toolhead.components[0].mesh = box_mesh(Vec3::Zero(), Vec3(10, 10, 10));
    p.fixtures.push_back({"Contained object", box_mesh(Vec3::Zero(), Vec3(1, 1, 1)), false});
    CollisionScene scene(p);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Collision);
}
void CoreTests::deposited_only() {
    auto p = small();
    p.part = box_mesh(Vec3::Zero(), Vec3(20, 20, 20));
    CollisionScene scene(p);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Clear);
    scene.deposit(Vec3(-1, 0, 0), Vec3(1, 0, 0), .4, .2, Role::Wall);
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Collision);
}
void CoreTests::tip_contact_and_heater() {
    auto p = small();
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .15, 2);
    p.toolhead.components[0].nozzle = true;
    p.toolhead.contactHeight = .6;
    CollisionScene scene(p);
    scene.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    JointState q;
    q.xyz = Vec3(0, 0, .1);
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Clear);
    Component heater;
    heater.name = "Heater";
    heater.mesh = box_mesh(Vec3::Zero(), Vec3(.5, .5, .5));
    p.toolhead.components.push_back(heater);
    CollisionScene next(p);
    next.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    QCOMPARE(next.check_pose(q, true, q.xyz).status, Status::Collision);
    p.toolhead.components.pop_back();
    CollisionScene neighboring(p);
    neighboring.deposit(Vec3(-.4, 0, 1), Vec3(.4, 0, 1), .4, .2, Role::Wall);
    QCOMPARE(neighboring.check_pose(q, true, q.xyz).status, Status::Collision);
}
static VolumeMesh unit_tetra() {
    VolumeMesh mesh;
    mesh.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
    mesh.cells.push_back({{0, 1, 2, 3}});
    return mesh;
}
void CoreTests::fea_patch() {
    auto mesh = unit_tetra();
    LoadCase load;
    load.fixedNodes = {0, 1, 2};
    load.nodalForces = {{3, Vec3(0, 0, -1)}};
    load.material.young = 1000;
    load.material.poisson = .25;
    auto result = solve_fea(mesh, load);
    QVERIFY(std::abs(result.displacement[3].z() + .005) < 1e-10);
    QVERIFY(std::abs(result.stress.tensors[0](2, 2) + 6) < 1e-8);
    QVERIFY(result.compliance > 0);
    load.fixedNodes = {0};
    bool failed = false;
    try {
        solve_fea(mesh, load);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
}
static void write_vtu(const QString &filename, const QString &arrays, bool mismatched = false) {
    QFile f(filename);
    QVERIFY(f.open(QIODevice::WriteOnly));
    QString xml =
        QString("<VTKFile type='UnstructuredGrid' version='0.1' "
                "byte_order='LittleEndian'><UnstructuredGrid><Piece NumberOfPoints='4' "
                "NumberOfCells='1'><Points><DataArray type='Float64' NumberOfComponents='3' format='ascii'>0 "
                "0 0 %1 0 0 0 1 0 0 0 1</DataArray></Points><Cells><DataArray type='Int32' "
                "Name='connectivity' format='ascii'>0 1 2 3</DataArray><DataArray type='Int32' "
                "Name='offsets' format='ascii'>4</DataArray><DataArray type='UInt8' Name='types' "
                "format='ascii'>10</DataArray></Cells><PointData>%2</PointData></Piece></UnstructuredGrid></"
                "VTKFile>")
            .arg(mismatched ? 2 : 1)
            .arg(arrays);
    f.write(xml.toUtf8());
}
void CoreTests::vtu_tensors() {
    QTemporaryDir dir;
    QString file = dir.filePath("tensor.vtu");
    write_vtu(file, "<DataArray type='Float64' Name='stress' NumberOfComponents='6' format='ascii'>10 1 1 0 "
                    "0 0 10 1 1 0 0 0 10 1 1 0 0 0 10 1 1 0 0 0</DataArray>");
    VtuOptions o;
    o.array = "stress";
    o.stressScale = 2;
    o.alignment = rotation_z(pi / 2);
    auto field = import_vtu({file}, o);
    QVERIFY(field.directional);
    bool covered;
    auto t = field.sample(Vec3(-.1, .1, .1), &covered);
    QVERIFY(covered);
    QVERIFY(std::abs(t(1, 1) - 20) < 1e-8);
    QVERIFY(std::abs(principal_tensile_direction(t).y()) > .999);
    field.sample(Vec3(50, 0, 0), &covered);
    QVERIFY(!covered);
}
void CoreTests::vtu_components_and_mismatch() {
    QTemporaryDir dir;
    QStringList files;
    for (int i = 0; i < 6; ++i) {
        auto file = dir.filePath(QString::number(i) + ".vtu");
        write_vtu(file,
                  QString("<DataArray type='Float64' Name='value' format='ascii'>%1 %1 %1 %1</DataArray>")
                      .arg(i + 1));
        files << file;
    }
    VtuOptions o;
    auto field = import_vtu(files, o);
    QVERIFY(field.directional);
    QCOMPARE(field.tensors[0](0, 1), 4.0);
    write_vtu(files[5], "<DataArray type='Float64' Name='value' format='ascii'>1 1 1 1</DataArray>", true);
    bool failed = false;
    try {
        import_vtu(files, o);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
    auto scalar = import_vtu({files[0]}, o);
    QVERIFY(!scalar.directional);
    QCOMPARE(scalar.scalar.size(), size_t(4));
}
void CoreTests::step_roundtrip() {
    QTemporaryDir dir;
    auto path = dir.filePath("nested.step");
    write_demo_step(path, true);
    auto model = load_step_assembly(path);
    QCOMPARE(model.components.size(), size_t(5));
    std::set<QString> ids;
    double maxX = -1e100;
    for (auto &c : model.components) {
        ids.insert(c.id);
        maxX = std::max(maxX, c.mesh.bounds().max().x());
        QVERIFY(!c.mesh.empty());
    }
    QCOMPARE(ids.size(), size_t(5));
    QVERIFY(maxX > 30);
    QVERIFY(!model.sourceStep.isEmpty());
    QCOMPARE(model.sourceHash.size(), 64);
}
void CoreTests::malformed_step() {
    QTemporaryDir dir;
    auto path = dir.filePath("bad.step");
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write("not a STEP file");
    file.close();
    bool failed = false;
    try {
        load_step_assembly(path);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
}
void CoreTests::project_roundtrip_and_stale_export() {
    QTemporaryDir dir;
    auto p = small();
    Sample a, b;
    a.joint.xyz = a.position = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    p.path = {a, b};
    p.report = validate_motion(p);
    QVERIFY(p.report.exportable());
    auto file = dir.filePath("project.5axis");
    save_project(p, file);
    auto loaded = load_project(file);
    QCOMPARE(loaded.fingerprint(), p.fingerprint());
    QVERIFY(!export_gcode(loaded).isEmpty());
    loaded.machine.clearance += .1;
    bool blocked = false;
    try {
        export_gcode(loaded);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::simulation_backward() {
    auto p = small();
    Sample a, b, c;
    a.position = a.joint.xyz = Vec3(0, 0, 1);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 1);
    b.deltaE = .1;
    b.time = 1;
    c = b;
    c.position = c.joint.xyz = Vec3(2, 0, 1);
    c.time = 2;
    p.path = {a, b, c};
    auto later = simulation_at(p, 1.5), earlier = simulation_at(p, .5), again = simulation_at(p, 1.5);
    QCOMPARE(later.deposited.size(), size_t(2));
    QCOMPARE(earlier.deposited.size(), size_t(1));
    QVERIFY((earlier.deposited[0].second - Vec3(.5, 0, 1)).norm() < 1e-9);
    QCOMPARE(later.joint.xyz.x(), again.joint.xyz.x());
}
void CoreTests::controller_replay() {
    for (auto layout : {Layout::RotaryBed, Layout::TiltingHead})
        for (auto controller : {Controller::Duet, Controller::Klipper}) {
            auto p = small();
            p.machine.layout = layout;
            p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, .5), .1, 1);
            p.toolhead.components[0].nozzle = true;
            p.machine.controller = controller;
            Sample a, b;
            a.joint.xyz = a.position = Vec3(0, 0, 5);
            b = a;
            b.joint.a = .1;
            b.joint.b = .2;
            b.position = part_position(p.machine, b.joint);
            b.direction = part_direction(p.machine, p.toolhead, b.joint);
            b.time = 1;
            b.deltaE = .02;
            b.role = Role::Wall;
            p.path = {a, b};
            p.report = validate_motion(p);
            QVERIFY(p.report.exportable());
            auto g = export_gcode(p);
            auto replay = replay_gcode(p, g);
            QVERIFY((replay.path.back().joint.xyz - b.joint.xyz).norm() < 1e-6);
            QVERIFY(std::abs(replay.path.back().joint.a - b.joint.a) < 1e-8);
            QVERIFY(std::abs(replay.path.back().deltaE - b.deltaE) < 1e-8);
        }
}
void CoreTests::head_motor_interpolation() {
    MachineProfile m;
    m.layout = Layout::TiltingHead;
    ToolheadAssembly t;
    t.tcp = Vec3(3, 0, 2);
    JointState a, b;
    a.xyz = b.xyz = Vec3(10, 10, 20);
    b.a = .4;
    b.b = .5;
    auto left = controller_axes(m, t, a), right = controller_axes(m, t, b),
         mid = interpolate_motion(m, t, a, b, .5);
    QVERIFY((controller_axes(m, t, mid).xyz - (left.xyz + right.xyz) / 2).norm() < 1e-9);
    QVERIFY((mid.xyz - a.xyz).norm() > .01);
    Component c;
    QVERIFY((component_pose(m, t, c, mid) * t.tcp - mid.xyz).norm() < 1e-9);
    m.layout = Layout::RotaryBed;
    m.minXYZ = Vec3(-10, -10, -10);
    m.maxXYZ = Vec3(10, 10, 10);
    t.tcp = Vec3(30, 0, 0);
    a.xyz = Vec3(35, 0, 5);
    a.a = a.b = 0;
    QVERIFY(in_limits(m, t, a));
    QVERIFY(!inverse_kinematics(m, a.xyz, Vec3::UnitZ(), a, t).empty());
}
void CoreTests::conical_nozzle_contact() {
    auto p = small();
    auto &c = p.toolhead.components[0];
    c.mesh = cylinder_mesh(Vec3(0, 0, 1), .1, 2);
    c.nozzle = true;
    for (auto &point : c.mesh.vertices)
        if (point.z() > 1) {
            point.x() *= 8;
            point.y() *= 8;
        }
    p.toolhead.contactHeight = .75;
    p.toolhead.contactRadius = .55;
    CollisionScene scene(p);
    scene.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Clear);
    scene.deposit(Vec3(-.4, 0, 1), Vec3(.4, 0, 1), .4, .2, Role::Wall);
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Collision);
}
void CoreTests::bounded_interface() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    auto &a = p.toolhead.components[0];
    a.id = "carriage";
    a.link = Link::Carriage;
    a.mesh = box_mesh(Vec3::Zero(), Vec3(4, 4, 4));
    Component b;
    b.id = b.name = "stage";
    b.mesh = box_mesh(Vec3::Zero(), Vec3(1, 1, 1));
    b.link = Link::Nozzle;
    p.toolhead.components.push_back(b);
    JointState q;
    q.xyz = Vec3(0, 0, 5);
    CollisionScene original(p);
    QCOMPARE(original.check_pose(q, false, q.xyz).status, Status::Collision);
    p.toolhead.interfaces.push_back({"carriage", "stage", Vec3::Zero(), Vec3(2, 2, 2)});
    CollisionScene allowed(p);
    QCOMPARE(allowed.check_pose(q, false, q.xyz).status, Status::Clear);
    p.toolhead.components[1].mesh = box_mesh(Vec3(1.4, 0, 0), Vec3(1, 1, 1));
    CollisionScene outside(p);
    QCOMPARE(outside.check_pose(q, false, q.xyz).status, Status::Collision);
}
void CoreTests::fem_step_metadata() {
    QTemporaryDir dir;QStringList files;
    for(int i=0;i<6;++i){QString file=dir.filePath(QString::number(i)+".vtu");
        write_vtu(file,"<DataArray type='Float64' Name='value' format='ascii'>1 1 1 1</DataArray>");
        QFile input(file);QVERIFY(input.open(QIODevice::ReadOnly));auto xml=input.readAll();input.close();
        xml.replace("<UnstructuredGrid>",QByteArray("<UnstructuredGrid><FieldData><DataArray type='Float64' Name='TimeValue' NumberOfTuples='1' format='ascii'>")+
                    QByteArray::number(i==5?2:1)+"</DataArray></FieldData>");
        QVERIFY(input.open(QIODevice::WriteOnly|QIODevice::Truncate));input.write(xml);input.close();files<<file;
    }
    VtuOptions options;bool rejected=false;try{import_vtu(files,options);}catch(...){rejected=true;}QVERIFY(rejected);
    options.resultStep="2";rejected=false;try{import_vtu({files[0]},options);}catch(...){rejected=true;}QVERIFY(rejected);
}
void CoreTests::scalar_field_slice() {
    auto p = demo_project();
    p.part = box_mesh(Vec3(0, 0, 1), Vec3(6, 6, 2));
    p.process.infill = .3;
    slice_project(p);
    QVERIFY(p.path.size() > 10);
    QVERIFY(p.depositionValid);
    for (auto &s : p.path) {
        QVERIFY(s.position.allFinite());
        QVERIFY(in_limits(p.machine, s.joint));
    }
}
void CoreTests::blocked_repair() {
    auto p = small();
    p.fixtures.push_back({"Enclosure", box_mesh(Vec3(0, 0, 5), Vec3(20, 20, 20)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 5);
    b.time = 1;
    p.path = {a, b};
    QVERIFY(!repair_motion(p));
    bool blocked = false;
    try {
        export_gcode(p);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::stale_kinematics() {
    auto p = small();
    Sample a, b;
    a.position = a.joint.xyz = Vec3(5, 0, 5);
    b = a;
    b.joint.a = .4;
    b.position = part_position(p.machine, b.joint);
    b.direction = part_direction(p.machine, p.toolhead, b.joint);
    b.time = 1;
    p.path = {a, b};
    p.machine.bedPivot = Vec3(50, 20, 0);
    QCOMPARE(validate_motion(p).status, Status::Unresolved);
}
void CoreTests::clearance_threshold() {
    auto p = small();
    p.fixtures.push_back({"Plate", box_mesh(Vec3(0, 0, -.6), Vec3(2, 2, 1)), false});
    JointState q;
    q.xyz = Vec3(0, 0, .1);
    p.machine.clearance = .09;
    CollisionScene scene(p);
    QCOMPARE(scene.check_pose(q, false, q.xyz).status, Status::Clear);
    p.machine.clearance = .11;
    QCOMPARE(scene.check_pose(q, false, q.xyz).status, Status::Collision);
    p.toolhead.components[0].mesh.error = .03;
    p.machine.clearance = .09;
    CollisionScene uncertain(p);
    QCOMPARE(uncertain.check_pose(q, false, q.xyz).status, Status::Collision);
}
void CoreTests::successful_travel_repair() {
    auto p = small();
    p.machine.minXYZ = Vec3(-6, -6, 3);
    p.machine.maxXYZ = Vec3(6, 6, 7);
    p.machine.minA = p.machine.minB = -.2;
    p.machine.maxA = p.machine.maxB = .2;
    p.fixtures.push_back({"Thin obstacle", box_mesh(Vec3(0, 0, 5), Vec3(.1, 2, 2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-5, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    b.deltaE = -.8;
    b.role = Role::Retract;
    p.path = {a, b};
    QVERIFY(repair_motion(p));
    QVERIFY(p.path.size() > 2);
    double E = 0;
    for (auto &s : p.path)
        E += s.deltaE;
    QVERIFY(std::abs(E + .8) < 1e-9);
    auto again = small();
    again.machine = p.machine;
    again.fixtures = p.fixtures;
    again.path = {a, b};
    QVERIFY(repair_motion(again));
    QCOMPARE(again.path.size(), p.path.size());
    for (size_t i = 0; i < p.path.size(); ++i)
        QVERIFY((again.path[i].joint.xyz - p.path[i].joint.xyz).norm() < 1e-9);
}
void CoreTests::quadratic_tetra_mapping() {
    auto mesh = unit_tetra();
    const int edges[][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (auto &e : edges)
        mesh.vertices.push_back((mesh.vertices[e[0]] + mesh.vertices[e[1]]) / 2);
    mesh.vertices[4].z() += .1;
    mesh.cells[0].nodes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Eigen::Vector4d w;
    w << .4, .2, .3, .1;
    Vec3 point = Vec3::Zero();
    for (int j = 0; j < 4; ++j)
        point += w[j] * (2 * w[j] - 1) * mesh.vertices[j];
    for (int j = 0; j < 6; ++j)
        point += 4 * w[edges[j][0]] * w[edges[j][1]] * mesh.vertices[4 + j];
    Eigen::Vector4d mapped;
    QVERIFY(barycentric(mesh, mesh.cells[0], point, mapped));
    QVERIFY((mapped - w).norm() < 1e-7);
}
void CoreTests::step_units() {
    QTemporaryDir dir;
    auto path = dir.filePath("inches.step");
    write_demo_step(path, false, true);
    auto model = load_step_assembly(path);
    QVERIFY(!model.sourceUnits.isEmpty());
    double max = 0;
    for (auto &c : model.components)
        max = std::max(max, c.mesh.bounds().max().z());
    QVERIFY(std::abs(max - 24) < 1e-4);
    auto cached = load_step_assembly(path);
    QCOMPARE(cached.components.size(), model.components.size());
    QVERIFY(!cached.components.front().renderMesh.empty());
}
void CoreTests::curved_strength_layers() {
    auto p = demo_project();
    p.part = box_mesh(Vec3(0, 0, 4), Vec3(5, 5, 8));
    p.process.layerHeight = .4;
    p.process.width = .6;
    p.volume = tetrahedralize(p.part, .5);
    p.stress.mesh = p.volume;
    p.stress.directional = true;
    Vec3 direction = Vec3(1, 0, 1).normalized();
    Mat3 tensor = Mat3::Identity() + 10 * direction * direction.transpose();
    p.stress.tensors.assign(p.volume.vertices.size(), tensor);
    slice_project(p);
    double tilt = 0;
    for (auto &s : p.path)
        tilt = std::max(tilt, std::abs(s.joint.a));
    QVERIFY(tilt > pi / 180);
    QVERIFY(p.deposition.checked);
    QVERIFY(p.deposition.coverage > .9);
}
void CoreTests::playback_invariance() {
    auto p = small();
    p.fixtures.push_back({"Plate", box_mesh(Vec3(0, 0, 5), Vec3(.01, 2, 2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-5, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    p.path = {a, b};
    auto r = validate_motion(p);
    for (double speed : {.25, 1.0, 10.0})
        for (double t = 0; t < 1; t += speed / 60)
            simulation_at(p, t, false);
    auto after = validate_motion(p);
    QCOMPARE(r.events.size(), after.events.size());
    QCOMPARE(r.checkedIntervals, after.checkedIntervals);
    QCOMPARE(r.events[0].time, after.events[0].time);
}
void CoreTests::unsupported_deposition() {
    auto p = small();
    Sample a, b;
    a.position = a.joint.xyz = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 5);
    b.time = 1;
    b.deltaE = bead_area(b.width, b.height) / (pi * std::pow(1.75 / 2, 2));
    b.role = Role::Wall;
    p.path = {a, b};
    check_deposition(p);
    QVERIFY(!p.depositionValid);
    QVERIFY(!p.deposition.unsupportedMoves.empty());
    p.report = validate_motion(p);
    bool blocked = false;
    try {
        export_gcode(p);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::nozzle_lift_and_penetration() {
    auto p = small();
    p.machine.clearance = .25;
    p.toolhead.components[0].nozzle = true;
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .15, 2);
    p.toolhead.contactHeight = .75;
    CollisionScene scene(p);
    scene.deposit(Vec3(-1, 0, .5), Vec3(1, 0, .5), .4, .2, Role::Wall);
    JointState a, b;
    a.xyz = Vec3(0, 0, .5001);
    b.xyz = Vec3(0, 0, 3);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, r));
    a.xyz.z() = .4;
    QCOMPARE(scene.check_pose(a, false, a.xyz).status, Status::Collision);
}
void CoreTests::successful_orientation_repair() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .1, 2);
    p.toolhead.components[0].nozzle = true;
    Component shroud;
    shroud.name = "Shroud";
    shroud.mesh = box_mesh(Vec3(1.5, 0, 8), Vec3(.4, .4, .4));
    p.toolhead.components.push_back(shroud);
    p.fixtures.push_back({"Bed", box_mesh(Vec3(0, 0, -.5), Vec3(20, 20, 1)), true});
    p.fixtures.push_back({"Obstacle", box_mesh(Vec3(1.5, 0, 8.2), Vec3(.2, .2, .2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-2, 0, .2);
    b = a;
    b.position = b.joint.xyz = Vec3(0, 0, .2);
    b.time = 1;
    b.role = Role::Wall;
    b.deltaE = 2 * bead_area(b.width, b.height) / (pi * std::pow(1.75 / 2, 2));
    p.path = {a, b};
    QCOMPARE(validate_motion(p).status, Status::Collision);
    QVERIFY(repair_motion(p));
    QVERIFY(p.path.back().direction.dot(Vec3::UnitZ()) >= std::cos(p.process.directionCone) - 1e-7);
}
void CoreTests::cavity_meshing() {
    Mesh m = box_mesh(Vec3::Zero(), Vec3(6, 6, 6));
    auto inner = box_mesh(Vec3::Zero(), Vec3(2, 2, 2));
    int offset = int(m.vertices.size());
    m.vertices.insert(m.vertices.end(), inner.vertices.begin(), inner.vertices.end());
    for (auto face : inner.triangles) {
        std::swap(face[1], face[2]);
        m.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
    }
    auto volume = tetrahedralize(m, 2);
    QVERIFY(!volume.cells.empty());
    for (auto &cell : volume.cells) {
        Vec3 center = Vec3::Zero();
        for (int i = 0; i < 4; ++i)
            center += volume.vertices[cell.nodes[i]] / 4;
        QVERIFY(center.cwiseAbs().maxCoeff() > 1 - 1e-6);
    }
}
void CoreTests::rotary_lever_bound() {
    MachineProfile m;
    ToolheadAssembly t;
    JointState a, b;
    a.xyz = Vec3(1000, 0, 0);
    b = a;
    b.b = .1;
    double actual = (part_position(m, b) - part_position(m, a)).norm();
    QVERIFY(actual <= movement_bound(m, t, a, b, 1));
}
void CoreTests::unresolved_budget() {
    auto p = small();
    p.machine.clearance = .25;
    p.fixtures.push_back({"Threshold plane", box_mesh(Vec3(0, 0, -.85), Vec3(2, 2, 1)), false});
    JointState a, b;
    a.xyz = Vec3(-.5, 0, 0);
    b.xyz = Vec3(.5, 0, 0);
    CollisionScene scene(p);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, r));
    QCOMPARE(r.status, Status::Unresolved);
    QVERIFY(r.checkedIntervals <= 2048);
    QVERIFY(!r.events.empty());
}
QTEST_GUILESS_MAIN(CoreTests)
#include "core_tests.moc"
