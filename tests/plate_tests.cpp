#include "core/collision.h"
#include "core/compute.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/plate.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/stress.h"
#include <QCryptographicHash>
#include <QJsonArray>
#include <QTemporaryDir>
#include <QtTest>
using namespace fiveaxis;
class PlateTests : public QObject {
    Q_OBJECT
  private:
    Project fixture() {
        Project p;
        p.machine.layout = Layout::Cartesian3Axis;
        p.machine.controller = Controller::Duet;
        p.machine.clearance = .01;
        p.process.curved = false;
        p.part = box_mesh(Vec3(0, 0, .2), Vec3(4, 4, .4));
        Component c;
        c.id = c.name = "nozzle";
        c.nozzle = true;
        c.mesh = cylinder_mesh(Vec3(0, 0, .3), .05, .2);
        p.toolhead.components.push_back(c);
        Sample a;
        a.position = a.joint.xyz = Vec3(-2, 0, .2);
        Sample b = a;
        b.position = b.joint.xyz = Vec3(2, 0, .2);
        b.role = Role::Wall;
        b.deltaE = 4 * bead_area(b.width, b.height) / (pi * std::pow(1.75 / 2, 2));
        p.path = {a, b};
        time_path(p);
        p.depositionValid = true;
        migrate_single_project(p);
        p.plateEnabled = true;
        p.depositionValid = true;
        p.plate.bedTemperature = 60;
        return p;
    }
  private Q_SLOTS:
    void schema_migration() {
        auto p = fixture();
        auto json = project_json(p);
        QCOMPARE(json["schema"].toInt(), 2);
        auto restored = project_from_json(json);
        QCOMPARE(restored.objects.size(), size_t(1));
        QCOMPARE(restored.tools.size(), size_t(1));
        QCOMPARE(restored.fingerprint(), p.fingerprint());
        auto legacy = json;
        legacy["schema"] = 1;
        legacy.remove("plateJob");
        auto migrated = project_from_json(legacy);
        QCOMPARE(migrated.objects.size(), size_t(1));
        QVERIFY(!migrated.report.exportable());
        QVERIFY(!migrated.depositionValid);
        QTemporaryDir directory;
        save_project(p, directory.filePath("plate.5axis"));
        auto loaded = load_project(directory.filePath("plate.5axis"));
        QCOMPARE(loaded.fingerprint(), p.fingerprint());
    }
    void deduplicated_assets() {
        auto p = fixture();
        p.tools.front().assembly.sourceStep = "ISO-10303-21;test fixture bytes";
        p.tools.front().assembly.sourceHash = QString::fromLatin1(
            QCryptographicHash::hash(p.tools.front().assembly.sourceStep, QCryptographicHash::Sha256)
                .toHex());
        p.tools.front().nozzle.geometryHash = p.tools.front().assembly.sourceHash;
        auto second = p.tools.front();
        second.id = "tool-2";
        second.controllerTool = 1;
        p.tools.push_back(second);
        auto json = project_json(p);
        QCOMPARE(json["plateJob"].toObject()["assets"].toObject().size(), 1);
        auto loaded = project_from_json(json);
        QCOMPARE(loaded.tools[1].assembly.sourceStep, p.tools[0].assembly.sourceStep);
        auto changed = json;
        auto job = changed["plateJob"].toObject();
        auto assets = job["assets"].toObject();
        assets[assets.keys().front()] = "Y29ycnVwdA==";
        job["assets"] = assets;
        changed["plateJob"] = job;
        QVERIFY_EXCEPTION_THROWN(project_from_json(changed), std::runtime_error);
    }
    void nozzle_geometry_and_invalidation() {
        auto p = fixture();
        auto &t = p.tools.front();
        auto old = t.assembly.components[0].mesh.vertices;
        t.nozzle.diameter = .8;
        t.nozzle.calibrated = false;
        QVERIFY_EXCEPTION_THROWN(validate_plate_setup(p), std::runtime_error);
        QCOMPARE(t.assembly.components[0].mesh.vertices.size(), old.size());
        for (size_t i = 0; i < old.size(); ++i)
            QVERIFY((old[i] - t.assembly.components[0].mesh.vertices[i]).norm() < 1e-12);
        t.useDummy = true;
        t.dummy = make_dummy_toolhead(.8);
        t.nozzle.calibrated = true;
        QVERIFY(installed_assembly(t).components.size() >= 4);
        auto fingerprint = p.fingerprint();
        p.compute.threads = 1;
        QCOMPARE(p.fingerprint(), fingerprint);
        p.tools[0].nozzle.diameter = .6;
        QVERIFY(p.fingerprint() != fingerprint);
    }
    void scoped_component_identity() {
        auto p = fixture();
        auto shared = p.tools[0].assembly.components[0];
        shared.nozzle = false;
        shared.link = Link::Carriage;
        shared.mesh = box_mesh(Vec3(0, 0, 5), Vec3(1, 1, 1));
        p.carriageAssembly.components = {shared};
        auto active = assembly_at(p, "tool-1");
        QCOMPARE(active.components.size(), size_t(2));
        QVERIFY(active.components[0].nozzle);
        QCOMPARE(active.components[1].id, QString("carriage/nozzle"));
        auto empty = assembly_at(p, {});
        QCOMPARE(empty.components.size(), size_t(1));
        QCOMPARE(empty.components[0].id, QString("carriage/nozzle"));
        QVERIFY(!empty.components[0].nozzle);
    }
    void tool_offsets_no_carriage_jump() {
        auto p = fixture();
        auto second = p.tools[0];
        second.id = "tool-2";
        second.controllerTool = 1;
        second.assembly.tcp = Vec3(1, 0, 0);
        p.tools.push_back(second);
        Sample first = p.path.back();
        first.carriage = JointState{};
        first.carriage->xyz = Vec3(0, 0, 5);
        first.joint = pose_from_axes(p.machine, assembly_at(p, first.toolId), *first.carriage);
        Sample next = first;
        next.toolId = second.id;
        next.event = EventKind::Attach;
        next.argument = second.id;
        next.joint = pose_from_axes(p.machine, assembly_at(p, next.toolId), *next.carriage);
        next.position = next.joint.xyz;
        first.deltaE = next.deltaE = 0;
        p.path = {first, next};
        time_plate(p);
        QVERIFY((sample_axes(p, p.path[0]).xyz - sample_axes(p, p.path[1]).xyz).norm() < 1e-12);
        auto state = simulation_at(p, p.path.back().time);
        QCOMPARE(state.toolId, second.id);
        QVERIFY((state.joint.xyz - Vec3(1, 0, 5)).norm() < 1e-12);
        auto before = simulation_at(p, 0);
        QCOMPARE(before.toolId, first.toolId);
    }
    void controller_replay_data() {
        QTest::addColumn<int>("controller");
        QTest::newRow("Duet") << 0;
        QTest::newRow("Klipper") << 1;
        QTest::newRow("XYZ") << 2;
    }
    void controller_replay() {
        QFETCH(int, controller);
        auto p = fixture();
        p.machine.controller = Controller(controller);
        p.report = validate_motion(p);
        QVERIFY2(p.report.exportable(),
                 p.report.events.empty() ? "no report" : qPrintable(p.report.events.front().reason));
        auto code = export_gcode(p);
        auto replay = replay_gcode(p, code);
        QCOMPARE(replay.path.size(), p.path.size());
        for (size_t i = 0; i < p.path.size(); ++i) {
            QVERIFY((sample_axes(p, replay.path[i]).xyz - sample_axes(p, p.path[i]).xyz).norm() < 1e-8);
            QVERIFY(std::abs(replay.path[i].deltaE - p.path[i].deltaE) < 1e-8);
        }
        auto malicious = code;
        malicious.replace("G1 X2.000000000", "G1 X2.000000000 A30");
        if (controller == 2)
            QVERIFY_EXCEPTION_THROWN(replay_gcode(p, malicious), std::runtime_error);
    }
    void collision_prefix_and_threads() {
        auto p = fixture();
        Sample a = p.path.back();
        a.deltaE = 0;
        a.role = Role::Travel;
        a.position = a.joint.xyz = Vec3(-2, 0, .2);
        p.path.push_back(a);
        time_plate(p);
        p.compute.threads = 1;
        auto one = validate_motion(p);
        p.compute.threads = 4;
        auto many = validate_motion(p);
        QCOMPARE(one.status, many.status);
        QVERIFY(one.status != Status::Clear);
        QCOMPARE(one.events.size(), many.events.size());
        auto future = fixture();
        future.path[1].deltaE = 0;
        future.depositionValid = true;
        auto report = validate_motion(future);
        QVERIFY(report.exportable());
    }
    void thin_obstacle() {
        auto p = fixture();
        p.path[1].deltaE = 0;
        p.path[0].joint.xyz.z() = p.path[1].joint.xyz.z() = 3;
        for (auto &s : p.path) {
            s.position = s.joint.xyz;
            s.carriage = controller_axes(p.machine, assembly_at(p, s.toolId), s.joint);
        }
        p.fixtures.push_back({"Thin obstacle", box_mesh(Vec3(0, 0, 3.3), Vec3(.01, 1, 1)), false});
        time_plate(p);
        QVERIFY(!validate_motion(p).exportable());
    }
    void final_bead_birth_collision() {
        auto p = fixture();
        p.fixtures.push_back({"Final bead obstacle", box_mesh(Vec3(2.20, 0, .1), Vec3(.02, .1, .1)), false});
        auto report = validate_motion(p);
        QVERIFY(!report.exportable());
        QVERIFY(std::any_of(report.events.begin(), report.events.end(), [](const CollisionEvent &e) {
            return e.second == "Final bead obstacle" && e.first.startsWith("Deposited");
        }));
    }
    void disconnected_step_containment() {
        auto p = fixture();
        p.plateEnabled = false;
        auto &component = p.toolhead.components[0];
        component.nozzle = false;
        component.mesh = box_mesh(Vec3(10, 0, 1), Vec3(1, 1, 1));
        auto inside = box_mesh(Vec3(0, 0, 1), Vec3(1, 1, 1));
        int offset = component.mesh.vertices.size();
        component.mesh.vertices.insert(component.mesh.vertices.end(), inside.vertices.begin(),
                                       inside.vertices.end());
        for (auto face : inside.triangles)
            component.mesh.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
        p.fixtures.push_back({"Containing fixture", box_mesh(Vec3(0, 0, 1), Vec3(3, 3, 3)), false});
        QVERIFY(contains(p.fixtures[0].mesh, inside.vertices.front()));
        CollisionScene scene(p);
        JointState pose;
        pose.xyz = Vec3::Zero();
        auto hit = scene.check_pose(pose, false, Vec3::Zero());
        QVERIFY(hit.status != Status::Clear);
    }
    void backward_restore() {
        auto p = fixture();
        auto full = simulation_at(p, p.path.back().time);
        auto beginning = simulation_at(p, 0);
        QVERIFY(full.filamentUsed["material-1"] > 0);
        QCOMPARE(beginning.filamentUsed["material-1"], 0.);
        QCOMPARE(beginning.deposited.size(), size_t(0));
        QVERIFY(!full.completedObjects.empty());
    }
    void deterministic_pool() {
        std::vector<size_t> values(1000);
        ComputeSettings s;
        s.threads = 1;
        parallel_for(values.size(), s, [&](size_t i) { values[i] = i * i; });
        auto expected = values;
        s.threads = 8;
        parallel_for(values.size(), s, [&](size_t i) { values[i] = i * i; });
        QVERIFY(values == expected);
        std::atomic_bool cancelled{true};
        QVERIFY_EXCEPTION_THROWN(parallel_for(100, s, [](size_t) {}, &cancelled), std::runtime_error);
    }
    void compute_parity_data() {
        QTest::addColumn<int>("backend");
        QTest::newRow("CPU") << 1;
        QTest::newRow("CUDA") << 2;
        QTest::newRow("Vulkan") << 3;
    }
    void compute_parity() {
        QFETCH(int, backend);
        std::vector<Vec3> vertices(8192);
        for (size_t i = 0; i < vertices.size(); ++i)
            vertices[i] = Vec3(double(i) * .01, std::sin(double(i)), 1e6 + double(i));
        auto transform = translation(Vec3(1, 2, 3)) * rotation_x(.4);
        ComputeSettings settings;
        settings.backend = ComputeBackend(backend);
        ComputeReport report;
        auto result = compute_transform(vertices, transform, settings, report);
        for (size_t i = 0; i < vertices.size(); ++i)
            QVERIFY((result[i] - transform * vertices[i]).norm() < 1e-8);
        if (backend != 1 && report.backend == "CPU")
            QSKIP(qPrintable("GPU unavailable/fallback: " + report.fallback));
        QCOMPARE(report.backend, backend == 1   ? QString("CPU")
                                 : backend == 2 ? QString("CUDA")
                                                : QString("Vulkan"));
        QVERIFY(report.batches > 0 || backend == 1);
    }
    void unavailable_gpu_and_nonfinite() {
        ComputeSettings s;
        s.backend = ComputeBackend::CUDA;
        s.device = "nonexistent GPU";
        ComputeReport report;
        auto result = compute_transform({Vec3(1, 2, 3)}, Transform::Identity(), s, report);
        QCOMPARE(report.backend, QString("CPU"));
        QVERIFY(!report.fallback.isEmpty());
        QVERIFY((result.front() - Vec3(1, 2, 3)).norm() < 1e-12);
        QVERIFY_EXCEPTION_THROWN(compute_transform({Vec3(NAN, 0, 0)}, Transform::Identity(), s, report),
                                 std::runtime_error);
    }
    void plate_generation() {
        auto p = plate_demo();
        p.compute.threads = 1;
        try {
            slice_project(p);
        } catch (const std::exception &e) {
            QFAIL(e.what());
        }
        QVERIFY(p.path.size() > 10);
        QVERIFY(p.events.size() > 0);
        QSet<QString> objects;
        for (const auto &s : p.path)
            if (s.deltaE > 0) {
                objects.insert(s.objectId);
                QVERIFY(s.carriage.has_value());
                QCOMPARE(s.toolId, QString("tool-1"));
                QVERIFY(std::abs(s.joint.a) + std::abs(s.joint.b) < 1e-12);
            }
        QCOMPARE(objects.size(), 2);
        auto single = p.path;
        p.compute.threads = 4;
        slice_project(p);
        QCOMPARE(p.path.size(), single.size());
        for (size_t i = 0; i < single.size(); ++i)
            QVERIFY((p.path[i].position - single[i].position).norm() < 1e-10);
    }
    void strict_replay_and_state() {
        auto p = fixture();
        p.report = validate_motion(p);
        auto code = export_gcode(p);
        auto offsets = code;
        offsets.replace("G10 P0 X0 Y0 Z0", "G10 P0 X10 Y0 Z0");
        QVERIFY_EXCEPTION_THROWN(replay_gcode(p, offsets), std::runtime_error);
        auto hidden = code;
        hidden.replace("M400\n", "M400\nG28\n");
        QVERIFY_EXCEPTION_THROWN(replay_gcode(p, hidden), std::runtime_error);
        p.path[1].materialId = "missing";
        QVERIFY(!validate_motion(p).exportable());
    }
    void neighboring_object_contact() {
        auto p = fixture();
        CollisionScene scene(p);
        auto q = p.path.back().joint;
        scene.deposit(Vec3(1, 0, .2), Vec3(2, 0, .2), .45, .2, Role::Wall, Vec3::UnitZ(), "neighbor");
        scene.set_active_object("object-1");
        // A low tip fully inside another object's bead must never receive the active-bead allowance.
        auto view = p;
        view.toolhead = assembly_at(p, "tool-1");
        view.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, .04), .05, .08);
        view.plateEnabled = false;
        CollisionScene other(view);
        other.deposit(Vec3(1, 0, .2), Vec3(2, 0, .2), .45, .2, Role::Wall, Vec3::UnitZ(), "neighbor");
        other.set_active_object("object-1");
        q.xyz = Vec3(1.5, 0, .15);
        QVERIFY(other.check_pose(q, true, Vec3(1.5, 0, .15)).status != Status::Clear);
    }
    void gpu_workloads_data() {
        compute_parity_data();
    }
    void gpu_workloads() {
        QFETCH(int, backend);
        ComputeSettings settings;
        settings.backend = ComputeBackend(backend);
        ComputeReport report;
        std::vector<double> ranges;
        for (int i = 0; i < 8192; ++i) {
            ranges.push_back(-1);
            ranges.push_back(1);
            ranges.push_back(i % 2 ? 1. : std::nextafter(1., 2.));
        }
        auto masks = compute_workload(ranges, 1, Transform::Identity(), settings, report);
        for (size_t i = 0; i < masks.size(); ++i)
            QCOMPARE(masks[i].z(), i % 2 ? 1. : 0.);
        if (backend != 1 && report.backend == "CPU")
            QSKIP(qPrintable(report.fallback));
        QCOMPARE(report.backend, backend == 1   ? QString("CPU")
                                 : backend == 2 ? QString("CUDA")
                                                : QString("Vulkan"));
        std::vector<double> triangles;
        for (int i = 0; i < 8192; ++i) {
            double data[] = {1e6, 0, 0, 1e6 + 1, 0, 0, 1e6, 1, 0};
            triangles.insert(triangles.end(), data, data + 9);
        }
        auto normals = compute_workload(triangles, 3, Transform::Identity(), settings, report);
        for (const auto &normal : normals)
            QVERIFY((normal - Vec3::UnitZ()).norm() < 1e-12);
        QCOMPARE(report.backend, backend == 1   ? QString("CPU")
                                 : backend == 2 ? QString("CUDA")
                                                : QString("Vulkan"));
        StressField field;
        field.directional = true;
        field.mesh.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
        field.mesh.cells = {Tetrahedron{{0, 1, 2, 3}}};
        for (int i = 0; i < 4; ++i)
            field.tensors.push_back(Mat3::Identity() * (i + 1));
        std::vector<Vec3> points(4096, Vec3(.25, .25, .25));
        auto tensors = sample_stress_batch(field, points, settings, report);
        for (const auto &tensor : tensors)
            QVERIFY((tensor - 2.5 * Mat3::Identity()).norm() < 1e-12);
        QCOMPARE(report.backend, backend == 1   ? QString("CPU")
                                 : backend == 2 ? QString("CUDA")
                                                : QString("Vulkan"));
    }
    void gpu_memory_and_cancel() {
        ComputeSettings settings;
        settings.backend = ComputeBackend::CUDA;
        settings.memoryFraction = 1e-15;
        ComputeReport report;
        auto output = compute_transform(std::vector<Vec3>(8192, Vec3(1, 2, 3)), Transform::Identity(),
                                        settings, report);
        QCOMPARE(report.backend, QString("CPU"));
        QVERIFY(!report.fallback.isEmpty());
        QVERIFY((output.front() - Vec3(1, 2, 3)).norm() < 1e-12);
        std::atomic_bool cancelled{true};
        QVERIFY_EXCEPTION_THROWN(
            compute_transform({Vec3(1, 2, 3)}, Transform::Identity(), settings, report, &cancelled),
            std::runtime_error);
    }
    void mixed_material_fea_threads() {
        VolumeMesh mesh;
        mesh.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
        mesh.cells = {Tetrahedron{{0, 1, 2, 3}}};
        LoadCase loads;
        loads.fixedNodes = {0, 1, 2};
        loads.nodalForces = {{3, Vec3(0, 0, 1)}};
        ComputeSettings one;
        one.threads = 1;
        ComputeSettings many;
        many.threads = 4;
        auto a = solve_fea(mesh, loads, {}, {}, one);
        auto b = solve_fea(mesh, loads, {}, {}, many);
        QCOMPARE(a.compliance, b.compliance);
        auto material = loads.material;
        material.young *= 2;
        material.transverseYoung *= 2;
        auto stiff = solve_fea(mesh, loads, {}, {material}, many);
        QVERIFY(std::abs(stiff.compliance * 2 - a.compliance) < 1e-10);
    }
    void shared_nozzle_boundaries_and_purge() {
        auto p = plate_demo();
        p.objects.resize(1);
        p.plate.order = {"object-1"};
        p.machine.clearance = .01;
        auto m = p.materials[0];
        m.id = "material-2";
        m.name = "Soluble";
        p.materials.push_back(m);
        auto &t = p.tools[0];
        t.nozzle.reset_bead_limits();
        t.routes[0].selectCommand = "M42 P1 S0";
        t.routes.push_back({m.id, "1", "M42 P1 S1", true});
        t.change.load = t.change.unload = 5;
        p.plate.purge = {{"material-1", m.id, 1}, {m.id, "material-1", 1}};
        MaterialAssignment a;
        a.id = "modifier";
        a.kind = AssignmentKind::Volume;
        a.materialId = m.id;
        a.volume = box_mesh(Vec3(-22, 0, 1), Vec3(4, 8, 2));
        p.objects[0].assignments.push_back(a);
        try {
            slice_project(p);
        } catch (const std::exception &e) {
            QFAIL(e.what());
        }
        QSet<QString> ids;
        int load = 0, unload = 0, purges = 0;
        for (const auto &s : p.path) {
            if (s.deltaE > 0 && s.event == EventKind::Motion)
                ids.insert(s.materialId);
            load += s.event == EventKind::Load;
            unload += s.event == EventKind::Unload;
            purges += s.role == Role::Purge && s.deltaE > 0;
            if (s.role == Role::Purge && s.deltaE > 0)
                QCOMPARE(s.height, t.nozzle.minHeight);
            QCOMPARE(s.toolId, QString("tool-1"));
        }
        QCOMPARE(ids.size(), 2);
        QVERIFY(load > 0 && unload > 0 && purges > 0);
        auto state = simulation_at(p, p.path.back().time);
        QVERIFY(state.purgeUsed[m.id] > 0);
        QCOMPARE(simulation_at(p, 0).purgeUsed.size(), size_t(0));
        a.volume = box_mesh(Vec3(-22, 0, 1), Vec3(.02, 8, 2));
        p.objects[0].assignments = {a};
        QVERIFY_EXCEPTION_THROWN(slice_project(p), std::runtime_error);
    }
    void sequential_obstacles_and_budget() {
        auto p = plate_demo(PrintSequence::Objects);
        p.compute.threads = 1;
        p.plate.searchBudget = 1;
        p.machine.clearance = .01;
        p.tools[0].dummy.components.erase(p.tools[0].dummy.components.begin() + 1,
                                          p.tools[0].dummy.components.end());
        slice_project(p);
        auto report = validate_motion(p);
        QVERIFY2(report.exportable(),
                 report.events.empty() ? "no report" : qPrintable(report.events.front().reason));
        p.fixtures.push_back({"Impossible enclosure", box_mesh(Vec3(0, 0, 25), Vec3(100, 100, 60)), false});
        QVERIFY(!repair_motion(p));
        QVERIFY(p.report.status != Status::Clear);
        QVERIFY_EXCEPTION_THROWN(export_gcode(p), std::runtime_error);
    }
    void travel_repair() {
        auto p = fixture();
        p.path[1].deltaE = 0;
        for (auto &sample : p.path) {
            sample.joint.xyz.z() = sample.position.z() = 3;
            sample.carriage = controller_axes(p.machine, assembly_at(p, sample.toolId), sample.joint);
        }
        p.fixtures.push_back({"Thin obstacle", box_mesh(Vec3(0, 0, 3.3), Vec3(.01, 1, 1)), false});
        time_plate(p);
        QVERIFY(!validate_motion(p).exportable());
        QVERIFY(repair_motion(p));
        QVERIFY(p.path.size() > 2);
    }
    void automatic_sequential_order() {
        auto p = plate_demo(PrintSequence::Objects);
        p.compute.threads = 1;
        p.machine.clearance = .01;
        p.plate.automaticOrder = true;
        p.plate.searchBudget = 2;
        p.process.wallCount = 1;
        p.objects[0].mesh = box_mesh(Vec3(6, 0, 2), Vec3(3, 3, 4), "Tall object");
        p.objects[1].mesh = box_mesh(Vec3(0, 0, .5), Vec3(3, 3, 1), "Short object");
        p.tools[0].dummy.components[3].mesh = box_mesh(Vec3(5, 0, 2), Vec3(2, 4, 2), "Fan sweep");
        p.tools[0].dummy.components[3].renderMesh = {};
        slice_plate(p);
        auto first = validate_motion(p);
        QVERIFY(!first.exportable());
        QVERIFY(repair_motion(p));
        QCOMPARE(p.plate.order.front(), QString("object-2"));
        QVERIFY(p.report.exportable());
    }
    void detachable_tools_replay_and_scrub() {
        auto p = fixture();
        p.path.clear();
        p.machine.controller = Controller::Duet;
        auto &first = p.tools[0];
        first.detachable = true;
        first.change.calibrated = true;
        first.dock = translation(Vec3(-6, 0, 5));
        RecipeStep release;
        release.carriage.xyz = Vec3(-6, 0, 5);
        RecipeStep guard;
        guard.kind = EventKind::Confirm;
        guard.argument = "M577 P1 S0";
        first.change.release = {release, guard};
        guard.argument = "M577 P1 S1";
        first.change.pickup = {release, guard};
        auto second = first;
        second.id = "tool-2";
        second.controllerTool = 1;
        second.nozzle.diameter = .8;
        second.assembly.tcp = Vec3(1, 0, 0);
        for (auto &v : second.assembly.components[0].mesh.vertices)
            v.x() += 1;
        second.dock = translation(Vec3(6, 0, 5));
        release.carriage.xyz = Vec3(6, 0, 5);
        second.change.release = {release, guard};
        second.change.pickup = {release, guard};
        p.tools.push_back(second);
        Component carriage;
        carriage.id = carriage.name = "coupler";
        carriage.link = Link::Carriage;
        carriage.mesh = box_mesh(Vec3(0, 0, 3), Vec3(.2, .2, .2));
        p.carriageAssembly.components = {carriage};
        auto add = [&](Vec3 axes, QString tool, EventKind event = EventKind::Motion, QString argument = {}) {
            Sample s;
            s.toolId = tool;
            s.materialId = "material-1";
            s.objectId = "object-1";
            s.event = event;
            s.argument = argument;
            JointState q;
            q.xyz = axes;
            s.carriage = q;
            s.joint = pose_from_axes(p.machine, assembly_at(p, tool), q);
            s.position = s.joint.xyz;
            s.direction = Vec3::UnitZ();
            p.path.push_back(s);
        };
        add(Vec3(-2, 0, 3), "tool-1");
        add(Vec3(-6, 0, 5), "tool-1");
        add(Vec3(-6, 0, 5), "tool-1", EventKind::Confirm, "M577 P1 S0");
        add(Vec3(-6, 0, 5), {}, EventKind::Detach, "tool-1");
        add(Vec3(6, 0, 5), {});
        add(Vec3(6, 0, 5), {}, EventKind::Confirm, "M577 P1 S1");
        add(Vec3(6, 0, 5), "tool-2", EventKind::Attach, "tool-2");
        add(Vec3(0, 0, 8), "tool-2");
        time_plate(p);
        p.depositionValid = true;
        p.report = validate_motion(p);
        QVERIFY2(p.report.exportable(),
                 p.report.events.empty() ? "no report" : qPrintable(p.report.events.front().reason));
        auto code = export_gcode(p);
        QVERIFY(code.contains("T-1 P0"));
        QVERIFY(code.contains("T1 P0"));
        auto replay = replay_gcode(p, code);
        QVERIFY(validate_motion(replay).exportable());
        auto empty = simulation_at(p, p.path[3].time);
        QVERIFY(empty.toolId.isEmpty());
        QCOMPARE(empty.parkedTools.size(), size_t(2));
        auto end = simulation_at(p, p.path.back().time);
        QCOMPARE(end.toolId, QString("tool-2"));
        auto beginning = simulation_at(p, 0);
        QCOMPARE(beginning.toolId, QString("tool-1"));
        p.fixtures.push_back({"Dock interference", box_mesh(Vec3(6, 0, 8), Vec3(1, 1, 1)), false});
        QVERIFY(!validate_motion(p).exportable());
    }
};
QTEST_GUILESS_MAIN(PlateTests)
#include "plate_tests.moc"
