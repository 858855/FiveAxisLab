#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/machine_settings.h"
#include "core/plate.h"
#include "core/project.h"
#include <QJsonArray>
#include <QTemporaryDir>
#include <QtTest/QtTest>
using namespace fiveaxis;
static Project motion(bool plate = false) {
    Project p;
    p.machine.layout = Layout::Cartesian3Axis;
    p.useDummyToolhead = true;
    p.dummyToolhead = make_dummy_toolhead(.4);
    for (auto pos : {Vec3(0, 0, 20), Vec3(5, 0, 20), Vec3(15, 0, 20), Vec3(20, 0, 20)}) {
        Sample s;
        s.position = s.joint.xyz = pos;
        s.speed = 25;
        s.role = p.path.size() == 1 || p.path.size() == 3 ? Role::Wall : Role::Travel;
        s.deltaE = s.role == Role::Wall ? (pos - p.path.back().position).norm() * bead_area(.4, .2) /
                                              (pi * std::pow(1.75 / 2, 2))
                                        : 0;
        s.width = .4;
        s.height = .2;
        p.path.push_back(s);
    }
    if (plate) {
        p.plateEnabled = true;
        ToolDefinition tool;
        tool.id = "t";
        tool.name = "Test";
        tool.dummy = p.dummyToolhead;
        tool.nozzle.calibrated = true;
        tool.nozzle.geometryHash = tool.dummy.sourceHash;
        tool.routes.push_back({"m", "", "", true});
        p.tools.push_back(tool);
        MaterialProfile material;
        material.id = "m";
        p.materials.push_back(material);
        PlateObject object;
        object.id = "o";
        object.materialId = "m";
        object.name = "Test object";
        object.mesh = box_mesh(Vec3(10, 0, 18), Vec3(30, 10, 4));
        p.objects.push_back(object);
        for (auto &s : p.path) {
            s.toolId = "t";
            s.materialId = "m";
            s.objectId = "o";
            s.carriage = s.joint;
        }
    }
    time_path(p);
    return p;
}
class MachineTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
    void settings_roundtrip_and_legacy_fingerprint() {
        auto p = motion(true);
        QVERIFY(p.machine.printer.isEmpty());
        auto fingerprint = p.fingerprint();
        auto j = project_json(p);
        QVERIFY(!j["machine"].toObject().contains("printer"));
        QCOMPARE(project_from_json(j).fingerprint(), fingerprint);
        time_path(p);
        QVERIFY(p.machine.printer.isEmpty());
        QCOMPARE(p.fingerprint(), fingerprint);
        p.machine.printer = {
            {"name", "My printer"}, {"notes", "Calibrated"}, {"relativeE", false}, {"speedX", 12}};
        p.tools[0].extruderSettings = {{"retractSpeed", 17}, {"hop", .6}, {"enabled", true}};
        QTemporaryDir directory;
        QVERIFY(directory.isValid());
        save_project(p, directory.filePath("printer.5machine"));
        auto loaded = load_project(directory.filePath("printer.5machine"));
        QCOMPARE(loaded.machine.printer, p.machine.printer);
        QCOMPARE(loaded.tools[0].extruderSettings, p.tools[0].extruderSettings);
        QVERIFY(loaded.fingerprint() != fingerprint);
        auto invalid = project_json(p);
        auto machine = invalid["machine"].toObject();
        auto settings = machine["printer"].toObject();
        settings["speedX"] = 0;
        machine["printer"] = settings;
        invalid["machine"] = machine;
        QVERIFY_EXCEPTION_THROWN(project_from_json(invalid), std::runtime_error);
    }
    void bed_shapes_and_exclusions() {
        MachineProfile m;
        m.printer = {{"bedEnabled", true}, {"bedWidth", 100},  {"bedDepth", 100},
                     {"bedOriginX", 50},   {"bedOriginY", 50}, {"printableHeight", 80}};
        QVERIFY(printable_point(m, Vec3(0, 0, 10), .3));
        QVERIFY(!printable_point(m, Vec3(49.9, 0, 10), .3));
        QVERIFY(!printable_point(m, Vec3(0, 0, 81)));
        auto mesh = printer_bed_mesh(m.printer);
        QVERIFY(contains(mesh, Vec3(0, 0, -1)));
        m.printer["bedShape"] = "circle";
        QVERIFY(!printable_point(m, Vec3(40, 40, 10)));
        m.printer["bedShape"] = "custom";
        m.printer["bedPolygon"] = QJsonArray{QJsonArray{0, 0},   QJsonArray{40, 0},  QJsonArray{40, 20},
                                             QJsonArray{20, 20}, QJsonArray{20, 40}, QJsonArray{0, 40}};
        validate_printer_settings(m);
        QVERIFY(!printable_point(m, Vec3(30, 30, 5)));
        QVERIFY(contains(printer_bed_mesh(m.printer), Vec3(10, 30, -1)));
        m.printer["bedExclusions"] = QJsonArray{
            QJsonArray{QJsonArray{8, 0}, QJsonArray{12, 0}, QJsonArray{12, 15}, QJsonArray{8, 15}}};
        auto p = motion();
        p.machine.printer = m.printer;
        p.path[1].position = p.path[1].joint.xyz = Vec3(5, 10, 5);
        p.path[2].position = p.path[2].joint.xyz = Vec3(15, 10, 5);
        p.path[2].deltaE = .1;
        QVERIFY_EXCEPTION_THROWN(check_printer_envelope(p), std::runtime_error);
        m.printer["bedPolygon"] =
            QJsonArray{QJsonArray{0, 0}, QJsonArray{30, 30}, QJsonArray{0, 30}, QJsonArray{30, 0}};
        QVERIFY_EXCEPTION_THROWN(validate_printer_settings(m), std::runtime_error);
    }
    void axis_timing_and_controller_units() {
        auto p = motion();
        p.machine.printer = {{"speedX", 1}, {"speedE", .05}, {"accelX", 4}, {"jerkX", 9}};
        time_path(p);
        QVERIFY(p.path[1].time >= 5);
        auto text = printer_controller_template(p.machine);
        QVERIFY(text.contains("M203 X60"));
        QVERIFY(text.contains("M566 X540"));
        p.machine.controller = Controller::GenericMarlin;
        QVERIFY(printer_controller_template(p.machine).contains("M203 X1 "));
        p.machine.controller = Controller::Klipper;
        QVERIFY(printer_controller_template(p.machine).contains("max_z_velocity"));
        QVERIFY(!printer_controller_template(p.machine).contains("square_corner_velocity:"));
    }
    void planned_retraction_hop_wipe() {
        auto p = motion(true);
        double initialExtrusion = p.path[1].deltaE + p.path[3].deltaE;
        p.tools[0].extruderSettings = {
            {"enabled", true},      {"retractLength", 1.4}, {"hop", .6},
            {"travelThreshold", 3}, {"retractSpeed", 10},   {"recoverSpeed", 8},
            {"wipe", true},         {"wipeDistance", 2},    {"retractBeforeWipe", 25}};
        apply_printer_path(p);
        double sum = 0;
        bool raised = false, wiped = false;
        for (auto &s : p.path) {
            sum += s.deltaE;
            raised |= s.position.z() > 20.5;
            wiped |= s.argument == "printer/wipe";
        }
        QVERIFY(raised && wiped);
        QVERIFY(std::abs(sum - initialExtrusion) < 1e-8);
        QVERIFY(p.path.size() > 7);
        auto sim = simulate_plate(p, p.path.back().time);
        QVERIFY(sim.toolId == "t");
        for (size_t i = 1; i < p.path.size(); ++i)
            QVERIFY(p.path[i].time > p.path[i - 1].time);
        p.depositionValid = true;
        p.report = validate_motion(p);
        if (!p.report.exportable())
            for (const auto &event : p.report.events)
                qWarning() << event.reason;
        QVERIFY(p.report.exportable());
        for (bool relative : {true, false}) {
            p.machine.printer["relativeE"] = relative;
            p.report.fingerprint = p.fingerprint();
            auto code = export_gcode(p);
            auto replay = replay_gcode(p, code);
            QCOMPARE(replay.path.size(), p.path.size());
            for (size_t i = 0; i < p.path.size(); ++i) {
                QVERIFY((replay.path[i].joint.xyz - p.path[i].joint.xyz).norm() < 1e-7);
                QVERIFY(std::abs(replay.path[i].deltaE - p.path[i].deltaE) < 1e-7);
                QVERIFY(std::abs(replay.path[i].time - p.path[i].time) < 1e-6);
            }
        }
    }
    void custom_scripts_are_collision_checked() {
        auto p = motion(true);
        p.machine.printer["gcode"] =
            QJsonObject{{"start", "G1 X40 F1200\nG4 P250\nM400"}, {"end", "G1 Y20 F1200"}};
        apply_printer_path(p);
        QVERIFY(p.path.size() > 8);
        QVERIFY(std::abs(p.path.back().position.y() - 20) < 1e-8);
        p.report.status = Status::Clear;
        p.depositionValid = true;
        p.report.fingerprint = p.fingerprint();
        auto replay = replay_gcode(p, export_gcode(p));
        QCOMPARE(replay.path.size(), p.path.size());
        p.fixtures.push_back({"Thin script obstacle", box_mesh(Vec3(25, 0, 20), Vec3(.1, 5, 5)), false, {}});
        auto report = validate_motion(p);
        QCOMPARE(report.status, Status::Collision);
        QVERIFY(std::any_of(report.events.begin(), report.events.end(), [](const auto &e) {
            return e.first.contains("Thin script obstacle") || e.second.contains("Thin script obstacle");
        }));
        auto blocked = motion();
        blocked.machine.printer["gcode"] = QJsonObject{{"start", "G28\nBAMBU_MACRO"}};
        QVERIFY_EXCEPTION_THROWN(apply_printer_path(blocked), std::runtime_error);
        blocked.machine.printer["gcode"] = QJsonObject{{"start", "G1 A20"}};
        QVERIFY_EXCEPTION_THROWN(apply_printer_path(blocked), std::runtime_error);
    }
};
QTEST_GUILESS_MAIN(MachineTests)
#include "machine_tests.moc"
