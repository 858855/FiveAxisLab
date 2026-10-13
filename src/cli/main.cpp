#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/project.h"
#include "core/process.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include "core/motion_preview.h"
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QTextStream>
#include <iostream>
using namespace fiveaxis;
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    QCommandLineParser parser;
    parser.setApplicationDescription("FiveAxisLab geometric research runner");
    parser.addHelpOption();
    parser.addOption({"demo", "Generate the reference project", "file"});
    parser.addOption({"fixtures", "Write STEP fixtures", "directory"});
    parser.addOption({"validate", "Validate project motion", "file"});
    parser.addOption({"slice", "Import and slice a mesh", "file"});
    parser.addOption({"output", "Save the resulting project", "file"});
    parser.addOption({"gcode", "Export a validated job", "file"});
    parser.addOption({"report", "Write JSON validation results", "file"});
    parser.addOption({"head", "Use the tilting-head layout"});
    parser.addOption({"klipper", "Use Klipper output"});
    parser.addOption({"repair", "Attempt collision avoidance"});
    parser.addOption({"toolhead", "Import and use a STEP toolhead with zero CAD TCP", "file"});
    parser.addOption({"dummy-toolhead", "Generate with synthetic toolhead geometry instead of requiring STEP"});
    parser.addOption({"orca-process", "Import supported settings from an Orca process JSON preset", "file"});
    parser.addOption({"supports", "Support generation: normal, reduced, or none", "mode"});
    parser.addOption({"maximum-bridge", "Maximum span used by the deposition support heuristic", "mm"});
    parser.addOption({"fem", "Import a complete tensor or scalar VTU file", "file"});
    parser.addOption({"stress-array", "Select the VTU array", "name"});
    parser.addOption({"stock-klipper", "Disable the optional flow extension"});
    parser.addOption({"replay", "Semantically replay and revalidate exported output"});
    parser.addOption({"profile", "Save the machine profile", "file"});
    parser.addOption({"plan-only", "Save planned motion with an unresolved validation status"});
    parser.addOption({"head-motion", "Generate a compensated tilting-head deposition fixture"});
    parser.addOption({"axis-demo", "Save a non-printing tilt/roll preview; requires --output", "bed|head"});
    parser.process(app);
    try {
        if(parser.isSet("axis-demo")){
            auto value=parser.value("axis-demo");if(value!="bed" && value!="head")throw std::runtime_error("Axis demo must be bed or head");
            if(!parser.isSet("output") || parser.isSet("gcode") || parser.isSet("slice") || parser.isSet("validate") || parser.isSet("replay"))throw std::runtime_error("Axis demo requires --output and cannot export or validate a print job");
            save_project(axis_motion_demo(value=="bed"?Layout::RotaryBed:Layout::TiltingHead),parser.value("output"));
            std::cout<<"Axis demonstration saved. No extrusion; export is blocked.\n";return 0;
        }
        Project p = demo_project();
        QStringList presetNotes;
        if (parser.isSet("head-motion")) {
            p = Project{};
            p.machine.layout = Layout::TiltingHead;
            Component c;
            c.id = c.name = "Reference nozzle";
            c.mesh = cylinder_mesh(Vec3(0, 0, .5), .1, 1);
            c.nozzle = true;
            p.toolhead.components.push_back(c);
            p.fixtures.push_back({"Bed", box_mesh(Vec3(10, 0, 9.3), Vec3(20, 20, 1)), true});
            Sample a, b;
            a.position = a.joint.xyz = Vec3(10, 0, 10);
            b = a;
            b.position = b.joint.xyz = Vec3(10, 1, 10);
            b.joint.a = b.joint.b = 2 * pi / 180;
            b.direction = part_direction(p.machine, p.toolhead, b.joint);
            b.role = Role::Wall;
            b.deltaE = bead_area(b.width, b.height) / (pi * std::pow(p.machine.filamentDiameter / 2, 2));
            p.path = {a, b};
            refine_joint_motion(p);
        }
        if (parser.isSet("fixtures")) {
            QDir().mkpath(parser.value("fixtures"));
            write_demo_step(parser.value("fixtures") + "/reference-toolhead.step");
            write_demo_step(parser.value("fixtures") + "/nested-toolhead.step", true);
            std::cout << "STEP fixtures written\n";
            return 0;
        }
        if (parser.isSet("validate"))
            p = load_project(parser.value("validate"));
        if(parser.isSet("slice")) {
            p.process.layerHeight=.4; p.process.width=.6;
            p.useDummyToolhead=false;
        }
        if(parser.isSet("orca-process")) {
            QFile file(parser.value("orca-process"));
            if(!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot open Orca process JSON");
            QJsonParseError error; auto doc=QJsonDocument::fromJson(file.readAll(),&error);
            if(error.error!=QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("Invalid Orca process JSON");
            p.process=doc.object().contains("layerHeight")?read_process(doc.object()):import_orca_process(doc.object(),p.process,presetNotes);
        }
        if(parser.isSet("supports")) {
            auto value=parser.value("supports");
            if(value=="normal") p.process.supportMode=SupportMode::Normal;
            else if(value=="reduced") p.process.supportMode=SupportMode::Reduced;
            else if(value=="none") p.process.supportMode=SupportMode::None;
            else throw std::runtime_error("Supports must be normal, reduced, or none");
            p.process.supports=p.process.supportMode!=SupportMode::None;
        }
        if(parser.isSet("maximum-bridge")) {
            bool ok=false; p.process.maximumBridge=parser.value("maximum-bridge").toDouble(&ok);
            if(!ok || !std::isfinite(p.process.maximumBridge) || p.process.maximumBridge<0)
                throw std::runtime_error("Invalid maximum bridge span");
        }
        if (parser.isSet("head")) {
            p.machine.layout = Layout::TiltingHead;
            p.machine.minB = -75 * pi / 180;
            p.machine.maxB = 75 * pi / 180;
            regenerate_joints(p);
        }
        if (parser.isSet("klipper"))
            p.machine.controller = Controller::Klipper;
        if (parser.isSet("stock-klipper"))
            p.machine.klipperExtension = false;
        if(parser.isSet("toolhead") && parser.isSet("dummy-toolhead"))
            throw std::runtime_error("Choose either a STEP toolhead or --dummy-toolhead");
        if (parser.isSet("toolhead")) {
            p.toolhead = load_step_assembly(parser.value("toolhead"));
            p.useDummyToolhead=false;
        }
        if(parser.isSet("dummy-toolhead")) {
            p.dummyToolhead=make_dummy_toolhead(p.process.nozzleDiameter);
            p.useDummyToolhead=true;
        }
        if(!parser.isSet("slice") && (parser.isSet("dummy-toolhead") || parser.isSet("toolhead")))
            regenerate_joints(p);
        if (parser.isSet("slice")) {
            p.part = load_mesh(parser.value("slice"));
            Vec3 shift = p.part.bounds().center();
            shift.z() = p.part.bounds().min().z();
            for (auto &v : p.part.vertices)
                v -= shift;
            p.partPlacement = translation(-shift);
            if (parser.isSet("fem")) {
                VtuOptions options;
                options.array = parser.value("stress-array");
                options.alignment = p.partPlacement;
                p.stress = import_vtu({parser.value("fem")}, options);
            }
            slice_project(p);
        }
        p.diagnostics.append(presetNotes);
        int progress = 0;
        if (!p.deposition.checked)
            check_deposition(p);
        std::cerr << "Generated " << p.path.size() << " movements\n";
        p.report = CollisionReport{};
        if (parser.isSet("output"))
            save_project(p, parser.value("output"));
        if (!parser.isSet("plan-only"))
            p.report = validate_motion(p, nullptr, [&](double value) {
                if (value * 100 >= progress) {
                    std::cerr << "Validated " << progress << "%\n";
                    progress += 10;
                }
            });
        if (parser.isSet("repair") && !p.report.exportable())
            repair_motion(p);
        if (parser.isSet("demo"))
            save_project(p, parser.value("demo"));
        if (parser.isSet("output"))
            save_project(p, parser.value("output"));
        if (parser.isSet("profile")) {
            Project profile;
            profile.machine = p.machine;
            profile.toolhead = p.toolhead;
            profile.dummyToolhead = p.dummyToolhead;
            profile.useDummyToolhead = p.useDummyToolhead;
            profile.fixtures = p.fixtures;
            save_project(profile, parser.value("profile"));
        }
        bool replayVerified = false;
        if (parser.isSet("replay") && p.report.exportable() && p.depositionValid) {
            auto replay = replay_gcode(p, export_gcode(p));
            replay.report = validate_motion(replay);
            if (!replay.report.exportable())
                throw std::runtime_error("Exported output failed semantic replay validation");
            replayVerified = true;
        }
        auto report = project_json(p)["report"].toObject();
        report["physicalPrintingValidated"] = false;
        report["dummyToolhead"] = p.useDummyToolhead;
        report["nativeFirmwareValidated"] = false;
        report["controllerSemanticReplayValidated"] = replayVerified;
        report["deposition"] = project_json(p)["deposition"];
        report["diagnostics"] = project_json(p)["diagnostics"];
        if (parser.isSet("report")) {
            QFile file(parser.value("report"));
            if (!file.open(QIODevice::WriteOnly))
                throw std::runtime_error("Cannot write report");
            file.write(QJsonDocument(report).toJson());
        }
        std::cout << QJsonDocument(report).toJson().toStdString();
        if (parser.isSet("gcode")) {
            auto text = export_gcode(p);
            QFile file(parser.value("gcode"));
            if (!file.open(QIODevice::WriteOnly))
                throw std::runtime_error("Cannot write G-code");
            file.write(text.toUtf8());
        }
        return p.report.exportable() && p.depositionValid ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
