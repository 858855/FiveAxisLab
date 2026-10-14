#include "core/collision.h"
#include "core/compute.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/motion_preview.h"
#include "core/nonplanar.h"
#include "core/plate.h"
#include "core/process.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
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
    parser.addOption(
        {"dummy-toolhead", "Generate with synthetic toolhead geometry instead of requiring STEP"});
    parser.addOption({"orca-process", "Import supported settings from an Orca process JSON preset", "file"});
    parser.addOption({"supports", "Support generation: normal, reduced, or none", "mode"});
    parser.addOption({"maximum-bridge", "Maximum span used by the deposition support heuristic", "mm"});
    parser.addOption({"fem", "Import a complete tensor or scalar VTU file", "file"});
    parser.addOption({"stress-array", "Select the VTU array", "name"});
    parser.addOption({"fea-load",
                      "Solve linear static FEA: fix the bottom face, load the top face with total force fx,fy,fz (N)",
                      "fx,fy,fz"});
    parser.addOption({"stock-klipper", "Disable the optional flow extension"});
    parser.addOption({"replay", "Semantically replay and revalidate exported output"});
    parser.addOption({"profile", "Save the machine profile", "file"});
    parser.addOption({"plan-only", "Save planned motion with an unresolved validation status"});
    parser.addOption({"head-motion", "Generate a compensated tilting-head deposition fixture"});
    parser.addOption({"axis-demo", "Save a non-printing tilt/roll preview; requires --output", "bed|head"});
    parser.addOption(
        {"spiral-demo", "Generate the profile-normal spiral test; requires --output", "bed|head"});
    parser.addOption(
        {"profile-guide", "Slice perpendicular to profile stations in source-model millimetres", "file"});
    parser.addOption({"three-axis", "Use a standard XYZ printer and conformal non-planar layers"});
    parser.addOption({"xyz-planar", "Use a standard XYZ printer with planar layers"});
    parser.addOption({"marlin", "Use generic / Marlin XYZ G-code"});
    parser.addOption({"nonplanar-demo", "Generate the standard XYZ non-planar test; requires --output"});
    parser.addOption({"plate-demo", "Generate the two-object plate test; requires --output"});
    parser.addOption({"complete-objects", "Finish each plate object before the next"});
    parser.addOption({"object-order", "Manual complete-object order, comma-separated IDs", "ids"});
    parser.addOption({"threads", "CPU thread budget; zero selects auto", "number"});
    parser.addOption({"compute", "Compute backend: auto, cpu, cuda, vulkan", "backend"});
    parser.addOption({"device", "Preferred compute device name", "name"});
    parser.addOption({"list-compute-devices", "List available CPU/GPU compute devices"});
    parser.addOption({"reslice", "Regenerate a loaded plate using its objects/materials/tools"});
    parser.addOption({"tool", "Physical tool ID for STEP/dummy setup in a plate", "id"});
    parser.addOption({"compute-benchmark", "Write measured CPU/CUDA/Vulkan workload timings", "file"});
    parser.process(app);
    try {
        auto configureCompute = [&](Project &p) {
            if (parser.isSet("threads")) {
                bool ok;
                int threads = parser.value("threads").toInt(&ok);
                if (!ok || threads < 0)
                    throw std::runtime_error("Invalid CPU budget");
                p.compute.threads = threads;
            }
            if (parser.isSet("compute")) {
                auto name = parser.value("compute");
                if (name == "auto")
                    p.compute.backend = ComputeBackend::Auto;
                else if (name == "cpu")
                    p.compute.backend = ComputeBackend::CPU;
                else if (name == "cuda")
                    p.compute.backend = ComputeBackend::CUDA;
                else if (name == "vulkan")
                    p.compute.backend = ComputeBackend::Vulkan;
                else
                    throw std::runtime_error("Unknown compute backend");
            }
            if (parser.isSet("device"))
                p.compute.device = parser.value("device");
        };
        if (parser.isSet("list-compute-devices")) {
            for (const auto &device : compute_devices())
                std::cout << device.toStdString() << "\n";
            return 0;
        }
        if (parser.isSet("compute-benchmark")) {
            QJsonArray results;
            for (size_t count : {8192u, 65536u, 262144u})
                for (uint32_t workload = 0; workload < 4; ++workload) {
                    std::vector<double> input(count * (workload == 2 ? 16 : workload == 3 ? 9 : 3));
                    for (size_t i = 0; i < count; ++i) {
                        if (workload == 3) {
                            double triangle[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
                            std::copy(triangle, triangle + 9, input.begin() + i * 9);
                        } else if (workload == 2) {
                            for (int n = 0; n < 4; ++n) {
                                input[i * 16 + n] = .25;
                                for (int c = 0; c < 3; ++c)
                                    input[i * 16 + 4 + n * 3 + c] = i * .01 + n + c;
                            }
                        } else {
                            input[i * 3] = -1;
                            input[i * 3 + 1] = 1;
                            input[i * 3 + 2] = i % 2 ? 1 : 1e6;
                        }
                    }
                    for (auto backend : {ComputeBackend::CPU, ComputeBackend::CUDA, ComputeBackend::Vulkan,
                                         ComputeBackend::Auto})
                        for (int repeat = 0; repeat < 3; ++repeat) {
                            Project configuration;
                            configureCompute(configuration);
                            auto settings = configuration.compute;
                            settings.backend = backend;
                            ComputeReport report;
                            QElapsedTimer timer;
                            timer.start();
                            auto output = compute_workload(input, workload, translation(Vec3(1, 2, 3)),
                                                           settings, report);
                            results.append(QJsonObject{{"requested", int(backend)},
                                                       {"actual", report.backend},
                                                       {"device", report.device},
                                                       {"precision", report.precision},
                                                       {"fallback", report.fallback},
                                                       {"count", qint64(count)},
                                                       {"workload", int(workload)},
                                                       {"repeat", repeat},
                                                       {"endToEndMs", timer.nsecsElapsed() / 1e6},
                                                       {"cpuReferenceMs", report.cpuMilliseconds},
                                                       {"gpuDispatchAndTransfersMs", report.gpuMilliseconds},
                                                       {"batches", qint64(report.batches)},
                                                       {"resultCount", qint64(output.size())}});
                        }
                }
            QFile file(parser.value("compute-benchmark"));
            if (!file.open(QIODevice::WriteOnly))
                throw std::runtime_error("Cannot write compute benchmark");
            file.write(QJsonDocument(QJsonObject{{"computeABI", 2},
                                                 {"measurements", results},
                                                 {"finalCollisionCertification", "CPU"},
                                                 {"physicalPrintingValidated", false}})
                           .toJson());
            return 0;
        }
        if (parser.isSet("plate-demo")) {
            if (!parser.isSet("output") || parser.isSet("gcode"))
                throw std::runtime_error("Plate demo requires --output and cannot authorize export");
            auto p =
                plate_demo(parser.isSet("complete-objects") ? PrintSequence::Objects : PrintSequence::Layers);
            configureCompute(p);
            slice_plate(p);
            save_project(p, parser.value("output"));
            std::cout << "Plate demonstration saved; validation remains required.\n";
            return 0;
        }

        if ((parser.isSet("three-axis") && parser.isSet("xyz-planar")) ||
            (parser.isSet("marlin") && parser.isSet("klipper")))
            throw std::runtime_error("Choose one XYZ slicing mode and one controller");
        if (parser.isSet("nonplanar-demo")) {
            if (!parser.isSet("output") || parser.isSet("slice") || parser.isSet("validate") ||
                parser.isSet("gcode") || parser.isSet("replay"))
                throw std::runtime_error(
                    "Non-planar demo requires --output; validate separately before export");
            auto p = nonplanar_cartesian_project();
            slice_project(p);
            save_project(p, parser.value("output"));
            std::cout << "XYZ non-planar test saved: " << p.path.size()
                      << " movements; validation remains required.\n";
            return 0;
        }
        if ((parser.isSet("three-axis") || parser.isSet("xyz-planar")) &&
            (parser.isSet("head") || parser.isSet("head-motion") || parser.isSet("profile-guide")))
            throw std::runtime_error("XYZ mode cannot use rotary-head or profile-normal options");
        if (parser.isSet("spiral-demo")) {
            auto value = parser.value("spiral-demo");
            if (value != "bed" && value != "head")
                throw std::runtime_error("Spiral demo must be bed or head");
            if (!parser.isSet("output") || parser.isSet("gcode") || parser.isSet("slice") ||
                parser.isSet("validate") || parser.isSet("replay"))
                throw std::runtime_error(
                    "Spiral demo requires --output; validate it separately before export");
            auto p = spiral_profile_project(value == "bed" ? Layout::RotaryBed : Layout::TiltingHead);
            slice_project(p);
            p.report = CollisionReport{};
            save_project(p, parser.value("output"));
            std::cout << "Spiral profile-normal test saved: " << p.path.size()
                      << " movements. Collision validation remains required.\n";
            return 0;
        }
        if (parser.isSet("axis-demo")) {
            auto value = parser.value("axis-demo");
            if (value != "bed" && value != "head")
                throw std::runtime_error("Axis demo must be bed or head");
            if (!parser.isSet("output") || parser.isSet("gcode") || parser.isSet("slice") ||
                parser.isSet("validate") || parser.isSet("replay"))
                throw std::runtime_error(
                    "Axis demo requires --output and cannot export or validate a print job");
            save_project(axis_motion_demo(value == "bed" ? Layout::RotaryBed : Layout::TiltingHead),
                         parser.value("output"));
            std::cout << "Axis demonstration saved. No extrusion; export is blocked.\n";
            return 0;
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
        if (p.plateEnabled && parser.isSet("slice"))
            throw std::runtime_error(
                "Add geometry through the plate editor; --slice creates a single-object job");
        configureCompute(p);
        if (parser.isSet("slice")) {
            p.process.layerHeight = .4;
            p.process.width = .6;
            p.useDummyToolhead = false;
        }
        if (parser.isSet("orca-process")) {
            QFile file(parser.value("orca-process"));
            if (!file.open(QIODevice::ReadOnly))
                throw std::runtime_error("Cannot open Orca process JSON");
            QJsonParseError error;
            auto doc = QJsonDocument::fromJson(file.readAll(), &error);
            if (error.error != QJsonParseError::NoError || !doc.isObject())
                throw std::runtime_error("Invalid Orca process JSON");
            p.process = doc.object().contains("layerHeight")
                            ? read_process(doc.object())
                            : import_orca_process(doc.object(), p.process, presetNotes);
        }
        if (parser.isSet("supports")) {
            auto value = parser.value("supports");
            if (value == "normal")
                p.process.supportMode = SupportMode::Normal;
            else if (value == "reduced")
                p.process.supportMode = SupportMode::Reduced;
            else if (value == "none")
                p.process.supportMode = SupportMode::None;
            else
                throw std::runtime_error("Supports must be normal, reduced, or none");
            p.process.supports = p.process.supportMode != SupportMode::None;
        }
        if (parser.isSet("maximum-bridge")) {
            bool ok = false;
            p.process.maximumBridge = parser.value("maximum-bridge").toDouble(&ok);
            if (!ok || !std::isfinite(p.process.maximumBridge) || p.process.maximumBridge < 0)
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
        if (parser.isSet("three-axis") || parser.isSet("xyz-planar")) {
            p.machine.layout = Layout::Cartesian3Axis;
            p.process.profileNormal = false;
            p.process.nonPlanar3Axis = parser.isSet("three-axis");
            if (!parser.isSet("klipper"))
                p.machine.controller = Controller::GenericMarlin;
        }
        if (parser.isSet("marlin"))
            p.machine.controller = Controller::GenericMarlin;
        if (parser.isSet("stock-klipper"))
            p.machine.klipperExtension = false;
        if (parser.isSet("toolhead") && parser.isSet("dummy-toolhead"))
            throw std::runtime_error("Choose either a STEP toolhead or --dummy-toolhead");
        if (parser.isSet("toolhead")) {
            p.toolhead = load_step_assembly(parser.value("toolhead"));
            p.useDummyToolhead = false;
        }
        if (parser.isSet("dummy-toolhead")) {
            p.dummyToolhead = make_dummy_toolhead(p.process.nozzleDiameter);
            p.useDummyToolhead = true;
        }
        if (p.plateEnabled && (parser.isSet("dummy-toolhead") || parser.isSet("toolhead"))) {
            auto id = parser.isSet("tool") ? parser.value("tool") : p.tools.front().id;
            auto tool = std::find_if(p.tools.begin(), p.tools.end(),
                                     [&](const ToolDefinition &t) { return t.id == id; });
            if (tool == p.tools.end())
                throw std::runtime_error("Unknown physical tool ID");
            if (parser.isSet("dummy-toolhead")) {
                tool->dummy = make_dummy_toolhead(tool->nozzle.diameter);
                tool->useDummy = true;
                tool->nozzle.calibrated = true;
            } else {
                tool->assembly = p.toolhead;
                tool->useDummy = false;
                tool->nozzle.calibrated = false;
            }
            synchronize_plate_preview(p);
            p.invalidate();
        }
        if (!p.plateEnabled && !parser.isSet("slice") &&
            (parser.isSet("dummy-toolhead") || parser.isSet("toolhead")))
            regenerate_joints(p);
        if (parser.isSet("slice")) {
            p.part = load_mesh(parser.value("slice"));
            Vec3 shift = p.part.bounds().center();
            shift.z() = p.part.bounds().min().z();
            if (!p.machine.rotary_axes())
                shift.head<2>() -= (p.machine.minXYZ + p.machine.maxXYZ).head<2>() / 2;
            for (auto &v : p.part.vertices)
                v -= shift;
            p.partPlacement = translation(-shift);
            if (parser.isSet("profile-guide")) {
                p.profileGuide = import_profile_guide(parser.value("profile-guide"), p.partPlacement);
                p.process.profileNormal = true;
            }
            if (parser.isSet("fem")) {
                VtuOptions options;
                options.array = parser.value("stress-array");
                options.alignment = p.partPlacement;
                p.stress = import_vtu({parser.value("fem")}, options);
            }
            if (parser.isSet("fea-load")) {
                auto parts = parser.value("fea-load").split(',');
                bool ok = parts.size() == 3;
                Vec3 force = Vec3::Zero();
                for (int k = 0; ok && k < 3; ++k)
                    force[k] = parts[k].toDouble(&ok);
                if (!ok || !force.allFinite() || force.norm() <= 0)
                    throw std::runtime_error("--fea-load expects a nonzero force fx,fy,fz");
                p.volume = tetrahedralize(p.part, std::max(.05, p.part.bounds().sizes().prod() / 1500));
                auto bounds = p.part.bounds();
                std::vector<int> loaded;
                p.loads = {};
                for (int i = 0; i < int(p.volume.vertices.size()); ++i) {
                    double z = p.volume.vertices[i].z();
                    if (z <= bounds.min().z() + .5)
                        p.loads.fixedNodes.push_back(i);
                    else if (z >= bounds.max().z() - 1)
                        loaded.push_back(i);
                }
                if (p.loads.fixedNodes.size() < 3 || loaded.empty())
                    throw std::runtime_error("Bottom/top faces do not map to enough volume nodes for FEA");
                for (int i : loaded)
                    p.loads.nodalForces.emplace_back(i, force / double(loaded.size()));
                auto result = solve_fea(p.volume, p.loads);
                p.stress = result.stress;
                std::cerr << "FEA: " << p.loads.fixedNodes.size() << " fixed / " << loaded.size()
                          << " loaded nodes, compliance " << result.compliance << " N*mm\n";
            }
            slice_project(p);
        }
        configureCompute(p);
        if (parser.isSet("complete-objects") || parser.isSet("object-order") || parser.isSet("reslice")) {
            enable_plate(p);
            if (parser.isSet("complete-objects") || parser.isSet("object-order"))
                p.plate.sequence = PrintSequence::Objects;
            if (parser.isSet("object-order")) {
                p.plate.automaticOrder = false;
                p.plate.order.clear();
                for (auto id : parser.value("object-order").split(',', Qt::SkipEmptyParts))
                    p.plate.order.push_back(id.trimmed());
            }
            slice_plate(p);
        }
        p.diagnostics.append(presetNotes);
        int progress = 0;
        if (!p.deposition.checked && !p.plateEnabled)
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
            profile.plateEnabled = p.plateEnabled;
            profile.tools = p.tools;
            profile.materials = p.materials;
            profile.carriageAssembly = p.carriageAssembly;
            profile.plate = p.plate;
            profile.compute = p.compute;
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
        report["compute"] = plate_json(p)["computation"];
        report["plate"] = p.plateEnabled;
        report["jobEvents"] = int(p.events.size());
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
