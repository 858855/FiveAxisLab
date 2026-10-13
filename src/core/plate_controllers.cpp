#include "controllers.h"
#include "plate.h"
#include "project.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QTextStream>
#include <stdexcept>
namespace fiveaxis {
static void need(bool condition, const QString &message) {
    if (!condition)
        throw std::runtime_error(message.toStdString());
}
static QString safe_name(const QString &value) {
    need(QRegularExpression("^[A-Za-z_][A-Za-z0-9_-]*$").match(value).hasMatch(),
         "Invalid controller identifier " + value);
    return value;
}
static QString state_command(const Project &p, const Sample &s) {
    auto tool = find_tool(p, s.toolId);
    auto material = find_material(p, s.materialId);
    bool duet = p.machine.controller == Controller::Duet,
         klipper = p.machine.controller == Controller::Klipper;
    switch (s.event) {
    case EventKind::Detach:
        return duet ? "T-1 P0" : klipper ? "FAL_TOOL_DETACH" : "M400";
    case EventKind::Attach:
        need(tool, "Unknown attached tool");
        return duet      ? QString("T%1 P0").arg(tool->controllerTool)
               : klipper ? QString("ACTIVATE_EXTRUDER EXTRUDER=%1\nSET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0")
                               .arg(safe_name(tool->extruder))
                         : QString("T%1 S1").arg(tool->controllerTool);
    case EventKind::Confirm:
        // A named sensor check is a real firmware guard. Empty generated confirmations
        // are synchronization events; calibrated pickup recipes must carry sensor guards.
        return s.argument.startsWith("M577 ") && duet                  ? s.argument
               : s.argument.startsWith("FAL_TOOL_CONFIRM ") && klipper ? s.argument
               : s.argument.startsWith("M226 ") && !duet && !klipper   ? s.argument
                                                                       : "M400";
    case EventKind::Temperature:
        need(tool && material, "Temperature requires an active tool/material");
        return duet ? QString("G10 P%1 S%2\nM116 P%1").arg(tool->controllerTool).arg(s.argument)
               : klipper
                   ? QString(
                         "SET_HEATER_TEMPERATURE HEATER=%1 TARGET=%2\nTEMPERATURE_WAIT SENSOR=%1 MINIMUM=%2")
                         .arg(safe_name(tool->heater), s.argument)
                   : QString("M109 S%1").arg(s.argument);
    case EventKind::IO:
        need(!s.argument.contains('\n') && !s.argument.contains('\r'),
             "Multiline tool action is not replayable");
        need(QRegularExpression(duet      ? "^M(42|280|577)\\s"
                                : klipper ? "^(SET_PIN|FAL_SELECT_FILAMENT|FAL_TOOL_CONFIRM)\\s"
                                          : "^M(42|280)\\s")
                 .match(s.argument)
                 .hasMatch(),
             "Unsupported/non-verifiable tool action " + s.argument);
        return s.argument;
    case EventKind::Material:
        need(material, "Unknown selected material");
        return QString("M400\nM106 S%1").arg(qRound(material->fan * 255));
    case EventKind::Standby: {
        auto parked = find_tool(p, s.argument);
        need(parked && material, "Unknown standby tool");
        return duet      ? QString("G10 P%1 S%2").arg(parked->controllerTool).arg(material->standby)
               : klipper ? QString("SET_HEATER_TEMPERATURE HEATER=%1 TARGET=%2")
                               .arg(safe_name(parked->heater))
                               .arg(material->standby)
                         : QString("M104 T%1 S%2").arg(parked->controllerTool).arg(material->standby);
    }
    case EventKind::Synchronize:
        return "M400";
    default:
        return {};
    }
}
QString export_plate_gcode(const Project &p) {
    need(p.report.exportable() && p.report.fingerprint == p.fingerprint() && p.depositionValid,
         "Export blocked: validate complete plate, deposition and tool changes");
    need(p.path.size() > 1, "No plate trajectory");
    if (p.machine.controller == Controller::GenericMarlin) {
        need(!p.machine.rotary_axes(), "Generic output supports XYZ only");
        for (const auto &t : p.tools)
            if (t.detachable)
                for (const auto *steps : {&t.change.release, &t.change.pickup})
                    need(std::any_of(steps->begin(), steps->end(),
                                     [](const RecipeStep &s) {
                                         return s.kind == EventKind::Confirm &&
                                                s.argument.startsWith("M226 ");
                                     }),
                         "Marlin toolchanger requires explicit M226 pin guards and firmware Tn S1 with zero "
                         "offsets");
    }
    QString output;
    QTextStream out(&output);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(9);
    out << "; FiveAxisLab plate job schema 2\n; Fingerprint " << p.fingerprint()
        << "\n; Carriage coordinates: firmware tool XYZ offsets must be zero.\nG21\nG90\nM83\n";
    if (p.machine.controller == Controller::Duet)
        for (const auto &tool : p.tools)
            out << "G10 P" << tool.controllerTool << " X0 Y0 Z0\nM572 D" << tool.duetDrive << " S0\n";
    if (p.machine.controller == Controller::Klipper) {
        for (const auto &tool : p.tools)
            out << "SET_PRESSURE_ADVANCE EXTRUDER=" << safe_name(tool.extruder) << " ADVANCE=0\n";
        if (p.machine.rotary_axes())
            out << "MANUAL_STEPPER STEPPER=rotary_a GCODE_AXIS=A LIMIT_VELOCITY="
                << p.machine.angularSpeed * 180 / pi << " LIMIT_ACCEL=" << p.machine.angularAccel * 180 / pi
                << "\nMANUAL_STEPPER STEPPER=rotary_b GCODE_AXIS=" << p.machine.axis2()
                << " LIMIT_VELOCITY=" << p.machine.angularSpeed * 180 / pi
                << " LIMIT_ACCEL=" << p.machine.angularAccel * 180 / pi << "\n";
    }
    if (p.machine.controller == Controller::GenericMarlin && p.tools.size() > 1)
        for (const auto &tool : p.tools)
            out << "M218 T" << tool.controllerTool << " X0 Y0 Z0\n";
    Sample first = p.path.front();
    if (p.machine.controller == Controller::Klipper)
        out << "ACTIVATE_EXTRUDER EXTRUDER=" << safe_name(find_tool(p, first.toolId)->extruder)
            << "\nSET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0\n";
    else if (p.machine.controller == Controller::Duet)
        out << "T" << find_tool(p, first.toolId)->controllerTool << " P0\n";
    else if (p.machine.controller == Controller::GenericMarlin && p.tools.size() > 1)
        out << "T" << find_tool(p, first.toolId)->controllerTool << " S1\n";
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1], &b = p.path[i];
        auto x = sample_axes(p, a), y = sample_axes(p, b);
        double duration = b.time - a.time;
        need(duration > 0, "Nonpositive event duration");
        QJsonObject metadata{{"object", b.objectId},  {"tool", b.toolId},       {"material", b.materialId},
                             {"event", int(b.event)}, {"argument", b.argument}, {"role", int(b.role)},
                             {"layer", b.layer},      {"region", b.region},     {"width", b.width},
                             {"height", b.height},    {"dwell", b.dwell}};
        metadata["contactComponent"] = b.contactComponent;
        metadata["contactFixture"] = b.contactFixture;
        metadata["contactCenter"] = QJsonArray{b.contactCenter.x(), b.contactCenter.y(), b.contactCenter.z()};
        metadata["contactSize"] = QJsonArray{b.contactSize.x(), b.contactSize.y(), b.contactSize.z()};
        if (b.surfaceNormal)
            metadata["normal"] = QJsonArray{b.surfaceNormal->x(), b.surfaceNormal->y(), b.surfaceNormal->z()};
        out << ";FAL " << QJsonDocument(metadata).toJson(QJsonDocument::Compact).toBase64() << "\n";
        auto command = state_command(p, b);
        if (!command.isEmpty())
            out << command << "\n";
        if (p.machine.controller == Controller::Klipper && p.machine.rotary_axes() &&
            p.machine.klipperExtension && !b.toolId.isEmpty() &&
            (b.event == EventKind::Attach || b.event == EventKind::Material || i == 1)) {
            auto t = find_tool(p, b.toolId);
            auto assembly = assembly_at(p, b.toolId);
            auto tip = assembly.mount * assembly.tcp;
            out << "FIVEAXIS_ENABLE EXTRUDER=" << safe_name(t->extruder)
                << " LAYOUT=" << (p.machine.layout == Layout::RotaryBed ? "BED" : "HEAD")
                << " AXIS2=" << p.machine.axis2()
                << " MAX_FLOW=" << std::min(t->nozzle.maxFlow, find_material(p, b.materialId)->maxFlow);
            const char *names[] = {"X", "Y", "Z"};
            for (int k = 0; k < 3; ++k)
                out << " TCP_" << names[k] << "=" << tip[k] << " PIVOT_" << names[k] << "="
                    << p.machine.bedPivot[k] << " PIVOT_A" << names[k] << "=" << p.machine.headPivotA[k]
                    << " PIVOT_B" << names[k] << "=" << p.machine.headPivotB[k];
            out << "\n";
        }
        double metric = feed_metric(p.machine, x, y, b.deltaE);
        if (metric < 1e-12) {
            out << "G4 P" << duration * 1000 << "\n";
        } else {
            if (p.machine.controller == Controller::Klipper &&
                (!p.machine.rotary_axes() || !p.machine.klipperExtension) && b.deltaE > 0) {
                double distance = (y.xyz - x.xyz).norm();
                need(distance > 1e-9 || b.deltaE <= 50, "Stock Klipper extrusion-only limit exceeded");
                need(distance < 1e-9 || b.deltaE / distance <= p.machine.stockExtrusionRatio,
                     "Stock Klipper extrusion ratio exceeded");
            }
            out << "G1 X" << y.xyz.x() << " Y" << y.xyz.y() << " Z" << y.xyz.z();
            if (p.machine.rotary_axes())
                out << " A" << y.a * 180 / pi << " " << p.machine.axis2() << y.b * 180 / pi;
            out << " E" << b.deltaE << " F" << metric / duration * 60 << "\n";
        }
        out << ";FAL_END\n";
    }
    out << "M400\n";
    if (p.machine.controller == Controller::Klipper && p.machine.rotary_axes() && p.machine.klipperExtension)
        out << "FIVEAXIS_DISABLE\n";
    return output;
}
Project replay_plate_gcode(const Project &context, const QString &code) {
    need(!context.path.empty(), "Replay requires initial machine/tool state");
    QStringList expectedOutside, actualOutside;
    std::vector<QStringList> expectedControls(1);
    bool inside = false;
    for (auto line : export_plate_gcode(context).split('\n')) {
        line = line.trimmed();
        if (line.startsWith(";FAL ")) {
            inside = true;
            expectedControls.push_back({});
            continue;
        }
        if (line == ";FAL_END") {
            inside = false;
            continue;
        }
        if (line.isEmpty() || line.startsWith(';'))
            continue;
        if (!inside)
            expectedOutside << line;
        else if (!line.startsWith("G1 ") && !line.startsWith("G4 "))
            expectedControls.back() << line;
    }
    Project result = context;
    result.path.clear();
    result.report = {};
    result.path.push_back(context.path.front());
    JointState axes = sample_axes(context, context.path.front());
    QString active = context.path.front().toolId, material = context.path.front().materialId;
    QJsonObject meta;
    QStringList commands;
    double elapsed = 0;
    bool frame = false;
    auto finish = [&] {
        need(frame, "Unexpected frame terminator");
        size_t index = result.path.size();
        need(index < context.path.size(), "Extra job frame");
        const auto &planned = context.path[index];
        auto controls = expectedControls[index];
        for (const auto &control : controls) {
            need(!commands.empty() && commands.front() == control,
                 "Controller configuration/action changed: " + control);
            commands.removeFirst();
        }
        Sample sample;
        sample.toolId = meta["tool"].toString();
        sample.materialId = meta["material"].toString();
        sample.objectId = meta["object"].toString();
        sample.event = EventKind(meta["event"].toInt());
        sample.argument = meta["argument"].toString();
        sample.role = Role(meta["role"].toInt());
        sample.layer = meta["layer"].toInt();
        sample.region = meta["region"].toInt();
        sample.width = meta["width"].toDouble();
        sample.height = meta["height"].toDouble();
        sample.dwell = meta["dwell"].toDouble();
        need(int(sample.event) >= 0 && int(sample.event) <= int(EventKind::Standby) &&
                 int(sample.role) >= 0 && int(sample.role) < role_count,
             "Invalid event metadata");
        if (meta.contains("normal")) {
            auto v = meta["normal"].toArray();
            need(v.size() == 3, "Invalid normal metadata");
            sample.surfaceNormal = Vec3(v[0].toDouble(), v[1].toDouble(), v[2].toDouble());
        }
        auto vector = [&](const char *name) {
            auto v = meta[name].toArray();
            need(v.size() == 3, "Invalid contact metadata");
            return Vec3(v[0].toDouble(), v[1].toDouble(), v[2].toDouble());
        };
        sample.contactComponent = meta["contactComponent"].toString();
        sample.contactFixture = meta["contactFixture"].toString();
        sample.contactCenter = vector("contactCenter");
        sample.contactSize = vector("contactSize");
        need(sample.toolId == planned.toolId && sample.materialId == planned.materialId &&
                 sample.objectId == planned.objectId && sample.event == planned.event &&
                 sample.argument == planned.argument && sample.role == planned.role &&
                 (sample.contactSize - planned.contactSize).norm() < 1e-9 &&
                 (sample.contactCenter - planned.contactCenter).norm() < 1e-9 &&
                 sample.contactComponent == planned.contactComponent &&
                 sample.contactFixture == planned.contactFixture,
             "Event metadata changed from the intended job");

        if (sample.event == EventKind::Attach)
            active = sample.argument;
        if (sample.event == EventKind::Detach)
            active.clear();
        if (sample.event == EventKind::Material)
            material = sample.argument;
        need(active == sample.toolId && material == sample.materialId,
             "Tool/material metadata disagrees with controller events");
        JointState next = axes;
        double dt = 0;
        bool movement = false;
        for (const auto &line : commands) {
            if (line.startsWith("G4 ")) {
                QRegularExpression re("P([-+0-9.eE]+)");
                auto match = re.match(line);
                need(match.hasMatch(), "Invalid dwell");
                need(QRegularExpression("^G4 P[-+0-9.eE]+$").match(line).hasMatch(),
                     "Unmodeled dwell parameter");
                dt += match.captured(1).toDouble() / 1000;
                continue;
            }
            need(line.startsWith("G1 "), "Unknown command in replay frame: " + line);
            need(!movement, "More than one motion per replay frame");
            movement = true;
            std::map<QString, double> values;
            QRegularExpression re("([XYZABCEF])\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
            auto matches = re.globalMatch(line);
            while (matches.hasNext()) {
                auto match = matches.next();
                double value = match.captured(2).toDouble();
                need(std::isfinite(value), "Non-finite replay coordinate");
                need(!values.count(match.captured(1)), "Duplicate motion coordinate");
                values[match.captured(1)] = value;
            }
            QString residue = line.mid(3);
            residue.remove(re);
            need(residue.trimmed().isEmpty(), "Unknown motion parameter in exported replay");
            for (int k = 0; k < 3; ++k) {
                QString key = QString("XYZ")[k];
                need(values.count(key), "Missing carriage coordinate");
                next.xyz[k] = values[key];
            }
            if (context.machine.rotary_axes()) {
                need(values.count("A") && values.count(context.machine.axis2()), "Missing rotary coordinate");
                next.a = values["A"] * pi / 180;
                next.b = values[context.machine.axis2()] * pi / 180;
            } else
                need(!values.count("A") && !values.count("B") && !values.count("C"),
                     "Rotary command in XYZ replay");
            sample.deltaE = values["E"];
            need(values.count("F") && values["F"] > 0, "Missing feedrate");
            dt += feed_metric(context.machine, axes, next, sample.deltaE) / (values["F"] / 60);
        }
        need((next.xyz - sample_axes(context, planned).xyz).norm() < 1e-7 &&
                 std::abs(next.a - sample_axes(context, planned).a) +
                         std::abs(next.b - sample_axes(context, planned).b) <
                     1e-7 &&
                 std::abs(sample.deltaE - planned.deltaE) < 1e-7,
             "Exported movement differs from intended carriage or extrusion");
        need(std::abs(dt - (planned.time - context.path[index - 1].time)) < 1e-6 * std::max(1., dt),
             "Controller timing differs from the intended job");
        need(dt > 0 && std::isfinite(dt), "Replay frame lacks motion or duration");
        elapsed += dt;
        sample.carriage = next;
        sample.joint = pose_from_axes(context.machine, assembly_at(context, active), next);
        sample.position = part_position(context.machine, sample.joint);
        sample.direction = part_direction(context.machine, assembly_at(context, active), sample.joint);
        sample.time = elapsed;
        result.path.push_back(sample);
        axes = next;
        frame = false;
        commands.clear();
    };
    for (auto line : code.split('\n')) {
        line = line.trimmed();
        if (line.startsWith(";FAL ")) {
            need(!frame, "Unclosed replay frame");
            QJsonParseError error;
            meta = QJsonDocument::fromJson(QByteArray::fromBase64(line.mid(5).toLatin1()), &error).object();
            need(error.error == QJsonParseError::NoError && !meta.isEmpty(), "Invalid replay event");
            frame = true;
            continue;
        }
        if (line == ";FAL_END") {
            finish();
            continue;
        }
        if (line.isEmpty() || line.startsWith(';'))
            continue;
        if (frame) {
            commands << line;
            continue;
        }
        actualOutside << line;
    }
    need(!frame, "Truncated replay frame");
    need(actualOutside == expectedOutside,
         "Startup/shutdown configuration differs from the generated controller profile");
    need(result.path.size() == context.path.size(), "Exported output lost job events");
    rebuild_job_events(result);
    return result;
}
} // namespace fiveaxis
