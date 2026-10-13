#include "controllers.h"
#include "collision.h"
#include "machine_settings.h"
#include "nonplanar.h"
#include "plate.h"
#include "profile.h"
#include "project.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QTextStream>
#include <cmath>
#include <map>
#include <stdexcept>

namespace fiveaxis {
double feed_metric(const MachineProfile &m, const JointState &a, const JointState &b, double deltaE) {
    double xyz = (b.xyz - a.xyz).norm();
    if (xyz > 1e-9)
        return xyz;
    double da = std::abs(b.a - a.a) * 180 / pi, db = std::abs(b.b - a.b) * 180 / pi;
    if (m.controller == Controller::Klipper)
        return std::max({da, db, std::abs(deltaE)});
    return std::max(std::hypot(da, db), std::abs(deltaE));
}
QString export_gcode(const Project &p) {
    if (p.plateEnabled)
        return export_plate_gcode(p);
    check_printer_envelope(p);
    if ((p.machine.controller == Controller::GenericMarlin && p.machine.rotary_axes()) ||
        (p.process.nonPlanar3Axis && p.machine.rotary_axes()) ||
        (!p.machine.rotary_axes() && cartesian_motion_violation(p) >= 0))
        throw std::runtime_error("Export blocked: incompatible XYZ controller or motion constraints");
    if (p.process.profileNormal && profile_motion_violation(p) >= 0)
        throw std::runtime_error("Export blocked: nozzle is not perpendicular to the profile");
    if (!p.report.exportable() || p.report.fingerprint != p.fingerprint())
        throw std::runtime_error("Export blocked: validate the current geometry, machine, and motion first");
    if (!p.depositionValid)
        throw std::runtime_error("Export blocked: deposition planning is incomplete");
    QString out;
    QTextStream stream(&out);
    stream.setRealNumberNotation(QTextStream::FixedNotation);
    stream.setRealNumberPrecision(9);
    stream << "; FiveAxisLab research G-code\n; Profile fingerprint: " << p.fingerprint()
           << "\n; Start in the calibrated initial pose; homing is machine-specific.\n; Geometry "
              "verification does not establish physical print safety.\n";
    auto setup = project_json(p);
    stream << "; Toolhead model: "
           << (p.useDummyToolhead ? "DUMMY - synthetic geometry only" : "imported / configured geometry")
           << "\n";
    auto initialAxes = controller_axes(p.machine, p.active_toolhead(), p.path.front().joint);
    QJsonObject metadata{
        {"machine", setup["machine"]},
        {"initial", setup["path"].toArray().first()},
        {"initialCarriage", QJsonArray{initialAxes.xyz.x(), initialAxes.xyz.y(), initialAxes.xyz.z()}}};
    stream << "; FIVEAXIS_METADATA " << QJsonDocument(metadata).toJson(QJsonDocument::Compact).toBase64()
           << "\nG21\nG90\n"
           << (relative_extrusion(p.machine) ? "M83\n" : "M82\nG92 E0\n");
    double absoluteE = 0;
    if (p.machine.controller == Controller::Duet)
        stream << "M572 D0 S0\n";
    else if (p.machine.controller == Controller::Klipper && p.machine.rotary_axes()) {
        stream
            << "SET_PRESSURE_ADVANCE ADVANCE=0\nMANUAL_STEPPER STEPPER=rotary_a GCODE_AXIS=\nMANUAL_STEPPER "
               "STEPPER=rotary_b GCODE_AXIS=\nMANUAL_STEPPER STEPPER=rotary_a GCODE_AXIS=A LIMIT_VELOCITY="
            << p.machine.angularSpeed * 180 / pi << " LIMIT_ACCEL=" << p.machine.angularAccel * 180 / pi
            << "\nMANUAL_STEPPER STEPPER=rotary_b GCODE_AXIS=" << p.machine.axis2()
            << " LIMIT_VELOCITY=" << p.machine.angularSpeed * 180 / pi
            << " LIMIT_ACCEL=" << p.machine.angularAccel * 180 / pi << "\n";
        if (p.machine.klipperExtension)
            stream << "FIVEAXIS_ENABLE LAYOUT=" << (p.machine.layout == Layout::RotaryBed ? "BED" : "HEAD")
                   << " AXIS2=" << p.machine.axis2() << " PIVOT_X=" << p.machine.bedPivot.x()
                   << " PIVOT_Y=" << p.machine.bedPivot.y() << " PIVOT_Z=" << p.machine.bedPivot.z()
                   << " MAX_FLOW=" << p.machine.maxFlow;
        if (p.machine.klipperExtension) {
            Vec3 tip = p.active_toolhead().mount * p.active_toolhead().tcp;
            const char *letters[] = {"X", "Y", "Z"};
            for (int j = 0; j < 3; ++j)
                stream << " TCP_" << letters[j] << "=" << tip[j] << " PIVOT_A" << letters[j] << "="
                       << p.machine.headPivotA[j] << " PIVOT_B" << letters[j] << "="
                       << p.machine.headPivotB[j];
            stream << "\n";
        }
    }
    if (p.machine.controller == Controller::Klipper && !p.machine.rotary_axes())
        stream << "SET_PRESSURE_ADVANCE ADVANCE=0\n";
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1];
        const auto &b = p.path[i];
        auto axesA = controller_axes(p.machine, p.active_toolhead(), a.joint),
             axesB = controller_axes(p.machine, p.active_toolhead(), b.joint);
        double metric = feed_metric(p.machine, axesA, axesB, b.deltaE), duration = b.time - a.time;
        if (metric < 1e-12) {
            stream << "G4 P" << duration * 1000 << "\n";
            continue;
        }
        if (duration <= 0)
            throw std::runtime_error("Invalid movement duration");
        if (p.machine.controller == Controller::Klipper &&
            (!p.machine.klipperExtension || !p.machine.rotary_axes()) && b.deltaE > 0) {
            double xyz = (axesB.xyz - axesA.xyz).norm();
            if (xyz > 1e-9 && b.deltaE / xyz > p.machine.stockExtrusionRatio)
                throw std::runtime_error("Stock Klipper extrusion ratio exceeded: enable the rotary-aware "
                                         "extension or change the path");
            if (xyz < 1e-9 && std::abs(b.deltaE) > 1)
                throw std::runtime_error("Stock Klipper extrusion-only limit exceeded");
        }
        stream << ";MOVE " << i << " ROLE " << int(b.role) << " LAYER " << b.layer << " WIDTH " << b.width
               << " HEIGHT " << b.height;
        if (b.surfaceNormal)
            stream << " NX " << b.surfaceNormal->x() << " NY " << b.surfaceNormal->y() << " NZ "
                   << b.surfaceNormal->z();
        stream << "\nG1 X" << axesB.xyz.x() << " Y" << axesB.xyz.y() << " Z" << axesB.xyz.z();
        if (p.machine.rotary_axes())
            stream << " A" << b.joint.a * 180 / pi << " " << p.machine.axis2() << b.joint.b * 180 / pi;
        absoluteE += b.deltaE;
        stream << " E" << (relative_extrusion(p.machine) ? b.deltaE : absoluteE) << " F"
               << metric / duration * 60 << "\n";
    }
    stream << "M400\n";
    if (p.machine.controller == Controller::Klipper && p.machine.klipperExtension && p.machine.rotary_axes())
        stream << "FIVEAXIS_DISABLE\n";
    return out;
}
Project replay_gcode(const Project &context, const QString &gcode) {
    if (context.plateEnabled)
        return replay_plate_gcode(context, gcode);
    Project p = context;
    p.path.clear();
    p.report = CollisionReport{};
    JointState state;
    Sample initial;
    if (!context.path.empty()) {
        initial = context.path.front();
        state = initial.joint;
    } else {
        initial.joint = state;
        initial.position = part_position(p.machine, state);
    }
    p.path.push_back(initial);
    state = controller_axes(p.machine, p.active_toolhead(), state);
    bool absolute = true, relativeE = true;
    double absoluteExtrusion = 0, feed = 0;
    Role role = Role::Travel;
    int layer = 0;
    double width = context.process.width, height = context.process.layerHeight;
    std::optional<Vec3> surfaceNormal;
    auto parse = [](const QString &line) {
        std::map<QString, double> values;
        QRegularExpression re("([XYZABCEFPS])\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
        auto it = re.globalMatch(line);
        while (it.hasNext()) {
            auto match = it.next();
            bool ok;
            double v = match.captured(2).toDouble(&ok);
            if (!ok || !std::isfinite(v))
                throw std::runtime_error("Invalid G-code number");
            values[match.captured(1)] = v;
        }
        return values;
    };
    for (auto line : gcode.split('\n')) {
        line = line.trimmed();
        if (line.startsWith(";MOVE ")) {
            auto a = line.split(' ', Qt::SkipEmptyParts);
            if (a.size() >= 10) {
                role = Role(a[3].toInt());
                layer = a[5].toInt();
                width = a[7].toDouble();
                height = a[9].toDouble();
                surfaceNormal.reset();
                if (a.size() >= 16 && a[10] == "NX" && a[12] == "NY" && a[14] == "NZ")
                    surfaceNormal = Vec3(a[11].toDouble(), a[13].toDouble(), a[15].toDouble());
            }
            continue;
        }
        if (line.startsWith(';') || line.isEmpty())
            continue;
        line = line.section(';', 0, 0).trimmed();
        QString command = line.section(' ', 0, 0).toUpper();
        if (!p.machine.rotary_axes() &&
            (command == "MANUAL_STEPPER" || command == "FIVEAXIS_ENABLE" || command == "FIVEAXIS_DISABLE"))
            throw std::runtime_error("Rotary command in an XYZ-only replay");
        if (command == "G90") {
            absolute = true;
            continue;
        }
        if (command == "G91") {
            absolute = false;
            continue;
        }
        if (command == "M83") {
            relativeE = true;
            continue;
        }
        if (command == "M82") {
            relativeE = false;
            continue;
        }
        if (command == "G92" && line == "G92 E0") {
            absoluteExtrusion = 0;
            continue;
        }
        if (command == "G4") {
            auto values = parse(line);
            double dt = values.count("P") ? values["P"] / 1000 : values["S"];
            auto sample = p.path.back();
            sample.deltaE = 0;
            sample.time += dt;
            sample.role = Role::Travel;
            sample.dwell = dt;
            p.path.push_back(sample);
            continue;
        }
        if (command == "G21" || command == "M400" || command == "M572" || command == "SET_PRESSURE_ADVANCE" ||
            command == "MANUAL_STEPPER" || command == "FIVEAXIS_ENABLE" || command == "FIVEAXIS_DISABLE")
            continue;
        if (command != "G1" && command != "G0")
            throw std::runtime_error("Replay cannot verify unsupported command: " + command.toStdString());
        auto v = parse(line);
        if (!p.machine.rotary_axes() && (v.count("A") || v.count("B") || v.count("C")))
            throw std::runtime_error("Rotary axis word in an XYZ-only replay");
        JointState next = state;
        for (int j = 0; j < 3; ++j) {
            QString key = QString("XYZ")[j];
            if (v.count(key))
                next.xyz[j] = absolute ? v[key] : next.xyz[j] + v[key];
        }
        if (v.count("A"))
            next.a = absolute ? v["A"] * pi / 180 : next.a + v["A"] * pi / 180;
        if (v.count(p.machine.axis2()))
            next.b = absolute ? v[p.machine.axis2()] * pi / 180 : next.b + v[p.machine.axis2()] * pi / 180;
        double e = 0;
        if (v.count("E")) {
            e = relativeE ? v["E"] : v["E"] - absoluteExtrusion;
            absoluteExtrusion = relativeE ? absoluteExtrusion + e : v["E"];
        }
        if (v.count("F"))
            feed = v["F"] / 60;
        if (feed <= 0)
            throw std::runtime_error("G-code move lacks positive feedrate");
        Sample s;
        s.joint = pose_from_axes(p.machine, p.active_toolhead(), next);
        s.position = part_position(p.machine, s.joint);
        s.direction = part_direction(p.machine, p.active_toolhead(), s.joint);
        s.surfaceNormal = surfaceNormal;
        s.deltaE = e;
        s.role = e > 0 ? role : e < 0 ? Role::Retract : Role::Travel;
        s.layer = layer;
        s.width = width;
        s.height = height;
        s.time = p.path.back().time + feed_metric(p.machine, state, next, e) / feed;
        s.speed = feed;
        p.path.push_back(s);
        state = next;
    }
    p.depositionValid = context.depositionValid;
    return p;
}
QString controller_template(const Project &p) {
    auto text = controller_template(p.machine) + printer_controller_template(p.machine);
    if (!p.plateEnabled)
        return text;
    bool klipper = p.machine.controller == Controller::Klipper,
         duet = p.machine.controller == Controller::Duet;
    QString comment = klipper ? "# " : "; ";
    text += comment + "Plate tool definitions: firmware XYZ tool offsets are zero; planned carriage paths "
                      "apply TCP calibration.\n";
    for (const auto &t : p.tools) {
        text += comment +
                QString("%1: installed nozzle %2 mm, extruder %3, heater %4, TCP in project CAD frame.\n")
                    .arg(t.id)
                    .arg(t.nozzle.diameter)
                    .arg(t.extruder, t.heater);
        if (duet)
            text += QString("M563 P%1 D%2 H%3\nG10 P%1 X0 Y0 Z0\nM572 D%2 S0\n")
                        .arg(t.controllerTool)
                        .arg(t.duetDrive)
                        .arg(t.duetHeater);
        else if (klipper)
            text += comment + QString("Configure [%1] with nozzle_diameter: %2 and pressure_advance: 0; "
                                      "calibrate its pins/heater.\n")
                                  .arg(t.extruder)
                                  .arg(t.nozzle.diameter);
        else
            text += QString("M218 T%1 X0 Y0 Z0\n").arg(t.controllerTool);
    }
    if (klipper && std::any_of(p.tools.begin(), p.tools.end(),
                               [](const ToolDefinition &t) { return t.detachable || t.routes.size() > 1; }))
        text +=
            "# Install fiveaxis_tools.py alongside fiveaxis_flow.py. Configure real filament_switch_sensor "
            "latch sensors.\n[fiveaxis_tools]\n# selectors: {\"0\":{\"pin\":\"selector\",\"value\":0}}\n";
    text += comment + "Dock waypoints, I/O pins, sensor checks and purge volumes require the calibrated "
                      "profile. Arbitrary motion macros are not replay-supported.\n";
    return text;
}
QString controller_template(const MachineProfile &m) {
    if (!m.rotary_axes()) {
        if (m.controller == Controller::Klipper)
            return "# Standard XYZ printer: use its calibrated printer.cfg.\n# Existing [stepper_x], "
                   "[stepper_y], [stepper_z] and [extruder] remain in use.\n# No rotary steppers or "
                   "fiveaxis_flow extension is required.\n# Keep pressure_advance: 0 until this non-planar "
                   "profile is qualified.\n";
        if (m.controller == Controller::Duet)
            return "; Standard XYZ printer: preserve its calibrated config.g and homing macros.\n; Cartesian "
                   "mapping example only:\nM584 X0 Y1 Z2 E3\nM572 D0 S0\n";
        return "; Generic / Marlin XYZ output: G21, G90, M83 and coordinated G1 XYZ/E.\n; Use the printer's "
               "calibrated settings, homing, heating and startup procedure.\n; No rotary axes are "
               "configured.\n";
    }
    if (m.controller == Controller::GenericMarlin)
        throw std::runtime_error("Generic / Marlin output is available only for standard XYZ machines");
    if (m.controller == Controller::Duet)
        return QString("; Reference only: calibrate drivers, endstops, limits and steps/degree for your "
                       "machine.\nM584 X0 Y1 Z2 E3\nM584 A4 %1%2 R1 S1\nM92 A<steps_per_degree> "
                       "%1<steps_per_degree>\nM203 A%3 %1%3\nM201 A%4 %1%4\n; Create homea.g and home%5.g "
                       "before using these axes.\nM572 D0 S0\n")
            .arg(m.axis2())
            .arg(5)
            .arg(m.angularSpeed * 180 / pi * 60)
            .arg(m.angularAccel * 180 / pi)
            .arg(m.axis2().toLower());
    auto config =
        QString("# Pin and qualify a Klipper revision with MANUAL_STEPPER GCODE_AXIS.\n# Configure "
                "rotary_a / rotary_b with real pins and endstops.\n# Coordinates use degrees: "
                "rotation_distance and gear_ratio must reflect the mechanics.\n[manual_stepper "
                "rotary_a]\nstep_pin: <pin>\ndir_pin: <pin>\nmicrosteps: 16\nrotation_distance: "
                "360\nvelocity: %1\naccel: "
                "%2\n\n[manual_stepper rotary_b]\nstep_pin: <pin>\ndir_pin: <pin>\nrotation_distance: "
                "360\nmicrosteps: 16\nvelocity: %1\naccel: %2\n\n# Home manual steppers BEFORE registering A "
                "and %3.\n# "
                "Copy fiveaxis_flow.py to klippy/extras, then add [fiveaxis_flow].\n[fiveaxis_flow]\n")
            .arg(m.angularSpeed * 180 / pi)
            .arg(m.angularAccel * 180 / pi)
            .arg(m.axis2());
    if (!m.klipperExtension)
        config.replace("[fiveaxis_flow]\n", "");
    return config;
}
} // namespace fiveaxis
