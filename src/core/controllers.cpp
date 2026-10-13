#include "controllers.h"
#include "collision.h"
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
    auto initialAxes = controller_axes(p.machine, p.toolhead, p.path.front().joint);
    QJsonObject metadata{
        {"machine", setup["machine"]},
        {"initial", setup["path"].toArray().first()},
        {"initialCarriage", QJsonArray{initialAxes.xyz.x(), initialAxes.xyz.y(), initialAxes.xyz.z()}}};
    stream << "; FIVEAXIS_METADATA " << QJsonDocument(metadata).toJson(QJsonDocument::Compact).toBase64()
           << "\nG21\nG90\nM83\n";
    if (p.machine.controller == Controller::Duet)
        stream << "M572 D0 S0\n";
    else {
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
            Vec3 tip = p.toolhead.mount * p.toolhead.tcp;
            const char *letters[] = {"X", "Y", "Z"};
            for (int j = 0; j < 3; ++j)
                stream << " TCP_" << letters[j] << "=" << tip[j] << " PIVOT_A" << letters[j] << "="
                       << p.machine.headPivotA[j] << " PIVOT_B" << letters[j] << "="
                       << p.machine.headPivotB[j];
            stream << "\n";
        }
    }
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1];
        const auto &b = p.path[i];
        auto axesA = controller_axes(p.machine, p.toolhead, a.joint),
             axesB = controller_axes(p.machine, p.toolhead, b.joint);
        double metric = feed_metric(p.machine, axesA, axesB, b.deltaE), duration = b.time - a.time;
        if (metric < 1e-12)
            continue;
        if (duration <= 0)
            throw std::runtime_error("Invalid movement duration");
        if (p.machine.controller == Controller::Klipper && !p.machine.klipperExtension && b.deltaE > 0) {
            double xyz = (axesB.xyz - axesA.xyz).norm();
            if (xyz > 1e-9 && b.deltaE / xyz > p.machine.stockExtrusionRatio)
                throw std::runtime_error("Stock Klipper extrusion ratio exceeded: enable the rotary-aware "
                                         "extension or change the path");
            if (xyz < 1e-9 && std::abs(b.deltaE) > 1)
                throw std::runtime_error("Stock Klipper extrusion-only limit exceeded");
        }
        stream << ";MOVE " << i << " ROLE " << int(b.role) << " LAYER " << b.layer << " WIDTH " << b.width
               << " HEIGHT " << b.height << "\nG1 X" << axesB.xyz.x() << " Y" << axesB.xyz.y() << " Z"
               << axesB.xyz.z() << " A" << b.joint.a * 180 / pi << " " << p.machine.axis2()
               << b.joint.b * 180 / pi << " E" << b.deltaE << " F" << metric / duration * 60 << "\n";
    }
    stream << "M400\n";
    if (p.machine.controller == Controller::Klipper && p.machine.klipperExtension)
        stream << "FIVEAXIS_DISABLE\n";
    return out;
}
Project replay_gcode(const Project &context, const QString &gcode) {
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
    state = controller_axes(p.machine, p.toolhead, state);
    bool absolute = true, relativeE = true;
    double absoluteExtrusion = 0, feed = 0;
    Role role = Role::Travel;
    int layer = 0;
    double width = context.process.width, height = context.process.layerHeight;
    auto parse = [](const QString &line) {
        std::map<QString, double> values;
        QRegularExpression re("([XYZABCEF])\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
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
            }
            continue;
        }
        if (line.startsWith(';') || line.isEmpty())
            continue;
        line = line.section(';', 0, 0).trimmed();
        QString command = line.section(' ', 0, 0).toUpper();
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
        if (command == "G21" || command == "M400" || command == "M572" || command == "SET_PRESSURE_ADVANCE" ||
            command == "MANUAL_STEPPER" || command == "FIVEAXIS_ENABLE" || command == "FIVEAXIS_DISABLE")
            continue;
        if (command != "G1" && command != "G0")
            throw std::runtime_error("Replay cannot verify unsupported command: " + command.toStdString());
        auto v = parse(line);
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
        s.joint = pose_from_axes(p.machine, p.toolhead, next);
        s.position = part_position(p.machine, s.joint);
        s.direction = part_direction(p.machine, p.toolhead, s.joint);
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
QString controller_template(const MachineProfile &m) {
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
