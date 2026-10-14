#include "machine_settings.h"
#include "geometry.h"
#include "plate.h"
#include "process.h"
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fiveaxis {
static void need(bool value, const QString &message) {
    if (!value)
        throw std::runtime_error(message.toStdString());
}
const std::vector<PrinterOption> &motion_options() {
    static const std::vector<PrinterOption> options{
        {"speedX", "Maximum speed X", "Speed limits", "mm/s", 100, .01, 10000, false},
        {"speedY", "Maximum speed Y", "Speed limits", "mm/s", 100, .01, 10000, false},
        {"speedZ", "Maximum speed Z", "Speed limits", "mm/s", 20, .01, 10000, false},
        {"speedE", "Maximum speed E", "Speed limits", "mm/s", 30, .01, 10000, false},
        {"accelX", "Maximum acceleration X", "Acceleration limits", "mm/s²", 500, .01, 100000, false},
        {"accelY", "Maximum acceleration Y", "Acceleration limits", "mm/s²", 500, .01, 100000, false},
        {"accelZ", "Maximum acceleration Z", "Acceleration limits", "mm/s²", 500, .01, 100000, false},
        {"accelE", "Maximum acceleration E", "Acceleration limits", "mm/s²", 5000, .01, 100000, false},
        {"accelExtrude", "Maximum acceleration while extruding", "Acceleration limits", "mm/s²", 500, .01,
         100000, false},
        {"accelRetract", "Maximum acceleration while retracting", "Acceleration limits", "mm/s²", 5000, .01,
         100000, false},
        {"jerkX", "Maximum jerk X", "Jerk limits", "mm/s", 9, 0, 1000, false},
        {"jerkY", "Maximum jerk Y", "Jerk limits", "mm/s", 9, 0, 1000, false},
        {"jerkZ", "Maximum jerk Z", "Jerk limits", "mm/s", 3, 0, 1000, false},
        {"jerkE", "Maximum jerk E", "Jerk limits", "mm/s", 2.5, 0, 1000, false}};
    return options;
}
const std::vector<PrinterOption> &extruder_options() {
    static const std::vector<PrinterOption> options{
        {"nozzleVolume", "Nozzle volume", "Basic information", "mm³", 107, 0, 100000},
        {"retractLength", "Length", "Retraction", "mm", .8, 0, 100, false},
        {"hop", "Z-hop when retracting", "Retraction", "mm", .4, 0, 100, false},
        {"hopMin", "Z-hop lower bound", "Retraction", "mm", 0, -10000, 10000},
        {"hopMax", "Z-hop upper bound", "Retraction", "mm", 1000, -10000, 10000},
        {"retractSpeed", "Retraction speed", "Retraction", "mm/s", 30, .01, 1000},
        {"recoverSpeed", "Deretraction speed", "Retraction", "mm/s", 30, .01, 1000},
        {"travelThreshold", "Travel distance threshold", "Retraction", "mm", 3, 0, 1000},
        {"wipeDistance", "Wipe distance", "Retraction", "mm", 2, 0, 100},
        {"retractBeforeWipe", "Retraction before wipe", "Retraction", "%", 0, 0, 100}};
    return options;
}
double printer_number(const QJsonObject &settings, const char *key, double fallback) {
    return settings.contains(key) ? settings[key].toDouble(fallback) : fallback;
}
static void validate_options(const QJsonObject &settings, const std::vector<PrinterOption> &options) {
    for (const auto &option : options)
        if (settings.contains(option.key)) {
            const auto value = settings[option.key];
            need(value.isDouble() && std::isfinite(value.toDouble()) && value.toDouble() >= option.minimum &&
                     value.toDouble() <= option.maximum,
                 QString("Invalid printer setting: %1").arg(option.label));
        }
}
void validate_extruder_settings(const QJsonObject &settings) {
    validate_options(settings, extruder_options());
    need(printer_number(settings, "hopMax", 1000) >= printer_number(settings, "hopMin", 0),
         "Z-hop upper bound must not be below its lower bound");
}
static double cross(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}
static std::vector<Vec3> polygon(const QJsonValue &value) {
    std::vector<Vec3> result;
    need(value.isArray(), "Bed polygon must be a list of XY coordinates");
    for (const auto &entry : value.toArray()) {
        auto a = entry.toArray();
        need(a.size() == 2 && a[0].isDouble() && a[1].isDouble() && std::isfinite(a[0].toDouble()) &&
                 std::isfinite(a[1].toDouble()),
             "Invalid bed polygon coordinate");
        result.emplace_back(a[0].toDouble(), a[1].toDouble(), 0);
    }
    need(result.size() >= 3 && result.size() <= 1024, "Bed polygon requires 3 to 1024 vertices");
    double area = 0;
    for (size_t i = 0; i < result.size(); ++i) {
        const auto &a = result[i], &b = result[(i + 1) % result.size()];
        need((a - b).norm() > 1e-6, "Duplicate bed polygon vertex");
        area += a.x() * b.y() - b.x() * a.y();
        for (size_t j = i + 2; j < result.size(); ++j) {
            if ((j + 1) % result.size() == i)
                continue;
            const auto &c = result[j], &d = result[(j + 1) % result.size()];
            // Reject touching or crossing nonadjacent edges as well as overlapping collinear edges.
            if (std::max(a.x(), b.x()) + 1e-8 < std::min(c.x(), d.x()) ||
                std::max(c.x(), d.x()) + 1e-8 < std::min(a.x(), b.x()) ||
                std::max(a.y(), b.y()) + 1e-8 < std::min(c.y(), d.y()) ||
                std::max(c.y(), d.y()) + 1e-8 < std::min(a.y(), b.y()))
                continue;
            need(!(cross(a, b, c) * cross(a, b, d) <= 0 && cross(c, d, a) * cross(c, d, b) <= 0),
                 "Bed polygon intersects itself");
        }
    }
    need(std::abs(area) > 1e-6, "Bed polygon has no area");
    if (area < 0)
        std::reverse(result.begin(), result.end());
    return result;
}
std::vector<Vec3> bed_outline(const QJsonObject &settings) {
    auto shape = settings["bedShape"].toString("rectangle");
    if (shape == "custom")
        return polygon(settings["bedPolygon"]);
    double w = printer_number(settings, "bedWidth", 320), h = printer_number(settings, "bedDepth", 320),
           ox = printer_number(settings, "bedOriginX", 160), oy = printer_number(settings, "bedOriginY", 160);
    need(w > 0 && h > 0 && w <= 10000 && h <= 10000 && std::isfinite(w + h + ox + oy),
         "Invalid printable bed dimensions");
    if (shape == "rectangle")
        return {Vec3(-ox, -oy, 0), Vec3(w - ox, -oy, 0), Vec3(w - ox, h - oy, 0), Vec3(-ox, h - oy, 0)};
    need(shape == "circle", "Unknown bed shape");
    need(std::abs(w - h) < 1e-6, "Circular bed width and depth must match");
    std::vector<Vec3> outline;
    for (int i = 0; i < 128; ++i)
        outline.emplace_back(w / 2 - ox + w / 2 * std::cos(2 * pi * i / 128),
                             h / 2 - oy + h / 2 * std::sin(2 * pi * i / 128), 0);
    return outline;
}
void validate_printer_settings(const MachineProfile &machine) {
    const auto &s = machine.printer;
    validate_options(s, motion_options());
    validate_extruder_settings(s["extruder"].toObject());
    if (s["bedEnabled"].toBool()) {
        bed_outline(s);
        auto height = printer_number(s, "printableHeight", 300);
        need(std::isfinite(height) && height > 0, "Printable height must be positive");
        for (const auto &entry : s["bedExclusions"].toArray())
            polygon(entry);
    }
    for (const auto *key : {"loadSeconds", "unloadSeconds", "extruderChangeSeconds", "hotendChangeSeconds",
                            "clearanceRadius", "rodDistance", "rodHeight", "lidHeight"}) {
        auto v = printer_number(s, key, 0);
        need(std::isfinite(v) && v >= 0, QString("Invalid printer setting: %1").arg(key));
    }
}
static bool inside(const std::vector<Vec3> &poly, const Vec3 &point, double *distance = nullptr) {
    bool hit = false;
    double d = 1e100;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const auto &a = poly[j], &b = poly[i];
        Vec3 edge = b - a, delta(point.x() - a.x(), point.y() - a.y(), 0);
        d = std::min(d, (delta - edge * std::clamp(delta.dot(edge) / edge.squaredNorm(), 0.0, 1.0)).norm());
        if ((a.y() > point.y()) != (b.y() > point.y()) &&
            point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x())
            hit = !hit;
    }
    if (distance)
        *distance = d;
    return hit || d <= 1e-8;
}
bool printable_point(const MachineProfile &machine, const Vec3 &point, double margin) {
    const auto &s = machine.printer;
    if (!s["bedEnabled"].toBool())
        return true;
    if (!point.allFinite() || point.z() < -1e-6 || point.z() > printer_number(s, "printableHeight", 300))
        return false;
    double distance;
    if (!inside(bed_outline(s), point, &distance) || distance + 1e-8 < margin)
        return false;
    for (const auto &entry : s["bedExclusions"].toArray())
        if (inside(polygon(entry), point, &distance) || distance < margin)
            return false;
    return true;
}
Mesh printer_bed_mesh(const QJsonObject &settings) {
    auto outline = bed_outline(settings);
    Mesh mesh;
    mesh.name = "Printer bed";
    mesh.error = settings["bedShape"].toString() == "circle"
                     ? printer_number(settings, "bedWidth", 320) / 2 * (1 - std::cos(pi / 128))
                     : 0;
    mesh.vertices = outline;
    for (auto v : outline) {
        v.z() = -2;
        mesh.vertices.push_back(v);
    }
    std::vector<int> remaining;
    for (int i = 0; i < int(outline.size()); ++i)
        remaining.push_back(i);
    while (remaining.size() > 2) {
        bool clipped = false;
        for (size_t j = 0; j < remaining.size(); ++j) {
            int a = remaining[(j + remaining.size() - 1) % remaining.size()], b = remaining[j],
                c = remaining[(j + 1) % remaining.size()];
            if (cross(outline[a], outline[b], outline[c]) <= 1e-9)
                continue;
            bool occupied = false;
            for (int k : remaining)
                if (k != a && k != b && k != c && cross(outline[a], outline[b], outline[k]) >= -1e-9 &&
                    cross(outline[b], outline[c], outline[k]) >= -1e-9 &&
                    cross(outline[c], outline[a], outline[k]) >= -1e-9)
                    occupied = true;
            if (occupied)
                continue;
            int n = int(outline.size());
            mesh.triangles.push_back({a, b, c});
            mesh.triangles.push_back({c + n, b + n, a + n});
            remaining.erase(remaining.begin() + j);
            clipped = true;
            break;
        }
        need(clipped, "Cannot triangulate bed polygon; remove collinear vertices");
    }
    int n = int(outline.size());
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        mesh.triangles.push_back({i, i + n, j + n});
        mesh.triangles.push_back({i, j + n, j});
    }
    return mesh;
}
Vec3 preferred_plate_position(const MachineProfile &m) {
    if (!m.printer.value("bedEnabled").toBool())
        return m.rotary_axes()
                   ? Vec3::Zero()
                   : Vec3((m.minXYZ.x() + m.maxXYZ.x()) / 2, (m.minXYZ.y() + m.maxXYZ.y()) / 2, 0);
    Eigen::AlignedBox3d bounds;
    for (auto v : bed_outline(m.printer))
        bounds.extend(v);
    return Vec3(bounds.min().x() + bounds.sizes().x() * printer_number(m.printer, "preferredX", .5),
                bounds.min().y() + bounds.sizes().y() * printer_number(m.printer, "preferredY", .5), 0);
}
double printer_move_seconds(const MachineProfile &m, const Vec3 &delta, double e, const QJsonObject &ex) {
    // Per-axis cruise limits only. Acceleration limits are firmware settings (see
    // printer_controller_template); they shape segments through look-ahead and are not
    // a per-segment stop-start time, which would also lower the exported feedrate.
    double result = 0;
    const char *speed[] = {"speedX", "speedY", "speedZ", "speedE"};
    for (int i = 0; i < 4; ++i) {
        double d = i < 3 ? std::abs(delta[i]) : std::abs(e);
        if (m.printer.contains(speed[i]))
            result = std::max(result, d / m.printer[speed[i]].toDouble());
    }
    if ((delta.norm() < 1e-8 || e < 0) && ex.contains(e < 0 ? "retractSpeed" : "recoverSpeed"))
        result =
            std::max(result, std::abs(e) / printer_number(ex, e < 0 ? "retractSpeed" : "recoverSpeed", 30));
    return result;
}
double estimated_print_seconds(const Project &p) {
    size_t n = p.path.size();
    if (n < 2)
        return 0;
    double accel = std::min(printer_number(p.machine.printer, "accelExtrude", p.machine.linearAccel),
                            std::min(printer_number(p.machine.printer, "accelX", p.machine.linearAccel),
                                     printer_number(p.machine.printer, "accelY", p.machine.linearAccel)));
    double corner = std::max(1.0, std::min(printer_number(p.machine.printer, "jerkX", 5),
                                           printer_number(p.machine.printer, "jerkY", 5)));
    struct Segment {
        Vec3 delta = Vec3::Zero();
        double length = 0, cruise = 0, fixed = 0;
    };
    std::vector<Segment> segments(n);
    for (size_t i = 1; i < n; ++i) {
        const auto &a = p.path[i - 1], &b = p.path[i];
        auto x = p.plateEnabled ? sample_axes(p, a) : controller_axes(p.machine, p.active_toolhead(), a.joint);
        auto y = p.plateEnabled ? sample_axes(p, b) : controller_axes(p.machine, p.active_toolhead(), b.joint);
        double dt = b.time - a.time;
        auto &s = segments[i];
        s.delta = y.xyz - x.xyz;
        s.length = s.delta.norm();
        // Rotary-only, extruder-only and dwell movements keep their planned duration.
        if (s.length < 1e-9 || std::abs(y.a - x.a) + std::abs(y.b - x.b) > 1e-9 || b.dwell > 0)
            s.fixed = dt;
        else
            s.cruise = s.length / std::max(dt, 1e-9);
    }
    // Junction speed: full speed when collinear, falling to the corner speed at 90 degrees or more.
    std::vector<double> junction(n + 1, 0);
    for (size_t i = 1; i + 1 < n; ++i) {
        const auto &a = segments[i], &b = segments[i + 1];
        if (a.cruise <= 0 || b.cruise <= 0)
            continue;
        double c = std::clamp(a.delta.dot(b.delta) / (a.length * b.length), -1.0, 1.0);
        double v = std::min(a.cruise, b.cruise);
        junction[i] = c <= 0 ? std::min(v, corner) : std::max(std::min(v, corner), v * c);
    }
    // junction[i] is the speed at the end of segment i / start of segment i + 1.
    for (size_t i = n - 1; i >= 1; --i)
        if (segments[i].cruise > 0)
            junction[i - 1] = std::min(junction[i - 1],
                                       std::sqrt(junction[i] * junction[i] + 2 * accel * segments[i].length));
    for (size_t i = 1; i < n; ++i)
        if (segments[i].cruise > 0)
            junction[i] = std::min(junction[i], std::sqrt(junction[i - 1] * junction[i - 1] +
                                                          2 * accel * segments[i].length));
    double total = 0;
    for (size_t i = 1; i < n; ++i) {
        const auto &s = segments[i];
        if (s.cruise <= 0) {
            total += s.fixed;
            continue;
        }
        double v0 = std::min(junction[i - 1], s.cruise), v1 = std::min(junction[i], s.cruise), vc = s.cruise;
        double up = (vc * vc - v0 * v0) / (2 * accel), down = (vc * vc - v1 * v1) / (2 * accel);
        if (up + down > s.length) {
            vc = std::sqrt(std::max(0.0, (2 * accel * s.length + v0 * v0 + v1 * v1) / 2));
            up = (vc * vc - v0 * v0) / (2 * accel);
            down = (vc * vc - v1 * v1) / (2 * accel);
        }
        double flat = std::max(0.0, s.length - up - down);
        total += (vc - v0) / accel + (vc - v1) / accel + flat / std::max(vc, 1e-9);
    }
    return total;
}
bool printer_homing(const QString &line) {
    return QRegularExpression("^G28(\\s+[XYZ]0?)*$", QRegularExpression::CaseInsensitiveOption)
        .match(line.trimmed())
        .hasMatch();
}
bool printer_passthrough(const QString &line, bool allowHoming) {
    static const QRegularExpression safe(
        "^(M10[49]|M14[019]|M19[01]|M10[67]|M117|M73|M204|M220|M221|M900)(\\s+[A-Z][-+0-9.]*)*(\\s+.*)?$",
        QRegularExpression::CaseInsensitiveOption);
    QString text = line.trimmed();
    if (text.contains('\n') || text.contains('\r'))
        return false;
    return (allowHoming && printer_homing(text)) || safe.match(text).hasMatch();
}
bool relative_extrusion(const MachineProfile &machine) {
    return machine.printer.value("relativeE").toBool(true);
}
QString printer_path_signature(const Project &p) {
    if (p.machine.printer.isEmpty())
        return process_fingerprint(p.process);
    auto settings = p.machine.printer;
    for (const char *key : {"relativeE", "name", "notes", "advanced", "bedTexture", "bedTextureName"})
        settings.remove(key);
    auto scripts = settings["gcode"].toObject();
    scripts.remove("pause");
    scripts.remove("custom");
    settings["gcode"] = scripts;
    QJsonObject inputs{{"process", process_fingerprint(p.process)}, {"printer", settings}};
    return QString::fromLatin1(QCryptographicHash::hash(QJsonDocument(inputs).toJson(QJsonDocument::Compact),
                                                        QCryptographicHash::Sha256)
                                   .toHex());
}
static QJsonObject extruder_for(const Project &p, const Sample &s) {
    if (auto tool = find_tool(p, s.toolId))
        return tool->extruderSettings;
    return p.machine.printer.value("extruder").toObject();
}
void check_printer_envelope(const Project &p) {
    validate_printer_settings(p.machine);
    if (!p.machine.printer.value("bedEnabled").toBool())
        return;
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1], &b = p.path[i];
        if (b.deltaE <= 0 || b.role == Role::Retract || b.event != EventKind::Motion)
            continue;
        double margin = std::max(b.width, b.height) / 2 + p.machine.chordTolerance;
        auto point = [&](double u) {
            return part_position(
                p.machine, p.plateEnabled
                               ? sample_motion(p, a, b, u)
                               : interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u));
        };
        // Lipschitz distance to polygon edges plus a conservative rotary movement bound certifies intervals.
        std::function<void(double, double, int)> check = [&](double lo, double hi, int depth) {
            Vec3 middle = point((lo + hi) / 2), left = point(lo), right = point(hi);
            need(printable_point(p.machine, middle, margin) && printable_point(p.machine, left, margin) &&
                     printable_point(p.machine, right, margin),
                 QString("Deposition outside printable area / excluded region at movement %1").arg(i));
            double radius = std::max(a.joint.xyz.norm(), b.joint.xyz.norm()) + p.machine.bedPivot.norm() + 1;
            double bound = (b.joint.xyz - a.joint.xyz).norm() * (hi - lo) / 2;
            if (p.machine.layout == Layout::RotaryBed)
                bound += radius * (std::abs(b.joint.a - a.joint.a) + std::abs(b.joint.b - a.joint.b)) *
                         (hi - lo) / 2;
            if (printable_point(p.machine, middle, margin + bound) && middle.z() - bound >= -1e-6 &&
                middle.z() + bound <= printer_number(p.machine.printer, "printableHeight", 300))
                return;
            need(depth < 22, QString("Printable-area interval unresolved at movement %1").arg(i));
            check(lo, (lo + hi) / 2, depth + 1);
            check((lo + hi) / 2, hi, depth + 1);
        };
        check(0, 1, 0);
    }
}
void apply_printer_path(Project &p) {
    validate_printer_settings(p.machine);
    if (p.path.size() < 2)
        return;
    std::vector<Sample> output;
    std::vector<int> mapped(p.path.size());
    auto axes = [&](const Sample &s) {
        return p.plateEnabled ? sample_axes(p, s) : controller_axes(p.machine, p.active_toolhead(), s.joint);
    };
    auto set_axes = [&](Sample s, const JointState &q) {
        auto assembly = p.plateEnabled ? assembly_at(p, s.toolId) : p.active_toolhead();
        s.joint = pose_from_axes(p.machine, assembly, q);
        s.position = part_position(p.machine, s.joint);
        s.direction = part_direction(p.machine, assembly, s.joint);
        if (p.plateEnabled)
            s.carriage = q;
        need(in_limits(p.machine, assembly, s.joint), "Printer script / Z-hop exceeds machine travel limits");
        return s;
    };
    auto travel = [&](Sample anchor, const JointState &q, const QString &reason) {
        anchor.deltaE = 0;
        anchor.role = Role::Travel;
        anchor.event = EventKind::Motion;
        anchor.argument = reason;
        output.push_back(set_axes(anchor, q));
    };
    auto script = [&](const char *key, bool restore) {
        auto text = p.machine.printer.value("gcode").toObject()[key].toString();
        Sample anchor = output.back();
        JointState original = axes(anchor);
        bool moved = false;
        for (auto line : text.split('\n')) {
            line = line.section(';', 0, 0).trimmed();
            if (line.isEmpty())
                continue;
            if (line == "M400") {
                auto s = output.back();
                s.deltaE = 0;
                s.event = EventKind::Synchronize;
                s.dwell = .001;
                output.push_back(s);
                continue;
            }
            // Heater, fan and similar commands move no axis and are emitted verbatim. Homing is
            // accepted only in single-object start scripts; export then re-enters the modeled pose.
            if (printer_passthrough(line, !p.plateEnabled && QString(key) == "start")) {
                auto s = output.back();
                s.deltaE = 0;
                s.role = Role::Travel;
                s.event = EventKind::IO;
                s.argument = line.simplified();
                output.push_back(s);
                continue;
            }
            QRegularExpression word("([XYZABCFPS])\\s*([-+]?(?:[0-9]*\\.)?[0-9]+(?:[eE][-+]?[0-9]+)?)");
            auto command = line.section(' ', 0, 0).toUpper();
            QString residue = line.mid(command.size());
            auto matches = word.globalMatch(residue);
            QMap<QString, double> values;
            while (matches.hasNext()) {
                auto match = matches.next();
                bool ok;
                double value = match.captured(2).toDouble(&ok);
                need(ok && std::isfinite(value) && !values.contains(match.captured(1)),
                     "Invalid printer G-code number");
                values[match.captured(1)] = value;
            }
            residue.remove(word);
            need(residue.trimmed().isEmpty() && (command == "G0" || command == "G1" || command == "G4"),
                 QString("Unmodeled %1 G-code: %2. Use explicit controller/tool recipes; verified export is "
                         "blocked.")
                     .arg(key, line));
            auto s = output.back();
            s.deltaE = 0;
            s.event = EventKind::Motion;
            s.role = Role::Travel;
            s.argument = QString("printer/%1").arg(key);
            if (command == "G4") {
                need(values.size() == 1 && (values.contains("P") || values.contains("S")),
                     "G4 requires P milliseconds or S seconds");
                s.dwell = values.contains("P") ? values["P"] / 1000 : values["S"];
                need(s.dwell >= 0 && s.dwell <= 86400, "Invalid printer dwell");
                s.event = EventKind::Synchronize;
                output.push_back(s);
            } else {
                need(!values.contains("P") && !values.contains("S") &&
                         (!values.contains("F") || values["F"] > 0),
                     "Invalid printer travel parameters");
                if (!p.machine.rotary_axes())
                    need(!values.contains("A") && !values.contains("B") && !values.contains("C"),
                         "Rotary word in XYZ printer script");
                else
                    need(!values.contains(p.machine.axis2() == "C" ? "B" : "C"),
                         "Incorrect rotary axis in printer script");
                auto q = axes(s);
                for (int j = 0; j < 3; ++j) {
                    QString k = QString("XYZ")[j];
                    if (values.contains(k))
                        q.xyz[j] = values[k];
                }
                if (values.contains("A"))
                    q.a = values["A"] * pi / 180;
                if (p.machine.rotary_axes() && values.contains(p.machine.axis2()))
                    q.b = values[p.machine.axis2()] * pi / 180;
                if (values.contains("F"))
                    s.speed = values["F"] / 60;
                output.push_back(set_axes(s, q));
                moved = true;
            }
        }
        if (restore && moved)
            travel(anchor, original, "printer/return");
    };
    output.push_back(p.path.front());
    script("start", true);
    double retracted = 0, hop = 0;
    QString retractTool;
    JointState unlifted;
    for (size_t i = 1; i < p.path.size(); ++i) {
        auto b = p.path[i];
        const auto &a = p.path[i - 1];
        auto ex = extruder_for(p, b);
        bool same = a.toolId == b.toolId && a.materialId == b.materialId;
        if (!same || b.event != EventKind::Motion) {
            need(retracted == 0, "Retraction cannot cross an unmodeled tool/material event");
            if (!same && a.materialId != b.materialId)
                script("filamentChange", true);
        }
        if (a.objectId != b.objectId && !a.objectId.isEmpty() && !b.objectId.isEmpty())
            script("betweenObjects", true);
        if (a.layer != b.layer && same) {
            script("layerChange", true);
            if (p.machine.printer.value("timelapseEnabled").toBool())
                script("timelapse", true);
        }
        if (same && b.event == EventKind::Motion && b.deltaE == 0 && a.deltaE > 0 &&
            a.role != Role::Retract && ex["enabled"].toBool() && retracted == 0) {
            double totalTravel = 0;
            size_t end = i;
            for (; end < p.path.size(); ++end) {
                const auto &s = p.path[end];
                if (s.toolId != b.toolId || s.materialId != b.materialId || s.event != EventKind::Motion ||
                    s.deltaE != 0)
                    break;
                totalTravel += (axes(s).xyz - axes(p.path[end - 1]).xyz).norm();
            }
            bool resumes = end < p.path.size() && p.path[end].deltaE > 0 && p.path[end].toolId == b.toolId &&
                           p.path[end].materialId == b.materialId && p.path[end].event == EventKind::Motion;
            if (resumes && printer_number(ex, "retractLength", .8) > 0 &&
                (totalTravel >= printer_number(ex, "travelThreshold", 3) ||
                 (ex["retractLayerChange"].toBool() && p.path[end].layer != a.layer))) {
                retracted = printer_number(ex, "retractLength", .8);
                retractTool = b.toolId;
                double wiped = 0;
                if (ex["wipe"].toBool() && i >= 2) {
                    auto prior = axes(p.path[i - 2]), current = axes(a);
                    Vec3 back = prior.xyz - current.xyz;
                    double distance = std::min(back.norm(), printer_number(ex, "wipeDistance", 2));
                    if (distance > 1e-8) {
                        double fraction = printer_number(ex, "retractBeforeWipe", 0) / 100;
                        if (fraction > 0) {
                            auto s = output.back();
                            s.deltaE = -retracted * fraction;
                            s.role = Role::Retract;
                            s.argument = "printer/retract before wipe";
                            output.push_back(s);
                        }
                        current.xyz += back.normalized() * distance;
                        auto s = set_axes(a, current);
                        s.deltaE = -retracted * (1 - fraction);
                        s.role = Role::Retract;
                        s.argument = "printer/wipe";
                        output.push_back(s);
                        wiped = retracted;
                    }
                }
                if (wiped == 0) {
                    auto s = output.back();
                    s.deltaE = -retracted;
                    s.role = Role::Retract;
                    s.argument = "printer/retract";
                    output.push_back(s);
                }
                double z = axes(a).xyz.z();
                hop = z >= printer_number(ex, "hopMin", 0) && z <= printer_number(ex, "hopMax", 1000)
                          ? printer_number(ex, "hop", .4)
                          : 0;
                unlifted = axes(output.back());
                if (hop > 0) {
                    auto lifted = unlifted;
                    lifted.xyz.z() += hop;
                    travel(a, lifted, "printer/hop");
                }
            }
        }
        if (retracted > 0 && b.deltaE > 0) {
            need(b.toolId == retractTool, "Printer retraction tool mismatch");
            if (hop > 0)
                travel(a, unlifted, "printer/lower");
            auto recover = output.back();
            recover.deltaE = retracted;
            recover.role = Role::Retract;
            recover.argument = "printer/recover";
            output.push_back(recover);
            retracted = hop = 0;
        }
        mapped[i] = int(output.size());
        if (hop > 0 && b.deltaE == 0 && b.event == EventKind::Motion) {
            unlifted = axes(b);
            auto lifted = unlifted;
            lifted.xyz.z() += hop;
            b = set_axes(b, lifted);
            b.argument = "printer/lifted travel";
        }
        output.push_back(b);
    }
    script("end", false);
    for (auto &index : p.deposition.unsupportedMoves)
        if (index >= 0 && index < int(mapped.size()))
            index = mapped[index];
    p.path = std::move(output);
    time_path(p);
    check_printer_envelope(p);
    if (!p.plateEnabled)
        p.pathProcessFingerprint = printer_path_signature(p);
}
QString printer_controller_template(const MachineProfile &m) {
    if (m.printer.isEmpty())
        return {};
    QString text;
    QTextStream out(&text);
    if (m.controller == Controller::Klipper) {
        out << "\n# Printer editor limits: copy these into calibrated printer.cfg "
               "sections.\n[printer]\nmax_velocity: "
            << std::min(printer_number(m.printer, "speedX", m.linearSpeed),
                        printer_number(m.printer, "speedY", m.linearSpeed))
            << "\nmax_accel: "
            << std::min(printer_number(m.printer, "accelX", m.linearAccel),
                        printer_number(m.printer, "accelY", m.linearAccel))
            << "\nmax_z_velocity: " << printer_number(m.printer, "speedZ", m.linearSpeed)
            << "\nmax_z_accel: " << printer_number(m.printer, "accelZ", m.linearAccel)
            << "\n# Classic XYZE jerk is not Klipper square_corner_velocity; tune that independently.\n";
    } else {
        out << "\n; Printer editor firmware limits\nM203";
        const char *speed[] = {"speedX", "speedY", "speedZ", "speedE"};
        const char *accel[] = {"accelX", "accelY", "accelZ", "accelE"};
        const char *jerk[] = {"jerkX", "jerkY", "jerkZ", "jerkE"};
        for (int j = 0; j < 4; ++j)
            out << " " << QString("XYZE")[j]
                << printer_number(m.printer, speed[j], 30) * (m.controller == Controller::Duet ? 60 : 1);
        out << "\nM201";
        for (int j = 0; j < 4; ++j)
            out << " " << QString("XYZE")[j] << printer_number(m.printer, accel[j], 500);
        out << (m.controller == Controller::Duet ? "\nM566" : "\nM205");
        for (int j = 0; j < 4; ++j)
            out << " " << QString("XYZE")[j]
                << printer_number(m.printer, jerk[j], 3) * (m.controller == Controller::Duet ? 60 : 1);
        out << "\n";
    }
    return text;
}
} // namespace fiveaxis
