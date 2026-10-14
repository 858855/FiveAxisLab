#include "types.h"
#include "machine_settings.h"
#include "plate.h"
#include <QCryptographicHash>
#include <QJsonDocument>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fiveaxis {
Eigen::AlignedBox3d Mesh::bounds() const {
    Eigen::AlignedBox3d b;
    for (const auto &v : vertices)
        b.extend(v);
    return b;
}
Transform translation(const Vec3 &p) {
    Transform t = Transform::Identity();
    t.translation() = p;
    return t;
}
Transform rotation_x(double a) {
    Transform t = Transform::Identity();
    t.linear() = Eigen::AngleAxisd(a, Vec3::UnitX()).toRotationMatrix();
    return t;
}
Transform rotation_y(double a) {
    Transform t = Transform::Identity();
    t.linear() = Eigen::AngleAxisd(a, Vec3::UnitY()).toRotationMatrix();
    return t;
}
Transform rotation_z(double a) {
    Transform t = Transform::Identity();
    t.linear() = Eigen::AngleAxisd(a, Vec3::UnitZ()).toRotationMatrix();
    return t;
}
static Transform head_a(const MachineProfile &m, const JointState &q) {
    return translation(m.headPivotA) * rotation_x(q.a) * translation(-m.headPivotA);
}
static Transform head_b(const MachineProfile &m, const JointState &q) {
    return translation(m.headPivotB) * rotation_y(q.b) * translation(-m.headPivotB);
}
Transform workpiece_pose(const MachineProfile &m, const JointState &q) {
    if (m.layout != Layout::RotaryBed)
        return Transform::Identity();
    return translation(m.bedPivot) * rotation_x(q.a) * rotation_z(q.b) * translation(-m.bedPivot);
}
Transform component_pose(const MachineProfile &m, const ToolheadAssembly &t, const Component &c,
                         const JointState &q) {
    Transform stage = Transform::Identity(), full = Transform::Identity();
    if (m.layout == Layout::TiltingHead) {
        full = head_a(m, q) * head_b(m, q);
        if (c.link == Link::RotaryStage)
            stage = head_a(m, q);
        if (c.link == Link::Nozzle)
            stage = full;
    }
    Vec3 origin = q.xyz - full * (t.mount * t.tcp);
    return translation(origin) * stage * t.mount;
}
Vec3 part_position(const MachineProfile &m, const JointState &q) {
    return workpiece_pose(m, q).inverse() * q.xyz;
}
Vec3 part_direction(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q) {
    Vec3 axis = (t.mount.linear() * t.axis).normalized();
    if (!m.rotary_axes())
        return axis;
    if (m.layout == Layout::RotaryBed)
        return workpiece_pose(m, q).linear().transpose() * axis;
    return (head_a(m, q) * head_b(m, q)).linear() * axis;
}
bool in_limits(const MachineProfile &m, const JointState &q) {
    if (!m.rotary_axes())
        return q.xyz.allFinite() && (q.xyz.array() >= m.minXYZ.array() - 1e-8).all() &&
               (q.xyz.array() <= m.maxXYZ.array() + 1e-8).all() && std::isfinite(q.a) && std::isfinite(q.b) &&
               std::abs(q.a) < 1e-10 && std::abs(q.b) < 1e-10;
    return q.xyz.allFinite() && (q.xyz.array() >= m.minXYZ.array() - 1e-8).all() &&
           (q.xyz.array() <= m.maxXYZ.array() + 1e-8).all() && std::isfinite(q.a) && std::isfinite(q.b) &&
           q.a >= m.minA - 1e-8 && q.a <= m.maxA + 1e-8 && q.b >= m.minB - 1e-8 && q.b <= m.maxB + 1e-8;
}
static Vec3 tip_offset(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q) {
    Vec3 tip = t.mount * t.tcp;
    return m.layout == Layout::TiltingHead ? (head_a(m, q) * head_b(m, q)) * tip : tip;
}
JointState controller_axes(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q) {
    JointState axes = q;
    axes.xyz -= tip_offset(m, t, q);
    return axes;
}
JointState pose_from_axes(const MachineProfile &m, const ToolheadAssembly &t, const JointState &axes) {
    JointState q = axes;
    q.xyz += tip_offset(m, t, axes);
    return q;
}
bool in_limits(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q) {
    return in_limits(m, controller_axes(m, t, q));
}
JointState interpolate_motion(const MachineProfile &m, const ToolheadAssembly &t, const JointState &a,
                              const JointState &b, double u) {
    return pose_from_axes(m, t, interpolate(controller_axes(m, t, a), controller_axes(m, t, b), u));
}
std::vector<JointState> inverse_kinematics(const MachineProfile &m, const Vec3 &p, const Vec3 &normal,
                                           const JointState &previous, const ToolheadAssembly &t) {
    if (!p.allFinite() || !normal.allFinite() || normal.norm() < 1e-10)
        throw std::runtime_error("Invalid target pose");
    Vec3 n = normal.normalized();
    std::vector<JointState> result;
    if (!m.rotary_axes()) {
        Vec3 axis = (t.mount.linear() * t.axis).normalized();
        if ((axis - Vec3::UnitZ()).norm() > 1e-6 || (n - axis).norm() > 1e-6)
            return result;
        JointState q;
        q.xyz = p;
        if (in_limits(m, t, q))
            result.push_back(q);
        return result;
    }
    std::vector<std::pair<double, double>> branches;
    Vec3 mounted = (t.mount.linear() * t.axis).normalized();
    if ((mounted - Vec3::UnitZ()).norm() > 1e-10) {
        if (m.layout == Layout::RotaryBed) {
            double radius = std::hypot(mounted.y(), mounted.z());
            if (radius < 1e-10) {
                if (std::abs(n.z()) > 1e-8)
                    return result;
                Vec3 target = rotation_x(-previous.a).linear() * mounted;
                branches.push_back(
                    {previous.a, std::atan2(target.y(), target.x()) - std::atan2(n.y(), n.x())});
            } else if (std::abs(n.z()) <= radius + 1e-10) {
                double phase = std::atan2(mounted.y(), mounted.z()),
                       angle = std::acos(std::clamp(n.z() / radius, -1.0, 1.0));
                for (double a : {angle - phase, -angle - phase}) {
                    Vec3 target = rotation_x(-a).linear() * mounted;
                    double b = std::hypot(n.x(), n.y()) < 1e-9
                                   ? previous.b
                                   : std::atan2(target.y(), target.x()) - std::atan2(n.y(), n.x());
                    branches.push_back({a, b});
                }
            }
        } else {
            double radius = std::hypot(mounted.x(), mounted.z());
            if (radius < 1e-10) {
                if (std::abs(n.x()) > 1e-8)
                    return result;
                branches.push_back(
                    {std::atan2(n.z(), n.y()) - std::atan2(mounted.z(), mounted.y()), previous.b});
            } else if (std::abs(n.x()) <= radius + 1e-10) {
                double phase = std::atan2(mounted.z(), mounted.x()),
                       angle = std::acos(std::clamp(n.x() / radius, -1.0, 1.0));
                for (double b : {phase + angle, phase - angle}) {
                    Vec3 tilted = rotation_y(b).linear() * mounted;
                    double a = std::hypot(n.y(), n.z()) < 1e-9
                                   ? previous.a
                                   : std::atan2(n.z(), n.y()) - std::atan2(tilted.z(), tilted.y());
                    branches.push_back({a, b});
                }
            }
        }
    } else if (m.layout == Layout::RotaryBed) {
        double h = std::hypot(n.x(), n.y()), a = std::atan2(h, n.z());
        double b = h < 1e-9 ? previous.b : std::atan2(n.x(), n.y());
        branches = {{a, b}, {-a, b + pi}};
    } else {
        double b = std::asin(std::clamp(n.x(), -1.0, 1.0));
        double a = std::hypot(n.y(), n.z()) < 1e-9 ? previous.a : std::atan2(-n.y(), n.z());
        branches = {{a, b}, {a + pi, pi - b}};
    }
    for (auto [a, b] : branches)
        for (int k = -1; k <= 1; ++k)
            for (int j = -1; j <= 1; ++j) {
                JointState q;
                q.a = a + k * 2 * pi;
                q.b = b + j * 2 * pi;
                q.xyz = m.layout == Layout::RotaryBed ? workpiece_pose(m, q) * p : p;
                if (in_limits(m, t, q))
                    result.push_back(q);
            }
    std::sort(result.begin(), result.end(), [&](const auto &x, const auto &y) {
        return std::abs(x.a - previous.a) + std::abs(x.b - previous.b) <
               std::abs(y.a - previous.a) + std::abs(y.b - previous.b);
    });
    return result;
}
JointState interpolate(const JointState &a, const JointState &b, double u) {
    JointState q;
    q.xyz = a.xyz + (b.xyz - a.xyz) * u;
    q.a = a.a + (b.a - a.a) * u;
    q.b = a.b + (b.b - a.b) * u;
    return q;
}
double movement_bound(const MachineProfile &m, const ToolheadAssembly &t, const JointState &a,
                      const JointState &b, double radius) {
    if (!m.rotary_axes())
        return (b.xyz - a.xyz).norm();
    const double rotation = std::abs(b.a - a.a) + std::abs(b.b - a.b);
    if (m.layout == Layout::RotaryBed) {
        double lever = radius + std::max((a.xyz - m.bedPivot).norm(), (b.xyz - m.bedPivot).norm());
        return (b.xyz - a.xyz).norm() + rotation * lever;
    }
    double lever = radius + m.headPivotA.norm() + 2 * m.headPivotB.norm() + (t.mount * t.tcp).norm();
    return (controller_axes(m, t, b).xyz - controller_axes(m, t, a).xyz).norm() + rotation * lever;
}
QString role_name(Role r) {
    switch (r) {
    case Role::Wall:
        return "Wall";
    case Role::OuterWall:
        return "Outer wall";
    case Role::SolidInfill:
        return "Solid infill";
    case Role::TopSurface:
        return "Top surface";
    case Role::BottomSurface:
        return "Bottom surface";
    case Role::Brim:
        return "Brim";
    case Role::Skirt:
        return "Skirt";
    case Role::Bridge:
        return "Bridge";
    case Role::Infill:
        return "Infill";
    case Role::Support:
        return "Support";
    case Role::SupportInterface:
        return "Support interface";
    case Role::Purge:
        return "Purge";
    case Role::Retract:
        return "Retract";
    default:
        return "Travel";
    }
}
QString layout_name(Layout layout) {
    return layout == Layout::RotaryBed     ? "Rotary bed"
           : layout == Layout::TiltingHead ? "Tilting head"
                                           : "Standard XYZ · fixed nozzle";
}
Vec3 surface_normal(const Sample &sample) {
    return sample.surfaceNormal.value_or(sample.direction);
}
void Project::invalidate() {
    report = CollisionReport{};
    depositionValid = false;
    comparison = {};
    deposition = {};
}
QString Project::fingerprint() const {
    auto json = project_json(*this);
    json.remove("report");
    json.remove("diagnostics");
    json.remove("depositionValid");
    json.remove("comparison");
    json.remove("deposition");
    json.remove("filament");
    auto job = json["plateJob"].toObject();
    job.remove("compute");
    job.remove("computation");
    json["plateJob"] = job;
    return QString::fromLatin1(QCryptographicHash::hash(QJsonDocument(json).toJson(QJsonDocument::Compact),
                                                        QCryptographicHash::Sha256)
                                   .toHex());
}
void time_path(Project &p) {
    if (p.plateEnabled) {
        time_plate(p);
        return;
    }
    if (p.path.empty())
        return;
    p.path.front().time = 0;
    for (size_t i = 1; i < p.path.size(); ++i) {
        auto &a = p.path[i - 1];
        auto &b = p.path[i];
        double cart = (controller_axes(p.machine, p.active_toolhead(), b.joint).xyz -
                       controller_axes(p.machine, p.active_toolhead(), a.joint).xyz)
                          .norm(),
               angle = std::max(std::abs(b.joint.a - a.joint.a), std::abs(b.joint.b - a.joint.b));
        double bead = (b.position - a.position).norm();
        // Segment time is the cruise time the exported feedrate commands. Firmware look-ahead
        // applies acceleration across consecutive segments; timing each short segment as a
        // stop-start move would cap printing near 2*sqrt(length/accel) per segment.
        double dt = std::max({cart / p.machine.linearSpeed, angle / p.machine.angularSpeed,
                              bead / std::max(0.1, b.speed), 0.001});
        double volume = std::max(0.0, b.deltaE) * pi * std::pow(p.machine.filamentDiameter / 2, 2);
        dt = std::max(dt, volume / p.machine.maxFlow);
        dt = std::max(dt, b.dwell);
        dt = std::max(dt,
                      printer_move_seconds(p.machine,
                                           controller_axes(p.machine, p.active_toolhead(), b.joint).xyz -
                                               controller_axes(p.machine, p.active_toolhead(), a.joint).xyz,
                                           b.deltaE, p.machine.printer.value("extruder").toObject()));
        b.time = a.time + dt;
    }
}
SimulationState simulation_at(const Project &p, double time, bool rebuildDeposition) {
    if (p.plateEnabled)
        return simulate_plate(p, time, rebuildDeposition);
    SimulationState s;
    if (p.path.empty())
        return s;
    s.time = std::clamp(time, 0.0, p.path.back().time);
    auto it = std::upper_bound(p.path.begin(), p.path.end(), s.time,
                               [](double t, const Sample &v) { return t < v.time; });
    int next = std::min<int>(it - p.path.begin(), int(p.path.size()) - 1);
    int previous = std::max(0, next - 1);
    s.move = next;
    double dt = p.path[next].time - p.path[previous].time;
    s.fraction = dt > 0 ? (s.time - p.path[previous].time) / dt : 0;
    s.joint = interpolate_motion(p.machine, p.active_toolhead(), p.path[previous].joint, p.path[next].joint,
                                 s.fraction);
    for (int i = 1; rebuildDeposition && i <= next; ++i)
        if (p.path[i].deltaE > 0 && p.path[i].role != Role::Retract) {
            Vec3 end = p.path[i].position;
            if (i == next)
                end = part_position(p.machine, s.joint);
            s.deposited.emplace_back(p.path[i - 1].position, end);
        }
    return s;
}
std::vector<ToolpathSegment> simulation_toolpaths(const Project &p) {
    std::vector<ToolpathSegment> result;
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1], &b = p.path[i];
        int count = 1;
        for (int attempt = 0; attempt < 12; ++attempt) {
            bool good = true;
            for (int j = 0; j < count; ++j) {
                auto point = [&](double u) {
                    return part_position(p.machine,
                                         (p.plateEnabled ? sample_motion(p, a, b, u)
                                                         : interpolate_motion(p.machine, p.active_toolhead(),
                                                                              a.joint, b.joint, u)));
                };
                double u = double(j) / count, v = double(j + 1) / count;
                if ((point((u + v) / 2) - (point(u) + point(v)) / 2).norm() > p.machine.chordTolerance ||
                    std::max(std::abs(b.joint.a - a.joint.a), std::abs(b.joint.b - a.joint.b)) / count >
                        p.machine.angularTolerance) {
                    good = false;
                    break;
                }
            }
            if (good || count >= 2048)
                break;
            count *= 2;
        }
        for (int j = 0; j < count; ++j) {
            double u = double(j) / count, v = double(j + 1) / count;
            Vec3 start = part_position(
                p.machine, interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u));
            Vec3 end = part_position(
                p.machine,
                (p.plateEnabled ? sample_motion(p, a, b, v)
                                : interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, v)));
            if ((end - start).norm() > 1e-9)
                result.push_back({start, end, int(i), b.layer, b.deltaE > 0 ? b.role : Role::Travel, u, v});
        }
    }
    return result;
}
} // namespace fiveaxis
