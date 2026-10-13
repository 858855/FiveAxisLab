#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <vector>

namespace fiveaxis {
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;
constexpr double pi = 3.14159265358979323846;
enum class Layout { RotaryBed, TiltingHead };
enum class Controller { Duet, Klipper };
enum class Link { Carriage, RotaryStage, Nozzle };
enum class Role { Travel, Wall, Infill, Support, Retract };
enum class Status { Clear, Collision, Unresolved, Cancelled };

struct Mesh {
    QString name;
    std::vector<Vec3> vertices;
    std::vector<Eigen::Vector3i> triangles;
    double error = 0;
    Eigen::AlignedBox3d bounds() const;
    bool empty() const {
        return vertices.empty() || triangles.empty();
    }
};
struct Tetrahedron {
    std::vector<int> nodes;
};
struct VolumeMesh {
    std::vector<Vec3> vertices;
    std::vector<Tetrahedron> cells;
};
struct Component {
    QString id, name;
    Mesh mesh;
    Link link = Link::Nozzle;
    bool visualOnly = false;
    bool nozzle = false;
    Mesh renderMesh;
    Transform assemblyPlacement = Transform::Identity();
};
struct ToolheadAssembly {
    struct InterfaceAllowance {
        QString first, second;
        Vec3 center = Vec3::Zero(), size = Vec3::Ones();
    };
    QString sourceName, sourceHash;
    QByteArray sourceStep;
    std::vector<Component> components;
    std::vector<InterfaceAllowance> interfaces;
    Transform mount = Transform::Identity();
    Vec3 tcp = Vec3::Zero();
    Vec3 axis = Vec3::UnitZ();
    double contactRadius = 0.55, contactHeight = 0.75;
    QString sourceUnits;
};
struct JointState {
    Vec3 xyz = Vec3(0, 0, 30);
    double a = 0, b = 0;
};
struct MachineProfile {
    Layout layout = Layout::RotaryBed;
    Controller controller = Controller::Duet;
    Vec3 bedPivot = Vec3::Zero(), headPivotA = Vec3(0, 0, 30), headPivotB = Vec3(0, 0, 15);
    Vec3 minXYZ = Vec3(-160, -160, -100), maxXYZ = Vec3(160, 160, 300);
    double minA = -75 * pi / 180, maxA = 75 * pi / 180, minB = -pi, maxB = pi;
    double linearSpeed = 100, linearAccel = 500, angularSpeed = 30 * pi / 180, angularAccel = 60 * pi / 180;
    double clearance = 0.5, chordTolerance = 0.02, angularTolerance = 1 * pi / 180;
    double filamentDiameter = 1.75, maxFlow = 10;
    bool klipperExtension = true;
    double stockExtrusionRatio = 0.25;
    QString klipperRevision;
    QString axis2() const {
        return layout == Layout::RotaryBed ? "C" : "B";
    }
};
struct Fixture {
    QString name;
    Mesh mesh;
    bool movesWithBed = false;
    QByteArray sourceStep;
};
struct Sample {
    Vec3 position = Vec3::Zero(), direction = Vec3::UnitZ();
    JointState joint;
    Role role = Role::Travel;
    double time = 0, deltaE = 0, width = 0.45, height = 0.2, speed = 25;
    int layer = 0, region = 0;
};
struct CollisionEvent {
    Status status = Status::Collision;
    QString first, second, reason;
    int move = 0;
    double time = 0, clearance = 0;
    Vec3 contact = Vec3::Zero();
};
struct CollisionReport {
    Status status = Status::Unresolved;
    QString fingerprint;
    std::vector<CollisionEvent> events;
    std::vector<QString> modeledComponents;
    double minimumClearance = 1e100;
    size_t checkedIntervals = 0;
    bool exportable() const {
        return status == Status::Clear;
    }
};
struct StressField {
    VolumeMesh mesh;
    std::vector<Mat3> tensors;
    bool pointData = true;
    bool directional = false;
    QString source, step;
    std::vector<double> scalar;
    Mat3 sample(const Vec3 &p, bool *covered = nullptr) const;
    double scalar_at(const Vec3 &p, bool *covered = nullptr) const;
};
struct Material {
    double young = 2200, poisson = 0.35, transverseYoung = 1200;
};
struct LoadCase {
    std::vector<int> fixedNodes;
    std::vector<std::pair<int, Vec3>> nodalForces;
    Material material;
};
struct FEAResult {
    StressField stress;
    std::vector<Vec3> displacement;
    double compliance = 0;
};
struct ProcessSettings {
    double layerHeight = 0.2, width = 0.45, infill = 0.25, speed = 25;
    double directionCone = 10 * pi / 180, maximumTilt = 65 * pi / 180;
    bool curved = true, supports = true;
    int wallCount = 2;
    double maximumBridge = 5;
};
struct DepositionReport {
    bool checked = false;
    std::vector<int> unsupportedMoves;
    std::vector<std::pair<int, int>> dependencies;
    double coverage = 0;
};
struct StructuralComparison {
    bool available = false;
    double planarCompliance = 0, curvedCompliance = 0, alignment = 0;
};
struct Project {
    Mesh part;
    Transform partPlacement = Transform::Identity();
    VolumeMesh volume;
    ToolheadAssembly toolhead;
    MachineProfile machine;
    ProcessSettings process;
    StressField stress;
    LoadCase loads;
    StructuralComparison comparison;
    DepositionReport deposition;
    std::vector<Fixture> fixtures;
    std::vector<Sample> path;
    CollisionReport report;
    QStringList diagnostics;
    bool depositionValid = false;
    QString fingerprint() const;
    void invalidate();
};
struct SimulationState {
    double time = 0, fraction = 0;
    int move = 0;
    JointState joint;
    std::vector<std::pair<Vec3, Vec3>> deposited;
};
Transform translation(const Vec3 &p);
Transform rotation_x(double a);
Transform rotation_y(double a);
Transform rotation_z(double a);
Transform workpiece_pose(const MachineProfile &m, const JointState &q);
Transform component_pose(const MachineProfile &m, const ToolheadAssembly &t, const Component &c,
                         const JointState &q);
Vec3 part_position(const MachineProfile &m, const JointState &q);
Vec3 part_direction(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q);
std::vector<JointState> inverse_kinematics(const MachineProfile &m, const Vec3 &p, const Vec3 &n,
                                           const JointState &previous, const ToolheadAssembly &t = {});
bool in_limits(const MachineProfile &m, const JointState &q);
bool in_limits(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q);
JointState controller_axes(const MachineProfile &m, const ToolheadAssembly &t, const JointState &q);
JointState pose_from_axes(const MachineProfile &m, const ToolheadAssembly &t, const JointState &axes);
JointState interpolate_motion(const MachineProfile &m, const ToolheadAssembly &t, const JointState &a,
                              const JointState &b, double u);
JointState interpolate(const JointState &a, const JointState &b, double u);
double movement_bound(const MachineProfile &m, const ToolheadAssembly &t, const JointState &a,
                      const JointState &b, double radius);
void time_path(Project &p);
SimulationState simulation_at(const Project &p, double time, bool rebuildDeposition = true);
Project demo_project();
QString role_name(Role r);
QJsonObject project_json(const Project &p);
Project project_from_json(const QJsonObject &json);
} // namespace fiveaxis
