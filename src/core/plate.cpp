#include "plate.h"
#include "collision.h"
#include "compute.h"
#include "geometry.h"
#include "machine_settings.h"
#include "process.h"
#include "project.h"
#include "slicing.h"
#include "stress.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <QCryptographicHash>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace fiveaxis {
static void require(bool condition, const QString &message) {
    if (!condition)
        throw std::runtime_error(message.toStdString());
}
const ToolDefinition *find_tool(const Project &p, const QString &id) {
    for (const auto &tool : p.tools)
        if (tool.id == id)
            return &tool;
    return nullptr;
}
const MaterialProfile *find_material(const Project &p, const QString &id) {
    for (const auto &m : p.materials)
        if (m.id == id)
            return &m;
    return nullptr;
}
ToolheadAssembly installed_assembly(const ToolDefinition &tool) {
    auto result = tool.geometry();
    require(!result.components.empty(), "Tool " + tool.id + ": import STEP or configure its dummy");
    if (tool.nozzle.dummyNozzle || !tool.nozzle.separateNozzle.components.empty()) {
        auto nozzle =
            tool.nozzle.dummyNozzle ? make_dummy_toolhead(tool.nozzle.diameter) : tool.nozzle.separateNozzle;
        result.components.erase(std::remove_if(result.components.begin(), result.components.end(),
                                               [](const Component &c) { return c.nozzle; }),
                                result.components.end());
        for (auto component : nozzle.components) {
            if (tool.nozzle.dummyNozzle && !component.nozzle)
                continue;
            component.id = "installed/" + component.id;
            for (auto &v : component.mesh.vertices)
                v = tool.nozzle.nozzleMount * v;
            for (auto &v : component.renderMesh.vertices)
                v = tool.nozzle.nozzleMount * v;
            component.link = Link::Nozzle;
            result.components.push_back(std::move(component));
        }
        result.tcp = tool.nozzle.nozzleMount * nozzle.tcp;
        result.axis = tool.nozzle.nozzleMount.linear() * nozzle.axis;
    }
    return result;
}
ToolheadAssembly assembly_at(const Project &p, const QString &id) {
    if (id.isEmpty()) {
        auto carriage = p.carriageAssembly;
        for (auto &c : carriage.components)
            c.id = "carriage/" + c.id;
        for (auto &rule : carriage.interfaces) {
            rule.first = "carriage/" + rule.first;
            if (!rule.second.startsWith("fixture:"))
                rule.second = "carriage/" + rule.second;
        }
        return carriage;
    }
    auto tool = find_tool(p, id);
    require(tool, "Unknown tool " + id);
    auto assembly = installed_assembly(*tool);
    for (const auto &shared : p.carriageAssembly.components) {
        auto c = shared;
        c.id = "carriage/" + c.id;
        auto transform = assembly.mount.inverse() * p.carriageAssembly.mount;
        for (auto &v : c.mesh.vertices)
            v = transform * v;
        for (auto &v : c.renderMesh.vertices)
            v = transform * v;
        assembly.components.push_back(std::move(c));
    }
    auto transform = assembly.mount.inverse() * p.carriageAssembly.mount;
    for (auto rule : p.carriageAssembly.interfaces) {
        rule.first = "carriage/" + rule.first;
        if (!rule.second.startsWith("fixture:"))
            rule.second = "carriage/" + rule.second;
        rule.regionFrame = transform * rule.regionFrame;
        assembly.interfaces.push_back(rule);
    }
    return assembly;
}
JointState sample_axes(const Project &p, const Sample &sample) {
    return sample.carriage.value_or(controller_axes(p.machine, assembly_at(p, sample.toolId), sample.joint));
}
JointState sample_motion(const Project &p, const Sample &a, const Sample &b, double u) {
    auto axes = interpolate(sample_axes(p, a), sample_axes(p, b), u);
    return pose_from_axes(p.machine, assembly_at(p, b.toolId), axes);
}
void migrate_single_project(Project &p) {
    if (!p.objects.empty())
        return;
    PlateObject o;
    o.id = "object-1";
    o.name = p.part.name;
    o.mesh = p.part;
    o.volume = p.volume;
    o.process = p.process;
    o.stress = p.stress;
    o.loads = p.loads;
    o.guide = p.profileGuide;
    o.materialId = "material-1";
    p.objects.push_back(o);
    MaterialProfile m;
    m.id = "material-1";
    m.name = p.filamentName;
    m.diameter = p.machine.filamentDiameter;
    m.maxFlow = p.machine.maxFlow;
    m.density = p.filamentDensity;
    m.cost = p.filamentCost;
    m.mechanical = p.loads.material;
    p.materials.push_back(m);
    ToolDefinition t;
    t.id = "tool-1";
    t.name = "Primary tool";
    t.assembly = p.toolhead;
    t.dummy = p.dummyToolhead;
    t.useDummy = p.useDummyToolhead;
    if (t.geometry().components.empty()) {
        t.dummy = make_dummy_toolhead(p.process.nozzleDiameter);
        t.useDummy = true;
    }
    t.nozzle.diameter = p.process.nozzleDiameter;
    t.nozzle.name = QString::number(t.nozzle.diameter) + " mm";
    t.nozzle.minWidth = std::min(p.process.width, t.nozzle.diameter * .8);
    t.nozzle.maxWidth = std::max(p.process.width, t.nozzle.diameter * 2);
    t.nozzle.maxHeight = std::max(p.process.layerHeight, t.nozzle.diameter * .8);
    t.nozzle.maxFlow = p.machine.maxFlow;
    t.nozzle.calibrated = true;
    t.nozzle.geometryHash = t.geometry().sourceHash;
    t.routes.push_back({m.id, "0", {}, true});
    p.tools.push_back(t);
    p.plate.order = {o.id};
    p.plate.bedTemperature = 60;
    p.pathProcessFingerprint.clear();
    for (auto &s : p.path) {
        s.objectId = o.id;
        s.toolId = t.id;
        s.materialId = m.id;
        s.carriage = controller_axes(p.machine, t.geometry(), s.joint);
    }
    rebuild_job_events(p);
}
void enable_plate(Project &p) {
    migrate_single_project(p);
    p.plateEnabled = true;
    p.invalidate();
    synchronize_plate_preview(p);
}
void synchronize_plate_preview(Project &p) {
    if (!p.plateEnabled)
        return;
    p.part = Mesh{};
    p.part.name = "Plate";
    p.volume = {};
    p.stress = {};
    for (const auto &o : p.objects) {
        size_t offset = p.part.vertices.size();
        for (const auto &v : o.mesh.vertices)
            p.part.vertices.push_back(o.placement * v);
        for (auto t : o.mesh.triangles) {
            t.array() += int(offset);
            p.part.triangles.push_back(t);
        }
        p.part.error = std::max(p.part.error, o.mesh.error);
    }
    for (const auto &o : p.objects) {
        const auto &field = o.stress;
        int node = p.stress.mesh.vertices.size();
        for (auto v : field.mesh.vertices)
            p.stress.mesh.vertices.push_back(o.placement * v);
        for (auto cell : field.mesh.cells) {
            for (auto &i : cell.nodes)
                i += node;
            p.stress.mesh.cells.push_back(cell);
        }
        for (auto tensor : field.tensors)
            p.stress.tensors.push_back(o.placement.linear() * tensor * o.placement.linear().transpose());
        p.stress.scalar.insert(p.stress.scalar.end(), field.scalar.begin(), field.scalar.end());
        p.stress.directional |= field.directional;
    }
    if (p.stress.mesh.cells.size() != p.stress.tensors.size() &&
        p.stress.mesh.vertices.size() != p.stress.tensors.size())
        p.stress.directional = false;
    if (!p.tools.empty()) {
        p.toolhead = installed_assembly(p.tools.front());
        p.useDummyToolhead = false;
    }
}
void validate_plate_setup(const Project &p) {
    auto finite = [](std::initializer_list<double> values) {
        return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
    };
    auto rigid = [](const Transform &frame) {
        return frame.matrix().allFinite() && std::abs(frame.linear().determinant() - 1) < 1e-6 &&
               (frame.linear().transpose() * frame.linear() - Mat3::Identity()).norm() < 1e-6;
    };
    require(
        finite({p.machine.clearance, p.machine.chordTolerance, p.machine.linearSpeed, p.machine.angularSpeed,
                p.machine.linearAccel, p.machine.angularAccel, p.plate.bedTemperature}) &&
            p.machine.clearance >= 0 && p.machine.chordTolerance > 0 && p.machine.linearSpeed > 0 &&
            p.machine.angularSpeed > 0 && p.machine.linearAccel > 0 && p.machine.angularAccel > 0,
        "Invalid machine accuracy or motion budget");
    require(!p.objects.empty() && !p.tools.empty() && !p.materials.empty(),
            "Configure at least one object, material and physical tool");
    require(p.plate.searchBudget > 0 && p.plate.searchBudget <= 4096, "Order search budget must be 1..4096");
    require(p.compute.threads >= 0 && p.compute.memoryFraction > 0 && p.compute.memoryFraction <= .9,
            "Invalid compute budget");
    QSet<QString> ids;
    auto unique = [&](const QString &id) {
        require(!id.isEmpty() && !ids.contains(id), "IDs must be nonempty and unique: " + id);
        ids.insert(id);
    };
    for (const auto &m : p.materials) {
        unique(m.id);
        require(finite({m.diameter, m.maxFlow, m.density, m.cost, m.fan, m.temperature, m.standby,
                        m.minTemperature, m.maxTemperature, m.bedMin, m.bedMax}),
                "Non-finite material profile");
        require(std::isfinite(m.diameter) && m.diameter > 0 && m.maxFlow > 0 && m.density > 0 &&
                    m.cost >= 0 && m.fan >= 0 && m.fan <= 1,
                "Invalid material " + m.id);
        require(m.temperature >= m.minTemperature && m.temperature <= m.maxTemperature &&
                    p.plate.bedTemperature >= m.bedMin && p.plate.bedTemperature <= m.bedMax,
                "Incompatible temperatures for " + m.id);
    }
    std::set<int> numbers;
    for (const auto &t : p.tools) {
        unique(t.id);
        require(numbers.insert(t.controllerTool).second && t.controllerTool >= 0,
                "Controller tool numbers must be unique");
        const auto &n = t.nozzle;
        require(finite({n.diameter, n.minHeight, n.maxHeight, n.minWidth, n.maxWidth, n.maxFlow}) &&
                    rigid(t.geometry().mount) && rigid(t.dock) && rigid(n.nozzleMount),
                "Invalid nozzle or tool calibration");
        require(n.calibrated && n.diameter > 0 && n.minHeight > 0 && n.maxHeight >= n.minHeight &&
                    n.minWidth > 0 && n.maxWidth >= n.minWidth && n.maxFlow > 0,
                "Calibrate installed nozzle for " + t.id);
        require(t.useDummy || n.geometryHash == t.assembly.sourceHash || n.dummyNozzle ||
                    !n.separateNozzle.components.empty(),
                "Nozzle CAD changed: confirm calibration for " + t.id);
        auto assembly = installed_assembly(t);
        require(assembly.axis.allFinite() && assembly.axis.norm() > 1e-8 &&
                    (assembly.mount.linear() * assembly.axis.normalized() - Vec3::UnitZ()).norm() < 1e-6 &&
                    finite({assembly.contactRadius, assembly.contactHeight}) && assembly.contactRadius >= 0 &&
                    assembly.contactHeight >= 0,
                "Align and calibrate the installed nozzle/contact frame for " + t.id);
        QSet<QString> componentIds;
        for (const auto &component : assembly.components) {
            require(!component.id.isEmpty() && !componentIds.contains(component.id) &&
                        !component.id.startsWith("carriage/"),
                    "Tool component IDs must be unique and cannot use the shared carriage namespace");
            componentIds.insert(component.id);
        }
        int nozzles = 0;
        for (const auto &c : assembly.components)
            if (!c.visualOnly && c.nozzle)
                ++nozzles;
        if (p.machine.layout == Layout::TiltingHead)
            for (const auto &component : assembly.components)
                if (component.nozzle && !component.visualOnly)
                    require(component.link == Link::Nozzle,
                            "A tilting-head nozzle must attach to the final nozzle link");
        require(nozzles == 1, "Each physical tool requires one installed nozzle: " + t.id);
        if (t.detachable)
            require(t.change.calibrated && !t.change.release.empty() && !t.change.pickup.empty(),
                    "Configure calibrated pickup and release recipes for " + t.id);
        if (t.detachable) {
            for (const auto *steps : {&t.change.release, &t.change.pickup})
                for (const auto &step : *steps)
                    require(step.carriage.xyz.allFinite() &&
                                finite({step.carriage.a, step.carriage.b, step.seconds}) &&
                                step.seconds >= 0 && step.contactCenter.allFinite() &&
                                step.contactSize.allFinite(),
                            "Invalid calibrated recipe waypoint or contact region");
            auto guarded = [&](const std::vector<RecipeStep> &steps) {
                const char *pattern =
                    p.machine.controller == Controller::Duet ? "^M577\\s+P[0-9]+\\s+S[01]$"
                    : p.machine.controller == Controller::Klipper
                        ? "^FAL_TOOL_CONFIRM\\s+SENSOR=[A-Za-z_][A-Za-z0-9_-]*\\s+PRESENT=[01]$"
                        : "^M226\\s+P[0-9]+\\s+S[01]$";
                return std::any_of(steps.begin(), steps.end(), [&](const RecipeStep &step) {
                    return step.kind == EventKind::Confirm &&
                           QRegularExpression(pattern).match(step.argument).hasMatch();
                });
            };
            require(guarded(t.change.pickup) && guarded(t.change.release),
                    "Pickup/release must include a controller sensor guard for " + t.id);
        }
        require(!t.routes.empty(), "Configure filament routes for " + t.id);
        QSet<QString> routes;
        for (const auto &r : t.routes) {
            require(find_material(p, r.materialId) && r.confirmed && !routes.contains(r.materialId),
                    "Invalid/unconfirmed filament route for " + t.id);
            routes.insert(r.materialId);
            require(t.routes.size() == 1 || !r.selectCommand.isEmpty(),
                    "Multiple filaments on one nozzle require explicit feeder-selector commands");
            require(!r.selectCommand.contains('\n') && !r.selectCommand.contains('\r'),
                    "A feeder action must be one non-motion command");
        }
    }
    for (const auto &o : p.objects) {
        unique(o.id);
        require(rigid(o.placement),
                "Object placement must be a rigid frame; scale the source geometry before slicing");
        require(!o.mesh.empty() && find_material(p, o.materialId),
                "Object requires geometry and a material: " + o.id);
        validate_process(o.process);
        require(o.toolId.isEmpty() || find_tool(p, o.toolId), "Unknown object tool");
        for (const auto &a : o.assignments) {
            double minimumHeight = 1e100, minimumWidth = 1e100;
            for (const auto &tool : p.tools) {
                if (!a.toolId.isEmpty() && tool.id != a.toolId)
                    continue;
                for (const auto &route : tool.routes)
                    if (route.materialId == a.materialId) {
                        minimumHeight = std::min(minimumHeight, tool.nozzle.minHeight);
                        minimumWidth = std::min(minimumWidth, tool.nozzle.minWidth);
                    }
            }
            if (a.kind == AssignmentKind::Volume && !a.volume.empty() &&
                a.volume.bounds().intersects(o.mesh.bounds())) {
                Vec3 center = Vec3::Zero();
                for (auto point : a.volume.vertices)
                    center += point / double(a.volume.vertices.size());
                Mat3 covariance = Mat3::Zero();
                for (auto point : a.volume.vertices) {
                    Vec3 delta = point - center;
                    covariance += delta * delta.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Mat3> eigen(covariance);
                Vec3 low = Vec3::Constant(1e100), high = -low;
                for (auto point : a.volume.vertices) {
                    Vec3 local = eigen.eigenvectors().transpose() * (point - center);
                    low = low.cwiseMin(local);
                    high = high.cwiseMax(local);
                }
                require((high - low).minCoeff() >= minimumHeight - 1e-8,
                        "Material volume feature is smaller than every compatible installed nozzle's minimum "
                        "layer thickness");
            }
            if (a.kind == AssignmentKind::Paint && !a.faces.empty()) {
                double area = 0;
                for (int face : a.faces)
                    if (face >= 0 && face < int(o.mesh.triangles.size())) {
                        auto t = o.mesh.triangles[face];
                        area += (o.mesh.vertices[t[1]] - o.mesh.vertices[t[0]])
                                    .cross(o.mesh.vertices[t[2]] - o.mesh.vertices[t[0]])
                                    .norm() /
                                2;
                    }
                require(area >= minimumWidth * minimumWidth * .25,
                        "Painted material feature is smaller than the compatible nozzle footprint");
            }

            require(find_material(p, a.materialId), "Unknown assigned material " + a.materialId);
            require(a.toolId.isEmpty() || find_tool(p, a.toolId), "Unknown assigned tool");
            for (int f : a.faces)
                require(f >= 0 && f < int(o.mesh.triangles.size()), "Invalid paint face");
        }
    }
    for (const auto &id : {p.plate.supportMaterial, p.plate.interfaceMaterial})
        require(id.isEmpty() || find_material(p, id), "Unknown support material");
    for (const auto &component : p.carriageAssembly.components)
        require(!component.nozzle, "The shared empty carriage cannot carry a nozzle contact allowance");
    require(p.plate.towerWidth > 0 && p.plate.purgePosition.allFinite(), "Invalid purge geometry");
    for (const auto &v : p.plate.purge)
        require(find_material(p, v.from) && find_material(p, v.to) && std::isfinite(v.volume) &&
                    v.volume >= 0,
                "Invalid purge transition");
    if (p.tools.size() > 1) {
        int permanent = 0;
        for (const auto &t : p.tools)
            permanent += !t.detachable;
        require(permanent == 0,
                "Multiple physical tools require detachable configurations; IDEX is not implemented");
    }
}
static bool skin_role(Role r) {
    return r == Role::Wall || r == Role::OuterWall || r == Role::TopSurface || r == Role::BottomSurface ||
           r == Role::SolidInfill;
}
static double triangle_distance(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    Vec3 ab = b - a, ac = c - a, n = ab.cross(ac);
    if (n.squaredNorm() < 1e-20)
        return (p - a).norm();
    n.normalize();
    Vec3 q = p - n * (p - a).dot(n);
    Eigen::Matrix2d gram;
    gram << ab.dot(ab), ab.dot(ac), ab.dot(ac), ac.dot(ac);
    auto uv = gram.ldlt().solve(Eigen::Vector2d((q - a).dot(ab), (q - a).dot(ac))).eval();
    if (uv.minCoeff() >= 0 && uv.sum() <= 1)
        return std::abs((p - a).dot(n));
    double best = 1e100;
    for (auto pair : {std::pair<Vec3, Vec3>{a, b}, {b, c}, {c, a}}) {
        auto d = pair.second - pair.first;
        double t = std::clamp((p - pair.first).dot(d) / std::max(1e-20, d.squaredNorm()), 0., 1.);
        best = std::min(best, (p - pair.first - d * t).norm());
    }
    return best;
}
static std::pair<QString, QString> assignment_at(const Project &p, const PlateObject &o, const Vec3 &world,
                                                 Role role) {
    Vec3 point = o.placement.inverse() * world;
    QString material = o.materialId, tool = o.toolId;
    int rank = -100000;
    if (role == Role::Support && !p.plate.supportMaterial.isEmpty()) {
        material = p.plate.supportMaterial;
        tool.clear();
    }
    if (role == Role::SupportInterface && !p.plate.interfaceMaterial.isEmpty()) {
        material = p.plate.interfaceMaterial;
        tool.clear();
    }
    int nearest = -1;
    if (skin_role(role) &&
        std::any_of(o.assignments.begin(), o.assignments.end(),
                    [](const MaterialAssignment &a) { return a.kind == AssignmentKind::Paint; })) {
        double best = 1e100;
        for (size_t i = 0; i < o.mesh.triangles.size(); ++i) {
            auto t = o.mesh.triangles[i];
            double distance =
                triangle_distance(point, o.mesh.vertices[t[0]], o.mesh.vertices[t[1]], o.mesh.vertices[t[2]]);
            if (distance < best) {
                best = distance;
                nearest = int(i);
            }
        }
    }
    for (const auto &a : o.assignments) {
        bool match = false;
        int priority = a.priority;
        if (a.kind == AssignmentKind::Object) {
            match = true;
            priority += 0;
        }
        if (a.kind == AssignmentKind::Role) {
            match = a.role == role;
            priority += 10000;
        }
        if (a.kind == AssignmentKind::Paint) {
            match = nearest >= 0 && std::find(a.faces.begin(), a.faces.end(), nearest) != a.faces.end();
            priority += 20000;
        }
        if (a.kind == AssignmentKind::Volume) {
            match = !a.volume.empty() && contains(a.volume, point);
            priority += 30000;
        }
        if (match && priority >= rank) {
            material = a.materialId;
            if (!a.toolId.isEmpty())
                tool = a.toolId;
            rank = priority;
        }
    }
    return {material, tool};
}
QString plate_generation_signature(const Project &p) {
    auto json = project_json(p);
    for (const char *key : {"path", "pathProcessFingerprint", "report", "diagnostics", "depositionValid",
                            "comparison", "deposition"})
        json.remove(key);
    auto job = json["plateJob"].toObject();
    job.remove("compute");
    job.remove("computation");
    json["plateJob"] = job;
    return QString::fromLatin1(QCryptographicHash::hash(QJsonDocument(json).toJson(QJsonDocument::Compact),
                                                        QCryptographicHash::Sha256)
                                   .toHex());
}
QString structural_signature(const Project &p, size_t index) {
    if (index >= p.objects.size())
        return {};
    auto json = plate_json(p);
    auto object = json["objects"].toArray()[int(index)].toObject();
    auto data = object["data"].toObject();
    data.remove("stress");
    data.remove("report");
    data.remove("path");
    data.remove("diagnostics");
    data.remove("plateJob");
    object["data"] = data;
    object.remove("structuralFingerprint");
    QJsonArray materials;
    for (const auto &m : p.materials) {
        QJsonObject value{{"id", m.id}};
        if (m.mechanical)
            value["mechanical"] = QJsonObject{{"young", m.mechanical->young},
                                              {"poisson", m.mechanical->poisson},
                                              {"transverseYoung", m.mechanical->transverseYoung}};
        materials.append(value);
    }
    return QString::fromLatin1(
        QCryptographicHash::hash(QJsonDocument(QJsonObject{{"object", object}, {"materials", materials}})
                                     .toJson(QJsonDocument::Compact),
                                 QCryptographicHash::Sha256)
            .toHex());
}
FEAResult solve_plate_fea(Project &p, size_t index, std::atomic_bool *cancel) {
    if (index >= p.objects.size())
        throw std::runtime_error("Unknown plate object");
    auto &o = p.objects[index];
    VolumeMesh volume = o.volume;
    if (volume.cells.empty()) {
        volume = tetrahedralize(o.mesh, std::max(.05, o.mesh.bounds().sizes().prod() / 1500));
        o.volume = volume;
    }
    for (auto &v : volume.vertices)
        v = o.placement * v;
    std::vector<Material> properties(volume.cells.size());
    parallel_for(
        volume.cells.size(), p.compute,
        [&](size_t i) {
            const auto &cell = volume.cells[i];
            Vec3 center = Vec3::Zero();
            for (int node : cell.nodes)
                center += volume.vertices[node];
            center /= cell.nodes.size();
            double closest = 1e100;
            Vec3 local = o.placement.inverse() * center;
            for (auto t : o.mesh.triangles)
                closest = std::min(closest, triangle_distance(local, o.mesh.vertices[t[0]],
                                                              o.mesh.vertices[t[1]], o.mesh.vertices[t[2]]));
            double skin = std::max(o.process.wallCount * o.process.width,
                                   std::max(o.process.topShellLayers, o.process.bottomShellLayers) *
                                       o.process.layerHeight);
            auto assigned = assignment_at(p, o, center, closest <= skin ? Role::SolidInfill : Role::Infill);
            auto material = find_material(p, assigned.first);
            require(material && material->mechanical.has_value(),
                    "Mechanical properties missing for " + assigned.first);
            properties[i] = *material->mechanical;
        },
        cancel);
    auto loads = o.loads;
    for (auto &force : loads.nodalForces)
        force.second = o.placement.linear() * force.second;
    auto result = solve_fea(volume, loads, {}, properties, p.compute, cancel);
    o.stress = result.stress;
    for (auto &v : o.stress.mesh.vertices)
        v = o.placement.inverse() * v;
    for (auto &tensor : o.stress.tensors)
        tensor = o.placement.linear().transpose() * tensor * o.placement.linear();
    o.stress.source += " · element materials, perfect bond";
    o.structuralFingerprint = structural_signature(p, index);
    p.invalidate();
    p.diagnostics << "Mixed-material FEA uses a perfect-bond dense-material assumption.";
    return result;
}
static const ToolDefinition &select_tool(const Project &p, const Sample &s, const QString &material,
                                         const QString &overrideId, const QString &previous) {
    require(p.plate.automaticTools || !overrideId.isEmpty() || p.tools.size() == 1,
            "Assign an explicit tool or enable automatic tool selection");
    const ToolDefinition *best = nullptr;
    double score = 1e100;
    for (const auto &tool : p.tools) {
        if (!overrideId.isEmpty() && tool.id != overrideId)
            continue;
        bool routed = false;
        for (const auto &r : tool.routes)
            routed |= r.materialId == material && r.confirmed;
        if (!routed)
            continue;
        const auto &n = tool.nozzle;
        if (s.height < n.minHeight - 1e-8 || s.height > n.maxHeight + 1e-8 || s.width < n.minWidth - 1e-8 ||
            s.width > n.maxWidth + 1e-8)
            continue;
        double quality = skin_role(s.role) ? n.diameter : -n.diameter;
        double cost = quality + (tool.id == previous ? 0 : .02 * tool.change.changeSeconds);
        if (cost < score) {
            best = &tool;
            score = cost;
        }
    }
    require(best, QString("No installed nozzle/filament route can print material %1 with width %2 and height "
                          "%3. Correct assignments or nozzle profiles.")
                      .arg(material)
                      .arg(s.width)
                      .arg(s.height));
    return *best;
}
static void intersect_cuts(const Mesh &mesh, const Vec3 &a, const Vec3 &b, std::vector<double> &cuts) {
    Vec3 d = b - a;
    for (auto triangle : mesh.triangles) {
        Vec3 x = mesh.vertices[triangle[0]], u = mesh.vertices[triangle[1]] - x,
             v = mesh.vertices[triangle[2]] - x, h = d.cross(v);
        double det = u.dot(h);
        if (std::abs(det) < 1e-12)
            continue;
        Vec3 t = a - x;
        double f = t.dot(h) / det;
        if (f < 0 || f > 1)
            continue;
        Vec3 q = t.cross(u);
        double g = d.dot(q) / det;
        if (g < 0 || f + g > 1)
            continue;
        double k = v.dot(q) / det;
        if (k > 1e-9 && k < 1 - 1e-9)
            cuts.push_back(k);
    }
}
static std::vector<Sample> label_paths(const Project &p, const PlateObject &o, Project &local) {
    std::vector<Sample> output;
    QString lastTool;
    for (size_t i = 0; i < local.path.size(); ++i) {
        auto b = local.path[i];
        b.objectId = o.id;
        if (b.role == Role::Support && b.deltaE > 0) {
            for (int layer = 1; layer <= p.plate.supportInterfaceLayers; ++layer)
                if (contains(local.part, b.position + surface_normal(b) * (b.height * layer))) {
                    b.role = Role::SupportInterface;
                    break;
                }
        }
        if (i == 0 || b.deltaE <= 0) {
            if (b.deltaE < 0 && !output.empty()) {
                b.toolId = output.back().toolId;
                b.materialId = output.back().materialId;
                output.push_back(b);
                continue;
            }
            Sample printing = b;
            Vec3 point = b.position;
            for (size_t next = i + 1; next < local.path.size(); ++next)
                if (local.path[next].deltaE > 0) {
                    printing = local.path[next];
                    point = (printing.position + local.path[next - 1].position) / 2;
                    break;
                }
            auto selected = assignment_at(p, o, point, printing.role);
            b.materialId = selected.first;
            const auto &tool = select_tool(p, printing, selected.first, selected.second, lastTool);
            b.toolId = tool.id;
            lastTool = tool.id;
            output.push_back(b);
            continue;
        }
        const auto &a = local.path[i - 1];
        std::vector<double> cuts{0, 1};
        for (const auto &assignment : o.assignments)
            if (assignment.kind == AssignmentKind::Volume)
                intersect_cuts(assignment.volume, o.placement.inverse() * a.position,
                               o.placement.inverse() * b.position, cuts);
        // Bound painting/material classification to the process resolution; volume intersections
        // are exact segment/triangle cuts. This never discards a short material segment.
        int count = std::any_of(o.assignments.begin(), o.assignments.end(),
                                [](const MaterialAssignment &a) { return a.kind == AssignmentKind::Paint; })
                        ? std::max(1, int(std::ceil((b.position - a.position).norm() /
                                                    std::max(.02, o.process.resolution))))
                        : 1;
        require(count <= 100000, "Material boundary subdivision budget exhausted");
        auto last = assignment_at(p, o, a.position, b.role);
        for (int k = 1; k < count; ++k) {
            double u = double(k) / count;
            auto current = assignment_at(p, o, a.position + (b.position - a.position) * u, b.role);
            if (current != last)
                cuts.push_back(u);
            last = current;
        }
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(
            std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }),
            cuts.end());
        struct Span {
            double u, v;
            std::pair<QString, QString> identity;
        };
        std::vector<Span> spans;
        for (size_t k = 1; k < cuts.size(); ++k) {
            double u = cuts[k - 1], v = cuts[k];
            auto identity =
                assignment_at(p, o, a.position + (b.position - a.position) * ((u + v) / 2), b.role);
            if (!spans.empty() && spans.back().identity == identity)
                spans.back().v = v;
            else
                spans.push_back({u, v, identity});
        }
        for (const auto &span : spans) {
            double u = span.u, v = span.v;
            const auto &identity = span.identity;
            auto sample = b;
            sample.position = a.position + (b.position - a.position) * v;
            sample.materialId = identity.first;
            const auto &tool = select_tool(p, sample, identity.first, identity.second, lastTool);
            sample.toolId = tool.id;
            auto material = find_material(p, identity.first);
            sample.speed = std::min(sample.speed, std::min(tool.nozzle.maxFlow, material->maxFlow) /
                                                      bead_area(sample.width, sample.height));
            sample.deltaE = (b.position - a.position).norm() * (v - u) *
                            bead_area(sample.width, sample.height) /
                            (pi * std::pow(material->diameter / 2, 2));
            output.push_back(sample);
            lastTool = tool.id;
        }
    }
    return output;
}
struct Planner {
    Project &p;
    QString active, material;
    double towerHeight = 0;
    std::atomic_bool *cancel;
    std::map<QString, QString> loaded;
    std::map<QString, double> retracted;
    double stationWaste = 0;
    void add(Sample s) {
        auto assembly = assembly_at(p, s.toolId);
        if (s.carriage)
            s.joint = pose_from_axes(p.machine, assembly, *s.carriage);
        else {
            auto branches = inverse_kinematics(p.machine, s.position, s.direction,
                                               p.path.empty() ? JointState{} : p.path.back().joint, assembly);
            require(!branches.empty(), "No feasible pose for tool " + s.toolId + " at " + s.objectId);
            s.joint = branches.front();
            s.carriage = controller_axes(p.machine, assembly, s.joint);
        }
        s.position = part_position(p.machine, s.joint);
        s.direction = part_direction(p.machine, assembly, s.joint);
        p.path.push_back(std::move(s));
    }
    Sample current() {
        require(!p.path.empty(), "Missing initial carriage pose");
        auto s = p.path.back();
        s.deltaE = 0;
        s.contactComponent.clear();
        s.contactFixture.clear();
        s.contactSize.setZero();
        s.event = EventKind::Motion;
        s.argument.clear();
        s.dwell = 0;
        s.role = Role::Travel;
        return s;
    }
    void state(EventKind kind, const QString &value, double seconds = 0) {
        auto s = current();
        s.event = kind;
        s.argument = value;
        s.dwell = seconds;
        s.toolId = active;
        s.materialId = material;
        s.carriage = sample_axes(p, p.path.back());
        s.contactComponent = p.path.back().contactComponent;
        s.contactFixture = p.path.back().contactFixture;
        s.contactCenter = p.path.back().contactCenter;
        s.contactSize = p.path.back().contactSize;
        add(s);
    }
    void move_axes(const JointState &axes, const RecipeStep *step = nullptr) {
        auto s = current();
        s.contactComponent.clear();
        s.contactFixture.clear();
        s.contactSize.setZero();
        if (step) {
            s.contactComponent = step->contactComponent;
            s.contactFixture = step->contactFixture;
            s.contactCenter = step->contactCenter;
            s.contactSize = step->contactSize;
        }
        s.carriage = axes;
        s.toolId = active;
        s.materialId = material;
        s.speed = p.process.travelSpeed;
        add(s);
    }
    void recipe(const std::vector<RecipeStep> &steps) {
        for (const auto &step : steps) {
            if (cancel && cancel->load())
                throw std::runtime_error("Job cancelled");
            if (step.kind == EventKind::Motion)
                move_axes(step.carriage, &step);
            else {
                require(step.kind == EventKind::IO || step.kind == EventKind::Confirm ||
                            step.kind == EventKind::Synchronize,
                        "Recipe contains an unsupported hidden state change");
                state(step.kind, step.argument, step.seconds);
            }
        }
    }
    void travel(const Sample &destination) {
        auto axes = sample_axes(p, p.path.back());
        auto goal = destination.carriage.value_or(
            controller_axes(p.machine, assembly_at(p, active), destination.joint));
        auto lifted = axes;
        lifted.xyz.z() = std::max(axes.xyz.z(), goal.xyz.z()) + p.process.travelLift;
        move_axes(lifted);
        auto overhead = goal;
        overhead.xyz.z() = lifted.xyz.z();
        move_axes(overhead);
        move_axes(goal);
        p.path.back().objectId = destination.objectId;
    }
    void purge(const ToolDefinition &tool, const MaterialProfile &incoming, const QString &old) {
        double volume = 0;
        bool defined = old.isEmpty() || old == incoming.id;
        for (const auto &v : p.plate.purge)
            if (v.from == old && v.to == incoming.id) {
                volume = v.volume;
                defined = true;
                break;
            }
        require(defined, "Configure purge volume for " + old + " → " + incoming.id);
        if (volume <= 0)
            return;
        auto origin = current();
        double width = std::clamp(default_line_width(tool.nozzle.diameter), tool.nozzle.minWidth,
                                  tool.nozzle.maxWidth),
               height = std::clamp(.2, tool.nozzle.minHeight, tool.nozzle.maxHeight);
        double area = bead_area(width, height), filament = pi * std::pow(incoming.diameter / 2, 2);
        if (!p.plate.purgeTower) {
            require(p.plate.purgeCapacity > 0 && stationWaste + volume <= p.plate.purgeCapacity &&
                        std::any_of(p.fixtures.begin(), p.fixtures.end(),
                                    [](const Fixture &f) {
                                        return f.name == "purge-waste-envelope" && !f.movesWithBed;
                                    }),
                    "Purge station requires a fixed, fully filled waste envelope and sufficient configured "
                    "capacity");
            stationWaste += volume;
            Sample target = origin;
            target.position = p.plate.purgePosition;
            target.direction = Vec3::UnitZ();
            target.carriage.reset();
            auto q = inverse_kinematics(p.machine, target.position, target.direction, origin.joint,
                                        assembly_at(p, active));
            require(!q.empty(), "Purge station unreachable");
            target.joint = q.front();
            target.carriage = controller_axes(p.machine, assembly_at(p, active), target.joint);
            travel(target);
            auto s = current();
            s.event = EventKind::Purge;
            s.role = Role::Purge;
            s.deltaE = volume / filament;
            s.dwell = volume / std::min(tool.nozzle.maxFlow, incoming.maxFlow);
            s.argument = "station";
            add(s);
            travel(origin);
            return;
        }
        double side = p.plate.towerWidth;
        require(side > width * 2, "Purge tower too narrow for installed nozzle");
        double remaining = volume;
        for (int x = 0; x <= 8; ++x)
            for (int y = 0; y <= 8; ++y) {
                Vec3 point = p.plate.purgePosition + Vec3(side * x / 8, side * y / 8, -.01);
                bool supported = false;
                for (const auto &f : p.fixtures)
                    if (f.movesWithBed && f.name.contains("bed", Qt::CaseInsensitive))
                        supported |= contains(f.mesh, point);
                require(supported, "Purge tower footprint must be supported by the modeled bed");
            }
        while (remaining > 1e-8) {
            towerHeight += height;
            Sample s = origin;
            s.toolId = active;
            s.materialId = incoming.id;
            s.objectId = "purge-tower";
            s.direction = Vec3::UnitZ();
            s.width = width;
            s.height = height;
            s.speed = std::min(p.process.speed, incoming.maxFlow / area);
            s.position = p.plate.purgePosition + Vec3(0, 0, towerHeight);
            s.surfaceNormal = Vec3::UnitZ();
            s.carriage.reset();
            auto poses = inverse_kinematics(p.machine, s.position, s.direction, p.path.back().joint,
                                            assembly_at(p, active));
            require(!poses.empty(), "Purge tower unreachable");
            s.joint = poses.front();
            s.carriage = controller_axes(p.machine, assembly_at(p, active), s.joint);
            travel(s);
            // Complete each tower layer even when the transition volume is smaller. Account
            // for the actual extra waste; partial unsupported tower layers are not produced.
            int lines = std::max(2, int(std::ceil(side / bead_spacing(width, height))));
            for (int line = 0; line < lines; ++line) {
                Sample start = s;
                start.position =
                    p.plate.purgePosition + Vec3(line % 2 ? side : 0, side * line / (lines - 1), towerHeight);
                start.carriage.reset();
                start.deltaE = 0;
                start.event = EventKind::Motion;
                start.role = Role::Travel;
                add(start);
                auto end = start;
                end.position.x() += line % 2 ? -side : side;
                end.carriage.reset();
                end.role = Role::Purge;
                end.deltaE = side * area / filament;
                end.event = EventKind::Motion;
                add(end);
                remaining -= side * area;
            }
        }
        travel(origin);
    }
    void switch_to(const QString &toolId, const QString &materialId) {
        if (active == toolId && material == materialId)
            return;
        auto incoming = find_tool(p, toolId);
        require(incoming, "Unknown incoming tool");
        bool change = active != toolId;
        QString previousLoaded = loaded[toolId];
        if (!active.isEmpty()) {
            auto outgoing = find_tool(p, active);
            auto retract = current();
            retract.deltaE = -outgoing->change.retract;
            retract.role = Role::Retract;
            add(retract);
            retracted[active] += outgoing->change.retract;
            if (change) {
                state(EventKind::Standby, active);
                require(outgoing->detachable, "Cannot detach a permanent tool");
                recipe(outgoing->change.release);
                auto old = active;
                active.clear();
                state(EventKind::Detach, old);
                state(EventKind::Synchronize, "empty carriage");
            }
        }
        if (change) {
            require(incoming->detachable, "A tool change requires a detachable tool");
            recipe(incoming->change.pickup);
            active = toolId;
            state(EventKind::Attach, toolId);
            state(EventKind::Confirm, toolId,
                  std::max(incoming->change.changeSeconds,
                           printer_number(p.machine.printer, "extruderChangeSeconds", 0) +
                               printer_number(p.machine.printer, "hotendChangeSeconds", 0)));
        }
        if (previousLoaded.isEmpty())
            previousLoaded = incoming->routes.front().materialId;
        bool swap = previousLoaded != materialId;
        if (swap) {
            require(incoming->change.unload > 0 && incoming->change.load > 0,
                    "Shared-nozzle changes require configured unload/load distances");
            if (material != previousLoaded) {
                material = previousLoaded;
                state(EventKind::Material, material);
            }
            state(EventKind::Temperature, QString::number(find_material(p, material)->temperature), 1);
            auto s = current();
            s.deltaE = -incoming->change.unload;
            s.event = EventKind::Unload;
            s.dwell = std::max(1.0, printer_number(p.machine.printer, "unloadSeconds", 0));
            add(s);
        }
        material = materialId;
        const auto &m = *find_material(p, material);
        state(EventKind::Material, material);
        state(EventKind::Temperature, QString::number(m.temperature), 1);
        for (const auto &route : incoming->routes)
            if (route.materialId == material && !route.selectCommand.isEmpty())
                state(EventKind::IO, route.selectCommand);
        if (swap) {
            auto s = current();
            s.deltaE = incoming->change.load;
            s.event = EventKind::Load;
            s.dwell = std::max(1.0, printer_number(p.machine.printer, "loadSeconds", 0));
            s.argument = "feeder";
            add(s);
        }
        if (retracted[active] > 0) {
            auto s = current();
            s.deltaE = retracted[active];
            s.event = EventKind::Load;
            s.dwell = 1;
            s.argument = "recover retraction";
            add(s);
            retracted[active] = 0;
        }
        loaded[active] = material;
        purge(*incoming, m, previousLoaded);
    }
};
void time_plate(Project &p) {
    if (p.path.empty())
        return;
    p.path.front().time = 0;
    for (size_t i = 1; i < p.path.size(); ++i) {
        auto &a = p.path[i - 1];
        auto &b = p.path[i];
        auto x = sample_axes(p, a), y = sample_axes(p, b);
        double distance = (y.xyz - x.xyz).norm(), angle = std::max(std::abs(y.a - x.a), std::abs(y.b - x.b));
        double dt = std::max({.001, b.dwell, distance / p.machine.linearSpeed, angle / p.machine.angularSpeed,
                              2 * std::sqrt(distance / p.machine.linearAccel),
                              2 * std::sqrt(angle / p.machine.angularAccel)});
        if (b.event == EventKind::Motion) {
            double length = (b.position - a.position).norm();
            dt = std::max(dt, length / std::max(.1, b.speed));
        }
        if (auto m = find_material(p, b.materialId))
            if (auto t = find_tool(p, b.toolId))
                dt = std::max(dt, std::abs(b.deltaE) * pi * std::pow(m->diameter / 2, 2) /
                                      std::min(m->maxFlow, t->nozzle.maxFlow));
        auto tool = find_tool(p, b.toolId);
        dt = std::max(dt, printer_move_seconds(p.machine, y.xyz - x.xyz, b.deltaE,
                                               tool ? tool->extruderSettings : QJsonObject{}));
        b.time = a.time + dt;
    }
    rebuild_job_events(p);
}
void rebuild_job_events(Project &p) {
    p.events.clear();
    for (size_t i = 0; i < p.path.size(); ++i) {
        auto &s = p.path[i];
        if (s.event != EventKind::Motion)
            p.events.push_back({s.event, int(i), s.objectId, s.toolId, s.materialId, s.argument, s.time});
    }
}
void slice_plate(Project &p, std::atomic_bool *cancel, std::function<void(double)> progress) {
    validate_plate_setup(p);
    validate_printer_settings(p.machine);
    p.invalidate();
    p.path.clear();
    p.events.clear();
    QElapsedTimer timer;
    timer.start();
    std::vector<Project> local(p.objects.size());
    std::vector<std::vector<Sample>> labeled(p.objects.size());
    std::vector<ComputeReport> compute(p.objects.size());
    parallel_for(
        p.objects.size(), p.compute,
        [&](size_t i) {
            auto &o = p.objects[i];
            Project child;
            child.machine = p.machine;
            // Printer hooks/retraction are scheduled once across the complete plate.
            child.machine.printer.remove("gcode");
            child.machine.printer.remove("extruder");
            child.process = o.inheritProcess ? p.process : o.process;
            child.part = o.mesh;
            child.part.vertices =
                compute_transform(o.mesh.vertices, o.placement, p.compute, compute[i], cancel);
            child.volume = o.volume;
            for (auto &v : child.volume.vertices)
                v = o.placement * v;
            child.partPlacement = o.placement;
            child.stress = o.stress;
            if (!o.structuralFingerprint.isEmpty() && o.structuralFingerprint != structural_signature(p, i)) {
                child.stress.directional = false;
                child.diagnostics << "Structural inputs changed; solve FEA again.";
            }
            for (auto &v : child.stress.mesh.vertices)
                v = o.placement * v;
            for (auto &tensor : child.stress.tensors)
                tensor = o.placement.linear() * tensor * o.placement.linear().transpose();
            child.loads = o.loads;
            child.profileGuide = o.guide;
            for (auto &v : child.profileGuide) {
                v.center = o.placement * v.center;
                v.normal = o.placement.linear() * v.normal;
            }
            const ToolDefinition *base = nullptr;
            double small = 1e100, large = 0;
            for (const auto &tool : p.tools)
                for (const auto &r : tool.routes)
                    if (r.materialId == o.materialId && r.confirmed) {
                        if (tool.nozzle.diameter < small) {
                            base = &tool;
                            small = tool.nozzle.diameter;
                        }
                        large = std::max(large, tool.nozzle.diameter);
                    }
            require(base, "No filament route for object " + o.id);
            if (!o.toolId.isEmpty())
                base = find_tool(p, o.toolId);
            child.toolhead = installed_assembly(*base);
            child.fixtures = p.fixtures;
            child.machine.filamentDiameter = find_material(p, o.materialId)->diameter;
            child.machine.maxFlow = std::min(base->nozzle.maxFlow, find_material(p, o.materialId)->maxFlow);
            child.compute = p.compute;
            child.process.nozzleDiameter = base->nozzle.diameter;
            if (p.plate.automaticTools) {
                double maximum = base->nozzle.maxHeight;
                QSet<QString> requiredMaterials{o.materialId};
                for (const auto &a : o.assignments)
                    requiredMaterials.insert(a.materialId);
                for (const auto &id : requiredMaterials) {
                    double possible = 0;
                    for (const auto &t : p.tools)
                        for (const auto &route : t.routes)
                            if (route.materialId == id)
                                possible = std::max(possible, t.nozzle.maxHeight);
                    maximum = std::min(maximum, possible);
                }
                require(maximum > 0, "No compatible shared layer sequence");
                child.process.layerHeight = std::min(child.process.layerHeight, maximum);
                if (child.process.firstLayerHeight > maximum)
                    child.process.firstLayerHeight = maximum;
            }
            if (p.plate.automaticTools && large > small && child.process.infillWidth == 0)
                child.process.infillWidth = default_line_width(large);
            QSet<QString> assigned{o.materialId};
            for (const auto &a : o.assignments)
                assigned.insert(a.materialId);
            if (assigned.size() > 1) {
                bool complete = true;
                for (const auto &id : assigned)
                    complete &= find_material(p, id)->mechanical.has_value();
                if (!complete) {
                    child.stress.directional = false;
                    child.diagnostics
                        << "Mixed material mechanical properties missing; strength optimization disabled.";
                } else if (!o.stress.source.contains("element materials, perfect bond") ||
                           o.structuralFingerprint != structural_signature(p, i)) {
                    child.stress.directional = false;
                    child.diagnostics << "Mixed-material assignment requires an elementwise FEA solve before "
                                         "directional optimization.";
                }
                child.comparison = {};
            }
            slice_project(child, cancel);
            labeled[i] = label_paths(p, o, child);
            local[i] = std::move(child);
        },
        cancel);
    struct Block {
        size_t object;
        int layer;
        std::vector<Sample> samples;
    };
    std::vector<Block> blocks;
    std::vector<QString> order = p.plate.order;
    if (order.empty())
        for (const auto &o : p.objects)
            order.push_back(o.id);
    require(order.size() == p.objects.size(), "Object order must include each object exactly once");
    QSet<QString> seen;
    for (const auto &id : order) {
        auto it = std::find_if(p.objects.begin(), p.objects.end(), [&](const auto &o) { return o.id == id; });
        require(it != p.objects.end() && !seen.contains(id), "Invalid object order");
        seen.insert(id);
        size_t i = it - p.objects.begin();
        for (size_t j = 0; j < labeled[i].size(); ++j) {
            const auto &sample = labeled[i][j];
            if (blocks.empty() || blocks.back().object != i || blocks.back().layer != sample.layer) {
                blocks.push_back({i, sample.layer, {}});
                if (j > 0 && sample.deltaE > 0) {
                    auto anchor = labeled[i][j - 1];
                    anchor.toolId = sample.toolId;
                    anchor.materialId = sample.materialId;
                    anchor.layer = sample.layer;
                    anchor.deltaE = 0;
                    anchor.role = Role::Travel;
                    blocks.back().samples.push_back(anchor);
                }
            }
            blocks.back().samples.push_back(sample);
        }
    }
    if (p.plate.sequence == PrintSequence::Layers)
        std::stable_sort(blocks.begin(), blocks.end(),
                         [](const Block &a, const Block &b) { return a.layer < b.layer; });
    require(!blocks.empty(), "No printable plate movements");
    Sample initial = blocks.front().samples.front();
    initial.carriage = controller_axes(p.machine, assembly_at(p, initial.toolId), initial.joint);
    initial.deltaE = 0;
    p.path.push_back(initial);
    Planner planner{p, initial.toolId, {}, 0, cancel};
    for (const auto &tool : p.tools)
        planner.loaded[tool.id] = tool.routes.front().materialId;
    planner.switch_to(initial.toolId, initial.materialId);
    p.depositionValid = true;
    p.deposition.checked = true;
    p.deposition.coverage = 1;
    int regionOffset = 0;
    for (const auto &child : local) {
        p.depositionValid &= child.depositionValid;
        p.deposition.coverage = std::min(p.deposition.coverage, child.deposition.coverage);
        p.diagnostics.append(child.diagnostics);
    }
    size_t done = 0;
    for (const auto &block : blocks) {
        auto &o = p.objects[block.object];
        for (size_t k = 0; k < block.samples.size(); ++k) {
            if (cancel && cancel->load())
                throw std::runtime_error("Job cancelled");
            auto sample = block.samples[k];
            bool changed = planner.active != sample.toolId || planner.material != sample.materialId;
            planner.switch_to(sample.toolId, sample.materialId);
            auto assembly = assembly_at(p, sample.toolId);
            auto poses = inverse_kinematics(p.machine, sample.position, sample.direction, p.path.back().joint,
                                            assembly);
            require(!poses.empty(), "Tool cannot reach object " + o.id);
            sample.joint = poses.front();
            sample.carriage = controller_axes(p.machine, assembly, sample.joint);
            sample.region += regionOffset;
            if (changed && sample.deltaE > 0 && k > 0) {
                auto start = block.samples[k - 1];
                start.toolId = sample.toolId;
                start.materialId = sample.materialId;
                auto poses = inverse_kinematics(p.machine, start.position, sample.direction,
                                                p.path.back().joint, assembly);
                require(!poses.empty(), "New tool cannot reach the material boundary");
                start.joint = poses.front();
                start.carriage = controller_axes(p.machine, assembly, start.joint);
                planner.travel(start);
            }
            if (k == 0 || sample.deltaE == 0 || p.path.back().objectId != sample.objectId) {
                auto entry = sample;
                entry.deltaE = 0;
                entry.role = Role::Travel;
                planner.travel(entry);
            }
            if (sample.deltaE != 0)
                planner.add(sample);
        }
        regionOffset += 10000;
        ++done;
        if (progress)
            progress(double(done) / blocks.size());
    }
    for (size_t i = 0; i < local.size(); ++i)
        if (!local[i].depositionValid)
            for (size_t j = 0; j < p.path.size(); ++j)
                if (p.path[j].objectId == p.objects[i].id && p.path[j].deltaE > 0)
                    p.deposition.unsupportedMoves.push_back(int(j));
    for (size_t i = 0; i < local.size(); ++i) {
        p.objects[i].process = local[i].process;
        if (p.objects[i].volume.cells.empty()) {
            p.objects[i].volume = local[i].volume;
            for (auto &v : p.objects[i].volume.vertices)
                v = p.objects[i].placement.inverse() * v;
        }
    }
    synchronize_plate_preview(p);
    apply_printer_path(p);
    time_plate(p);
    p.computation = compute.front();
    p.computation.milliseconds = timer.nsecsElapsed() / 1e6;
    p.pathProcessFingerprint = plate_generation_signature(p);
    p.diagnostics
        << "Plate planned: tool changes, purge, and object transitions require full collision validation.";
}
SimulationState simulate_plate(const Project &p, double time, bool rebuild) {
    SimulationState state;
    if (p.path.empty())
        return state;
    state.time = std::clamp(time, 0., p.path.back().time);
    auto it = std::upper_bound(p.path.begin(), p.path.end(), state.time,
                               [](double t, const Sample &s) { return t < s.time; });
    int next = std::min(int(it - p.path.begin()), int(p.path.size()) - 1), previous = std::max(0, next - 1);
    state.move = next;
    double dt = p.path[next].time - p.path[previous].time;
    state.fraction = dt > 0 ? (state.time - p.path[previous].time) / dt : 0;
    const Sample &active =
        state.fraction < 1 && p.path[next].event != EventKind::Motion ? p.path[previous] : p.path[next];
    state.toolId = active.toolId;
    state.materialId = active.materialId;
    state.objectId = active.objectId;
    auto axes = interpolate(sample_axes(p, p.path[previous]), sample_axes(p, p.path[next]), state.fraction);
    state.joint = pose_from_axes(p.machine, assembly_at(p, state.toolId), axes);
    for (const auto &tool : p.tools)
        if (tool.detachable && tool.id != state.toolId)
            state.parkedTools.push_back(tool.id);
    std::map<QString, int> last;
    for (size_t i = 0; i < p.path.size(); ++i)
        if (p.path[i].deltaE > 0 && p.path[i].role != Role::Purge && p.path[i].role != Role::Retract)
            last[p.path[i].objectId] = int(i);
    for (const auto &[id, index] : last)
        if (index < next || (index == next && state.fraction >= 1))
            state.completedObjects.push_back(id);
    for (int i = 1; i <= next; ++i) {
        const auto &s = p.path[i];
        double fraction = i == next ? state.fraction : 1;
        if (s.deltaE > 0 && s.event != EventKind::Load && s.role != Role::Retract) {
            state.filamentUsed[s.materialId] += s.deltaE * fraction;
            if (s.role == Role::Purge)
                state.purgeUsed[s.materialId] += s.deltaE * fraction;
            if (rebuild && s.event == EventKind::Motion && fraction > 1e-12) {
                Vec3 end = i == next ? part_position(p.machine, state.joint) : s.position;
                state.deposited.emplace_back(p.path[i - 1].position, end);
            }
        }
    }
    return state;
}
Project plate_demo(PrintSequence sequence) {
    auto p = demo_project();
    p.machine.layout = Layout::Cartesian3Axis;
    p.machine.controller = Controller::GenericMarlin;
    p.process.curved = false;
    p.process.supports = false;
    p.process.topShellLayers = 0;
    p.process.bottomShellLayers = 0;
    p.process.layerHeight = .5;
    enable_plate(p);
    p.plate.sequence = sequence;
    p.plate.automaticOrder = false;
    p.objects[0].mesh = box_mesh(Vec3(-22, 0, 1), Vec3(8, 8, 2), "First block");
    p.objects[0].name = "First block";
    p.objects[0].process = p.process;
    p.objects[0].volume = {};
    auto second = p.objects.front();
    second.id = "object-2";
    second.name = "Second block";
    second.mesh = box_mesh(Vec3(22, 0, 1), Vec3(8, 8, 2), "Second block");
    p.objects.push_back(second);
    p.plate.order.push_back(second.id);
    synchronize_plate_preview(p);
    return p;
}
} // namespace fiveaxis
