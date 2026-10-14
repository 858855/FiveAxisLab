#include "collision.h"
#include "compute.h"
#include "geometry.h"
#include "machine_settings.h"
#include "nonplanar.h"
#include "plate.h"
#include "process.h"
#include "profile.h"
#include "slicing.h"
#include "stress.h"
#include <algorithm>
#include <cmath>
#include <fcl/fcl.h>
#include <map>
#include <mutex>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <stdexcept>

namespace fiveaxis {
static QString component_label(const Component &c) {
    return c.name + " [" + c.id + "]";
}
using BVH = fcl::BVHModel<fcl::OBBRSSd>;
static std::shared_ptr<fcl::CollisionGeometryd> geometry(const Mesh &mesh) {
    auto b = std::make_shared<BVH>();
    std::vector<fcl::Vector3d> v(mesh.vertices.begin(), mesh.vertices.end());
    std::vector<fcl::Triangle> faces;
    for (auto t : mesh.triangles)
        faces.emplace_back(t[0], t[1], t[2]);
    b->beginModel();
    b->addSubModel(v, faces);
    b->endModel();
    return b;
}
static Mesh subtract_region(const Mesh &source, const std::vector<std::pair<Vec3, double>> &planes) {
    Mesh out;
    out.name = source.name;
    out.error = source.error;
    auto append = [&](const std::vector<Vec3> &polygon) {
        if (polygon.size() < 3)
            return;
        int offset = int(out.vertices.size());
        out.vertices.insert(out.vertices.end(), polygon.begin(), polygon.end());
        for (size_t i = 1; i + 1 < polygon.size(); ++i)
            if ((polygon[i] - polygon[0]).cross(polygon[i + 1] - polygon[0]).norm() > 1e-14)
                out.triangles.emplace_back(offset, offset + int(i), offset + int(i) + 1);
    };
    // Subtract an inscribed convex contact prism, including both axial limits. Each outside
    // fragment is retained immediately; only the intersection of all inside halfspaces is discarded.
    for (auto face : source.triangles) {
        std::vector<Vec3> inside = {source.vertices[face[0]], source.vertices[face[1]],
                                    source.vertices[face[2]]};
        for (auto &plane : planes) {
            if (inside.empty())
                break;
            std::vector<Vec3> next, outside;
            for (size_t i = 0; i < inside.size(); ++i) {
                Vec3 a = inside[i], b = inside[(i + 1) % inside.size()];
                double da = plane.first.dot(a) - plane.second, db = plane.first.dot(b) - plane.second;
                bool ia = da <= 1e-12, ib = db <= 1e-12;
                if (ia)
                    next.push_back(a);
                else
                    outside.push_back(a);
                if (ia != ib) {
                    Vec3 point = a + (b - a) * (da / (da - db));
                    next.push_back(point);
                    outside.push_back(point);
                }
            }
            append(outside);
            inside = std::move(next);
        }
    }
    return out;
}
static Mesh noncontact_mesh(const Component &c, const ToolheadAssembly &t) {
    Vec3 axis = t.axis.normalized(), u = axis.unitOrthogonal(), v = axis.cross(u);
    std::vector<std::pair<Vec3, double>> planes = {{axis, axis.dot(t.tcp) + t.contactHeight},
                                                   {-axis, -axis.dot(t.tcp)}};
    for (int i = 0; i < 32; ++i) {
        double angle = 2 * pi * i / 32;
        Vec3 n = u * std::cos(angle) + v * std::sin(angle);
        planes.emplace_back(n, n.dot(t.tcp) + t.contactRadius * std::cos(pi / 32));
    }
    return subtract_region(c.mesh, planes);
}
struct CollisionScene::Impl {
    const Project &project;
    struct Tool {
        const Component *component;
        std::shared_ptr<fcl::CollisionGeometryd> full, masked;
        Mesh maskedMesh;
        Eigen::AlignedBox3d bounds;
    };
    struct Obstacle {
        QString name;
        std::shared_ptr<fcl::CollisionObjectd> object;
        const Mesh *solid = nullptr;
        double error = 0;
        bool deposited = false, bed = false;
        QString objectId;
        Vec3 center = Vec3::Zero();
        double radius = 0;
        // Deposited beads are boxes: pose of the box centre and half extents.
        Transform boxPose = Transform::Identity();
        Vec3 half = Vec3::Zero();
        // A group box contains consecutive collinear bead pieces (its members). Only the group is
        // in the broad phase; members are evaluated when the group cannot rule them out.
        std::vector<Obstacle *> members;
    };
    std::vector<Tool> tools;
    struct Interface {
        size_t first, second;
        Eigen::AlignedBox3d box;
        std::shared_ptr<fcl::CollisionGeometryd> outside;
        Transform regionFrame = Transform::Identity();
    };
    std::map<std::pair<size_t, size_t>, Interface> interfaces;
    struct FixtureInterface {
        Eigen::AlignedBox3d box;
        std::shared_ptr<fcl::CollisionGeometryd> outside;
        Transform regionFrame = Transform::Identity();
    };
    QString activeObject;
    std::map<std::pair<QString, QString>, FixtureInterface> fixtureInterfaces;
    bool movingPairs = false;
    std::vector<std::unique_ptr<Obstacle>> world, part, staged, groups;
    fcl::DynamicAABBTreeCollisionManagerd worldManager, partManager;
    double radius = 1;
    mutable size_t checkedStaticBodies = 0;
    mutable double staticBodyClearance = 1e100;
    mutable std::optional<CollisionEvent> staticBodyConflict;
    explicit Impl(const Project &p) : project(p) {
        for (auto &c : p.active_toolhead().components) {
            if (c.visualOnly)
                continue;
            if (c.mesh.empty())
                throw std::runtime_error("Empty collision component");
            Tool t;
            t.component = &c;
            t.bounds = c.mesh.bounds();
            t.full = geometry(c.mesh);
            if (c.nozzle) {
                t.maskedMesh = noncontact_mesh(c, p.active_toolhead());
                if (!t.maskedMesh.empty())
                    t.masked = geometry(t.maskedMesh);
            }
            tools.push_back(t);
            for (auto &v : c.mesh.vertices)
                radius = std::max(radius, p.machine.layout == Layout::RotaryBed
                                              ? (v - p.active_toolhead().tcp).norm()
                                              : (p.active_toolhead().mount * v).norm());
        }
        for (size_t i = 0; i < tools.size(); ++i)
            for (size_t j = i + 1; j < tools.size(); ++j)
                if (tools[i].component->link != tools[j].component->link)
                    movingPairs = true;
        for (const auto &rule : p.active_toolhead().interfaces) {
            auto find = [&](const QString &id) {
                for (size_t i = 0; i < tools.size(); ++i)
                    if (tools[i].component->id == id)
                        return i;
                throw std::runtime_error("Interface references a missing physical component");
            };
            size_t a = find(rule.first);
            if (rule.second.startsWith("fixture:")) {
                if (rule.size.minCoeff() <= 0)
                    throw std::runtime_error("Invalid fixture interface box");
                QString target = rule.second.mid(8);
                if (std::none_of(p.fixtures.begin(), p.fixtures.end(),
                                 [&](const Fixture &f) { return f.name == target; }))
                    throw std::runtime_error("Unknown interface fixture");
                std::vector<std::pair<Vec3, double>> planes;
                for (int j = 0; j < 3; ++j) {
                    Vec3 n = rule.regionFrame.linear() * Vec3::Unit(j);
                    Vec3 center = rule.regionFrame * rule.center;
                    planes.emplace_back(n, n.dot(center) + rule.size[j] / 2);
                    planes.emplace_back(-n, -n.dot(center) + rule.size[j] / 2);
                }
                auto mask = subtract_region(tools[a].component->mesh, planes);
                fixtureInterfaces[{rule.first, target}] = {
                    Eigen::AlignedBox3d(rule.center - rule.size / 2, rule.center + rule.size / 2),
                    mask.empty() ? nullptr : geometry(mask), rule.regionFrame};
                continue;
            }
            size_t b = find(rule.second);
            if (a == b || rule.size.minCoeff() <= 0 || interfaces.count({a, b}) || interfaces.count({b, a}))
                throw std::runtime_error("Invalid or duplicate interface pair");
            std::vector<std::pair<Vec3, double>> planes;
            for (int j = 0; j < 3; ++j) {
                Vec3 n = rule.regionFrame.linear() * Vec3::Unit(j);
                Vec3 center = rule.regionFrame * rule.center;
                planes.emplace_back(n, n.dot(center) + rule.size[j] / 2);
                planes.emplace_back(-n, -n.dot(center) + rule.size[j] / 2);
            }
            auto mask = subtract_region(tools[a].component->mesh, planes);
            interfaces.emplace(
                std::make_pair(a, b),
                Interface{a, b, Eigen::AlignedBox3d(rule.center - rule.size / 2, rule.center + rule.size / 2),
                          mask.empty() ? nullptr : geometry(mask), rule.regionFrame});
        }
        for (auto &fixture : p.fixtures) {
            if (fixture.mesh.empty())
                throw std::runtime_error("Empty fixture collision geometry");
            auto o = std::make_unique<Obstacle>();
            o->name = fixture.name;
            o->solid = &fixture.mesh;
            o->error = fixture.mesh.error;
            o->bed = fixture.name.contains("bed", Qt::CaseInsensitive);
            o->object = std::make_shared<fcl::CollisionObjectd>(geometry(fixture.mesh));
            o->object->setUserData(o.get());
            if (fixture.movesWithBed) {
                partManager.registerObject(o->object.get());
                part.push_back(std::move(o));
            } else {
                worldManager.registerObject(o->object.get());
                world.push_back(std::move(o));
            }
        }
        worldManager.setup();
        partManager.setup();
        double width = p.process.width, height = p.process.layerHeight;
        for (auto &s : p.path)
            if (s.deltaE > 0) {
                width = std::max(width, s.width);
                height = std::max(height, s.height);
            }
        radius = std::max(radius, p.active_toolhead().contactRadius + p.machine.clearance +
                                      Vec3((.2 + width) / 2, width / 2, height / 2).norm() + .1);
    }
};
CollisionScene::CollisionScene(const Project &p) : impl(std::make_unique<Impl>(p)) {}
CollisionScene::~CollisionScene() = default;
void CollisionScene::deposit(const Vec3 &a, const Vec3 &b, double width, double height, Role role,
                             const Vec3 &normal, const QString &objectId) {
    stage_deposit(a, b, width, height, role, normal, objectId);
    for (auto &o : impl->staged) {
        impl->partManager.registerObject(o->object.get());
        impl->part.push_back(std::move(o));
        if (impl->part.size() % 128 == 0)
            impl->partManager.update();
    }
    impl->staged.clear();
}
size_t CollisionScene::staged_count() const {
    return impl->staged.size();
}
void CollisionScene::commit_staged() {
    // Validation steps lay beads down in 0.2 mm pieces that overlap along their line. Consecutive
    // pieces with one cross-section and orientation on one line are covered by a single group box
    // (at most 5 mm long): one distance query to the group then bounds all of its pieces.
    auto &staged = impl->staged;
    for (size_t i = 0; i < staged.size();) {
        auto *first = staged[i].get();
        Vec3 axis = first->boxPose.linear().col(0);
        double along = first->boxPose.translation().dot(axis);
        double low = along - first->half.x(), high = along + first->half.x();
        size_t j = i + 1;
        while (j < staged.size() && j - i < 64) {
            const auto *next = staged[j].get();
            if (next->name != first->name || next->objectId != first->objectId ||
                (next->boxPose.linear() - first->boxPose.linear()).norm() > 1e-9 ||
                std::abs(next->half.y() - first->half.y()) > 1e-12 ||
                std::abs(next->half.z() - first->half.z()) > 1e-12)
                break;
            Vec3 offset = next->boxPose.translation() - first->boxPose.translation();
            if ((offset - axis * axis.dot(offset)).norm() > 1e-9)
                break;
            double centre = next->boxPose.translation().dot(axis);
            double nextLow = centre - next->half.x(), nextHigh = centre + next->half.x();
            // A piece must touch the group so the group box stays covered by its pieces.
            if (nextLow > high + 1e-9 || nextHigh < low - 1e-9 ||
                std::max(high, nextHigh) - std::min(low, nextLow) > 5)
                break;
            low = std::min(low, nextLow);
            high = std::max(high, nextHigh);
            ++j;
        }
        if (j - i > 1) {
            auto group = std::make_unique<Impl::Obstacle>();
            group->name = first->name;
            group->objectId = first->objectId;
            group->deposited = true;
            group->error = first->error;
            group->boxPose = first->boxPose;
            group->boxPose.translation() += axis * ((low + high) / 2 - along);
            group->half = Vec3((high - low) / 2, first->half.y(), first->half.z());
            group->center = group->boxPose.translation();
            group->radius = group->half.norm();
            group->object =
                std::make_shared<fcl::CollisionObjectd>(std::make_shared<fcl::Boxd>(2 * group->half), group->boxPose);
            group->object->setUserData(group.get());
            for (size_t k = i; k < j; ++k)
                group->members.push_back(staged[k].get());
            impl->partManager.registerObject(group->object.get());
            impl->groups.push_back(std::move(group));
        } else
            impl->partManager.registerObject(first->object.get());
        for (size_t k = i; k < j; ++k)
            impl->part.push_back(std::move(staged[k]));
        i = j;
    }
    if (!staged.empty())
        impl->partManager.update();
    staged.clear();
}
size_t CollisionScene::stage_deposit(const Vec3 &a, const Vec3 &b, double width, double height, Role role,
                                     const Vec3 &normal, const QString &objectId) {
    double length = (b - a).norm();
    if (length < 1e-10)
        return impl->staged.size();
    int count = std::max(1, int(std::ceil(length / std::max(width, .2))));
    for (int i = 0; i < count; ++i) {
        Vec3 x = a + (b - a) * double(i) / count, y = a + (b - a) * double(i + 1) / count;
        auto o = std::make_unique<Impl::Obstacle>();
        o->name = (role == Role::Support || role == Role::SupportInterface) ? "Deposited support"
                  : role == Role::Purge                                     ? "Deposited purge"
                                                                            : "Deposited part";
        o->objectId = objectId;
        if (!objectId.isEmpty())
            o->name += "/" + objectId;
        o->deposited = true;
        o->center = (x + y) / 2 - normal.normalized() * height / 2;
        o->radius = Vec3(((y - x).norm() + width) / 2, width / 2, height / 2).norm();
        if (impl->project.checkWorkpieceBodies && impl->project.machine.layout == Layout::RotaryBed)
            impl->radius =
                std::max(impl->radius, (o->center - impl->project.machine.bedPivot).norm() + o->radius);
        Transform pose = translation(o->center);
        Vec3 tangent = (y - x).normalized(), side = normal.cross(tangent);
        if (side.norm() < 1e-8)
            side = tangent.unitOrthogonal();
        side.normalize();
        Vec3 up = tangent.cross(side).normalized();
        if (up.dot(normal) < 0) {
            up = -up;
            side = -side;
        }
        pose.linear().col(0) = tangent;
        pose.linear().col(1) = side;
        pose.linear().col(2) = up;
        // Bound the rounded cross section without extending above its nozzle-top surface.
        auto g = std::make_shared<fcl::Boxd>((y - x).norm() + width, width, height);
        o->boxPose = pose;
        o->half = Vec3((y - x).norm() + width, width, height) / 2;
        o->object = std::make_shared<fcl::CollisionObjectd>(g, pose);
        o->object->setUserData(o.get());
        impl->staged.push_back(std::move(o));
    }
    return impl->staged.size();
}
void CollisionScene::set_active_object(const QString &id) {
    impl->activeObject = id;
}
CollisionEvent CollisionScene::check_pose(const JointState &q, bool extruding, const Vec3 &activePoint,
                                          double margin, size_t staged) const {
    const auto &p = impl->project;
    CollisionEvent result;
    result.status = Status::Clear;
    result.clearance = 1e100;
    if (!in_limits(p.machine, p.active_toolhead(), q)) {
        result.status = Status::Collision;
        result.first = "Machine";
        result.second = "Axis limits";
        result.reason = "Joint target exceeds configured machine limits";
        result.clearance = -1;
        return result;
    }
    double required = margin < 0 ? p.machine.clearance : margin;
    Transform bed = workpiece_pose(p.machine, q);
    struct Query {
        const Impl::Tool *tool;
        const ToolheadAssembly *assembly;
        const JointState *q;
        CollisionEvent *result;
        bool extruding;
        Vec3 active;
        Transform transform;
        double required;
        const std::map<std::pair<QString, QString>, Impl::FixtureInterface> *interfaces;
        QString activeObject;
    };
    static auto evaluate = [](fcl::CollisionObjectd *first, fcl::CollisionObjectd *second, void *data,
                              double &distance) -> bool {
        auto &query = *static_cast<Query *>(data);
        auto *obstacle = static_cast<Impl::Obstacle *>(second->getUserData());
        if (!obstacle) {
            std::swap(first, second);
            obstacle = static_cast<Impl::Obstacle *>(second->getUserData());
        }
        if (!obstacle)
            return false;
        fcl::DistanceRequestd request(true, false, 0, 0, 1e-8, fcl::GST_LIBCCD);
        fcl::DistanceResultd d;
        fcl::distance(first, second, request, d);
        double raw = d.min_distance;
        if (!std::isfinite(raw))
            raw = 0;
        auto rule = query.interfaces->find({query.tool->component->id, obstacle->name});
        const Impl::FixtureInterface *interface = rule == query.interfaces->end() ? nullptr : &rule->second;
        bool tipNearby = query.tool->component->nozzle && (obstacle->deposited || obstacle->bed);
        double allowance = query.assembly->contactRadius + obstacle->radius + query.required +
                           query.tool->component->mesh.error + obstacle->error;
        if (tipNearby && obstacle->deposited)
            tipNearby =
                (obstacle->center - query.active).norm() <= allowance &&
                (obstacle->center - query.active)
                        .dot(first->getTransform().linear() * query.assembly->axis.normalized()) < -1e-7;
        bool contactAllowed = query.extruding && tipNearby &&
                              (!obstacle->deposited || obstacle->objectId == query.activeObject);
        if (obstacle->deposited && obstacle->objectId != query.activeObject)
            tipNearby = false;
        double clearance = raw - query.tool->component->mesh.error - obstacle->error;
        if (tipNearby && query.tool->masked) {
            fcl::CollisionObjectd masked(query.tool->masked, first->getTransform());
            fcl::DistanceResultd maskedResult;
            fcl::distance(&masked, second, request, maskedResult);
            clearance = maskedResult.min_distance - query.tool->component->mesh.error - obstacle->error;
            if (obstacle->deposited) {
                double gate = allowance - (obstacle->center - query.active).norm();
                double lowerGate =
                    -(obstacle->center - query.active)
                         .dot(first->getTransform().linear() * query.assembly->axis.normalized());
                gate = std::min(gate, lowerGate);
                // Include the changing locality allowance in interval certification.
                clearance =
                    std::min(clearance, std::max(raw - query.tool->component->mesh.error - obstacle->error,
                                                 query.required + gate));
            }
            if (!contactAllowed && raw <= 0) {
                fcl::CollisionRequestd request;
                request.enable_contact = true;
                request.num_max_contacts = 32;
                fcl::CollisionResultd contacts;
                fcl::collide(first, second, request, contacts);
                for (size_t i = 0; i < contacts.numContacts(); ++i)
                    if (contacts.getContact(i).penetration_depth > 1e-7)
                        clearance = -1;
            }
            // The permitted tip region cannot penetrate the physical bed.
            if (obstacle->bed && raw <= 0)
                clearance = -1;
        }
        if (interface) {
            if (interface->outside) {
                fcl::CollisionObjectd masked(interface->outside, first->getTransform());
                fcl::DistanceResultd outside;
                fcl::distance(&masked, second, request, outside);
                clearance = outside.min_distance - query.tool->component->mesh.error - obstacle->error;
            } else
                clearance = 1e100;
        }
        if (obstacle->solid) {
            Transform inverse = first->getTransform().inverse();
            // STEP compounds may have disconnected islands. Test every witness,
            // with a cheap box rejection, rather than relying on the first vertex.
            if (first->getAABB().overlap(second->getAABB())) {
                const auto bounds = obstacle->solid->bounds();
                for (const auto &vertex : query.tool->component->mesh.vertices) {
                    Vec3 worldPoint = first->getTransform() * vertex;
                    if (bounds.contains(worldPoint) && contains(*obstacle->solid, worldPoint) &&
                        !(interface && interface->box.contains(interface->regionFrame.inverse() * vertex))) {
                        clearance = -1;
                        break;
                    }
                }
            }
            for (const auto &v : obstacle->solid->vertices) {
                Vec3 local = inverse * v;
                if (!query.tool->bounds.contains(local))
                    continue;
                if (contains(query.tool->component->mesh, local)) {
                    Vec3 delta = local - query.assembly->tcp;
                    double h = delta.dot(query.assembly->axis.normalized());
                    bool inContact = contactAllowed && h >= -1e-6 && h <= query.assembly->contactHeight &&
                                     (delta - query.assembly->axis.normalized() * h).norm() <=
                                         query.assembly->contactRadius;
                    if (!inContact &&
                        !(interface && interface->box.contains(interface->regionFrame.inverse() * local))) {
                        clearance = -1;
                        break;
                    }
                }
            }
        }
        if (obstacle->deposited) {
            Vec3 local = first->getTransform().inverse() * obstacle->center;
            if (query.tool->bounds.contains(local) && contains(query.tool->component->mesh, local)) {
                Vec3 delta = local - query.assembly->tcp;
                Vec3 axis = query.assembly->axis.normalized();
                double h = delta.dot(axis);
                bool allowed = contactAllowed && h >= -1e-6 && h <= query.assembly->contactHeight &&
                               (delta - axis * h).norm() <= query.assembly->contactRadius;
                if (!allowed)
                    clearance = -1;
            }
        }
        if (clearance < query.result->clearance) {
            query.result->clearance = clearance;
            query.result->first = component_label(*query.tool->component);
            query.result->second = obstacle->name;
            Vec3 point = d.nearest_points[0];
            if (raw <= 0) {
                fcl::CollisionRequestd cr;
                cr.enable_contact = true;
                cr.num_max_contacts = 8;
                fcl::CollisionResultd contacts;
                fcl::collide(first, second, cr, contacts);
                if (contacts.numContacts())
                    point = contacts.getContact(0).pos;
                else
                    point = obstacle->deposited ? obstacle->center : first->getTransform().translation();
            }
            query.result->contact = query.transform * point;
        }
        distance = std::max(0.0, query.result->clearance);
        if (clearance < query.required - 1e-8) {
            query.result->status = Status::Collision;
            query.result->reason = clearance < 0 ? "Physical intersection" : "Required clearance is not met";
            return true;
        }
        return false;
    };
    auto callback = [](fcl::CollisionObjectd *first, fcl::CollisionObjectd *second, void *data,
                       double &distance) -> bool {
        auto *obstacle = static_cast<Impl::Obstacle *>(second->getUserData());
        if (!obstacle) {
            std::swap(first, second);
            obstacle = static_cast<Impl::Obstacle *>(second->getUserData());
        }
        if (!obstacle || obstacle->members.empty())
            return evaluate(first, second, data, distance);
        // Every member lies inside the group box, so a member's surface distance is at least the
        // group's, and its clearance is at least that minus the errors (masked and gated nozzle
        // clearances are never below the full-mesh distance). A group that cannot reach below the
        // current minimum or the required clearance is skipped, unless the group box lies inside
        // the tool (it does not touch the tool surface and its centre is inside the mesh), where
        // the members' containment checks apply.
        auto &query = *static_cast<Query *>(data);
        double errors = query.tool->component->mesh.error + obstacle->error;
        auto threshold = [&] { return std::max(query.required, std::max(0.0, query.result->clearance)); };
        // Cheap bound first: the gap between the tool's local bounding box and a box expressed in
        // tool coordinates. It never exceeds the surface distance, and a positive gap places the
        // box outside the tool, which excludes containment.
        Transform toolInverse = first->getTransform().inverse();
        auto gap = [&](const Impl::Obstacle &box) {
            Transform local = toolInverse * box.boxPose;
            Vec3 extent = local.linear().cwiseAbs() * box.half, centre = local.translation();
            return (query.tool->bounds.min() - (centre + extent))
                .cwiseMax((centre - extent) - query.tool->bounds.max())
                .cwiseMax(Vec3::Zero())
                .norm();
        };
        if (gap(*obstacle) - errors >= threshold())
            return false;
        fcl::DistanceRequestd request(true, false, 0, 0, 1e-8, fcl::GST_LIBCCD);
        fcl::DistanceResultd d;
        fcl::distance(first, second, request, d);
        if (std::isfinite(d.min_distance) && d.min_distance > 0 && d.min_distance - errors >= threshold()) {
            Vec3 local = toolInverse * obstacle->center;
            if (!(query.tool->bounds.contains(local) && contains(query.tool->component->mesh, local)))
                return false;
        }
        // Members nearest the tool first, each behind the same cheap bound; the threshold tightens
        // as members are evaluated.
        std::vector<std::pair<double, Impl::Obstacle *>> members;
        members.reserve(obstacle->members.size());
        for (auto *member : obstacle->members)
            members.emplace_back(gap(*member), member);
        std::sort(members.begin(), members.end(),
                  [](const auto &a, const auto &b) { return a.first < b.first; });
        for (const auto &[lower, member] : members) {
            if (lower > 0 && lower - errors >= threshold())
                break;
            if (evaluate(first, member->object.get(), data, distance))
                return true;
        }
        return false;
    };
    for (auto &tool : impl->tools) {
        Transform world = component_pose(p.machine, p.active_toolhead(), *tool.component, q);
        for (int frame = 0; frame < 2; ++frame) {
            Transform tr = frame == 0 ? world : bed.inverse() * world;
            fcl::CollisionObjectd object(tool.full, tr);
            object.setUserData(nullptr);
            Query query{&tool,
                        &p.active_toolhead(),
                        &q,
                        &result,
                        extruding,
                        frame == 0 ? bed * activePoint : activePoint,
                        frame == 0 ? Transform::Identity() : bed,
                        required,
                        &impl->fixtureInterfaces,
                        impl->activeObject};
            if (frame == 0)
                impl->worldManager.distance(&object, &query, callback);
            else {
                impl->partManager.distance(&object, &query, callback);
                // Staged deposits visible to this query, pruned like the broad phase by the
                // distance between bounding boxes and the clearance found so far.
                object.computeAABB();
                for (size_t k = 0; k < staged && k < impl->staged.size() && result.status == Status::Clear; ++k) {
                    auto *other = impl->staged[k]->object.get();
                    if (object.getAABB().distance(other->getAABB()) > std::max(0.0, result.clearance))
                        continue;
                    double bound = std::max(0.0, result.clearance);
                    if (callback(&object, other, &query, bound))
                        break;
                }
            }
            if (result.status != Status::Clear)
                return result;
        }
    }
    // A declared interface removes only a bounded region of the first component's surface.
    for (size_t i = 0; i < impl->tools.size(); ++i)
        for (size_t j = i + 1; j < impl->tools.size(); ++j) {
            auto found = impl->interfaces.find({i, j});
            if (found == impl->interfaces.end())
                found = impl->interfaces.find({j, i});
            const Impl::Interface *rule = found == impl->interfaces.end() ? nullptr : &found->second;
            auto &a = impl->tools[rule ? rule->first : i];
            auto &b = impl->tools[rule ? rule->second : j];
            if (a.component->link == b.component->link || p.machine.layout != Layout::TiltingHead)
                continue;
            Transform ta = component_pose(p.machine, p.active_toolhead(), *a.component, q),
                      tb = component_pose(p.machine, p.active_toolhead(), *b.component, q);
            auto mask = rule ? rule->outside : a.full;
            double clearance = 1e100;
            Vec3 witness = ta.translation();
            if (mask) {
                fcl::CollisionObjectd x(mask, ta), y(b.full, tb);
                fcl::DistanceRequestd request(true);
                fcl::DistanceResultd distance;
                fcl::distance(&x, &y, request, distance);
                clearance = distance.min_distance - a.component->mesh.error - b.component->mesh.error;
                witness = distance.nearest_points[0];
            }
            double gate = 1e100;
            auto containment = [&](const Impl::Tool &object, const Transform &pose,
                                   const Impl::Tool &container, const Transform &other) {
                for (const auto &v : object.component->mesh.vertices) {
                    Vec3 point = pose * v, local = other.inverse() * point;
                    if (!container.bounds.contains(local) || !contains(container.component->mesh, local))
                        continue;
                    Vec3 interfacePoint = ta.inverse() * point;
                    if (!rule || !rule->box.contains(rule->regionFrame.inverse() * interfacePoint)) {
                        clearance = -1;
                        witness = point;
                        return;
                    }
                    interfacePoint = rule->regionFrame.inverse() * interfacePoint;
                    double inside = (interfacePoint - rule->box.min())
                                        .cwiseMin(rule->box.max() - interfacePoint)
                                        .minCoeff();
                    gate = std::min(gate, inside);
                }
            };
            containment(b, tb, a, ta);
            if (clearance >= 0)
                containment(a, ta, b, tb);
            if (rule && clearance >= 0)
                clearance = std::min(clearance, required + gate);
            result.clearance = std::min(result.clearance, clearance);
            if (clearance < required - 1e-8) {
                result.status = Status::Collision;
                result.first = component_label(*a.component);
                result.second = component_label(*b.component);
                result.reason = clearance < 0 ? "Link intersection outside declared interface"
                                              : "Moving links do not maintain clearance";
                result.clearance = clearance;
                result.contact = witness;
                return result;
            }
        }

    if (p.checkWorkpieceBodies && !impl->world.empty()) {
        struct Pair {
            const Impl::Obstacle *moving, *fixed;
            Transform pose;
            double lower;
        };
        std::vector<Pair> pairs;
        std::vector<double> bounds;
        bool stationary = p.machine.layout != Layout::RotaryBed;
        if (stationary) {
            result.clearance = std::min(result.clearance, impl->staticBodyClearance);
            if (impl->staticBodyConflict)
                return *impl->staticBodyConflict;
        }
        for (size_t index = stationary ? impl->checkedStaticBodies : 0; index < impl->part.size(); ++index) {
            const auto &moving = impl->part[index];
            fcl::CollisionObjectd body(*moving->object);
            auto pose = bed * moving->object->getTransform();
            body.setTransform(pose);
            body.computeAABB();
            auto a = body.getAABB();
            for (const auto &fixed : impl->world) {
                const auto &b = fixed->object->getAABB();
                double error = moving->error + fixed->error;
                Vec3 gap = Vec3::Zero();
                for (int k = 0; k < 3; ++k) {
                    gap[k] = std::max({0., a.min_[k] - b.max_[k], b.min_[k] - a.max_[k]});
                    bounds.push_back(a.min_[k] - b.max_[k] - required - error - 1e-9);
                    bounds.push_back(a.max_[k] - b.min_[k] + required + error + 1e-9);
                    bounds.push_back(0);
                }
                pairs.push_back({moving.get(), fixed.get(), pose, gap.norm() - error});
            }
        }
        ComputeReport candidatesReport;
        auto candidates = compute_workload(bounds, 1, Transform::Identity(), p.compute, candidatesReport);
        for (size_t i = 0; i < pairs.size(); ++i) {
            const auto &pair = pairs[i];
            if (candidates[i * 3].z() == 0 || candidates[i * 3 + 1].z() == 0 ||
                candidates[i * 3 + 2].z() == 0) {
                result.clearance = std::min(result.clearance, pair.lower);
                if (stationary)
                    impl->staticBodyClearance = std::min(impl->staticBodyClearance, pair.lower);
                continue;
            }
            auto moving = pair.moving, fixed = pair.fixed;
            fcl::CollisionObjectd body(*moving->object);
            body.setTransform(pair.pose);
            body.computeAABB();
            fcl::DistanceRequestd request(true, false, 0, 0, 1e-8, fcl::GST_LIBCCD);
            fcl::DistanceResultd distance;
            fcl::distance(&body, fixed->object.get(), request, distance);
            double clearance = distance.min_distance - moving->error - fixed->error;
            result.clearance = std::min(result.clearance, clearance);
            if (stationary)
                impl->staticBodyClearance = std::min(impl->staticBodyClearance, clearance);
            if (fixed->solid && contains(*fixed->solid, body.getTransform() *
                                                            moving->object->collisionGeometry()->aabb_center))
                clearance = -1;
            if (moving->solid &&
                contains(*moving->solid, bed.inverse() * (fixed->object->getTransform() *
                                                          fixed->object->collisionGeometry()->aabb_center)))
                clearance = -1;
            if (clearance < required - 1e-8) {
                result.status = Status::Collision;
                result.first = moving->name;
                result.second = fixed->name;
                result.reason =
                    "Workpiece/finished material does not clear a fixed machine body or parked tool";
                result.clearance = clearance;
                result.contact = distance.nearest_points[0];
                if (stationary)
                    impl->staticBodyConflict = result;
                return result;
            }
        }
        if (stationary)
            impl->checkedStaticBodies = impl->part.size();
    }
    return result;
}
bool CollisionScene::check_interval(const JointState &a, const JointState &b, bool extruding, const Vec3 &pa,
                                    const Vec3 &pb, int move, double ta, double tb, CollisionReport &report,
                                    std::atomic_bool *cancel, size_t staged) const {
    int iterations = 0;
    bool exhausted = false;
    std::function<bool(double, double, int)> check = [&](double left, double right, int depth) {
        if (cancel && cancel->load()) {
            report.status = Status::Cancelled;
            return false;
        }
        if (++iterations > 2048) {
            if (!exhausted) {
                CollisionEvent e;
                e.status = Status::Unresolved;
                e.move = move;
                e.time = ta + (tb - ta) * (left + right) / 2;
                e.first = "Motion interval";
                e.second = "Accuracy budget";
                e.reason = "Clearance certificate exceeded 2048 interval queries";
                report.events.push_back(e);
                if (report.status == Status::Clear)
                    report.status = Status::Unresolved;
                exhausted = true;
            }
            return false;
        }
        double mid = (left + right) / 2;
        JointState q = interpolate_motion(impl->project.machine, impl->project.active_toolhead(), a, b, mid);
        Vec3 point = part_position(impl->project.machine, q);
        auto hit = check_pose(q, extruding, point, -1, staged);
        ++report.checkedIntervals;
        report.minimumClearance = std::min(report.minimumClearance, hit.clearance);
        if (hit.status != Status::Clear) {
            hit.move = move;
            hit.time = ta + (tb - ta) * mid;
            report.events.push_back(hit);
            report.status = Status::Collision;
            return false;
        }
        auto x = interpolate_motion(impl->project.machine, impl->project.active_toolhead(), a, b, left),
             y = interpolate_motion(impl->project.machine, impl->project.active_toolhead(), a, b, right);
        double bound =
            movement_bound(impl->project.machine, impl->project.active_toolhead(), x, y, impl->radius);
        if (impl->project.machine.layout == Layout::TiltingHead && impl->movingPairs)
            bound *= 2;
        double margin = impl->project.machine.clearance;
        if (hit.clearance - bound >= margin + 1e-7 || bound < 1e-10) {
            report.minimumClearance = std::min(report.minimumClearance, hit.clearance - bound);
            return true;
        }
        if (depth >= 20) {
            CollisionEvent e;
            e.status = Status::Unresolved;
            e.first = hit.first;
            e.second = hit.second;
            e.move = move;
            e.time = ta + (tb - ta) * mid;
            e.clearance = hit.clearance;
            e.contact = hit.contact;
            e.reason = "Motion interval cannot establish clearance within subdivision budget";
            report.events.push_back(e);
            if (report.status == Status::Clear)
                report.status = Status::Unresolved;
            return false;
        }
        bool c1 = check(left, mid, depth + 1), c2 = check(mid, right, depth + 1);
        return c1 && c2;
    };
    (void)pa;
    (void)pb;
    return check(0, 1, 0);
}
CollisionReport validate_motion(const Project &p, std::atomic_bool *cancel,
                                std::function<void(double)> progress) {
    CollisionReport r;
    if (p.plateEnabled)
        return validate_plate_motion(p, cancel, progress);
    r.fingerprint = p.fingerprint();
    r.status = Status::Clear;
    try {
        check_printer_envelope(p);
    } catch (const std::exception &e) {
        r.status = Status::Unresolved;
        r.events.push_back(
            {Status::Unresolved, "Printer settings", "Printable area", QString::fromUtf8(e.what())});
        return r;
    }
    if ((p.machine.controller == Controller::GenericMarlin && p.machine.rotary_axes()) ||
        (p.process.nonPlanar3Axis && p.machine.rotary_axes()) ||
        (!p.machine.rotary_axes() && cartesian_motion_violation(p) >= 0)) {
        r.status = Status::Unresolved;
        r.events.push_back({Status::Unresolved, "Machine mode", "Motion constraints",
                            "Standard XYZ output requires zero rotary motion, an upright nozzle and allowed "
                            "non-planar slopes"});
        return r;
    }
    if (p.process.profileNormal) {
        try {
            int move = profile_motion_violation(p);
            if (move >= 0) {
                r.status = Status::Unresolved;
                r.events.push_back({Status::Unresolved, "Nozzle axis", "Profile plane",
                                    "Nozzle orientation or TCP departs from the required profile plane", move,
                                    p.path[move].time});
                return r;
            }
        } catch (const std::exception &e) {
            r.status = Status::Unresolved;
            r.events.push_back(
                {Status::Unresolved, "Profile guide", "Invalid guide", QString::fromUtf8(e.what())});
            return r;
        }
    }
    if ((!p.pathProcessFingerprint.isEmpty() || !p.machine.printer.isEmpty()) &&
        p.pathProcessFingerprint != printer_path_signature(p)) {
        r.status = Status::Unresolved;
        r.events.push_back({Status::Unresolved, "Print process", "Stale toolpaths",
                            "Process settings changed; slice again to generate matching motion"});
        return r;
    }
    for (auto &c : p.active_toolhead().components)
        if (!c.visualOnly)
            r.modeledComponents.push_back(component_label(c));
    for (auto &f : p.fixtures)
        r.modeledComponents.push_back(f.name);
    if (p.active_toolhead().components.empty() || p.path.size() < 2) {
        r.status = Status::Unresolved;
        r.events.push_back({Status::Unresolved, "Setup", "Missing geometry or path",
                            "Import a toolhead and generate a path first"});
        return r;
    }
    int physical = 0, nozzles = 0;
    bool hasExtrusion =
        std::any_of(p.path.begin(), p.path.end(), [](const Sample &s) { return s.deltaE > 0; });
    for (const auto &c : p.active_toolhead().components)
        if (!c.visualOnly) {
            ++physical;
            if (c.nozzle) {
                ++nozzles;
                if (p.machine.layout == Layout::TiltingHead && c.link != Link::Nozzle) {
                    r.status = Status::Unresolved;
                    r.events.push_back({Status::Unresolved, "Setup", "Nozzle attachment",
                                        "A tilting-head nozzle must be attached to the final nozzle link"});
                    return r;
                }
            }
        }
    if (physical == 0 || (hasExtrusion && nozzles != 1)) {
        r.status = Status::Unresolved;
        r.events.push_back({Status::Unresolved, "Setup", "Physical nozzle",
                            "Extrusion requires one physical nozzle and a modeled toolhead"});
        return r;
    }
    if (p.active_toolhead().axis.norm() < 1e-8 ||
        (p.active_toolhead().mount.linear() * p.active_toolhead().axis.normalized() - Vec3::UnitZ()).norm() >
            1e-6) {
        r.status = Status::Unresolved;
        r.events.push_back({Status::Unresolved, "Setup", "Nozzle frame",
                            "Align the zero-pose nozzle axis to +Z in the setup wizard"});
        return r;
    }
    double lastTime = -1;
    for (size_t i = 0; i < p.path.size(); ++i) {
        const auto &s = p.path[i];
        if (!s.position.allFinite() || !s.direction.allFinite() || s.direction.norm() < 1e-8 ||
            !std::isfinite(s.time) || !std::isfinite(s.deltaE) || s.time < lastTime ||
            (part_position(p.machine, s.joint) - s.position).norm() > p.machine.chordTolerance + 1e-5 ||
            (part_direction(p.machine, p.active_toolhead(), s.joint) - s.direction.normalized()).norm() >
                1e-5) {
            r.status = Status::Unresolved;
            CollisionEvent e;
            e.status = Status::Unresolved;
            e.move = int(i);
            e.first = "Motion";
            e.second = "Machine calibration";
            e.reason = "Stored path and machine transforms disagree; regenerate motion before validation";
            r.events.push_back(e);
            return r;
        }
        lastTime = s.time;
    }
    CollisionScene scene(p);
    auto initial = scene.check_pose(p.path.front().joint, false, p.path.front().position);
    if (initial.status != Status::Clear) {
        initial.move = 0;
        r.events.push_back(initial);
        r.status = Status::Collision;
    }
    // Every check must see the material deposited before it. Steps are processed in blocks:
    // all steps of a block are checked in parallel against the material committed before the
    // block plus that block's earlier steps, held as staged deposits, and results are merged in
    // path order, so the outcome matches a sequential pass.
    struct Step {
        int move = 0;
        JointState x, y;
        Vec3 pa = Vec3::Zero(), pb = Vec3::Zero();
        bool extruding = false;
        double ta = 0, tb = 0;
        size_t staged = 0;
    };
    struct Outcome {
        CollisionEvent end;
        CollisionReport interval;
    };
    // Workpiece-body checks keep mutable per-scene caches, so they stay sequential.
    int threads = p.checkWorkpieceBodies ? 1 : compute_threads(p.compute);
    size_t blockSize = threads > 1 ? size_t(threads) * 48 : 1;
    std::vector<Step> block;
    bool stop = false;
    auto flush = [&] {
        if (block.empty() || stop)
            return;
        std::vector<Outcome> outcomes(block.size());
        std::atomic_bool halted{false};
        ComputeSettings settings = p.compute;
        settings.threads = threads;
        parallel_for(block.size(), settings, [&](size_t j) {
            if (halted.load() || (cancel && cancel->load())) {
                halted = true;
                return;
            }
            const auto &s = block[j];
            auto &o = outcomes[j];
            o.interval.status = Status::Clear;
            o.end = scene.check_pose(s.y, s.extruding, s.pb, -1, s.staged);
            if (o.end.status == Status::Clear)
                scene.check_interval(s.x, s.y, s.extruding, s.pa, s.pb, s.move, s.ta, s.tb, o.interval, cancel,
                                     s.staged);
        });
        for (size_t j = 0; j < block.size() && !stop; ++j) {
            if (cancel && cancel->load()) {
                r.status = Status::Cancelled;
                stop = true;
                break;
            }
            auto &o = outcomes[j];
            r.minimumClearance = std::min(r.minimumClearance, o.end.clearance);
            if (o.end.status != Status::Clear) {
                o.end.move = block[j].move;
                o.end.time = block[j].tb;
                r.events.push_back(o.end);
                r.status = Status::Collision;
            } else {
                r.checkedIntervals += o.interval.checkedIntervals;
                r.minimumClearance = std::min(r.minimumClearance, o.interval.minimumClearance);
                r.events.insert(r.events.end(), o.interval.events.begin(), o.interval.events.end());
                if (o.interval.status == Status::Cancelled)
                    r.status = Status::Cancelled;
                else if (o.interval.status == Status::Collision && r.status != Status::Cancelled)
                    r.status = Status::Collision;
                else if (o.interval.status == Status::Unresolved && r.status == Status::Clear)
                    r.status = Status::Unresolved;
            }
            if (r.status == Status::Cancelled)
                stop = true;
            else if (r.events.size() > 200) {
                r.status = Status::Collision;
                stop = true;
            }
        }
        scene.commit_staged();
        block.clear();
    };
    for (size_t i = 1; i < p.path.size() && !stop; ++i) {
        auto &a = p.path[i - 1];
        auto &b = p.path[i];
        if (cancel && cancel->load()) {
            r.status = Status::Cancelled;
            break;
        }
        // Grow even the current movement in bounded increments. A later point on a long bead
        // must see material extruded earlier in that same movement.
        int steps = b.deltaE > 0 ? std::max(1, int(std::ceil((b.position - a.position).norm() / .2))) : 1;
        if (steps > 100000)
            throw std::runtime_error("Deposition movement exceeds accuracy budget");
        for (int k = 0; k < steps; ++k) {
            double u = double(k) / steps, v = double(k + 1) / steps;
            Step s;
            s.move = int(i);
            s.x = interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u);
            s.y = interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, v);
            s.pa = part_position(p.machine, s.x);
            s.pb = part_position(p.machine, s.y);
            s.extruding = b.deltaE > 0;
            s.ta = a.time + (b.time - a.time) * u;
            s.tb = a.time + (b.time - a.time) * v;
            s.staged = scene.staged_count();
            block.push_back(s);
            if (b.deltaE > 0 && b.role != Role::Retract)
                scene.stage_deposit(
                    s.pa, s.pb, b.width, b.height, b.role,
                    p.machine.rotary_axes()
                        ? part_direction(p.machine, p.active_toolhead(),
                                         interpolate_motion(p.machine, p.active_toolhead(), s.x, s.y, .5))
                        : (surface_normal(a) * (1 - (u + v) / 2) + surface_normal(b) * ((u + v) / 2))
                              .normalized());
            if (block.size() >= blockSize)
                flush();
        }
        if (progress && !stop)
            progress(double(i) / p.path.size());
    }
    flush();
    if (p.deposition.checked && !p.depositionValid && r.status != Status::Cancelled) {
        if (r.status == Status::Clear)
            r.status = Status::Unresolved;
        if (p.deposition.coverage < .95) {
            CollisionEvent e;
            e.status = Status::Unresolved;
            e.first = "Deposition";
            e.second = "Coverage";
            e.reason = "Sampled material coverage is below 95%; complete the missing region before export";
            r.events.push_back(e);
        }
        for (int move : p.deposition.unsupportedMoves) {
            if (r.events.size() > 200)
                break;
            CollisionEvent e;
            e.status = Status::Unresolved;
            e.first = "Deposition";
            e.second = "Support / process constraints";
            e.move = move;
            e.time = p.path.at(move).time;
            e.contact = workpiece_pose(p.machine, p.path[move].joint) * p.path[move].position;
            e.reason = "Accumulated support, gravity angle, or actual-motion extrusion check failed";
            r.events.push_back(e);
        }
    }
    return r;
}
static std::mutex plannerMutex;
class SeededSampler : public ompl::base::RealVectorStateSampler {
  public:
    explicit SeededSampler(const ompl::base::StateSpace *space) : RealVectorStateSampler(space) {
        rng_.setLocalSeed(42);
    }
};
class SeededConnect : public ompl::geometric::RRTConnect {
  public:
    explicit SeededConnect(const ompl::base::SpaceInformationPtr &si) : RRTConnect(si) {
        rng_.setLocalSeed(42);
    }
};
std::vector<JointState> repair_travel(CollisionScene &scene, const MachineProfile &machine,
                                      const ToolheadAssembly &assembly, const JointState &first,
                                      const JointState &last, std::atomic_bool *cancel) {
    std::lock_guard<std::mutex> lock(plannerMutex);
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    if (scene.check_pose(first, false, part_position(machine, first)).status != Status::Clear ||
        scene.check_pose(last, false, part_position(machine, last)).status != Status::Clear)
        return {};
    int dimensions = machine.rotary_axes() ? 5 : 3;
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimensions);
    ob::RealVectorBounds bounds(dimensions);
    for (int k = 0; k < 3; ++k) {
        bounds.setLow(k, machine.minXYZ[k]);
        bounds.setHigh(k, machine.maxXYZ[k]);
    }
    if (dimensions == 5) {
        bounds.setLow(3, machine.minA);
        bounds.setHigh(3, machine.maxA);
        bounds.setLow(4, machine.minB);
        bounds.setHigh(4, machine.maxB);
    }
    space->setBounds(bounds);
    space->setStateSamplerAllocator(
        [](const ob::StateSpace *s) { return std::make_shared<SeededSampler>(s); });
    og::SimpleSetup setup(space);
    auto pose = [&](const ob::State *state) {
        auto values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        JointState axes;
        axes.xyz = Vec3(values[0], values[1], values[2]);
        if (dimensions == 5) {
            axes.a = values[3];
            axes.b = values[4];
        }
        return pose_from_axes(machine, assembly, axes);
    };
    setup.setStateValidityChecker([&](const ob::State *state) {
        auto q = pose(state);
        return in_limits(machine, assembly, q) &&
               scene.check_pose(q, false, part_position(machine, q)).status == Status::Clear;
    });
    setup.getSpaceInformation()->setStateValidityCheckingResolution(.0005);
    ob::ScopedState<> start(space), goal(space);
    auto a = controller_axes(machine, assembly, first), b = controller_axes(machine, assembly, last);
    for (int k = 0; k < 3; ++k) {
        start[k] = a.xyz[k];
        goal[k] = b.xyz[k];
    }
    if (dimensions == 5) {
        start[3] = a.a;
        start[4] = a.b;
        goal[3] = b.a;
        goal[4] = b.b;
    }
    setup.setStartAndGoalStates(start, goal);
    setup.setPlanner(std::make_shared<SeededConnect>(setup.getSpaceInformation()));
    int budget = 0;
    if (setup.solve(ob::PlannerTerminationCondition([&] {
            return ++budget >= 5000 || (cancel && cancel->load());
        })) != ob::PlannerStatus::EXACT_SOLUTION)
        return {};
    auto path = setup.getSolutionPath();
    std::vector<JointState> result;
    auto previous = first;
    for (size_t i = 1; i < path.getStateCount(); ++i) {
        auto next = pose(path.getState(i));
        CollisionReport report;
        report.status = Status::Clear;
        if (!scene.check_interval(previous, next, false, Vec3::Zero(), Vec3::Zero(), 0, 0, 1, report, cancel))
            return {};
        result.push_back(next);
        previous = next;
    }
    return result;
}
bool repair_motion(Project &p, std::atomic_bool *cancel) {
    if (p.plateEnabled)
        return repair_plate_motion(p, cancel);
    std::lock_guard<std::mutex> lock(plannerMutex);
    CollisionScene scene(p);
    bool changed = false;
    int trials = 0, conflicts = 0;
    static bool seeded = false;
    if (!seeded) {
        ompl::RNG::setSeed(42);
        seeded = true;
    }
    for (size_t i = 1; i < p.path.size(); ++i) {
        if (cancel && cancel->load())
            return false;
        auto previous = p.path[i - 1];
        auto target = p.path[i];
        CollisionReport local;
        local.status = Status::Clear;
        bool good = scene.check_interval(previous.joint, target.joint, target.deltaE > 0, previous.position,
                                         target.position, int(i), previous.time, target.time, local, cancel);
        if (!good && ++conflicts > 16) {
            p.diagnostics << "Collision repair budget exhausted after 16 conflicts.";
            break;
        }
        if (!good && target.deltaE > 0) {
            Vec3 original = target.direction;
            Vec3 x = original.unitOrthogonal(), y = original.cross(x);
            for (int cone = 0; cone <= 2 && !good && trials < 256; ++cone)
                for (int angle = 0; angle < 12 && !good && trials < 256; ++angle) {
                    double tilt = p.process.profileNormal || !p.machine.rotary_axes()
                                      ? 0
                                      : cone * p.process.directionCone / 2,
                           az = angle * 2 * pi / 12;
                    Vec3 n =
                        (original * std::cos(tilt) + (x * std::cos(az) + y * std::sin(az)) * std::sin(tilt))
                            .normalized();
                    for (auto q : inverse_kinematics(p.machine, target.position, n, previous.joint,
                                                     p.active_toolhead())) {
                        if (++trials > 256)
                            break;
                        CollisionReport test;
                        test.status = Status::Clear;
                        if (scene.check_interval(previous.joint, q, true, previous.position, target.position,
                                                 int(i), previous.time, target.time, test, cancel)) {
                            p.path[i].joint = q;
                            p.path[i].direction = n;
                            target = p.path[i];
                            good = true;
                            changed = true;
                            break;
                        }
                    }
                }
        }
        if (!good && target.deltaE <= 0) {
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            int dimensions = p.machine.rotary_axes() ? 5 : 3;
            auto space = std::make_shared<ob::RealVectorStateSpace>(dimensions);
            ob::RealVectorBounds bounds(dimensions);
            for (int k = 0; k < 3; ++k) {
                bounds.setLow(k, p.machine.minXYZ[k]);
                bounds.setHigh(k, p.machine.maxXYZ[k]);
            }
            if (p.machine.rotary_axes()) {
                bounds.setLow(3, p.machine.minA);
                bounds.setHigh(3, p.machine.maxA);
                bounds.setLow(4, p.machine.minB);
                bounds.setHigh(4, p.machine.maxB);
            }
            space->setBounds(bounds);
            space->setStateSamplerAllocator(
                [](const ob::StateSpace *s) { return std::make_shared<SeededSampler>(s); });
            og::SimpleSetup setup(space);
            auto from = [&p](const ob::State *state) {
                auto s = state->as<ob::RealVectorStateSpace::StateType>();
                JointState q;
                q.xyz = Vec3(s->values[0], s->values[1], s->values[2]);
                q.a = p.machine.rotary_axes() ? s->values[3] : 0;
                q.b = p.machine.rotary_axes() ? s->values[4] : 0;
                return pose_from_axes(p.machine, p.active_toolhead(), q);
            };
            setup.setStateValidityChecker([&](const ob::State *state) {
                auto q = from(state);
                return scene.check_pose(q, false, part_position(p.machine, q)).status == Status::Clear;
            });
            setup.getSpaceInformation()->setStateValidityCheckingResolution(.0005);
            ob::ScopedState<> start(space), goal(space);
            for (int k = 0; k < 3; ++k) {
                start[k] = controller_axes(p.machine, p.active_toolhead(), previous.joint).xyz[k];
                goal[k] = controller_axes(p.machine, p.active_toolhead(), target.joint).xyz[k];
            }
            if (p.machine.rotary_axes()) {
                start[3] = previous.joint.a;
                start[4] = previous.joint.b;
                goal[3] = target.joint.a;
                goal[4] = target.joint.b;
            }
            setup.setStartAndGoalStates(start, goal);
            setup.setPlanner(std::make_shared<SeededConnect>(setup.getSpaceInformation()));
            int budget = 0;
            auto solved = setup.solve(ob::PlannerTerminationCondition(
                [&] { return ++budget >= 5000 || (cancel && cancel->load()); }));
            if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
                auto path = setup.getSolutionPath();
                std::vector<Sample> replacement;
                JointState last = previous.joint;
                bool valid = true;
                for (size_t k = 1; k < path.getStateCount(); ++k) {
                    Sample s = target;
                    s.deltaE = k + 1 == path.getStateCount() ? target.deltaE : 0;
                    s.joint = from(path.getState(k));
                    s.position = part_position(p.machine, s.joint);
                    s.direction = part_direction(p.machine, p.active_toolhead(), s.joint);
                    CollisionReport test;
                    test.status = Status::Clear;
                    if (!scene.check_interval(last, s.joint, false, Vec3::Zero(), Vec3::Zero(), int(i), 0, 1,
                                              test, cancel)) {
                        valid = false;
                        break;
                    }
                    replacement.push_back(s);
                    last = s.joint;
                }
                if (valid && !replacement.empty()) {
                    p.path.erase(p.path.begin() + i);
                    p.path.insert(p.path.begin() + i, replacement.begin(), replacement.end());
                    i += replacement.size() - 1;
                    target = p.path[i];
                    good = true;
                    changed = true;
                }
            }
        }
        if (target.deltaE > 0)
            scene.deposit(previous.position, target.position, target.width, target.height, target.role,
                          (surface_normal(previous) + surface_normal(target)).normalized());
    }
    if (changed)
        refine_joint_motion(p);
    if (changed)
        check_deposition(p);
    if (changed && p.stress.directional) {
        p.diagnostics << QString("Repaired trajectory stress alignment: %1%.")
                             .arg(stress_alignment(p) * 100, 0, 'f', 1);
        if (p.loads.fixedNodes.size() >= 3 && !p.loads.nodalForces.empty()) {
            try {
                p.comparison = compare_strength(p);
            } catch (const std::exception &e) {
                p.depositionValid = false;
                p.diagnostics << QString("Strength recheck failed: %1").arg(e.what());
            }
        }
    }
    p.report = validate_motion(p, cancel);
    // One bounded retry changes the orientation of the affected curve as a region.
    if (p.machine.rotary_axes() && !p.process.profileNormal && !p.report.exportable() &&
        !p.report.events.empty() && !(cancel && cancel->load())) {
        auto event = p.report.events.front();
        if (event.move > 0 && event.move < int(p.path.size()) && p.path[event.move].deltaE > 0) {
            Project candidate = p;
            int region = p.path[event.move].region;
            Vec3 away = p.path[event.move].position -
                        workpiece_pose(p.machine, p.path[event.move].joint).inverse() * event.contact;
            for (auto &s : candidate.path)
                if (s.region == region) {
                    Vec3 tangent = away - s.direction * s.direction.dot(away);
                    if (tangent.norm() < 1e-8)
                        tangent = s.direction.unitOrthogonal();
                    s.direction = (s.direction * std::cos(p.process.directionCone) +
                                   tangent.normalized() * std::sin(p.process.directionCone))
                                      .normalized();
                }
            try {
                regenerate_joints(candidate);
                check_deposition(candidate);
                candidate.report = validate_motion(candidate, cancel);
                if (candidate.depositionValid && candidate.report.exportable()) {
                    candidate.diagnostics
                        << "Regenerated the affected region orientation once; all checks passed.";
                    p = std::move(candidate);
                }
            } catch (const std::exception &e) {
                p.diagnostics << QString("Bounded region retry rejected: %1").arg(e.what());
            }
        }
    }
    return p.report.exportable() && p.depositionValid;
}
} // namespace fiveaxis
