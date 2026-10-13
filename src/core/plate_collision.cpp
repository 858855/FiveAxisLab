#include "collision.h"
#include "compute.h"
#include "geometry.h"
#include "machine_settings.h"
#include "nonplanar.h"
#include "plate.h"
#include "profile.h"
#include "slicing.h"
#include <algorithm>
#include <stdexcept>
namespace fiveaxis {
static Project collision_view(const Project &p, const QString &active) {
    Project view;
    view.checkWorkpieceBodies = true;
    view.machine = p.machine;
    view.process = p.process;
    view.compute = p.compute;
    view.toolhead = assembly_at(p, active);
    view.fixtures = p.fixtures;
    for (const auto &tool : p.tools)
        if (tool.detachable && tool.id != active) {
            auto assembly = installed_assembly(tool);
            for (const auto &component : assembly.components) {
                if (component.visualOnly)
                    continue;
                auto mesh = component.mesh;
                for (auto &point : mesh.vertices)
                    point = tool.dock * point;
                view.fixtures.push_back({"parked/" + tool.id + "/" + component.id, std::move(mesh), false});
            }
        }
    for (auto &component : view.toolhead.components)
        component.name = "tool/" + active + "/" + component.id;
    return view;
}
static bool deposited(const Sample &s) {
    return s.deltaE > 0 && s.event == EventKind::Motion && s.role != Role::Retract;
}
static void deposit_interval(CollisionScene &scene, const Project &p, size_t i) {
    const auto &a = p.path[i - 1], &b = p.path[i];
    if (!deposited(b))
        return;
    int count = std::max(1, int(std::ceil((b.position - a.position).norm() / .2)));
    for (int k = 0; k < count; ++k) {
        double u = double(k) / count, v = double(k + 1) / count;
        auto q = sample_motion(p, a, b, (u + v) / 2);
        Vec3 normal =
            p.machine.rotary_axes()
                ? part_direction(p.machine, assembly_at(p, b.toolId), q)
                : (surface_normal(a) * (1 - (u + v) / 2) + surface_normal(b) * ((u + v) / 2)).normalized();
        scene.deposit(part_position(p.machine, sample_motion(p, a, b, u)),
                      part_position(p.machine, sample_motion(p, a, b, v)), b.width, b.height, b.role, normal,
                      b.objectId);
    }
}
CollisionReport validate_plate_motion(const Project &p, std::atomic_bool *cancel,
                                      std::function<void(double)> progress) {
    CollisionReport report;
    report.fingerprint = p.fingerprint();
    report.status = Status::Clear;
    try {
        validate_plate_setup(p);
        check_printer_envelope(p);
    } catch (const std::exception &e) {
        report.status = Status::Unresolved;
        report.events.push_back({Status::Unresolved, "Plate setup", "Profiles", QString::fromUtf8(e.what())});
        return report;
    }
    if (!p.pathProcessFingerprint.isEmpty() && p.pathProcessFingerprint != plate_generation_signature(p)) {
        report.status = Status::Unresolved;
        report.events.push_back({Status::Unresolved, "Plate configuration", "Stale material/toolpaths",
                                 "Planning inputs changed; regenerate the plate"});
        return report;
    }
    if (p.path.size() < 2) {
        report.status = Status::Unresolved;
        report.events.push_back({Status::Unresolved, "Plate", "Trajectory", "Generate plate motion first"});
        return report;
    }
    for (const auto &tool : p.tools)
        for (const auto &c : installed_assembly(tool).components)
            if (!c.visualOnly)
                report.modeledComponents.push_back("tool/" + tool.id + "/" + c.id);
    for (const auto &c : p.carriageAssembly.components)
        report.modeledComponents.push_back("carriage/" + c.id);
    for (const auto &f : p.fixtures)
        report.modeledComponents.push_back(f.name);
    for (const auto &o : p.objects)
        report.modeledComponents.push_back("deposited/" + o.id);
    report.modeledComponents.push_back("deposited/purge-tower");
    double previousTime = -1;
    QString active = p.path.front().toolId, material = p.path.front().materialId;
    for (size_t i = 0; i < p.path.size(); ++i) {
        const auto &s = p.path[i];
        auto fail = [&](const QString &reason) {
            report.status = Status::Unresolved;
            report.events.push_back({Status::Unresolved, "Job state", s.objectId, reason, int(i), s.time});
        };
        if (!std::isfinite(s.time) || s.time < previousTime || !std::isfinite(s.deltaE) ||
            !s.position.allFinite() || !s.direction.allFinite() || std::abs(s.direction.norm() - 1) > 1e-6 ||
            int(s.event) < 0 || int(s.event) > int(EventKind::Standby)) {
            fail("Invalid job event or timeline");
            return report;
        }
        previousTime = s.time;
        if (i) {
            if (s.event == EventKind::Attach) {
                auto incoming = find_tool(p, s.argument);
                if (!incoming || !active.isEmpty()) {
                    fail("Attach requires an empty carriage and a known tool");
                    return report;
                }
                active = s.argument;
            }
            if (s.event == EventKind::Detach) {
                if (s.argument != active) {
                    fail("Detach does not match the attached tool");
                    return report;
                }
                active.clear();
            }
            if (s.event == EventKind::Material)
                material = s.argument;
        }
        if (s.toolId != active || s.materialId != material) {
            fail("Tool/material changed without a corresponding job event");
            return report;
        }
        if (!s.contactComponent.isEmpty()) {
            bool declared = false;
            auto axes = sample_axes(p, s);
            for (const auto &tool : p.tools)
                for (const auto *steps : {&tool.change.release, &tool.change.pickup})
                    for (const auto &step : *steps)
                        if (step.contactComponent == s.contactComponent &&
                            step.contactFixture == s.contactFixture &&
                            (step.contactCenter - s.contactCenter).norm() < 1e-9 &&
                            (step.contactSize - s.contactSize).norm() < 1e-9 &&
                            (step.carriage.xyz - axes.xyz).norm() <= p.machine.chordTolerance &&
                            std::abs(step.carriage.a - axes.a) + std::abs(step.carriage.b - axes.b) < 1e-8)
                            declared = true;
            if (!declared) {
                fail("Motion contains an undeclared or misplaced docking-contact allowance");
                return report;
            }
        }
        if (s.event == EventKind::Attach || s.event == EventKind::Detach) {
            auto t = find_tool(p, s.argument);
            if (!t) {
                fail("Unknown docking tool");
                return report;
            }
            auto geometry = installed_assembly(*t);
            auto q = pose_from_axes(p.machine, geometry, sample_axes(p, s));
            for (const auto &component : geometry.components) {
                if (component.visualOnly)
                    continue;
                auto pose = component_pose(p.machine, geometry, component, q);
                if ((pose.matrix() - t->dock.matrix()).norm() > p.machine.chordTolerance) {
                    fail("Pickup/drop-off does not match calibrated parked CAD pose");
                    return report;
                }
            }
        }
        if (deposited(s)) {
            auto t = find_tool(p, s.toolId);
            auto m = find_material(p, s.materialId);
            if (!t || !m) {
                fail("Extrusion requires a physical tool and known material");
                return report;
            }
            bool routed = false;
            for (const auto &r : t->routes)
                routed |= r.materialId == m->id && r.confirmed;
            if (!routed || s.width < t->nozzle.minWidth - 1e-8 || s.width > t->nozzle.maxWidth + 1e-8 ||
                s.height < t->nozzle.minHeight - 1e-8 || s.height > t->nozzle.maxHeight + 1e-8) {
                fail("Extrusion is incompatible with the installed nozzle or filament route");
                return report;
            }
            if (i) {
                double length = (s.position - p.path[i - 1].position).norm();
                double expected = length * bead_area(s.width, s.height) / (pi * std::pow(m->diameter / 2, 2));
                if (std::abs(expected - s.deltaE) > std::max(1e-7, expected * .03)) {
                    fail("Extrusion does not match the selected material and actual path");
                    return report;
                }
            }
            if (s.role != Role::Purge) {
                auto o = std::find_if(p.objects.begin(), p.objects.end(),
                                      [&](const PlateObject &o) { return o.id == s.objectId; });
                if (o == p.objects.end()) {
                    fail("Extrusion references a missing object");
                    return report;
                }
                if (o->process.profileNormal) {
                    Project local;
                    local.process = o->process;
                    local.profileGuide = o->guide;
                    for (auto &v : local.profileGuide) {
                        v.center = o->placement * v.center;
                        v.normal = o->placement.linear() * v.normal;
                    }
                    auto section = profile_layer(local, s.layer);
                    if (s.direction.dot(section.normal) < 1 - 1e-8 ||
                        std::abs((s.position - section.center).dot(section.normal)) >
                            p.machine.chordTolerance) {
                        fail("Nozzle lost the required profile-normal constraint");
                        return report;
                    }
                    if (i && (std::abs(s.joint.a - p.path[i - 1].joint.a) +
                                  std::abs(s.joint.b - p.path[i - 1].joint.b) >
                              1e-8)) {
                        fail("Profile-normal extrusion must keep rotary joints fixed");
                        return report;
                    }
                }
            }
        }
    }
    size_t workers =
        std::min<size_t>(compute_threads(p.compute), std::max<size_t>(1, (p.path.size() - 1) / 64));
    std::vector<CollisionReport> parts(workers);
    parallel_for(
        workers, p.compute,
        [&](size_t worker) {
            auto &r = parts[worker];
            r.status = Status::Clear;
            size_t first = 1 + (p.path.size() - 1) * worker / workers,
                   last = 1 + (p.path.size() - 1) * (worker + 1) / workers;
            QString active, contactComponent, contactFixture;
            Vec3 contactCenter = Vec3::Zero(), contactSize = Vec3::Zero();
            std::unique_ptr<CollisionScene> scene;
            Project view;
            auto restore = [&](const QString &id, size_t prefix) {
                scene.reset();
                view = collision_view(p, id);
                if (!contactComponent.isEmpty() && contactSize.minCoeff() > 0) {
                    ToolheadAssembly::InterfaceAllowance rule{contactComponent, "fixture:" + contactFixture,
                                                              contactCenter, contactSize};
                    bool shared = contactComponent.startsWith("carriage/");
                    if (!shared &&
                        !std::any_of(view.toolhead.components.begin(), view.toolhead.components.end(),
                                     [&](const Component &c) { return c.id == contactComponent; }))
                        shared = std::any_of(p.carriageAssembly.components.begin(),
                                             p.carriageAssembly.components.end(),
                                             [&](const Component &c) { return c.id == contactComponent; });
                    if (shared) {
                        if (!rule.first.startsWith("carriage/"))
                            rule.first = "carriage/" + rule.first;
                        rule.regionFrame = view.toolhead.mount.inverse() * p.carriageAssembly.mount;
                    }
                    if (contactFixture.startsWith("tool:")) {
                        auto reference = contactFixture.mid(5);
                        int separator = reference.indexOf('/');
                        if (separator <= 0 || separator + 1 >= reference.size())
                            throw std::runtime_error("Tool interface target must be tool:ID/component");
                        auto tool = reference.left(separator), component = reference.mid(separator + 1);
                        rule.second = tool == id ? component : "fixture:parked/" + tool + "/" + component;
                    }
                    view.toolhead.interfaces.push_back(rule);
                }
                if (view.toolhead.components.empty())
                    throw std::runtime_error(
                        "Model the empty carriage/coupler before tool-changing validation");
                scene = std::make_unique<CollisionScene>(view);
                for (size_t j = 1; j < prefix; ++j)
                    deposit_interval(*scene, p, j);
                active = id;
            };
            try {
                for (size_t i = first; i < last; ++i) {
                    if (r.events.size() >= 200)
                        break;
                    if (cancel && cancel->load()) {
                        r.status = Status::Cancelled;
                        break;
                    }
                    const auto &a = p.path[i - 1], &b = p.path[i];
                    QString intervalTool = b.event == EventKind::Motion ? b.toolId : a.toolId;
                    const auto &contact = b.contactComponent.isEmpty() ? a : b;
                    bool changed = contact.contactComponent != contactComponent ||
                                   contact.contactFixture != contactFixture ||
                                   (contact.contactCenter - contactCenter).norm() > 1e-9 ||
                                   (contact.contactSize - contactSize).norm() > 1e-9;
                    contactComponent = contact.contactComponent;
                    contactFixture = contact.contactFixture;
                    contactCenter = contact.contactCenter;
                    contactSize = contact.contactSize;
                    if (!scene || intervalTool != active || changed)
                        restore(intervalTool, i);
                    scene->set_active_object(b.objectId);
                    auto assembly = assembly_at(p, intervalTool);
                    auto axesA = sample_axes(p, a), axesB = sample_axes(p, b);
                    auto x = pose_from_axes(p.machine, assembly, axesA),
                         y = pose_from_axes(p.machine, assembly, axesB);
                    if (!in_limits(p.machine, assembly, x) || !in_limits(p.machine, assembly, y)) {
                        r.status = Status::Unresolved;
                        r.events.push_back({Status::Unresolved, "tool/" + intervalTool, b.objectId,
                                            "Carriage or rotary limit exceeded", int(i), b.time});
                        continue;
                    }
                    if (b.event == EventKind::Motion) {
                        auto expected = part_position(p.machine, y);
                        auto direction = part_direction(p.machine, assembly, y);
                        if ((expected - b.position).norm() > p.machine.chordTolerance ||
                            (direction - b.direction).norm() > 1e-6) {
                            r.status = Status::Unresolved;
                            r.events.push_back({Status::Unresolved, "Motion", b.objectId,
                                                "Stored TCP disagrees with selected tool calibration", int(i),
                                                b.time});
                            continue;
                        }
                    }
                    if (b.event == EventKind::Load || b.event == EventKind::Unload) {
                        auto event = scene->check_pose(y, false, part_position(p.machine, y));
                        if (event.status != Status::Clear) {
                            event.move = int(i);
                            event.time = b.time;
                            r.events.push_back(event);
                            r.status = event.status;
                        }
                        continue;
                    }
                    int steps =
                        deposited(b) ? std::max(1, int(std::ceil((b.position - a.position).norm() / .2))) : 1;
                    if (steps > 100000)
                        throw std::runtime_error("Deposition interval budget exhausted");
                    for (int k = 0; k < steps; ++k) {
                        double u = double(k) / steps, v = double(k + 1) / steps;
                        auto qa = pose_from_axes(p.machine, assembly, interpolate(axesA, axesB, u)),
                             qb = pose_from_axes(p.machine, assembly, interpolate(axesA, axesB, v));
                        Vec3 pa = part_position(p.machine, qa), pb = part_position(p.machine, qb);
                        scene->check_interval(qa, qb, deposited(b), pa, pb, int(i),
                                              a.time + (b.time - a.time) * u, a.time + (b.time - a.time) * v,
                                              r, cancel);
                        auto endpoint = scene->check_pose(qb, deposited(b), pb);
                        r.minimumClearance = std::min(r.minimumClearance, endpoint.clearance);
                        if (endpoint.status != Status::Clear) {
                            endpoint.move = int(i);
                            endpoint.time = a.time + (b.time - a.time) * v;
                            endpoint.reason +=
                                QString(" [object %1, material %2]").arg(b.objectId, b.materialId);
                            r.events.push_back(endpoint);
                            if (r.status == Status::Clear)
                                r.status = endpoint.status;
                        }
                        if (deposited(b)) {
                            Vec3 normal =
                                p.machine.rotary_axes()
                                    ? part_direction(p.machine, assembly,
                                                     pose_from_axes(p.machine, assembly,
                                                                    interpolate(axesA, axesB, (u + v) / 2)))
                                    : (surface_normal(a) * (1 - (u + v) / 2) +
                                       surface_normal(b) * ((u + v) / 2))
                                          .normalized();
                            scene->deposit(pa, pb, b.width, b.height, b.role, normal, b.objectId);
                            // The final bead must also see fixed bodies; there may be no
                            // later movement that would otherwise query its birth prefix.
                            auto born = scene->check_pose(qb, true, pb);
                            r.minimumClearance = std::min(r.minimumClearance, born.clearance);
                            if (born.status != Status::Clear) {
                                born.move = int(i);
                                born.time = a.time + (b.time - a.time) * v;
                                r.events.push_back(born);
                                if (r.status == Status::Clear)
                                    r.status = born.status;
                            }
                        }
                    }
                    if (b.toolId != intervalTool) {
                        restore(b.toolId, i + 1);
                        auto q = pose_from_axes(p.machine, assembly_at(p, b.toolId), axesB);
                        auto endpoint = scene->check_pose(q, false, part_position(p.machine, q));
                        if (endpoint.status != Status::Clear) {
                            endpoint.move = int(i);
                            endpoint.time = b.time;
                            r.events.push_back(endpoint);
                            r.status = endpoint.status;
                        }
                    }
                }
            } catch (const std::exception &e) {
                r.status = cancel && cancel->load() ? Status::Cancelled : Status::Unresolved;
                r.events.push_back(
                    {r.status, "Plate", "Validation", QString::fromUtf8(e.what()), int(first)});
            }
        },
        nullptr);
    for (size_t i = 0; i < parts.size(); ++i) {
        auto &r = parts[i];
        report.checkedIntervals += r.checkedIntervals;
        report.minimumClearance = std::min(report.minimumClearance, r.minimumClearance);
        report.events.insert(report.events.end(), r.events.begin(), r.events.end());
        if (r.status == Status::Cancelled)
            report.status = Status::Cancelled;
        else if (report.status != Status::Cancelled && r.status == Status::Unresolved)
            report.status = Status::Unresolved;
        else if (report.status == Status::Clear && r.status == Status::Collision)
            report.status = Status::Collision;
        if (progress)
            progress(double(i + 1) / parts.size());
    }
    std::stable_sort(
        report.events.begin(), report.events.end(), [](const CollisionEvent &a, const CollisionEvent &b) {
            return std::tie(a.move, a.time, a.first, a.second) < std::tie(b.move, b.time, b.first, b.second);
        });
    if (!p.depositionValid && report.status == Status::Clear) {
        report.status = Status::Unresolved;
        report.events.push_back({Status::Unresolved, "Plate deposition", "Coverage",
                                 "One or more object/material deposition plans are incomplete"});
    }
    return report;
}
static bool repair_plate_travels(Project &p, std::atomic_bool *cancel) {
    for (int attempt = 0; attempt < 16; ++attempt) {
        p.report = validate_plate_motion(p, cancel);
        if (p.report.exportable())
            return true;
        if (cancel && cancel->load())
            return false;
        bool changed = false;
        int searchAttempts = 0;
        for (const auto &event : p.report.events) {
            if (++searchAttempts > 16)
                break;
            int i = event.move;
            if (i <= 0 || i >= int(p.path.size()))
                continue;
            const auto &a = p.path[i - 1], &b = p.path[i];
            if (b.event != EventKind::Motion || b.deltaE != 0 || a.toolId != b.toolId ||
                !a.contactComponent.isEmpty() || !b.contactComponent.isEmpty())
                continue;
            auto view = collision_view(p, b.toolId);
            CollisionScene scene(view);
            for (int j = 1; j < i; ++j)
                deposit_interval(scene, p, j);
            scene.set_active_object(b.objectId);
            auto replacement = repair_travel(
                scene, p.machine, view.toolhead, pose_from_axes(p.machine, view.toolhead, sample_axes(p, a)),
                pose_from_axes(p.machine, view.toolhead, sample_axes(p, b)), cancel);
            if (replacement.empty())
                continue;
            auto target = b;
            std::vector<Sample> samples;
            for (auto q : replacement) {
                auto sample = target;
                sample.joint = q;
                sample.carriage = controller_axes(p.machine, view.toolhead, q);
                sample.position = part_position(p.machine, q);
                sample.direction = part_direction(p.machine, view.toolhead, q);
                samples.push_back(sample);
            }
            p.path.erase(p.path.begin() + i);
            p.path.insert(p.path.begin() + i, samples.begin(), samples.end());
            time_plate(p);
            changed = true;
            p.diagnostics << "Repaired plate travel with bounded joint-space RRT-Connect; checking the "
                             "complete job again.";
            break;
        }
        if (!changed)
            return false;
    }
    p.report = validate_plate_motion(p, cancel);
    return p.report.exportable();
}
bool repair_plate_motion(Project &p, std::atomic_bool *cancel) {
    if (repair_plate_travels(p, cancel))
        return true;
    // A single region retry is permitted only when deposition can be rechecked
    // using one physical nozzle calibration throughout that object.
    if (p.machine.rotary_axes() && !(cancel && cancel->load()))
        for (const auto &event : p.report.events) {
            if (event.move <= 0 || event.move >= int(p.path.size()))
                continue;
            const auto &hit = p.path[event.move];
            if (!deposited(hit) || hit.role == Role::Purge)
                continue;
            auto object = std::find_if(p.objects.begin(), p.objects.end(),
                                       [&](const PlateObject &o) { return o.id == hit.objectId; });
            if (object == p.objects.end() || object->process.profileNormal)
                continue;
            bool oneCalibration = std::all_of(p.path.begin(), p.path.end(), [&](const Sample &s) {
                return s.objectId != hit.objectId || !deposited(s) ||
                       (s.toolId == hit.toolId &&
                        std::abs(find_material(p, s.materialId)->diameter -
                                 find_material(p, hit.materialId)->diameter) < 1e-9);
            });
            if (!oneCalibration)
                break;
            auto candidate = p;
            auto assembly = assembly_at(p, hit.toolId);
            bool feasible = true;
            for (auto &sample : candidate.path)
                if (sample.objectId == hit.objectId && sample.region == hit.region &&
                    sample.event == EventKind::Motion) {
                    auto normal = sample.direction;
                    auto tangent = normal.unitOrthogonal();
                    sample.direction = (normal * std::cos(object->process.directionCone) +
                                        tangent * std::sin(object->process.directionCone))
                                           .normalized();
                    auto poses = inverse_kinematics(p.machine, sample.position, sample.direction,
                                                    sample.joint, assembly);
                    if (poses.empty()) {
                        feasible = false;
                        break;
                    }
                    sample.joint = poses.front();
                    sample.carriage = controller_axes(p.machine, assembly, sample.joint);
                }
            if (feasible) {
                Project local;
                local.machine = p.machine;
                local.machine.filamentDiameter = find_material(p, hit.materialId)->diameter;
                local.process = object->process;
                local.toolhead = assembly;
                local.fixtures = p.fixtures;
                local.part = object->mesh;
                local.volume = object->volume;
                for (auto &v : local.part.vertices)
                    v = object->placement * v;
                for (auto &v : local.volume.vertices)
                    v = object->placement * v;
                for (const auto &sample : candidate.path)
                    if (sample.objectId == hit.objectId && sample.event == EventKind::Motion)
                        local.path.push_back(sample);
                check_deposition(local);
                if (local.depositionValid) {
                    time_plate(candidate);
                    candidate.report = validate_plate_motion(candidate, cancel);
                    if (candidate.report.exportable()) {
                        p = std::move(candidate);
                        p.diagnostics << "Regenerated one affected object region orientation; deposition and "
                                         "complete plate collision checks passed.";
                        return true;
                    }
                }
            }
            p.diagnostics << "Bounded extrusion orientation retry did not produce a validated region.";
            break;
        }
    if (p.plate.sequence == PrintSequence::Objects && p.plate.automaticOrder) {
        std::vector<QString> order;
        for (const auto &o : p.objects)
            order.push_back(o.id);
        std::sort(order.begin(), order.end());
        int budget = p.plate.searchBudget;
        bool exhausted = false;
        std::optional<Project> best;
        double bestCost = 1e100;
        do {
            if (cancel && cancel->load()) {
                p.report.status = Status::Cancelled;
                return false;
            }
            if (budget-- <= 0) {
                exhausted = true;
                break;
            }
            auto candidate = p;
            candidate.plate.order = order;
            try {
                slice_plate(candidate, cancel);
                if (!repair_plate_travels(candidate, cancel))
                    continue;
                double cost = candidate.path.back().time;
                for (const auto &sample : candidate.path)
                    if (sample.role == Role::Purge && sample.deltaE > 0)
                        cost += sample.deltaE;
                if (cost < bestCost) {
                    bestCost = cost;
                    best = std::move(candidate);
                }
            } catch (const std::exception &e) {
                p.diagnostics << QString("Order candidate rejected: ") + e.what();
            }
        } while (std::next_permutation(order.begin(), order.end()));
        if (best) {
            p = std::move(*best);
            p.diagnostics << "Selected the lowest time/waste cost among validated object orders within the "
                             "search budget.";
            return true;
        }
        p.report.status = Status::Unresolved;
        p.report.events.push_back(
            {Status::Unresolved, "Object order",
             exhausted ? "Search budget exhausted" : "Candidates rejected",
             "No validated object order found; this is not proof that printing is impossible"});
    }
    return false;
}
} // namespace fiveaxis
