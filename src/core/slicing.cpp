#include "slicing.h"
#include "compute.h"
#include "geometry.h"
#include "machine_settings.h"
#include "nonplanar.h"
#include "plate.h"
#include "process.h"
#include "profile.h"
#include "stress.h"
#include <Eigen/LU>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

namespace fiveaxis {
namespace bg = boost::geometry;
using Point2 = bg::model::d2::point_xy<double>;
using Polygon2 = bg::model::polygon<Point2>;
using MultiPolygon2 = bg::model::multi_polygon<Polygon2>;
struct SurfaceGrid {
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    double cell = 1;
    int nx = 1, ny = 1;
    std::vector<std::vector<int>> cells;
};
// A flat layer island taken directly from the part's surface mesh (planar layer fields).
struct PlanarSection {
    Polygon2 shape;
    double z = 0, reach = 1;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments;
    Eigen::AlignedBox2d bounds;
    std::vector<std::vector<int>> rows;
};
struct Surface {
    Mesh mesh;
    std::vector<Vec3> normals;
    std::vector<double> distance;
    std::vector<double> heightScale;
    std::shared_ptr<const SurfaceGrid> grid;
    // Set for planar islands; mesh, normals, distance and heightScale are then empty.
    std::shared_ptr<const PlanarSection> section;
};
// One planar island built directly from a mesh section, with its boundary segments indexed in
// horizontal bands for point-in-polygon and near-boundary distance queries.
static void index_section(PlanarSection &s) {
    s.segments.clear();
    auto add = [&](const auto &ring) {
        for (size_t i = 1; i < ring.size(); ++i)
            s.segments.push_back({Eigen::Vector2d(bg::get<0>(ring[i - 1]), bg::get<1>(ring[i - 1])),
                                  Eigen::Vector2d(bg::get<0>(ring[i]), bg::get<1>(ring[i]))});
    };
    add(s.shape.outer());
    for (const auto &inner : s.shape.inners())
        add(inner);
    s.bounds.setEmpty();
    for (const auto &[a, b] : s.segments) {
        s.bounds.extend(a);
        s.bounds.extend(b);
    }
    int count = std::max(1, int(std::ceil(s.bounds.sizes().y() / s.reach)) + 1);
    s.rows.assign(count, {});
    for (int i = 0; i < int(s.segments.size()); ++i) {
        const auto &[a, b] = s.segments[i];
        int r0 = std::clamp(int((std::min(a.y(), b.y()) - s.bounds.min().y()) / s.reach), 0, count - 1),
            r1 = std::clamp(int((std::max(a.y(), b.y()) - s.bounds.min().y()) / s.reach), 0, count - 1);
        for (int r = r0; r <= r1; ++r)
            s.rows[r].push_back(i);
    }
}
static bool section_point(const PlanarSection &s, const Eigen::Vector2d &q, double &distance) {
    if (!s.bounds.contains(q))
        return false;
    int row = std::clamp(int((q.y() - s.bounds.min().y()) / s.reach), 0, int(s.rows.size()) - 1);
    bool inside = false;
    for (int id : s.rows[row]) {
        const auto &[a, b] = s.segments[id];
        if ((a.y() > q.y()) != (b.y() > q.y()) && q.x() < (b.x() - a.x()) * (q.y() - a.y()) / (b.y() - a.y()) + a.x())
            inside = !inside;
    }
    if (!inside)
        return false;
    // Exact within the indexed reach; farther points only need to know they are at least that far.
    double best = s.reach;
    for (int r = std::max(0, row - 1); r <= std::min(int(s.rows.size()) - 1, row + 1); ++r)
        for (int id : s.rows[r]) {
            const auto &[a, b] = s.segments[id];
            Eigen::Vector2d d = b - a;
            double u = d.squaredNorm() > 1e-20 ? std::clamp((q - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
            best = std::min(best, (q - a - u * d).norm());
        }
    distance = best;
    return true;
}
// XY triangle index for surface_point. Lists keep ascending triangle order so lookups
// return the same triangle as a linear scan on folded surfaces.
static void index_surface(Surface &s) {
    if (s.mesh.triangles.empty())
        return;
    auto grid = std::make_shared<SurfaceGrid>();
    auto bounds = s.mesh.bounds();
    Eigen::Vector2d size = bounds.sizes().head<2>().cwiseMax(1e-6);
    grid->cell = std::max(size.maxCoeff() / std::max(1.0, std::sqrt(double(s.mesh.triangles.size()))), 1e-4);
    grid->origin = bounds.min().head<2>();
    grid->nx = std::max(1, int(std::ceil(size.x() / grid->cell)));
    grid->ny = std::max(1, int(std::ceil(size.y() / grid->cell)));
    grid->cells.resize(size_t(grid->nx) * grid->ny);
    for (int i = 0; i < int(s.mesh.triangles.size()); ++i) {
        Eigen::AlignedBox2d b;
        for (int j = 0; j < 3; ++j)
            b.extend(s.mesh.vertices[s.mesh.triangles[i][j]].head<2>());
        auto index = [&](double v, double o, int count) {
            return std::clamp(int(std::floor((v - o) / grid->cell)), 0, count - 1);
        };
        int x0 = index(b.min().x() - 1e-6, grid->origin.x(), grid->nx),
            x1 = index(b.max().x() + 1e-6, grid->origin.x(), grid->nx),
            y0 = index(b.min().y() - 1e-6, grid->origin.y(), grid->ny),
            y1 = index(b.max().y() + 1e-6, grid->origin.y(), grid->ny);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                grid->cells[size_t(y) * grid->nx + x].push_back(i);
    }
    s.grid = std::move(grid);
}
static std::vector<double> scalar_field(const Project &p) {
    if (!p.machine.rotary_axes()) {
        if (p.process.nonPlanar3Axis)
            return conformal_field(p);
        std::vector<double> field;
        for (const auto &v : p.volume.vertices)
            field.push_back(v.z());
        return field;
    }
    const auto &mesh = p.volume;
    size_t n = mesh.vertices.size();
    std::vector<double> values;
    for (auto &v : mesh.vertices)
        values.push_back(v.z());
    if (!p.process.curved || !p.stress.directional)
        return values;
    double low = p.part.bounds().min().z(), high = p.part.bounds().max().z();
    std::set<int> anchors;
    for (size_t i = 0; i < n; ++i)
        if (mesh.vertices[i].z() <= low + 2 * p.process.layerHeight ||
            mesh.vertices[i].z() >= high - .1 * p.process.layerHeight)
            anchors.insert(int(i));
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<Vec3> centers;
    for (const auto &cell : mesh.cells) {
        Vec3 center = Vec3::Zero();
        for (int j = 0; j < 4; ++j)
            center += mesh.vertices[cell.nodes[j]] / 4;
        centers.push_back(center);
    }
    ComputeReport samplingReport;
    auto sampled = sample_stress_batch(p.stress, centers, p.compute, samplingReport);
    size_t sampleIndex = 0;
    for (auto &cell : mesh.cells) {
        Mat3 tensor = sampled[sampleIndex++];
        Eigen::Matrix4d M;
        Vec3 center = Vec3::Zero();
        for (int j = 0; j < 4; ++j) {
            auto v = mesh.vertices[cell.nodes[j]];
            M.row(j) << 1, v.transpose();
            center += v / 4;
        }
        double volume = std::abs(M.determinant()) / 6;
        if (volume < 1e-12)
            continue;
        // FEM coverage of every cell center is enforced by sample_stress_batch.
        Eigen::Matrix<double, 3, 4> gradient = M.inverse().bottomRows<3>();
        double confidence;
        Vec3 direction = principal_tensile_direction(tensor, &confidence);
        Mat3 D = Mat3::Identity() + 4 * std::clamp(confidence, 0.0, 1.0) * direction * direction.transpose();
        Eigen::Matrix4d K = gradient.transpose() * D * gradient * volume;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                int a = cell.nodes[i], b = cell.nodes[j];
                if (anchors.count(a))
                    continue;
                if (anchors.count(b))
                    rhs[a] -= K(i, j) * values[b];
                else
                    triplets.emplace_back(a, b, K(i, j));
            }
    }
    for (int i : anchors) {
        triplets.emplace_back(i, i, 1);
        rhs[i] = values[i];
    }
    Eigen::SparseMatrix<double> K(n, n);
    K.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("Scalar-field solve failed");
    Eigen::VectorXd f = solver.solve(rhs);
    if (!f.allFinite())
        throw std::runtime_error("Invalid scalar field");
    // Preserve positive build progression and bound spacing distortion by blending with the anchored
    // reference field.
    double blend = 1;
    for (int pass = 0; pass < 12; ++pass) {
        bool valid = true;
        for (auto &cell : mesh.cells) {
            Eigen::Matrix4d M;
            Eigen::Vector4d node;
            for (int j = 0; j < 4; ++j) {
                Vec3 v = mesh.vertices[cell.nodes[j]];
                M.row(j) << 1, v.transpose();
                node[j] = (1 - blend) * v.z() + blend * f[cell.nodes[j]];
            }
            Vec3 g = M.inverse().bottomRows<3>() * node;
            if (g.z() < std::cos(p.process.maximumTilt) * g.norm() || g.norm() < .65 || g.norm() > 1.4) {
                valid = false;
                break;
            }
        }
        if (valid)
            break;
        blend *= .5;
    }
    for (size_t i = 0; i < n; ++i)
        values[i] = (1 - blend) * values[i] + blend * f[i];
    return values;
}
// exactRadius: boundary distances up to this value are exact Euclidean distances to the
// boundary edges (walls and the infill limit); larger values propagate along mesh edges.
static Surface iso_surface(const VolumeMesh &mesh, const std::vector<double> &values, double level,
                           const ComputeSettings &compute, ComputeReport &report, std::atomic_bool *cancel,
                           double exactRadius = 0) {
    Surface out;
    std::vector<int> normalCounts;
    std::map<std::array<long long, 3>, int> ids;
    const int edges[][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    std::vector<double> ranges(mesh.cells.size() * 3);
    parallel_for(
        mesh.cells.size(), compute,
        [&](size_t i) {
            double low = 1e100, high = -1e100;
            for (int j = 0; j < 4; ++j) {
                double value = values[mesh.cells[i].nodes[j]];
                low = std::min(low, value);
                high = std::max(high, value);
            }
            ranges[i * 3] = std::nextafter(low, -INFINITY);
            ranges[i * 3 + 1] = std::nextafter(high, INFINITY);
            ranges[i * 3 + 2] = level;
        },
        cancel);
    auto candidates = compute_workload(ranges, 1, Transform::Identity(), compute, report, cancel);
    size_t candidateIndex = 0;
    for (auto &t : mesh.cells) {
        if (candidates[candidateIndex++].z() == 0)
            continue;
        std::vector<Vec3> intersections;
        Eigen::Matrix4d M;
        Eigen::Vector4d v;
        for (int j = 0; j < 4; ++j) {
            auto x = mesh.vertices[t.nodes[j]];
            M.row(j) << 1, x.transpose();
            v[j] = values[t.nodes[j]];
        }
        if (v.minCoeff() > level || v.maxCoeff() < level)
            continue;
        Vec3 normal = M.inverse().bottomRows<3>() * v;
        if (normal.norm() < 1e-10)
            continue;
        double scale = 1 / normal.norm();
        normal.normalize();
        for (auto &e : edges) {
            int a = t.nodes[e[0]], b = t.nodes[e[1]];
            double x = values[a] - level, y = values[b] - level;
            if ((x < 0) == (y < 0) || std::abs(x - y) < 1e-12)
                continue;
            Vec3 q = mesh.vertices[a] + (mesh.vertices[b] - mesh.vertices[a]) * x / (x - y);
            if (std::none_of(intersections.begin(), intersections.end(),
                             [&](auto &p) { return (p - q).norm() < 1e-7; }))
                intersections.push_back(q);
        }
        if (intersections.size() < 3)
            continue;
        Vec3 center = Vec3::Zero();
        for (auto &x : intersections)
            center += x / double(intersections.size());
        Vec3 u = normal.unitOrthogonal(), w = normal.cross(u);
        std::sort(intersections.begin(), intersections.end(), [&](auto &a, auto &b) {
            return std::atan2((a - center).dot(w), (a - center).dot(u)) <
                   std::atan2((b - center).dot(w), (b - center).dot(u));
        });
        std::vector<int> index;
        for (auto &q : intersections) {
            std::array<long long, 3> key = {llround(q.x() * 1e7), llround(q.y() * 1e7), llround(q.z() * 1e7)};
            auto found = ids.find(key);
            int id;
            if (found == ids.end()) {
                id = int(out.mesh.vertices.size());
                ids[key] = id;
                out.mesh.vertices.push_back(q);
                out.normals.push_back(normal);
                out.heightScale.push_back(scale);
                normalCounts.push_back(1);
            } else {
                id = found->second;
                out.normals[id] += normal;
                out.heightScale[id] += scale;
                ++normalCounts[id];
            }
            index.push_back(id);
        }
        for (size_t j = 1; j + 1 < index.size(); ++j)
            out.mesh.triangles.emplace_back(index[0], index[j], index[j + 1]);
    }
    for (auto &n : out.normals)
        n.normalize();
    for (size_t i = 0; i < out.heightScale.size(); ++i)
        out.heightScale[i] /= normalCounts[i];
    std::vector<std::vector<std::pair<int, double>>> graph(out.mesh.vertices.size());
    std::map<std::pair<int, int>, int> counts;
    for (auto t : out.mesh.triangles)
        for (int j = 0; j < 3; ++j) {
            int a = t[j], b = t[(j + 1) % 3];
            double d = (out.mesh.vertices[a] - out.mesh.vertices[b]).norm();
            graph[a].emplace_back(b, d);
            graph[b].emplace_back(a, d);
            if (a > b)
                std::swap(a, b);
            ++counts[{a, b}];
        }
    out.distance.assign(out.mesh.vertices.size(), 1e100);
    std::vector<std::pair<int, int>> boundary;
    for (auto &[edge, count] : counts)
        if (count == 1) {
            boundary.push_back(edge);
            out.distance[edge.first] = out.distance[edge.second] = 0;
        }
    // Path length along mesh edges overestimates the distance to the contour and made walls
    // wander on coarse volume meshes, so the band used by walls and infill is computed exactly.
    if (exactRadius > 0 && !boundary.empty()) {
        using Key = std::array<long long, 3>;
        auto key = [&](const Vec3 &v) {
            return Key{(long long)std::floor(v.x() / exactRadius), (long long)std::floor(v.y() / exactRadius),
                       (long long)std::floor(v.z() / exactRadius)};
        };
        std::map<Key, std::vector<int>> buckets;
        for (int k = 0; k < int(boundary.size()); ++k) {
            Key a = key(out.mesh.vertices[boundary[k].first]), b = key(out.mesh.vertices[boundary[k].second]);
            for (long long x = std::min(a[0], b[0]); x <= std::max(a[0], b[0]); ++x)
                for (long long y = std::min(a[1], b[1]); y <= std::max(a[1], b[1]); ++y)
                    for (long long z = std::min(a[2], b[2]); z <= std::max(a[2], b[2]); ++z)
                        buckets[{x, y, z}].push_back(k);
        }
        for (size_t v = 0; v < out.mesh.vertices.size(); ++v) {
            const Vec3 &q = out.mesh.vertices[v];
            Key c = key(q);
            double best = out.distance[v];
            for (int x = -1; x <= 1; ++x)
                for (int y = -1; y <= 1; ++y)
                    for (int z = -1; z <= 1; ++z) {
                        auto found = buckets.find({c[0] + x, c[1] + y, c[2] + z});
                        if (found == buckets.end())
                            continue;
                        for (int k : found->second) {
                            const Vec3 &a = out.mesh.vertices[boundary[k].first],
                                       &b = out.mesh.vertices[boundary[k].second];
                            Vec3 d = b - a;
                            double u = d.squaredNorm() > 1e-20 ? std::clamp((q - a).dot(d) / d.squaredNorm(), 0.0, 1.0)
                                                               : 0.0;
                            best = std::min(best, (q - a - u * d).norm());
                        }
                    }
            if (best <= exactRadius)
                out.distance[v] = best;
        }
    }
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    for (size_t v = 0; v < out.distance.size(); ++v)
        if (out.distance[v] < 1e100)
            queue.emplace(out.distance[v], int(v));
    while (!queue.empty()) {
        auto [d, u] = queue.top();
        queue.pop();
        if (d > out.distance[u])
            continue;
        for (auto [v, w] : graph[u])
            if (d + w < out.distance[v]) {
                out.distance[v] = d + w;
                queue.emplace(d + w, v);
            }
    }
    return out;
}
static bool surface_point(const Surface &s, const Eigen::Vector2d &xy, Vec3 &point, Vec3 &normal,
                          double &distance, double *scale = nullptr) {
    if (s.section) {
        if (!section_point(*s.section, xy, distance))
            return false;
        point = Vec3(xy.x(), xy.y(), s.section->z);
        normal = Vec3::UnitZ();
        if (scale)
            *scale = 1;
        return true;
    }
    auto test = [&](const Eigen::Vector3i &t) {
        const Vec3 &a = s.mesh.vertices[t[0]];
        Vec3 b = s.mesh.vertices[t[1]], c = s.mesh.vertices[t[2]];
        Eigen::Matrix2d M;
        M.col(0) = (b - a).head<2>();
        M.col(1) = (c - a).head<2>();
        if (std::abs(M.determinant()) < 1e-12)
            return false;
        Eigen::Vector2d w = M.inverse() * (xy - a.head<2>());
        double u = 1 - w.sum();
        if (u < -1e-7 || w.minCoeff() < -1e-7)
            return false;
        point = u * a + w[0] * b + w[1] * c;
        normal = (u * s.normals[t[0]] + w[0] * s.normals[t[1]] + w[1] * s.normals[t[2]]).normalized();
        distance = u * s.distance[t[0]] + w[0] * s.distance[t[1]] + w[1] * s.distance[t[2]];
        if (scale)
            *scale = u * s.heightScale[t[0]] + w[0] * s.heightScale[t[1]] + w[1] * s.heightScale[t[2]];
        return true;
    };
    if (s.grid) {
        const auto &g = *s.grid;
        int x = int(std::floor((xy.x() - g.origin.x()) / g.cell)), y = int(std::floor((xy.y() - g.origin.y()) / g.cell));
        if (x < 0 || y < 0 || x >= g.nx || y >= g.ny)
            return false;
        for (int id : g.cells[size_t(y) * g.nx + x])
            if (test(s.mesh.triangles[id]))
                return true;
        return false;
    }
    for (const auto &t : s.mesh.triangles)
        if (test(t))
            return true;
    return false;
}
static std::vector<Surface> decompose_surface(const Surface &s) {
    std::vector<int> parent(s.mesh.vertices.size());
    for (size_t i = 0; i < parent.size(); ++i)
        parent[i] = int(i);
    std::function<int(int)> root = [&](int i) { return parent[i] == i ? i : parent[i] = root(parent[i]); };
    for (auto face : s.mesh.triangles) {
        parent[root(face[0])] = root(face[1]);
        parent[root(face[1])] = root(face[2]);
    }
    std::map<int, Surface> islands;
    std::map<int, std::map<int, int>> maps;
    for (auto face : s.mesh.triangles) {
        int id = root(face[0]);
        auto &out = islands[id];
        auto &remap = maps[id];
        Eigen::Vector3i tri;
        for (int j = 0; j < 3; ++j) {
            int v = face[j];
            auto it = remap.find(v);
            if (it == remap.end()) {
                int n = int(out.mesh.vertices.size());
                remap[v] = n;
                tri[j] = n;
                out.mesh.vertices.push_back(s.mesh.vertices[v]);
                out.normals.push_back(s.normals[v]);
                out.distance.push_back(s.distance[v]);
                out.heightScale.push_back(s.heightScale[v]);
            } else
                tri[j] = it->second;
        }
        out.mesh.triangles.push_back(tri);
    }
    std::vector<Surface> result;
    for (auto &item : islands) {
        index_surface(item.second);
        result.push_back(std::move(item.second));
    }
    return result;
}
struct CurvePoint {
    Vec3 first, second;
    double heightScale = 1;
};
using Curve = std::vector<CurvePoint>;
static Curve simplify_curve(const Curve &curve, double tolerance, double angularTolerance) {
    if (curve.size() < 3 || tolerance <= 0)
        return curve;
    std::vector<bool> keep(curve.size(), false);
    keep.front() = keep.back() = true;
    std::function<void(size_t, size_t)> visit = [&](size_t first, size_t last) {
        if (last <= first + 1)
            return;
        Vec3 delta = curve[last].first - curve[first].first;
        double worst = -1;
        size_t chosen = first;
        for (size_t i = first + 1; i < last; ++i) {
            double u =
                delta.squaredNorm() > 1e-16
                    ? std::clamp((curve[i].first - curve[first].first).dot(delta) / delta.squaredNorm(), 0.0,
                                 1.0)
                    : 0;
            Vec3 normal = curve[first].second * (1 - u) + curve[last].second * u;
            double error = (curve[i].first - curve[first].first - u * delta).norm() / tolerance;
            double scale = curve[first].heightScale * (1 - u) + curve[last].heightScale * u;
            error = std::max(error, std::abs(curve[i].heightScale - scale) /
                                        std::max(.005, curve[i].heightScale * .02));
            if (normal.norm() < 1e-8 || normal.normalized().dot(curve[i].second) < std::cos(angularTolerance))
                error = std::max(error, 2.0);
            if (error > worst) {
                worst = error;
                chosen = i;
            }
        }
        if (worst > 1) {
            keep[chosen] = true;
            visit(first, chosen);
            visit(chosen, last);
        }
    };
    visit(0, curve.size() - 1);
    Curve out;
    for (size_t i = 0; i < curve.size(); ++i)
        if (keep[i])
            out.push_back(curve[i]);
    return out;
}
static Vec3 nozzle_axis(const Project &p, const Sample &s) {
    return p.machine.rotary_axes() ? s.direction.normalized() : Vec3::UnitZ().eval();
}
// Same-layer travels move directly when short or when they stay over the part's own material
// (Orca's "only retract when crossing perimeters"); others lift by travelLift along the nozzle axis.
static bool travel_needs_lift(const Project &p, const InsideTester &inside, const Vec3 &target, int layer,
                              double width, double height) {
    if (p.path.empty() || p.process.travelLift <= 0)
        return false;
    const auto &from = p.path.back();
    if (from.layer != layer)
        return true;
    double length = (target - from.position).norm();
    if (length <= std::max(2.0, 4 * width))
        return false;
    Vec3 below = nozzle_axis(p, from) * (height / 2);
    int steps = std::max(2, int(std::ceil(length / std::max(.5, width))));
    for (int i = 0; i <= steps; ++i)
        if (!inside.contains(from.position + (target - from.position) * (double(i) / steps) - below))
            return true;
    return false;
}
static void add_curve(Project &p, const InsideTester &inside, const Curve &input, Role role, int layer,
                      int region) {
    auto curve = simplify_curve(input, std::min(p.process.resolution, p.machine.chordTolerance),
                                p.machine.angularTolerance * .5);
    if (curve.size() < 2)
        return;
    double width = extrusion_width(p.process, role, layer), height = extrusion_height(p.process, layer);
    double area = bead_area(width, height), filament = pi * std::pow(p.machine.filamentDiameter / 2, 2);
    Sample start;
    start.position = curve.front().first;
    start.direction = curve.front().second;
    if (!p.machine.rotary_axes()) {
        start.surfaceNormal = start.direction;
        start.direction = Vec3::UnitZ();
    }
    start.role = Role::Travel;
    start.layer = layer;
    start.region = region;
    start.speed = p.process.travelSpeed;
    start.width = width;
    start.height = height;
    if (travel_needs_lift(p, inside, start.position, layer, width, height)) {
        auto lift = p.path.back();
        lift.position += nozzle_axis(p, lift) * p.process.travelLift;
        lift.deltaE = 0;
        lift.role = Role::Travel;
        lift.speed = p.process.travelSpeed;
        p.path.push_back(lift);
        auto above = start;
        above.position += nozzle_axis(p, start) * p.process.travelLift;
        p.path.push_back(above);
    }
    p.path.push_back(start);
    double curveLength = 0;
    for (size_t i = 1; i < curve.size(); ++i)
        curveLength += (curve[i].first - curve[i - 1].first).norm();
    double requestedSpeed = process_speed(p.process, role, layer);
    // Orca's small_perimeter_threshold is a radius: loops shorter than 2*pi*r are slowed.
    if ((role == Role::Wall || role == Role::OuterWall) && p.process.smallWallThreshold > 0 &&
        curveLength < 2 * pi * p.process.smallWallThreshold)
        requestedSpeed = p.process.smallWallAbsoluteSpeed > 0
                             ? std::min(requestedSpeed, p.process.smallWallAbsoluteSpeed)
                             : requestedSpeed * p.process.smallWallSpeed;
    for (size_t i = 1; i < curve.size(); ++i) {
        double length = (curve[i].first - curve[i - 1].first).norm();
        // Segments whose nozzle orientation or non-planar bead height changes need samples; with
        // both constant the move is exact as one linear motion (overhang slowdown keeps samples).
        bool varying = p.process.slowOverhangs || (curve[i].second - curve[i - 1].second).norm() > 1e-9 ||
                       (p.process.nonPlanar3Axis && std::abs(curve[i].heightScale - curve[i - 1].heightScale) > 1e-12);
        int count = varying ? std::max(1, int(std::ceil(length / .7))) : 1;
        for (int j = 1; j <= count; ++j) {
            Sample s;
            s.position = curve[i - 1].first + (curve[i].first - curve[i - 1].first) * double(j) / count;
            s.direction = (curve[i - 1].second + (curve[i].second - curve[i - 1].second) * double(j) / count)
                              .normalized();
            s.role = role;
            s.layer = layer;
            s.region = region;
            s.width = width;
            s.height = height;
            if (!p.machine.rotary_axes()) {
                s.surfaceNormal = s.direction;
                s.direction = Vec3::UnitZ();
                if (p.process.nonPlanar3Axis && role != Role::Support)
                    s.height *= curve[i - 1].heightScale * (1 - double(j) / count) +
                                curve[i].heightScale * double(j) / count;
            }
            s.speed = requestedSpeed;
            if (p.process.slowOverhangs && layer > 0 && role != Role::Support && role != Role::Brim &&
                role != Role::Skirt) {
                int unsupported = 0;
                for (int k = 0; k < 10; ++k) {
                    Vec3 normal = surface_normal(s),
                         side = normal.cross((curve[i].first - curve[i - 1].first).normalized());
                    Vec3 probe = s.position - normal * s.height * 1.25 + side * width * (double(k) / 9 - .5);
                    unsupported += !inside.contains(probe);
                }
                int band = unsupported <= 1   ? 0
                           : unsupported <= 2 ? 1
                           : unsupported <= 5 ? 2
                           : unsupported <= 7 ? 3
                                              : 4;
                if (p.process.overhangSpeeds[band] > 0)
                    s.speed = std::min(s.speed, p.process.overhangSpeeds[band]);
            }
            s.deltaE = (s.position - p.path.back().position).norm() *
                       (!p.machine.rotary_axes() ? bead_area(width, s.height) : area) / filament;
            p.path.push_back(s);
        }
    }
}
// Orca PerimeterGenerator spacing: the outer wall centerline sits half its width inside the
// contour; each further wall is offset by the mean of the neighbouring bead spacings.
static double wall_offset(const ProcessSettings &s, int wall, int layer) {
    double height = extrusion_height(s, layer);
    auto spacing = [&](int i) {
        return bead_spacing(extrusion_width(s, i == 0 ? Role::OuterWall : Role::Wall, layer), height);
    };
    double distance = extrusion_width(s, Role::OuterWall, layer) / 2;
    for (int i = 1; i <= wall; ++i)
        distance += (spacing(i - 1) + spacing(i)) / 2;
    return distance;
}
static double wall_band(const ProcessSettings &s, int layer) {
    int count = effective_walls(s, layer);
    return count > 0 ? wall_offset(s, count - 1, layer) +
                           extrusion_width(s, count == 1 ? Role::OuterWall : Role::Wall, layer) / 2
                     : 0;
}
static bool inside_xy(const Curve &curve, const Vec3 &point) {
    bool inside = false;
    for (size_t i = 0, j = curve.size() - 1; i < curve.size(); j = i++) {
        const auto &a = curve[i].first;
        const auto &b = curve[j].first;
        if ((a.y() > point.y()) != (b.y() > point.y()) &&
            point.x() < (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) + a.x())
            inside = !inside;
    }
    return inside;
}
static void offset_curve(Curve &curve, double amount) {
    if (std::abs(amount) < 1e-10 || curve.size() < 4 ||
        (curve.front().first - curve.back().first).norm() > 1e-6)
        return;
    namespace bg = boost::geometry;
    using Point = bg::model::d2::point_xy<double>;
    using Polygon = bg::model::polygon<Point>;
    auto original = simplify_curve(curve, .001, .001);
    Polygon polygon;
    for (const auto &p : original)
        bg::append(polygon.outer(), Point(p.first.x(), p.first.y()));
    bg::correct(polygon);
    bg::model::multi_polygon<Polygon> result;
    bg::strategy::buffer::distance_symmetric<double> distance(amount);
    bg::strategy::buffer::side_straight side;
    bg::strategy::buffer::join_miter join(4);
    bg::strategy::buffer::end_flat end;
    bg::strategy::buffer::point_circle point(12);
    bg::buffer(polygon, result, distance, side, join, end, point);
    if (result.size() != 1 || result.front().outer().size() < 4 || !result.front().inners().empty())
        throw std::runtime_error(
            "XY offset collapses or splits a contour; reduce compensation or adhesion width");
    Curve output;
    for (const auto &p : result.front().outer()) {
        Eigen::Vector2d xy(bg::get<0>(p), bg::get<1>(p));
        double best = 1e100, scale = 1;
        Vec3 surface = Vec3::Zero(), normal = Vec3::UnitZ();
        for (size_t i = 1; i < original.size(); ++i) {
            Vec3 delta = original[i].first - original[i - 1].first;
            double length = delta.head<2>().squaredNorm();
            double u = length > 1e-16
                           ? std::clamp((xy - original[i - 1].first.head<2>()).dot(delta.head<2>()) / length,
                                        0.0, 1.0)
                           : 0;
            Vec3 candidate = original[i - 1].first + u * delta;
            double error = (xy - candidate.head<2>()).squaredNorm();
            if (error < best) {
                best = error;
                surface = candidate;
                normal = (original[i - 1].second * (1 - u) + original[i].second * u).normalized();
                scale = original[i - 1].heightScale * (1 - u) + original[i].heightScale * u;
            }
        }
        if (std::abs(normal.z()) < .1)
            throw std::runtime_error("XY compensation requires layer normals with a Z component");
        Vec3 displacement(xy.x() - surface.x(), xy.y() - surface.y(), 0);
        displacement.z() = -(normal.x() * displacement.x() + normal.y() * displacement.y()) / normal.z();
        output.push_back({surface + displacement, normal, scale});
    }
    output.pop_back();
    size_t seam = 0;
    double nearest = 1e100;
    for (size_t i = 0; i < output.size(); ++i) {
        double d = (output[i].first - curve.front().first).squaredNorm();
        if (d < nearest) {
            nearest = d;
            seam = i;
        }
    }
    std::rotate(output.begin(), output.begin() + seam, output.end());
    output.push_back(output.front());
    curve = std::move(output);
}

// Point predicate restricting solid infill or concentric surface loops to one surface class.
using SurfaceMask = std::function<bool(const Vec3 &, const Vec3 &)>;
// A surface is planar when every vertex shares one height and one normal (planar Z fields).
static bool planar_surface(const Surface &s, double &z, Vec3 &normal, double &scale) {
    if (s.mesh.vertices.empty())
        return false;
    z = s.mesh.vertices.front().z();
    normal = s.normals.front();
    scale = 0;
    for (size_t i = 0; i < s.mesh.vertices.size(); ++i) {
        if (std::abs(s.mesh.vertices[i].z() - z) > 1e-6 || (s.normals[i] - normal).norm() > 1e-6)
            return false;
        scale += s.heightScale[i] / s.mesh.vertices.size();
    }
    return std::abs(normal.z()) > 1 - 1e-9;
}
// Closed boundary loops of a surface island, chained from edges used by one triangle.
static std::vector<std::vector<Vec3>> boundary_loops(const Surface &s) {
    std::map<std::pair<int, int>, int> counts;
    for (const auto &t : s.mesh.triangles)
        for (int j = 0; j < 3; ++j) {
            int a = t[j], b = t[(j + 1) % 3];
            ++counts[{std::min(a, b), std::max(a, b)}];
        }
    std::map<int, std::vector<int>> adjacency;
    for (const auto &[edge, count] : counts)
        if (count == 1) {
            adjacency[edge.first].push_back(edge.second);
            adjacency[edge.second].push_back(edge.first);
        }
    std::set<std::pair<int, int>> used;
    std::vector<std::vector<Vec3>> loops;
    for (const auto &[start, neighbours] : adjacency)
        for (int first : neighbours) {
            if (used.count({std::min(start, first), std::max(start, first)}))
                continue;
            std::vector<Vec3> loop{s.mesh.vertices[start]};
            int current = first;
            used.insert({std::min(start, first), std::max(start, first)});
            while (current != start) {
                loop.push_back(s.mesh.vertices[current]);
                int next = -1;
                for (int v : adjacency[current])
                    if (!used.count({std::min(current, v), std::max(current, v)})) {
                        next = v;
                        break;
                    }
                if (next < 0)
                    break;
                used.insert({std::min(current, next), std::max(current, next)});
                current = next;
            }
            if (current == start && loop.size() >= 3)
                loops.push_back(std::move(loop));
        }
    return loops;
}
// Polygons with holes from closed XY loops, nested by even-odd containment.
static MultiPolygon2 polygons_from_loops(const std::vector<std::vector<Vec3>> &loops) {
    std::vector<Polygon2> rings;
    std::vector<double> areas;
    std::vector<Point2> probes;
    for (const auto &loop : loops) {
        Polygon2 ring;
        for (const auto &v : loop)
            bg::append(ring.outer(), Point2(v.x(), v.y()));
        bg::correct(ring);
        double area = std::abs(bg::area(ring));
        if (area < 1e-9)
            continue;
        rings.push_back(std::move(ring));
        areas.push_back(area);
        probes.emplace_back(loop.front().x(), loop.front().y());
    }
    std::vector<int> depth(rings.size(), 0), parent(rings.size(), -1);
    for (size_t i = 0; i < rings.size(); ++i)
        for (size_t j = 0; j < rings.size(); ++j)
            if (i != j && areas[j] > areas[i] && bg::within(probes[i], rings[j])) {
                ++depth[i];
                if (parent[i] < 0 || areas[j] < areas[parent[i]])
                    parent[i] = int(j);
            }
    MultiPolygon2 shape;
    std::map<int, size_t> polygonOf;
    for (size_t i = 0; i < rings.size(); ++i)
        if (depth[i] % 2 == 0) {
            polygonOf[int(i)] = shape.size();
            shape.push_back(rings[i]);
        }
    for (size_t i = 0; i < rings.size(); ++i)
        if (depth[i] % 2 == 1 && polygonOf.count(parent[i]))
            shape[polygonOf[parent[i]]].inners().emplace_back(rings[i].outer().begin(), rings[i].outer().end());
    for (auto &polygon : shape)
        bg::correct(polygon);
    return shape;
}
// Exact planar wall loops: the outline offset inward by the target distance, as Orca does
// with Clipper. Returned curves are closed (front == back).
static std::vector<Curve> offset_shape_curves(const MultiPolygon2 &shape, double target, double z,
                                              const Vec3 &normal, double scale) {
    MultiPolygon2 result;
    if (target < 1e-5)
        result = shape;
    else
        bg::buffer(shape, result, bg::strategy::buffer::distance_symmetric<double>(-target),
                   bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_miter(3),
                   bg::strategy::buffer::end_flat(), bg::strategy::buffer::point_circle(12));
    std::vector<Curve> curves;
    auto appendRing = [&](const auto &ring) {
        Curve curve;
        for (const auto &q : ring)
            curve.push_back({Vec3(bg::get<0>(q), bg::get<1>(q), z), normal, scale});
        // Boost rings are clockwise outers / counter-clockwise holes; print the reverse, as Orca does.
        std::reverse(curve.begin(), curve.end());
        if (curve.size() >= 4)
            curves.push_back(std::move(curve));
    };
    for (const auto &polygon : result) {
        appendRing(polygon.outer());
        for (const auto &inner : polygon.inners())
            appendRing(inner);
    }
    return curves;
}
static std::vector<Curve> planar_offset_curves(const Surface &s, double target, double z, const Vec3 &normal,
                                               double scale) {
    return offset_shape_curves(polygons_from_loops(boundary_loops(s)), target, z, normal, scale);
}
// Cross-section of a closed triangle mesh by the plane at height z. Crossing points are keyed
// by the mesh edge they lie on, so segments chain into loops through shared edges without
// any distance tolerance; sliver or slightly self-intersecting CAD tessellation is tolerated.
static MultiPolygon2 mesh_section(const Mesh &mesh, double z) {
    using EdgeKey = std::pair<int, int>;
    std::map<EdgeKey, int> pointOf;
    std::vector<Vec3> points;
    std::vector<std::array<int, 2>> segments;
    auto crossing = [&](int a, int b) {
        EdgeKey key{std::min(a, b), std::max(a, b)};
        auto found = pointOf.find(key);
        if (found != pointOf.end())
            return found->second;
        const Vec3 &p = mesh.vertices[key.first], &q = mesh.vertices[key.second];
        double u = (z - p.z()) / (q.z() - p.z());
        points.push_back(p + (q - p) * u);
        return pointOf[key] = int(points.size()) - 1;
    };
    for (const auto &t : mesh.triangles) {
        // A vertex on the plane counts as above it, so every crossing edge has distinct sides.
        bool above[3];
        for (int j = 0; j < 3; ++j)
            above[j] = mesh.vertices[t[j]].z() >= z;
        if (above[0] == above[1] && above[1] == above[2])
            continue;
        std::vector<int> ends;
        for (int j = 0; j < 3; ++j)
            if (above[j] != above[(j + 1) % 3])
                ends.push_back(crossing(t[j], t[(j + 1) % 3]));
        if (ends.size() == 2 && ends[0] != ends[1])
            segments.push_back({ends[0], ends[1]});
    }
    std::vector<std::vector<int>> adjacency(points.size());
    for (int i = 0; i < int(segments.size()); ++i)
        for (int end : segments[i])
            adjacency[end].push_back(i);
    std::vector<char> used(segments.size(), 0);
    std::vector<std::vector<Vec3>> loops;
    for (int start = 0; start < int(segments.size()); ++start) {
        if (used[start])
            continue;
        used[start] = 1;
        int first = segments[start][0], current = segments[start][1];
        std::vector<Vec3> loop{points[first]};
        while (current != first) {
            loop.push_back(points[current]);
            int next = -1;
            for (int id : adjacency[current])
                if (!used[id]) {
                    next = id;
                    break;
                }
            if (next < 0)
                break;
            used[next] = 1;
            current = segments[next][0] == current ? segments[next][1] : segments[next][0];
        }
        if (current == first && loop.size() >= 3)
            loops.push_back(std::move(loop));
    }
    auto shape = polygons_from_loops(loops);
    if (!bg::is_valid(shape)) {
        // Self-touching tessellation noise: rebuild a valid outline with a closing pair of tiny offsets.
        MultiPolygon2 grown, cleaned;
        bg::buffer(shape, grown, bg::strategy::buffer::distance_symmetric<double>(1e-5),
                   bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_miter(3),
                   bg::strategy::buffer::end_flat(), bg::strategy::buffer::point_circle(12));
        bg::buffer(grown, cleaned, bg::strategy::buffer::distance_symmetric<double>(-1e-5),
                   bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_miter(3),
                   bg::strategy::buffer::end_flat(), bg::strategy::buffer::point_circle(12));
        shape = cleaned;
    }
    return shape;
}
// Planar layer islands sliced at sliceZ and placed at layerZ, the height the nozzle runs at.
static std::vector<Surface> section_islands(const Mesh &mesh, double sliceZ, double layerZ, double reach) {
    std::vector<Surface> islands;
    for (const auto &polygon : mesh_section(mesh, sliceZ)) {
        auto section = std::make_shared<PlanarSection>();
        section->shape = polygon;
        section->z = layerZ;
        section->reach = std::max(reach, 1e-3);
        index_section(*section);
        if (section->segments.size() < 3)
            continue;
        Surface s;
        s.section = std::move(section);
        islands.push_back(std::move(s));
    }
    return islands;
}
static bool surface_empty(const Surface &s) {
    return s.section ? s.section->segments.empty() : s.mesh.empty();
}
static Eigen::AlignedBox3d surface_bounds(const Surface &s) {
    if (!s.section)
        return s.mesh.bounds();
    Eigen::AlignedBox3d b;
    b.extend(Vec3(s.section->bounds.min().x(), s.section->bounds.min().y(), s.section->z));
    b.extend(Vec3(s.section->bounds.max().x(), s.section->bounds.max().y(), s.section->z));
    return b;
}
// Upper bound of the distance from the boundary to any interior point.
static double surface_max_distance(const Surface &s) {
    if (s.section)
        return s.section->bounds.sizes().minCoeff() / 2;
    return s.distance.empty() ? 0 : *std::max_element(s.distance.begin(), s.distance.end());
}
static std::vector<Vec3> outline_points(const Surface &s) {
    std::vector<Vec3> points;
    if (s.section) {
        for (const auto &q : s.section->shape.outer())
            points.emplace_back(bg::get<0>(q), bg::get<1>(q), s.section->z);
        return points;
    }
    for (const auto &loop : boundary_loops(s))
        points.insert(points.end(), loop.begin(), loop.end());
    return points;
}
// Iso-distance lines of the boundary distance field on curved surfaces. Closed loops repeat
// their first point at the end.
static std::vector<Curve> isoline_curves(const Surface &s, double target) {
    Curve ends;
    std::vector<std::pair<int, int>> edges;
    std::map<std::array<long long, 3>, int> ids;
    for (auto t : s.mesh.triangles) {
        Curve segment;
        for (int j = 0; j < 3; ++j) {
            int a = t[j], b = t[(j + 1) % 3];
            double da = s.distance[a] - target, db = s.distance[b] - target;
            if ((da < 0) == (db < 0) || std::abs(da - db) < 1e-12)
                continue;
            double u = da / (da - db);
            segment.push_back({s.mesh.vertices[a] + u * (s.mesh.vertices[b] - s.mesh.vertices[a]),
                               (s.normals[a] + u * (s.normals[b] - s.normals[a])).normalized(),
                               s.heightScale[a] * (1 - u) + s.heightScale[b] * u});
        }
        if (segment.size() != 2)
            continue;
        int index[2];
        for (int j = 0; j < 2; ++j) {
            auto &q = segment[j].first;
            std::array<long long, 3> key = {llround(q.x() * 1e6), llround(q.y() * 1e6), llround(q.z() * 1e6)};
            auto it = ids.find(key);
            if (it == ids.end()) {
                index[j] = int(ends.size());
                ids[key] = index[j];
                ends.push_back(segment[j]);
            } else
                index[j] = it->second;
        }
        edges.emplace_back(index[0], index[1]);
    }
    std::vector<std::vector<int>> adjacency(ends.size());
    for (auto [a, b] : edges) {
        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }
    std::set<std::pair<int, int>> used;
    std::vector<Curve> curves;
    for (auto edge : edges) {
        auto key = std::minmax(edge.first, edge.second);
        if (used.count(key))
            continue;
        Curve curve;
        int first = edge.first, current = first, next = edge.second;
        curve.push_back(ends[first]);
        for (size_t step = 0; step <= edges.size(); ++step) {
            used.insert(std::minmax(current, next));
            curve.push_back(ends[next]);
            current = next;
            if (current == first)
                break;
            next = -1;
            for (int v : adjacency[current])
                if (!used.count(std::minmax(current, v))) {
                    next = v;
                    break;
                }
            if (next < 0)
                break;
        }
        curves.push_back(std::move(curve));
    }
    return curves;
}
static bool closed_curve(const Curve &curve) {
    return curve.size() > 3 && (curve.front().first - curve.back().first).norm() < 1e-6;
}
// Rotate a closed curve (front == back) so it starts at the configured seam.
static void place_seam(const Project &p, Curve &curve, int layer, int wall) {
    curve.pop_back();
    size_t seam = 0;
    double cost = 1e100;
    for (size_t i = 0; i < curve.size(); ++i) {
        double v = 0;
        if (p.process.seam == SeamPosition::LowStress && p.stress.directional) {
            bool covered;
            auto t = p.stress.sample(curve[i].first, &covered);
            v = covered ? t.norm() : 1e100;
        } else if (p.process.seam == SeamPosition::Back)
            v = -curve[i].first.y() + 1e-6 * curve[i].first.x();
        else if (p.process.seam == SeamPosition::Nearest && !p.path.empty())
            v = (curve[i].first - p.path.back().position).squaredNorm();
        else
            v = curve[i].first.x() + 1e-6 * curve[i].first.y();
        if (v < cost) {
            cost = v;
            seam = i;
        }
    }
    if (p.process.seam == SeamPosition::Random)
        seam = (unsigned(layer + 1) * 1664525u + unsigned(wall + 1) * 1013904223u) % curve.size();
    std::rotate(curve.begin(), curve.begin() + seam, curve.end());
    curve.push_back(curve.front());
}
// Split a curve into the runs whose points satisfy the mask.
static std::vector<Curve> masked_runs(const Curve &curve, const SurfaceMask &keep) {
    std::vector<Curve> runs;
    Curve run;
    for (const auto &q : curve) {
        if (keep(q.first, q.second))
            run.push_back(q);
        else if (!run.empty()) {
            if (run.size() > 1)
                runs.push_back(std::move(run));
            run.clear();
        }
    }
    if (run.size() > 1)
        runs.push_back(std::move(run));
    return runs;
}

static void walls(Project &p, const InsideTester &inside, const Surface &s, int layer, int &region,
                  int count = -1, Role forced = Role::Travel, double firstOffset = 0, double spacing = 0,
                  double outside = 0, int boundaryKind = 0, const SurfaceMask &keep = {}) {
    if (count < 0)
        count = effective_walls(p.process, layer);
    double planeZ = 0, planeScale = 1;
    Vec3 planeNormal = Vec3::UnitZ();
    bool planar = planar_surface(s, planeZ, planeNormal, planeScale);
    for (int index = 0; index < count; ++index) {
        int wall = forced != Role::Travel || p.process.outerWallFirst ? index : count - index - 1;
        double target =
            forced == Role::Travel ? wall_offset(p.process, wall, layer) : firstOffset + index * spacing;
        auto curves = s.section ? offset_shape_curves(MultiPolygon2{s.section->shape}, target, s.section->z,
                                                      Vec3::UnitZ(), 1)
                      : planar  ? planar_offset_curves(s, target, planeZ, planeNormal, planeScale)
                                : isoline_curves(s, target);
        for (auto &curve : curves)
            if (closed_curve(curve))
                place_seam(p, curve, layer, wall);
        for (auto &curve : curves) {
            int depth = 0;
            for (const auto &other : curves)
                if (&other != &curve && inside_xy(other, curve.front().first))
                    ++depth;
            if ((boundaryKind == 1 && depth % 2) || (boundaryKind == 2 && !(depth % 2)))
                continue;
            if (forced == Role::Travel && closed_curve(curve)) {
                double amount = depth % 2 ? p.process.holeCompensation : p.process.contourCompensation;
                if (layer == 0)
                    amount += (depth % 2 ? 1 : -1) * p.process.elephantFoot;
                offset_curve(curve, amount);
            }
            if (outside != 0)
                offset_curve(curve, (depth % 2 ? -1 : 1) * outside);
            Role role = forced != Role::Travel ? forced : wall == 0 ? Role::OuterWall : Role::Wall;
            if (keep)
                for (const auto &run : masked_runs(curve, keep))
                    add_curve(p, inside, run, role, layer, region++);
            else
                add_curve(p, inside, curve, role, layer, region++);
        }
    }
}
static void infill(Project &p, const InsideTester &inside, const Surface &surface, int layer, int &region,
                   Role role, const SurfaceMask &keep = {}) {
    bool solid = role != Role::Infill;
    auto allowed = [&](const Vec3 &q, const Vec3 &n) { return !keep || keep(q, n); };
    if ((!solid && p.process.infill <= 0) || surface_empty(surface))
        return;
    double density = role == Role::TopSurface      ? p.process.topDensity
                     : role == Role::BottomSurface ? p.process.bottomDensity
                     : solid                       ? 1
                                                   : p.process.infill;
    if (density <= 0)
        return;
    int families = !solid && p.process.infillPattern == InfillPattern::Grid ? 2 : 1;
    double width = extrusion_width(p.process, role, layer);
    double spacing = bead_spacing(width, extrusion_height(p.process, layer)) / density * families;
    auto pattern = role == Role::TopSurface ? p.process.topPattern : p.process.bottomPattern;
    if (solid && pattern == SurfacePattern::Concentric) {
        double start = wall_band(p.process, layer) + width / 2;
        double maximum = surface_max_distance(surface);
        walls(p, inside, surface, layer, region, std::max(0, int(std::ceil((maximum - start) / spacing))), role,
              start, spacing, 0, 0, keep);
        return;
    }
    auto b = surface_bounds(surface);
    std::vector<Curve> curves;
    for (int family = 0; family < families; ++family) {
        // Covered infill points hashed in cells of one line spacing; queries reach at most .8 spacing.
        std::map<std::pair<long long, long long>, std::vector<Eigen::Vector2d>> covered;
        auto cellOf = [&](const Eigen::Vector2d &q) {
            return std::make_pair((long long)std::floor(q.x() / spacing), (long long)std::floor(q.y() / spacing));
        };
        auto cover = [&](const Eigen::Vector2d &q) { covered[cellOf(q)].push_back(q); };
        auto near = [&](Eigen::Vector2d q, double distance) {
            auto c = cellOf(q);
            for (long long x = -1; x <= 1; ++x)
                for (long long y = -1; y <= 1; ++y) {
                    auto found = covered.find({c.first + x, c.second + y});
                    if (found != covered.end())
                        for (auto &v : found->second)
                            if ((v - q).squaredNorm() < distance * distance)
                                return true;
                }
            return false;
        };
        for (double y = b.min().y(); y <= b.max().y(); y += spacing)
            for (double x = b.min().x(); x <= b.max().x(); x += spacing) {
                Eigen::Vector2d seed(x, y);
                Vec3 point, n;
                double d, scale;
                if (near(seed, .8 * spacing) || !surface_point(surface, seed, point, n, d, &scale) ||
                    d < wall_band(p.process, layer) || !allowed(point, n))
                    continue;
                Curve halves[2];
                for (int sign = 0; sign < 2; ++sign) {
                    Eigen::Vector2d xy = seed;
                    double angle =
                        p.process.infillAngle + (families == 2 ? family * pi / 2 : (layer % 2) * pi / 2);
                    Vec3 previous(std::cos(angle), std::sin(angle), 0);
                    previous *= sign ? 1 : -1;
                    for (int step = 0; step < 1000; ++step) {
                        if (!surface_point(surface, xy, point, n, d, &scale) ||
                            d < wall_band(p.process, layer) || !allowed(point, n))
                            break;
                        if (step > 2 && near(xy, .7 * spacing))
                            break;
                        halves[sign].push_back({point, n, scale});
                        Vec3 direction = previous;
                        if (!solid && p.process.infillPattern == InfillPattern::StressAligned &&
                            p.stress.directional) {
                            bool valid;
                            auto tensor = p.stress.sample(point, &valid);
                            double confidence;
                            if (valid) {
                                Vec3 principal = principal_tensile_direction(tensor, &confidence);
                                if (confidence > .1) {
                                    direction = principal - n * n.dot(principal);
                                    if (direction.norm() > 1e-6) {
                                        direction.normalize();
                                        if (direction.dot(previous) < 0)
                                            direction = -direction;
                                    } else
                                        direction = previous;
                                }
                            }
                        }
                        direction -= n * n.dot(direction);
                        if (direction.head<2>().norm() < 1e-6)
                            break;
                        previous = direction.normalized();
                        xy += previous.head<2>().normalized() * std::min(.6, width);
                    }
                }
                Curve curve;
                for (auto it = halves[0].rbegin(); it != halves[0].rend(); ++it)
                    curve.push_back(*it);
                if (!halves[1].empty())
                    curve.insert(curve.end(), halves[1].begin() + 1, halves[1].end());
                if (curve.size() > 1) {
                    for (auto &q : curve)
                        cover(q.first.head<2>());
                    curves.push_back(std::move(curve));
                }
            }
    }
    if (solid && pattern == SurfacePattern::Monotonic) {
        double angle = p.process.infillAngle + (layer % 2) * pi / 2;
        Vec3 direction(std::cos(angle), std::sin(angle), 0), side(-direction.y(), direction.x(), 0);
        for (auto &curve : curves)
            if ((curve.back().first - curve.front().first).dot(direction) < 0)
                std::reverse(curve.begin(), curve.end());
        std::stable_sort(curves.begin(), curves.end(), [&](const Curve &a, const Curve &b) {
            return a.front().first.dot(side) < b.front().first.dot(side);
        });
    }
    for (const auto &curve : curves)
        add_curve(p, inside, curve, role, layer, region++);
}
static void support_columns(Project &p, const InsideTester &inside) {
    if (!p.process.supports || p.process.supportMode == SupportMode::None) {
        p.diagnostics << "Support generation disabled; unsupported deposition is still checked.";
        return;
    }
    double width = extrusion_width(p.process, Role::Support, 1);
    double pitch = width + p.process.supportSpacing;
    if (p.process.supportMode == SupportMode::Reduced)
        pitch *= p.process.supportReduction;
    std::map<std::pair<int, int>, double> columns;
    // A bead rests on the previous layer along its layer normal (vertical for planar layers),
    // as check_deposition assumes; probing straight down on tilted layers flags beads whose
    // material below them exists.
    auto probe = [&](const Vec3 &position, const Vec3 &normal, double height) {
        Vec3 below = position - normal * (1.5 * p.process.layerHeight);
        if (position.z() <= 2 * p.process.layerHeight || inside.contains(below))
            return;
        if (p.process.supportAngle > 0) {
            double reach = 1.5 * p.process.layerHeight / std::tan(p.process.supportAngle);
            Vec3 u = normal.unitOrthogonal(), v = normal.cross(u);
            for (const Vec3 &axis : {u, v, (-u).eval(), (-v).eval()})
                if (inside.contains(below + axis * reach))
                    return;
        }
        auto key = std::make_pair(int(std::round(position.x() / pitch)), int(std::round(position.y() / pitch)));
        auto found = columns.find(key);
        double top = position.z() - height;
        if (found == columns.end())
            columns[key] = top;
        else
            found->second = std::max(found->second, top);
    };
    // Probe along each bead at half the column pitch: a straight move is a single path sample,
    // and checking endpoints alone misses overhangs between them.
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1], &b = p.path[i];
        if (b.deltaE <= 0 || b.role == Role::Brim || b.role == Role::Skirt)
            continue;
        int steps = std::max(1, int(std::ceil((b.position - a.position).norm() / (pitch / 2))));
        Vec3 normal = surface_normal(b).normalized();
        if (!normal.allFinite() || normal.z() <= 0)
            normal = Vec3::UnitZ();
        for (int k = 1; k <= steps; ++k)
            probe(a.position + (b.position - a.position) * (double(k) / steps), normal, b.height);
    }
    if (columns.empty())
        return;
    if (p.process.supportBedOnly)
        for (auto it = columns.begin(); it != columns.end();) {
            bool blocked = false;
            for (double z = p.process.layerHeight; z < it->second - p.process.layerHeight;
                 z += p.process.layerHeight)
                if (inside.contains(Vec3(it->first.first * pitch, it->first.second * pitch, z))) {
                    blocked = true;
                    break;
                }
            if (blocked)
                it = columns.erase(it);
            else
                ++it;
        }
    if (p.process.removeSmallOverhangs) {
        auto snapshot = columns;
        for (auto it = columns.begin(); it != columns.end();) {
            bool neighbor = false;
            for (auto offset :
                 {std::make_pair(1, 0), std::make_pair(-1, 0), std::make_pair(0, 1), std::make_pair(0, -1)})
                neighbor |=
                    snapshot.count({it->first.first + offset.first, it->first.second + offset.second}) > 0;
            if (!neighbor)
                it = columns.erase(it);
            else
                ++it;
        }
    }
    auto original = std::move(p.path);
    p.path.clear();
    double maxZ = 0;
    for (auto &[k, z] : columns)
        maxZ = std::max(maxZ, z);
    int region = -1;
    int supportLayer = 0;
    // Each layer joins neighbouring columns into continuous lines, alternating X and Y rows,
    // instead of depositing an isolated dash per column.
    for (double z = extrusion_height(p.process, 0); z <= maxZ + 1e-8;
         z += p.process.layerHeight, ++supportLayer) {
        bool alongX = supportLayer % 2 == 0;
        Vec3 axis = alongX ? Vec3::UnitX().eval() : Vec3::UnitY().eval();
        auto center = [&](int row, int along) {
            return alongX ? Vec3(along * pitch, row * pitch, z) : Vec3(row * pitch, along * pitch, z);
        };
        std::map<int, std::vector<int>> rows;
        for (auto &[key, top] : columns)
            if (z <= top) {
                int row = alongX ? key.second : key.first, along = alongX ? key.first : key.second;
                if (!inside.contains(center(row, along)))
                    rows[row].push_back(along);
            }
        for (auto &[row, positions] : rows) {
            std::sort(positions.begin(), positions.end());
            for (size_t i = 0; i < positions.size();) {
                size_t j = i;
                while (j + 1 < positions.size() && positions[j + 1] == positions[j] + 1 &&
                       !inside.contains((center(row, positions[j]) + center(row, positions[j + 1])) / 2))
                    ++j;
                add_curve(p, inside,
                          {{center(row, positions[i]) - axis * (width / 3), Vec3::UnitZ()},
                           {center(row, positions[j]) + axis * (width / 3), Vec3::UnitZ()}},
                          Role::Support, supportLayer, region--);
                i = j + 1;
            }
        }
    }
    // Schedule deposited support and part curves in layer order. Future support is never an obstacle.
    struct Block {
        int layer;
        bool support;
        std::vector<Sample> samples;
    };
    std::vector<Block> blocks;
    auto collect = [&](const std::vector<Sample> &path, bool support) {
        for (size_t i = 1; i < path.size();) {
            if (path[i].deltaE <= 0) {
                ++i;
                continue;
            }
            Block block{path[i].layer, support, {path[i - 1]}};
            int region = path[i].region;
            while (i < path.size() && path[i].deltaE > 0 && path[i].region == region)
                block.samples.push_back(path[i++]);
            blocks.push_back(std::move(block));
        }
    };
    collect(p.path, true);
    collect(original, false);
    std::stable_sort(blocks.begin(), blocks.end(), [](const Block &a, const Block &b) {
        return a.layer != b.layer ? a.layer < b.layer : a.support > b.support;
    });
    p.path.clear();
    for (const auto &block : blocks) {
        auto entry = block.samples.front();
        entry.deltaE = 0;
        entry.role = Role::Travel;
        entry.layer = block.layer;
        entry.speed = p.process.travelSpeed;
        if (travel_needs_lift(p, inside, entry.position, block.layer, block.samples.back().width,
                              block.samples.back().height)) {
            auto lift = p.path.back();
            lift.position += nozzle_axis(p, lift) * p.process.travelLift;
            lift.role = Role::Travel;
            lift.deltaE = 0;
            lift.speed = p.process.travelSpeed;
            lift.layer = block.layer;
            p.path.push_back(lift);
            auto above = entry;
            above.position += nozzle_axis(p, entry) * p.process.travelLift;
            p.path.push_back(above);
        }
        p.path.push_back(entry);
        p.path.insert(p.path.end(), block.samples.begin() + 1, block.samples.end());
    }
    p.diagnostics << QString("Generated %1 support columns (%2, spacing %3 mm); deposition and collision "
                             "checks remain required.")
                         .arg(columns.size())
                         .arg(p.process.supportMode == SupportMode::Reduced ? "reduced" : "normal")
                         .arg(pitch);
}
void regenerate_joints(Project &p) {
    JointState previous;
    for (size_t i = 0; i < p.path.size(); ++i) {
        auto &s = p.path[i];
        if (!p.machine.rotary_axes()) {
            if (!s.surfaceNormal)
                s.surfaceNormal = s.direction;
            s.direction = Vec3::UnitZ();
        }
        if (p.process.profileNormal && s.role != Role::Support && s.region >= 0)
            s.direction = profile_layer(p, s.layer).normal;
        double uprightCone = std::min(p.process.directionCone * .1, p.machine.angularTolerance * .25);
        if (!p.process.profileNormal && s.direction.dot(Vec3::UnitZ()) >= std::cos(uprightCone))
            s.direction = Vec3::UnitZ();
        auto solutions =
            inverse_kinematics(p.machine, s.position, s.direction, previous, p.active_toolhead());
        if (solutions.empty())
            throw std::runtime_error("No reachable machine pose at movement " + std::to_string(i));
        s.joint = solutions.front();
        previous = s.joint;
    }
    refine_joint_motion(p);
}
void refine_joint_motion(Project &p) {
    // Resample the true joint-interpolated TCP curve before collision checking or export.
    std::vector<Sample> refined;
    if (p.path.empty())
        return;
    refined.push_back(p.path.front());
    for (size_t i = 1; i < p.path.size(); ++i) {
        const auto &a = p.path[i - 1];
        const auto &b = p.path[i];
        int count = 1;
        for (int attempt = 0; attempt < 16; ++attempt) {
            bool good = true;
            for (int j = 0; j < count; ++j) {
                double u = (j + .5) / count;
                JointState q = interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u);
                Vec3 left = part_position(p.machine, interpolate_motion(p.machine, p.active_toolhead(),
                                                                        a.joint, b.joint, double(j) / count));
                Vec3 right =
                    part_position(p.machine, interpolate_motion(p.machine, p.active_toolhead(), a.joint,
                                                                b.joint, double(j + 1) / count));
                Vec3 intended = (left + right) / 2;
                Vec3 actual = part_position(p.machine, q);
                double turn =
                    std::max(std::abs(b.joint.a - a.joint.a), std::abs(b.joint.b - a.joint.b)) / count;
                if ((actual - intended).norm() > p.machine.chordTolerance ||
                    turn > p.machine.angularTolerance) {
                    good = false;
                    break;
                }
            }
            if (good)
                break;
            count *= 2;
            if (count > 32768)
                throw std::runtime_error("Motion interpolation budget exhausted");
        }
        for (int j = 1; j <= count; ++j) {
            Sample s = b;
            double u = double(j) / count;
            s.joint = interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u);
            s.position = part_position(p.machine, s.joint);
            s.direction = part_direction(p.machine, p.active_toolhead(), s.joint);
            if (!p.machine.rotary_axes()) {
                s.surfaceNormal = (surface_normal(a) * (1 - u) + surface_normal(b) * u).normalized();
                if (b.deltaE > 0 && a.deltaE > 0 && a.layer == b.layer)
                    s.height = a.height * (1 - u) + b.height * u;
            }
            // Use this sample's (possibly interpolated) bead height, matching check_deposition.
            s.deltaE = b.deltaE > 0
                           ? (s.position - refined.back().position).norm() * bead_area(s.width, s.height) /
                                 (pi * std::pow(p.machine.filamentDiameter / 2, 2))
                           : (j == count ? b.deltaE : 0);
            refined.push_back(s);
        }
    }
    p.path = std::move(refined);
    time_path(p);
}
void check_deposition(Project &p) {
    p.deposition = {};
    p.deposition.checked = true;
    p.depositionValid = true;
    struct Deposit {
        Vec3 point, normal;
        int region;
    };
    using Key = std::array<int, 3>;
    std::map<Key, std::vector<Deposit>> grid;
    // Cells match the support search radius (lateral reach + 2 bead heights) so each query
    // visits 3x3x3 cells instead of thousands of bead-sized cells.
    double pitch = std::max({p.process.width * .8, p.process.maximumBridge / 2, .2}) + 2 * p.process.layerHeight;
    auto key = [&](const Vec3 &v) {
        return Key{int(std::floor(v.x() / pitch)), int(std::floor(v.y() / pitch)),
                   int(std::floor(v.z() / pitch))};
    };
    auto visit = [&](const Vec3 &point, double radius, auto task) {
        Key lo = key(point - Vec3::Constant(radius)), hi = key(point + Vec3::Constant(radius));
        for (int x = lo[0]; x <= hi[0]; ++x)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int z = lo[2]; z <= hi[2]; ++z) {
                    auto found = grid.find({x, y, z});
                    if (found != grid.end())
                        for (auto &d : found->second)
                            if (task(d))
                                return true;
                }
        return false;
    };
    std::set<std::pair<int, int>> dependencies;
    for (size_t i = 1; i < p.path.size(); ++i) {
        auto &a = p.path[i - 1];
        auto &b = p.path[i];
        if (b.deltaE <= 0 || b.role == Role::Retract)
            continue;
        bool valid = true;
        double length = (b.position - a.position).norm();
        int count = std::max(1, int(std::ceil(length / std::max(.05, b.width * .4))));
        double expected =
            length * bead_area(b.width, b.height) / (pi * std::pow(p.machine.filamentDiameter / 2, 2));
        if (std::abs(expected - b.deltaE) > std::max(1e-7, expected * .03))
            valid = false;
        for (int j = 0; j < count; ++j) {
            double u = (j + .5) / count;
            JointState q = interpolate_motion(p.machine, p.active_toolhead(), a.joint, b.joint, u);
            Vec3 point = part_position(p.machine, q),
                 n = p.machine.rotary_axes()
                         ? part_direction(p.machine, p.active_toolhead(), q)
                         : (surface_normal(a) * (1 - u) + surface_normal(b) * u).normalized();
            Vec3 antiGravity = workpiece_pose(p.machine, q).linear().transpose() * Vec3::UnitZ();
            if (n.dot(antiGravity) < std::cos(p.process.maximumTilt) - 1e-6)
                valid = false;
            bool supported = false;
            for (auto &f : p.fixtures)
                if (f.movesWithBed && f.name.contains("bed", Qt::CaseInsensitive)) {
                    auto bounds = f.mesh.bounds();
                    Vec3 base = point - n * b.height;
                    if (base.x() >= bounds.min().x() && base.x() <= bounds.max().x() &&
                        base.y() >= bounds.min().y() && base.y() <= bounds.max().y() &&
                        std::abs(base.z() - bounds.max().z()) <= b.height * .25 + 1e-5)
                        supported = true;
                }
            double lateral = std::max(b.width * .8, p.process.maximumBridge / 2);
            if (!supported)
                supported = visit(point, lateral + 2 * b.height, [&](const Deposit &d) {
                    Vec3 delta = point - d.point;
                    double h = delta.dot(n);
                    if (h < b.height * .25 || h > b.height * 1.8 || (delta - n * h).norm() > lateral)
                        return false;
                    if (d.region != b.region)
                        dependencies.emplace(d.region, b.region);
                    return true;
                });
            if (!supported)
                valid = false;
            grid[key(point)].push_back({point, n, b.region});
        }
        if (!valid)
            p.deposition.unsupportedMoves.push_back(int(i));
    }
    p.deposition.dependencies.assign(dependencies.begin(), dependencies.end());
    size_t covered = 0;
    double reach = p.process.infill > 0
                       ? bead_spacing(p.process.width, p.process.layerHeight) / p.process.infill * .8 +
                             p.process.layerHeight
                       : p.part.bounds().sizes().norm();
    std::vector<Vec3> samples;
    for (auto &cell : p.volume.cells) {
        Vec3 center = Vec3::Zero();
        for (int j = 0; j < 4; ++j)
            center += p.volume.vertices[cell.nodes[j]] / 4;
        samples.push_back(center);
    }
    if (samples.empty() && !p.part.empty()) {
        // No volume mesh (planar slicing of a surface the mesher rejected): sample a lattice inside the part.
        InsideTester inside(p.part);
        auto box = p.part.bounds();
        double step = std::max(std::cbrt(std::max(box.volume(), 1e-9) / 20000), 1e-3);
        for (double x = box.min().x() + step / 2; x < box.max().x(); x += step)
            for (double y = box.min().y() + step / 2; y < box.max().y(); y += step)
                for (double z = box.min().z() + step / 2; z < box.max().z(); z += step)
                    if (inside.contains(Vec3(x, y, z)))
                        samples.emplace_back(x, y, z);
    }
    for (const auto &center : samples)
        if (visit(center, reach, [&](const Deposit &d) { return (d.point - center).norm() <= reach; }))
            ++covered;
    p.deposition.coverage = samples.empty() ? 1 : double(covered) / samples.size();
    p.depositionValid = p.deposition.unsupportedMoves.empty() && p.deposition.coverage >= .95;
    p.diagnostics << QString("Deposition support / gravity check: %1 flagged movements, %2 dependency edges, "
                             "%3% sampled volume coverage.")
                         .arg(p.deposition.unsupportedMoves.size())
                         .arg(dependencies.size())
                         .arg(p.deposition.coverage * 100, 0, 'f', 1);
}
void slice_project(Project &p, std::atomic_bool *cancel, std::function<void(double)> progress) {
    if (p.plateEnabled) {
        slice_plate(p, cancel, progress);
        return;
    }
    p.invalidate();
    p.path.clear();
    p.diagnostics.clear();
    if (p.part.empty())
        throw std::runtime_error("Import a printable part first");
    validate_process(p.process);
    if ((p.process.nonPlanar3Axis && p.machine.rotary_axes()) ||
        (p.process.profileNormal && !p.machine.rotary_axes()))
        throw std::runtime_error("Select a compatible machine layout for the requested slicing mode");
    if (p.active_toolhead().components.empty())
        throw std::runtime_error("Import a toolhead STEP or enable the dummy toolhead option");
    if (p.useDummyToolhead)
        p.diagnostics << "Dummy toolhead selected: collision results cover synthetic geometry only.";
    // Pure-height fields give flat layers. Like Orca, those are cut directly from the surface
    // mesh at mid-layer and placed at the layer top where the nozzle runs; only curved fields
    // need the tetrahedral volume.
    bool planarField = !p.process.profileNormal && !p.process.nonPlanar3Axis &&
                       (!p.machine.rotary_axes() || !p.process.curved || !p.stress.directional);
    if (p.volume.cells.empty()) {
        try {
            p.volume = tetrahedralize(p.part, std::max(.05, p.part.bounds().sizes().prod() / 1500));
        } catch (const std::exception &e) {
            if (!planarField)
                throw;
            // Planar layers do not need the volume (deposition coverage then samples the part
            // directly), so CAD tessellation the tetrahedral mesher rejects still slices.
            p.diagnostics << QString("Volume meshing failed (%1); planar layers are cut from the surface mesh.")
                                 .arg(QString::fromUtf8(e.what()));
        }
    }
    if (p.process.profileNormal)
        validate_profile_guide(p.profileGuide);
    InsideTester inside(p.part);
    auto field = p.process.profileNormal || planarField ? std::vector<double>{} : scalar_field(p);
    double low = p.process.profileNormal ? 0
                 : planarField          ? p.part.bounds().min().z()
                                        : *std::min_element(field.begin(), field.end()),
           high = p.process.profileNormal ? profile_length(p.profileGuide)
                  : planarField           ? p.part.bounds().max().z()
                                          : *std::max_element(field.begin(), field.end());
    int layer = 0, region = 0;
    double previousLevel = low;
    for (double level = p.process.preciseZ ? std::min(high, low + extrusion_height(p.process, 0))
                                           : low + extrusion_height(p.process, 0);
         level <= high + 1e-7; level += p.process.layerHeight, ++layer) {
        if (level > high)
            break;
        if (cancel && cancel->load())
            throw std::runtime_error("Slicing cancelled");
        double thickness = layer > 0 && level - previousLevel > 1e-9
                               ? std::min(extrusion_height(p.process, layer), level - previousLevel)
                               : extrusion_height(p.process, layer);
        double isoLevel =
            planarField ? std::min(level - thickness / 2, high - 1e-7) : std::min(level, high - 1e-7);
        ProfileStation section;
        if (p.process.profileNormal) {
            section = profile_section(p.profileGuide, isoLevel);
            if (section.normal.z() < std::cos(p.process.maximumTilt) - 1e-8)
                throw std::runtime_error("Profile normal exceeds the permitted deposition tilt");
            field.clear();
            for (const auto &v : p.volume.vertices)
                field.push_back((v - section.center).dot(section.normal));
            isoLevel = 0;
        }
        double exactRadius = wall_band(p.process, layer) + 4 * p.process.width +
                             std::abs(p.process.contourCompensation) + std::abs(p.process.holeCompensation) + 1;
        std::vector<Surface> islands;
        if (planarField)
            islands = section_islands(p.part, isoLevel, std::min(level, high), exactRadius);
        else {
            auto surface =
                iso_surface(p.volume, field, isoLevel, p.compute, p.computation, cancel, exactRadius);
            if (surface.mesh.empty() && p.process.profileNormal)
                throw std::runtime_error("Profile plane misses the model; correct the centerline guide");
            islands = decompose_surface(surface);
        }
        if (islands.empty())
            continue;
        size_t begin = p.path.size();
        if (p.process.profileNormal) {
            int chosen = -1;
            double best = 1e100;
            for (size_t i = 0; i < islands.size(); ++i) {
                auto bounds = islands[i].mesh.bounds();
                if (bounds.exteriorDistance(section.center) > p.machine.chordTolerance)
                    continue;
                double score = (bounds.center() - section.center).squaredNorm();
                if (score < best) {
                    best = score;
                    chosen = int(i);
                }
            }
            if (chosen < 0)
                throw std::runtime_error("Profile center is outside every cross-section; correct the guide");
            auto selected = std::move(islands[chosen]);
            islands.clear();
            islands.push_back(std::move(selected));
        }
        double nominalSpacing = bead_spacing(extrusion_width(p.process, Role::Brim, layer),
                                             extrusion_height(p.process, layer));
        // One skirt around everything printed on the layer (Orca uses the convex hull).
        if (layer < p.process.skirtHeight && p.process.skirtLoops > 0) {
            bg::model::multi_point<Point2> points;
            double z = 1e100;
            for (const auto &island : islands)
                for (const auto &v : outline_points(island)) {
                    bg::append(points, Point2(v.x(), v.y()));
                    z = std::min(z, v.z());
                }
            Polygon2 hull;
            bg::convex_hull(points, hull);
            for (int i = 0; i < p.process.skirtLoops && bg::area(hull) != 0; ++i) {
                MultiPolygon2 ring;
                bg::buffer(hull, ring,
                           bg::strategy::buffer::distance_symmetric<double>(p.process.skirtDistance +
                                                                            (i + .5) * nominalSpacing),
                           bg::strategy::buffer::side_straight(), bg::strategy::buffer::join_round(72),
                           bg::strategy::buffer::end_flat(), bg::strategy::buffer::point_circle(72));
                for (const auto &polygon : ring) {
                    Curve curve;
                    for (const auto &q : polygon.outer())
                        curve.push_back({Vec3(bg::get<0>(q), bg::get<1>(q), z), Vec3::UnitZ()});
                    std::reverse(curve.begin(), curve.end());
                    add_curve(p, inside, curve, Role::Skirt, layer, region++);
                }
            }
        }
        for (auto &island : islands) {
            bool autoBrim = p.process.brimType == 1 &&
                            p.part.bounds().sizes().z() >
                                5 * std::min(p.part.bounds().sizes().x(), p.part.bounds().sizes().y());
            if (layer == 0 && (p.process.brimType > 1 || autoBrim)) {
                int count = int(std::ceil(p.process.brimWidth / nominalSpacing));
                int kind = p.process.brimType == 3 ? 2 : p.process.brimType == 4 ? 0 : 1;
                for (int i = 0; i < count; ++i)
                    walls(p, inside, island, layer, region, 1, Role::Brim, 1e-6, 0,
                          p.process.brimGap + (i + .5) * nominalSpacing, kind);
            }
            int topCount = std::max(p.process.topShellLayers,
                                    int(std::ceil(p.process.topThickness / p.process.layerHeight)));
            int bottomCount = std::max(p.process.bottomShellLayers,
                                       int(std::ceil(p.process.bottomThickness / p.process.layerHeight)));
            // Classify each infill point by probing the solid above and below it, so top and
            // bottom shells follow every exposed surface (shelves, overhangs), not only the
            // part's global top and bottom. Probes sit at the middle of neighbouring layers.
            double h = p.process.layerHeight;
            auto category = [&](const Vec3 &q, const Vec3 &normal) {
                if (layer == 0)
                    return bottomCount > 0 ? Role::BottomSurface : Role::Infill;
                for (int k = 1; k <= topCount; ++k)
                    if (!inside.contains(q + normal * ((k - .5) * h)))
                        return k == 1 ? Role::TopSurface : Role::SolidInfill;
                for (int k = 1; k <= bottomCount; ++k)
                    if (!inside.contains(q - normal * ((k + .5) * h)))
                        return k == 1 ? Role::BottomSurface : Role::SolidInfill;
                return Role::Infill;
            };
            auto fill = [&] {
                if (layer > 0 && topCount == 0 && bottomCount == 0) {
                    infill(p, inside, island, layer, region, Role::Infill);
                    return;
                }
                for (Role role : {Role::TopSurface, Role::BottomSurface, Role::SolidInfill, Role::Infill})
                    infill(p, inside, island, layer, region, role,
                           [&, role](const Vec3 &q, const Vec3 &normal) { return category(q, normal) == role; });
            };
            if (p.process.infillFirst)
                fill();
            walls(p, inside, island, layer, region);
            if (!p.process.infillFirst)
                fill();
        }
        if (p.process.preciseZ && level - previousLevel < extrusion_height(p.process, layer) - 1e-7) {
            double actual = level - previousLevel;
            for (size_t i = begin; i < p.path.size(); ++i) {
                auto &s = p.path[i];
                double next = p.process.nonPlanar3Axis
                                  ? s.height * actual / extrusion_height(p.process, layer)
                                  : actual;
                if (s.deltaE > 0)
                    s.deltaE *= bead_area(s.width, next) / bead_area(s.width, s.height);
                s.height = next;
            }
        }
        previousLevel = level;
        if (progress)
            progress((level - low) / (high - low));
        if (level >= high - 1e-7) {
            ++layer;
            break;
        }
        if (p.process.preciseZ && level + p.process.layerHeight > high)
            level = high - p.process.layerHeight;
    }
    if (p.path.size() < 2)
        throw std::runtime_error("No printable paths: increase meshing detail or reduce extrusion width");
    support_columns(p, inside);
    p.pathProcessFingerprint = process_fingerprint(p.process);
    regenerate_joints(p);
    apply_printer_path(p);
    if (p.process.profileNormal && profile_motion_violation(p) >= 0)
        throw std::runtime_error("Generated nozzle motion is not perpendicular to the profile");
    if (!p.machine.rotary_axes() && cartesian_motion_violation(p) >= 0)
        throw std::runtime_error("Three-axis paths exceed the fixed-nozzle or non-planar slope constraints");
    check_deposition(p);
    p.diagnostics << QString("%1 layers, %2 independently scheduled curves, %3 coordinated movements.")
                         .arg(layer)
                         .arg(region)
                         .arg(p.path.size());
    p.diagnostics
        << (p.process.nonPlanar3Axis ? "Standard XYZ conformal layers retain a planar base, fixed upright "
                                       "nozzle and local field-based bead spacing. Slope, deposition and "
                                       "complete toolhead clearance checks remain required."
            : p.process.profileNormal
                ? "Profile-normal slicing follows explicit cross-section planes in centerline arc order. "
                  "Extrusion orientation is locked; deposition and collision checks remain required."
                : "Research scalar-field solver uses stress-weighted diffusion with bounded spacing; "
                  "material behavior and strength require calibration.");
}
double stress_alignment(const Project &p) {
    double sum = 0, weight = 0;
    for (size_t i = 1; i < p.path.size(); ++i)
        if (p.path[i].deltaE > 0 && p.path[i].role != Role::Support) {
            Vec3 delta = p.path[i].position - p.path[i - 1].position;
            if (delta.norm() < 1e-8)
                continue;
            bool covered;
            auto tensor = p.stress.sample(p.path[i].position, &covered);
            if (!covered)
                continue;
            double confidence;
            Vec3 direction = principal_tensile_direction(tensor, &confidence);
            double w = delta.norm() * confidence;
            sum += std::abs(direction.dot(delta.normalized())) * w;
            weight += w;
        }
    return weight > 0 ? sum / weight : 0;
}
} // namespace fiveaxis
