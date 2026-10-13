#include "slicing.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include "geometry.h"
#include "stress.h"
#include "process.h"
#include <Eigen/LU>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

namespace fiveaxis {
struct Surface {
    Mesh mesh;
    std::vector<Vec3> normals;
    std::vector<double> distance;
};
static std::vector<double> scalar_field(const Project &p) {
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
    for (auto &cell : mesh.cells) {
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
        Eigen::Matrix<double, 3, 4> gradient = M.inverse().bottomRows<3>();
        bool coverage;
        Mat3 tensor = p.stress.sample(center, &coverage);
        if (!coverage)
            throw std::runtime_error("FEM field does not cover the sliced volume");
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
static Surface iso_surface(const VolumeMesh &mesh, const std::vector<double> &values, double level) {
    Surface out;
    std::map<std::array<long long, 3>, int> ids;
    const int edges[][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (auto &t : mesh.cells) {
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
            } else {
                id = found->second;
                out.normals[id] += normal;
            }
            index.push_back(id);
        }
        for (size_t j = 1; j + 1 < index.size(); ++j)
            out.mesh.triangles.emplace_back(index[0], index[j], index[j + 1]);
    }
    for (auto &n : out.normals)
        n.normalize();
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
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    for (auto &[edge, count] : counts)
        if (count == 1)
            for (int v : {edge.first, edge.second}) {
                out.distance[v] = 0;
                queue.emplace(0, v);
            }
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
                          double &distance) {
    for (auto t : s.mesh.triangles) {
        const Vec3 &a = s.mesh.vertices[t[0]];
        Vec3 b = s.mesh.vertices[t[1]], c = s.mesh.vertices[t[2]];
        Eigen::Matrix2d M;
        M.col(0) = (b - a).head<2>();
        M.col(1) = (c - a).head<2>();
        if (std::abs(M.determinant()) < 1e-12)
            continue;
        Eigen::Vector2d w = M.inverse() * (xy - a.head<2>());
        double u = 1 - w.sum();
        if (u < -1e-7 || w.minCoeff() < -1e-7)
            continue;
        point = u * a + w[0] * b + w[1] * c;
        normal = (u * s.normals[t[0]] + w[0] * s.normals[t[1]] + w[1] * s.normals[t[2]]).normalized();
        distance = u * s.distance[t[0]] + w[0] * s.distance[t[1]] + w[1] * s.distance[t[2]];
        return true;
    }
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
            } else
                tri[j] = it->second;
        }
        out.mesh.triangles.push_back(tri);
    }
    std::vector<Surface> result;
    for (auto &item : islands)
        result.push_back(std::move(item.second));
    return result;
}
using Curve = std::vector<std::pair<Vec3, Vec3>>;
static Curve simplify_curve(const Curve &curve, double tolerance, double angularTolerance) {
    if (curve.size()<3 || tolerance<=0) return curve;
    std::vector<bool> keep(curve.size(),false); keep.front()=keep.back()=true;
    std::function<void(size_t,size_t)> visit=[&](size_t first,size_t last) {
        if(last<=first+1) return;
        Vec3 delta=curve[last].first-curve[first].first; double worst=-1; size_t chosen=first;
        for(size_t i=first+1;i<last;++i) {
            double u=delta.squaredNorm()>1e-16 ? std::clamp((curve[i].first-curve[first].first).dot(delta)/delta.squaredNorm(),0.0,1.0) : 0;
            Vec3 normal=curve[first].second*(1-u)+curve[last].second*u;
            double error=(curve[i].first-curve[first].first-u*delta).norm()/tolerance;
            if(normal.norm()<1e-8 || normal.normalized().dot(curve[i].second)<std::cos(angularTolerance)) error=std::max(error,2.0);
            if(error>worst) { worst=error; chosen=i; }
        }
        if(worst>1) { keep[chosen]=true; visit(first,chosen); visit(chosen,last); }
    };
    visit(0,curve.size()-1); Curve out;
    for(size_t i=0;i<curve.size();++i) if(keep[i]) out.push_back(curve[i]);
    return out;
}
static void add_curve(Project &p, const Curve &input, Role role, int layer,
                      int region) {
    auto curve=simplify_curve(input,std::min(p.process.resolution,p.machine.chordTolerance),p.machine.angularTolerance*.5);
    if (curve.size() < 2)
        return;
    double width = extrusion_width(p.process, role, layer), height = extrusion_height(p.process, layer);
    double area = bead_area(width, height),
           filament = pi * std::pow(p.machine.filamentDiameter / 2, 2);
    if (!p.path.empty()) {
        auto lift = p.path.back();
        lift.position += Vec3(0, 0, p.process.travelLift);
        lift.deltaE = 0;
        lift.role = Role::Travel;
        lift.speed = p.process.travelSpeed;
        p.path.push_back(lift);
    }
    Sample start;
    start.position = curve.front().first + Vec3(0, 0, p.process.travelLift);
    start.direction = curve.front().second;
    start.role = Role::Travel;
    start.layer = layer;
    start.region = region;
    start.speed = p.process.travelSpeed;
    start.width = width;
    start.height = height;
    p.path.push_back(start);
    start.position = curve.front().first;
    p.path.push_back(start);
    double curveLength=0; for(size_t i=1;i<curve.size();++i) curveLength+=(curve[i].first-curve[i-1].first).norm();
    double requestedSpeed=process_speed(p.process,role,layer);
    if((role==Role::Wall || role==Role::OuterWall) && p.process.smallWallThreshold>0 &&
       curveLength<pi*p.process.smallWallThreshold) requestedSpeed=p.process.smallWallAbsoluteSpeed>0?std::min(requestedSpeed,p.process.smallWallAbsoluteSpeed):requestedSpeed*p.process.smallWallSpeed;
    for (size_t i = 1; i < curve.size(); ++i) {
        double length = (curve[i].first - curve[i - 1].first).norm();
        int count = std::max(1, int(std::ceil(length / .7)));
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
            s.speed = requestedSpeed;
            if(p.process.slowOverhangs && layer>0 && role!=Role::Support && role!=Role::Brim && role!=Role::Skirt) {
                int unsupported=0;
                for(int k=0;k<10;++k) {
                    Vec3 side=s.direction.cross((curve[i].first-curve[i-1].first).normalized());
                    Vec3 probe=s.position-s.direction*height*1.25+side*width*(double(k)/9-.5);
                    unsupported+=!contains(p.part,probe);
                }
                int band=unsupported<=1?0:unsupported<=2?1:unsupported<=5?2:unsupported<=7?3:4;
                if(p.process.overhangSpeeds[band]>0) s.speed=std::min(s.speed,p.process.overhangSpeeds[band]);
            }
            s.deltaE = (s.position - p.path.back().position).norm() * area / filament;
            p.path.push_back(s);
        }
    }
}
static double wall_offset(const ProcessSettings &s, int wall, int layer) {
    double distance=0;
    for(int i=0;i<=wall;++i)
        distance += bead_spacing(extrusion_width(s,i==0?Role::OuterWall:Role::Wall,layer),extrusion_height(s,layer))*(i==wall?.5:1);
    return distance;
}
static double wall_band(const ProcessSettings &s, int layer) {
    int count=effective_walls(s,layer);
    return count>0 ? wall_offset(s,count-1,layer)+extrusion_width(s,Role::Wall,layer)/2 : 0;
}
static bool inside_xy(const Curve &curve,const Vec3 &point) {
    bool inside=false;
    for(size_t i=0,j=curve.size()-1;i<curve.size();j=i++) {
        const auto &a=curve[i].first; const auto &b=curve[j].first;
        if((a.y()>point.y())!=(b.y()>point.y()) && point.x()<(b.x()-a.x())*(point.y()-a.y())/(b.y()-a.y())+a.x()) inside=!inside;
    }
    return inside;
}
static void offset_curve(Curve &curve,double amount) {
    if(std::abs(amount)<1e-10 || curve.size()<4 || (curve.front().first-curve.back().first).norm()>1e-6) return;
    namespace bg=boost::geometry;
    using Point=bg::model::d2::point_xy<double>;
    using Polygon=bg::model::polygon<Point>;
    auto original=simplify_curve(curve,.001,.001); Polygon polygon;
    for(const auto &p:original) bg::append(polygon.outer(),Point(p.first.x(),p.first.y()));
    bg::correct(polygon);
    bg::model::multi_polygon<Polygon> result;
    bg::strategy::buffer::distance_symmetric<double> distance(amount);
    bg::strategy::buffer::side_straight side;
    bg::strategy::buffer::join_miter join(4);
    bg::strategy::buffer::end_flat end;
    bg::strategy::buffer::point_circle point(12);
    bg::buffer(polygon,result,distance,side,join,end,point);
    if(result.size()!=1 || result.front().outer().size()<4 || !result.front().inners().empty())
        throw std::runtime_error("XY offset collapses or splits a contour; reduce compensation or adhesion width");
    Curve output;
    for(const auto &p:result.front().outer()) {
        Eigen::Vector2d xy(bg::get<0>(p),bg::get<1>(p)); double best=1e100; Vec3 surface=Vec3::Zero(),normal=Vec3::UnitZ();
        for(size_t i=1;i<original.size();++i) {
            Vec3 delta=original[i].first-original[i-1].first; double length=delta.head<2>().squaredNorm();
            double u=length>1e-16?std::clamp((xy-original[i-1].first.head<2>()).dot(delta.head<2>())/length,0.0,1.0):0;
            Vec3 candidate=original[i-1].first+u*delta; double error=(xy-candidate.head<2>()).squaredNorm();
            if(error<best) {best=error;surface=candidate;normal=(original[i-1].second*(1-u)+original[i].second*u).normalized();}
        }
        if(std::abs(normal.z())<.1) throw std::runtime_error("XY compensation requires layer normals with a Z component");
        Vec3 displacement(xy.x()-surface.x(),xy.y()-surface.y(),0);
        displacement.z()=-(normal.x()*displacement.x()+normal.y()*displacement.y())/normal.z();
        output.push_back({surface+displacement,normal});
    }
    output.pop_back(); size_t seam=0; double nearest=1e100;
    for(size_t i=0;i<output.size();++i) {double d=(output[i].first-curve.front().first).squaredNorm();if(d<nearest){nearest=d;seam=i;}}
    std::rotate(output.begin(),output.begin()+seam,output.end()); output.push_back(output.front()); curve=std::move(output);
}

static void walls(Project &p, const Surface &s, int layer, int &region, int count=-1,
                  Role forced=Role::Travel,double firstOffset=0,double spacing=0,double outside=0,int boundaryKind=0) {
    if(count<0) count=effective_walls(p.process,layer);
    for (int index = 0; index < count; ++index) {
        int wall=forced!=Role::Travel || p.process.outerWallFirst?index:count-index-1;
        double target = forced==Role::Travel ? wall_offset(p.process,wall,layer) : firstOffset+index*spacing;
        std::vector<std::pair<Vec3, Vec3>> ends;
        std::vector<std::pair<int, int>> edges;
        std::map<std::array<long long, 3>, int> ids;
        for (auto t : s.mesh.triangles) {
            std::vector<std::pair<Vec3, Vec3>> segment;
            for (int j = 0; j < 3; ++j) {
                int a = t[j], b = t[(j + 1) % 3];
                double da = s.distance[a] - target, db = s.distance[b] - target;
                if ((da < 0) == (db < 0) || std::abs(da - db) < 1e-12)
                    continue;
                double u = da / (da - db);
                segment.push_back({s.mesh.vertices[a] + u * (s.mesh.vertices[b] - s.mesh.vertices[a]),
                                   (s.normals[a] + u * (s.normals[b] - s.normals[a])).normalized()});
            }
            if (segment.size() != 2)
                continue;
            int index[2];
            for (int j = 0; j < 2; ++j) {
                auto &q = segment[j].first;
                std::array<long long, 3> key = {llround(q.x() * 1e6), llround(q.y() * 1e6),
                                                llround(q.z() * 1e6)};
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
            std::vector<std::pair<Vec3, Vec3>> curve;
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
            if (curve.size() > 3 &&
                (curve.front().first - curve.back().first).norm() < 1e-6) {
                curve.pop_back();
                size_t seam = 0;
                double cost = 1e100;
                for (size_t i = 0; i < curve.size(); ++i) {
                    double v=0;
                    if(p.process.seam==SeamPosition::LowStress && p.stress.directional) {
                        bool covered; auto t=p.stress.sample(curve[i].first,&covered); v=covered?t.norm():1e100;
                    } else if(p.process.seam==SeamPosition::Back)
                        v=-curve[i].first.y()+1e-6*curve[i].first.x();
                    else if(p.process.seam==SeamPosition::Nearest && !p.path.empty())
                        v=(curve[i].first-p.path.back().position).squaredNorm();
                    else v=curve[i].first.x()+1e-6*curve[i].first.y();
                    if (v < cost) {
                        cost = v;
                        seam = i;
                    }
                }
                if(p.process.seam==SeamPosition::Random)
                    seam=(unsigned(layer+1)*1664525u+unsigned(wall+1)*1013904223u)%curve.size();
                std::rotate(curve.begin(), curve.begin() + seam, curve.end());
                curve.push_back(curve.front());
            }
            curves.push_back(std::move(curve));
        }
        for(auto &curve:curves) {
            int depth=0; for(const auto &other:curves) if(&other!=&curve && inside_xy(other,curve.front().first)) ++depth;
            if((boundaryKind==1 && depth%2) || (boundaryKind==2 && !(depth%2))) continue;
            if(forced==Role::Travel && curve.size()>3 && (curve.front().first-curve.back().first).norm()<1e-6) {
                double amount=depth%2 ? p.process.holeCompensation : p.process.contourCompensation;
                if(layer==0) amount+=(depth%2?1:-1)*p.process.elephantFoot;
                offset_curve(curve,amount);
            }
            if(outside!=0) offset_curve(curve,(depth%2?-1:1)*outside);
            add_curve(p, curve, forced!=Role::Travel?forced:wall==0?Role::OuterWall:Role::Wall, layer, region++);
        }
    }
}
static void infill(Project &p, const Surface &surface, int layer, int &region, Role role) {
    bool solid=role!=Role::Infill;
    if ((!solid && p.process.infill <= 0) || surface.mesh.empty())
        return;
    double density=role==Role::TopSurface?p.process.topDensity:role==Role::BottomSurface?p.process.bottomDensity:solid?1:p.process.infill;
    if(density<=0) return;
    int families=!solid && p.process.infillPattern==InfillPattern::Grid?2:1;
    double width=extrusion_width(p.process,role,layer);
    double spacing = bead_spacing(width, extrusion_height(p.process,layer)) / density * families;
    auto pattern=role==Role::TopSurface?p.process.topPattern:p.process.bottomPattern;
    if(solid && pattern==SurfacePattern::Concentric) {
        double start=wall_band(p.process,layer)+width/2;
        double maximum=*std::max_element(surface.distance.begin(),surface.distance.end());
        walls(p,surface,layer,region,std::max(0,int(std::ceil((maximum-start)/spacing))),role,start,spacing); return;
    }
    auto b = surface.mesh.bounds();
    std::vector<Curve> curves;
    for(int family=0;family<families;++family) {
    std::vector<Eigen::Vector2d> covered;
    auto near = [&](Eigen::Vector2d q, double distance) {
        for (auto &c : covered)
            if ((c - q).squaredNorm() < distance * distance)
                return true;
        return false;
    };
    for (double y = b.min().y(); y <= b.max().y(); y += spacing)
        for (double x = b.min().x(); x <= b.max().x(); x += spacing) {
            Eigen::Vector2d seed(x, y);
            Vec3 point, n;
            double d;
            if (near(seed, .8 * spacing) || !surface_point(surface, seed, point, n, d) ||
                d < wall_band(p.process,layer))
                continue;
            std::vector<std::pair<Vec3, Vec3>> halves[2];
            for (int sign = 0; sign < 2; ++sign) {
                Eigen::Vector2d xy = seed;
                double angle=p.process.infillAngle+(families==2?family*pi/2:(layer%2)*pi/2);
                Vec3 previous(std::cos(angle),std::sin(angle),0);
                previous *= sign ? 1 : -1;
                for (int step = 0; step < 1000; ++step) {
                    if (!surface_point(surface, xy, point, n, d) || d < wall_band(p.process,layer))
                        break;
                    if (step > 2 && near(xy, .7 * spacing))
                        break;
                    halves[sign].push_back({point, n});
                    Vec3 direction = previous;
                    if (!solid && p.process.infillPattern==InfillPattern::StressAligned && p.stress.directional) {
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
            std::vector<std::pair<Vec3, Vec3>> curve;
            for (auto it = halves[0].rbegin(); it != halves[0].rend(); ++it)
                curve.push_back(*it);
            if (!halves[1].empty())
                curve.insert(curve.end(), halves[1].begin() + 1, halves[1].end());
            if (curve.size() > 1) {
                for (auto &q : curve)
                    covered.push_back(q.first.head<2>());
                curves.push_back(std::move(curve));
            }
        }
    }
    if(solid && pattern==SurfacePattern::Monotonic) {
        double angle=p.process.infillAngle+(layer%2)*pi/2; Vec3 direction(std::cos(angle),std::sin(angle),0),side(-direction.y(),direction.x(),0);
        for(auto &curve:curves) if((curve.back().first-curve.front().first).dot(direction)<0) std::reverse(curve.begin(),curve.end());
        std::stable_sort(curves.begin(),curves.end(),[&](const Curve &a,const Curve &b) { return a.front().first.dot(side)<b.front().first.dot(side); });
    }
    for(const auto &curve:curves) add_curve(p,curve,role,layer,region++);
}
static void support_columns(Project &p) {
    if (!p.process.supports || p.process.supportMode==SupportMode::None) {
        p.diagnostics << "Support generation disabled; unsupported deposition is still checked.";
        return;
    }
    double width = extrusion_width(p.process,Role::Support,1);
    double pitch=width+p.process.supportSpacing;
    if(p.process.supportMode==SupportMode::Reduced) pitch*=p.process.supportReduction;
    std::map<std::pair<int, int>, double> columns;
    for (auto &s : p.path)
        if (s.deltaE > 0 && s.role!=Role::Brim && s.role!=Role::Skirt && s.position.z() > 2 * p.process.layerHeight &&
            !contains(p.part, s.position - Vec3(0, 0, 1.5 * p.process.layerHeight))) {
            if(p.process.supportAngle>0) {
                double reach=1.5*p.process.layerHeight/std::tan(p.process.supportAngle); bool supported=false;
                for(const Vec3 &axis:{Vec3::UnitX().eval(),Vec3::UnitY().eval(),(-Vec3::UnitX()).eval(),(-Vec3::UnitY()).eval()})
                    supported|=contains(p.part,s.position-Vec3(0,0,1.5*p.process.layerHeight)+axis*reach);
                if(supported) continue;
            }
            int x = int(std::round(s.position.x() / pitch)), y = int(std::round(s.position.y() / pitch));
            double top = s.position.z() - s.height;
            auto key = std::make_pair(x, y);
            columns[key] = std::max(columns[key], top);
        }
    if (columns.empty())
        return;
    if(p.process.supportBedOnly) for(auto it=columns.begin();it!=columns.end();) {
        bool blocked=false;
        for(double z=p.process.layerHeight;z<it->second-p.process.layerHeight;z+=p.process.layerHeight)
            if(contains(p.part,Vec3(it->first.first*pitch,it->first.second*pitch,z))) { blocked=true; break; }
        if(blocked) it=columns.erase(it); else ++it;
    }
    if(p.process.removeSmallOverhangs) {
        auto snapshot=columns;
        for(auto it=columns.begin();it!=columns.end();) {
            bool neighbor=false;
            for(auto offset:{std::make_pair(1,0),std::make_pair(-1,0),std::make_pair(0,1),std::make_pair(0,-1)})
                neighbor|=snapshot.count({it->first.first+offset.first,it->first.second+offset.second})>0;
            if(!neighbor) it=columns.erase(it); else ++it;
        }
    }
    auto original = std::move(p.path);
    p.path.clear();
    double maxZ = 0;
    for (auto &[k, z] : columns)
        maxZ = std::max(maxZ, z);
    int region = -1;
    int supportLayer=0;
    for (double z = extrusion_height(p.process,0); z <= maxZ + 1e-8; z += p.process.layerHeight,++supportLayer)
        for (auto &[key, top] : columns)
            if (z <= top) {
                Vec3 center(key.first * pitch, key.second * pitch, z);
                if (contains(p.part, center))
                    continue;
                add_curve(p,
                          {{center - Vec3(width / 3, 0, 0), Vec3::UnitZ()},
                           {center + Vec3(width / 3, 0, 0), Vec3::UnitZ()}},
                          Role::Support, supportLayer, region--);
            }
    // Schedule deposited support and part curves in layer order. Future support is never an obstacle.
    struct Block { int layer; bool support; std::vector<Sample> samples; };
    std::vector<Block> blocks;
    auto collect=[&](const std::vector<Sample> &path,bool support) {
        for(size_t i=1;i<path.size();) {
            if(path[i].deltaE<=0) { ++i; continue; }
            Block block{path[i].layer,support,{path[i-1]}}; int region=path[i].region;
            while(i<path.size() && path[i].deltaE>0 && path[i].region==region) block.samples.push_back(path[i++]);
            blocks.push_back(std::move(block));
        }
    };
    collect(p.path,true); collect(original,false);
    std::stable_sort(blocks.begin(),blocks.end(),[](const Block &a,const Block &b) { return a.layer!=b.layer?a.layer<b.layer:a.support>b.support; });
    p.path.clear();
    for(const auto &block:blocks) {
        if(!p.path.empty()) { auto lift=p.path.back(); lift.position+=Vec3(0,0,p.process.travelLift); lift.role=Role::Travel; lift.deltaE=0; lift.speed=p.process.travelSpeed; lift.layer=block.layer; p.path.push_back(lift); }
        auto entry=block.samples.front(); entry.deltaE=0; entry.role=Role::Travel; entry.layer=block.layer; entry.speed=p.process.travelSpeed;
        auto lift=entry; lift.position+=Vec3(0,0,p.process.travelLift); p.path.push_back(lift); p.path.push_back(entry);
        p.path.insert(p.path.end(),block.samples.begin()+1,block.samples.end());
    }
    p.diagnostics << QString(
                         "Generated %1 support columns (%2, spacing %3 mm); deposition and collision checks remain required.")
                         .arg(columns.size()).arg(p.process.supportMode==SupportMode::Reduced?"reduced":"normal").arg(pitch);
}
void regenerate_joints(Project &p) {
    JointState previous;
    for (size_t i = 0; i < p.path.size(); ++i) {
        auto &s = p.path[i];
        double uprightCone = std::min(p.process.directionCone * .1, p.machine.angularTolerance * .25);
        if (s.direction.dot(Vec3::UnitZ()) >= std::cos(uprightCone))
            s.direction = Vec3::UnitZ();
        auto solutions = inverse_kinematics(p.machine, s.position, s.direction, previous, p.active_toolhead());
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
                Vec3 left = part_position(p.machine, interpolate_motion(p.machine, p.active_toolhead(), a.joint,
                                                                        b.joint, double(j) / count));
                Vec3 right = part_position(p.machine, interpolate_motion(p.machine, p.active_toolhead(), a.joint,
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
            s.deltaE = b.deltaE > 0
                           ? (s.position - refined.back().position).norm() * bead_area(b.width, b.height) /
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
    double pitch = std::max(p.process.width, .2);
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
        if (b.deltaE <= 0)
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
            Vec3 point = part_position(p.machine, q), n = part_direction(p.machine, p.active_toolhead(), q);
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
    for (auto &cell : p.volume.cells) {
        Vec3 center = Vec3::Zero();
        for (int j = 0; j < 4; ++j)
            center += p.volume.vertices[cell.nodes[j]] / 4;
        if (visit(center, reach, [&](const Deposit &d) { return (d.point - center).norm() <= reach; }))
            ++covered;
    }
    p.deposition.coverage = p.volume.cells.empty() ? 1 : double(covered) / p.volume.cells.size();
    p.depositionValid = p.deposition.unsupportedMoves.empty() && p.deposition.coverage >= .95;
    p.diagnostics << QString("Deposition support / gravity check: %1 flagged movements, %2 dependency edges, "
                             "%3% sampled volume coverage.")
                         .arg(p.deposition.unsupportedMoves.size())
                         .arg(dependencies.size())
                         .arg(p.deposition.coverage * 100, 0, 'f', 1);
}
void slice_project(Project &p, std::atomic_bool *cancel, std::function<void(double)> progress) {
    p.invalidate();
    p.path.clear();
    p.diagnostics.clear();
    if (p.part.empty())
        throw std::runtime_error("Import a printable part first");
    validate_process(p.process);
    if(p.active_toolhead().components.empty())
        throw std::runtime_error("Import a toolhead STEP or enable the dummy toolhead option");
    if(p.useDummyToolhead) p.diagnostics << "Dummy toolhead selected: collision results cover synthetic geometry only.";
    if (p.volume.cells.empty())
        p.volume = tetrahedralize(p.part, std::max(.05, p.part.bounds().sizes().prod() / 1500));
    auto field = scalar_field(p);
    double low = *std::min_element(field.begin(), field.end()),
           high = *std::max_element(field.begin(), field.end());
    int layer = 0, region = 0;
    double previousLevel=low;
    for (double level = p.process.preciseZ?std::min(high,low + extrusion_height(p.process,0)):low+extrusion_height(p.process,0); level <= high + 1e-7;
         level += p.process.layerHeight, ++layer) {
        if(level>high) break;
        if (cancel && cancel->load())
            throw std::runtime_error("Slicing cancelled");
        auto surface = iso_surface(p.volume, field, std::min(level, high - 1e-7));
        if (surface.mesh.empty())
            continue;
        size_t begin=p.path.size();
        for (auto &island : decompose_surface(surface)) {
            double nominalSpacing=bead_spacing(extrusion_width(p.process,Role::Brim,layer),extrusion_height(p.process,layer));
            bool autoBrim=p.process.brimType==1 && p.part.bounds().sizes().z()>5*std::min(p.part.bounds().sizes().x(),p.part.bounds().sizes().y());
            if(layer==0 && (p.process.brimType>1 || autoBrim)) {
                int count=int(std::ceil(p.process.brimWidth/nominalSpacing));
                int kind=p.process.brimType==3?2:p.process.brimType==4?0:1;
                for(int i=0;i<count;++i) walls(p,island,layer,region,1,Role::Brim,1e-6,0,p.process.brimGap+(i+.5)*nominalSpacing,kind);
            }
            if(layer<p.process.skirtHeight) for(int i=0;i<p.process.skirtLoops;++i)
                walls(p,island,layer,region,1,Role::Skirt,1e-6,0,p.process.skirtDistance+(i+.5)*nominalSpacing,1);
            int topCount=std::max(p.process.topShellLayers,int(std::ceil(p.process.topThickness/p.process.layerHeight)));
            int bottomCount=std::max(p.process.bottomShellLayers,int(std::ceil(p.process.bottomThickness/p.process.layerHeight)));
            bool bottom=layer<bottomCount,top=topCount>0 && level+topCount*p.process.layerHeight>high+1e-7;
            bool finalLayer=level+p.process.layerHeight>high+1e-7;
            Role role=top && finalLayer?Role::TopSurface:bottom && layer==0?Role::BottomSurface:top || bottom?Role::SolidInfill:Role::Infill;
            if(p.process.infillFirst) infill(p,island,layer,region,role);
            walls(p, island, layer, region);
            if(!p.process.infillFirst) infill(p,island,layer,region,role);
        }
        if(p.process.preciseZ && level-previousLevel<extrusion_height(p.process,layer)-1e-7) {
            double actual=level-previousLevel;
            for(size_t i=begin;i<p.path.size();++i) { auto &s=p.path[i]; if(s.deltaE>0) s.deltaE*=bead_area(s.width,actual)/bead_area(s.width,s.height); s.height=actual; }
        }
        previousLevel=level;
        if (progress)
            progress((level - low) / (high - low));
        if(level>=high-1e-7) { ++layer; break; }
        if(p.process.preciseZ && level+p.process.layerHeight>high) level=high-p.process.layerHeight;
    }
    if (p.path.size() < 2)
        throw std::runtime_error("No printable paths: increase meshing detail or reduce extrusion width");
    support_columns(p);
    p.pathProcessFingerprint=process_fingerprint(p.process);
    regenerate_joints(p);
    check_deposition(p);
    p.diagnostics << QString("%1 layers, %2 independently scheduled curves, %3 coordinated movements.")
                         .arg(layer)
                         .arg(region)
                         .arg(p.path.size());
    p.diagnostics << "Research scalar-field solver uses stress-weighted diffusion with bounded spacing; "
                     "material behavior and strength require calibration.";
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
