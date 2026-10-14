#include "geometry.h"
#include "step_import.h"
#include <Eigen/LU>
#include <QDataStream>
#include <QFile>
#include <QRegularExpression>
#include <QXmlStreamReader>
#include <algorithm>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tetgen.h>
#include <zip.h>

namespace fiveaxis {
ToolheadAssembly make_dummy_toolhead(double nozzleDiameter) {
    if (!std::isfinite(nozzleDiameter) || nozzleDiameter <= 0)
        throw std::runtime_error("Invalid dummy nozzle diameter");
    ToolheadAssembly t;
    t.sourceName = "Dummy toolhead";
    t.sourceUnits = "synthetic millimetres";
    t.components.push_back({"dummy-nozzle", "Dummy nozzle",
                            cylinder_mesh(Vec3(0, 0, 1), nozzleDiameter / 2, 2), Link::Nozzle, false, true});
    t.components.push_back(
        {"dummy-break", "Dummy heat break", cylinder_mesh(Vec3(0, 0, 5), 1, 6), Link::Nozzle});
    t.components.push_back(
        {"dummy-heater", "Dummy heater block", box_mesh(Vec3(0, 0, 12), Vec3(10, 8, 8)), Link::Nozzle});
    t.components.push_back(
        {"dummy-fan", "Dummy fan shroud", box_mesh(Vec3(11, 0, 18), Vec3(8, 12, 12)), Link::Nozzle});
    t.contactRadius = std::max(.55, nozzleDiameter * .8);
    t.contactHeight = .75;
    return t;
}
Mesh box_mesh(const Vec3 &center, const Vec3 &size, const QString &name) {
    Mesh m;
    m.name = name;
    for (int z = 0; z < 2; ++z)
        for (int y = 0; y < 2; ++y)
            for (int x = 0; x < 2; ++x)
                m.vertices.push_back(center +
                                     Vec3((x - .5) * size.x(), (y - .5) * size.y(), (z - .5) * size.z()));
    const int f[][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                        {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    for (auto &t : f)
        m.triangles.emplace_back(t[0], t[1], t[2]);
    return m;
}
Mesh cylinder_mesh(const Vec3 &center, double radius, double height, const QString &name, int segments) {
    Mesh m;
    m.name = name;
    for (int z = 0; z < 2; ++z)
        for (int i = 0; i < segments; ++i) {
            double a = 2 * pi * i / segments;
            m.vertices.push_back(center +
                                 Vec3(radius * std::cos(a), radius * std::sin(a), (z - .5) * height));
        }
    m.vertices.push_back(center - Vec3(0, 0, height / 2));
    m.vertices.push_back(center + Vec3(0, 0, height / 2));
    for (int i = 0; i < segments; ++i) {
        int j = (i + 1) % segments;
        m.triangles.emplace_back(i, j, i + segments);
        m.triangles.emplace_back(j, j + segments, i + segments);
        m.triangles.emplace_back(2 * segments, j, i);
        m.triangles.emplace_back(2 * segments + 1, i + segments, j + segments);
    }
    return m;
}
bool contains(const Mesh &mesh, const Vec3 &point) {
    if (!mesh.bounds().contains(point))
        return false;
    const Vec3 ray = Vec3(1, 0.371390676, 0.529177211).normalized();
    std::vector<double> hits;
    for (const auto &t : mesh.triangles) {
        const Vec3 &v = mesh.vertices[t[0]];
        Vec3 a = mesh.vertices[t[1]] - v, b = mesh.vertices[t[2]] - v, h = ray.cross(b);
        double det = a.dot(h);
        if (std::abs(det) < 1e-12)
            continue;
        Vec3 s = point - v;
        double u = s.dot(h) / det;
        if (u < 0 || u > 1)
            continue;
        Vec3 q = s.cross(a);
        double w = ray.dot(q) / det;
        if (w < 0 || u + w > 1)
            continue;
        double d = b.dot(q) / det;
        if (d > 1e-9)
            hits.push_back(d);
    }
    std::sort(hits.begin(), hits.end());
    hits.erase(
        std::unique(hits.begin(), hits.end(), [](double a, double b) { return std::abs(a - b) < 1e-7; }),
        hits.end());
    return hits.size() % 2 != 0;
}
// A nearly vertical ray keeps each query inside a few XY grid cells while avoiding
// the axis-aligned edge and vertex hits of a perfectly vertical ray.
static const Vec3 testerRay = Vec3(0.0123456, 0.0071234, 1).normalized();
InsideTester::InsideTester(const Mesh &m) : mesh(&m), box(m.bounds()) {
    if (m.triangles.empty())
        return;
    Eigen::Vector2d size = box.sizes().head<2>().cwiseMax(1e-6);
    double area = 0;
    for (const auto &t : m.triangles) {
        Eigen::AlignedBox2d b;
        for (int j = 0; j < 3; ++j)
            b.extend(m.vertices[t[j]].head<2>());
        area += b.volume();
    }
    cell = std::max({std::sqrt(area / m.triangles.size()) * 2, size.maxCoeff() / 1024, 1e-6});
    origin = box.min().head<2>();
    nx = std::max(1, int(std::ceil(size.x() / cell)));
    ny = std::max(1, int(std::ceil(size.y() / cell)));
    cells.resize(size_t(nx) * ny);
    for (int i = 0; i < int(m.triangles.size()); ++i) {
        Eigen::AlignedBox2d b;
        for (int j = 0; j < 3; ++j)
            b.extend(m.vertices[m.triangles[i][j]].head<2>());
        int x0 = std::clamp(int((b.min().x() - origin.x()) / cell), 0, nx - 1),
            x1 = std::clamp(int((b.max().x() - origin.x()) / cell), 0, nx - 1),
            y0 = std::clamp(int((b.min().y() - origin.y()) / cell), 0, ny - 1),
            y1 = std::clamp(int((b.max().y() - origin.y()) / cell), 0, ny - 1);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                cells[size_t(y) * nx + x].push_back(i);
    }
}
bool InsideTester::contains(const Vec3 &point) const {
    if (cells.empty() || !box.contains(point))
        return false;
    Eigen::Vector2d end = point.head<2>() + testerRay.head<2>() / testerRay.z() * (box.max().z() - point.z());
    Eigen::AlignedBox2d footprint;
    footprint.extend(point.head<2>());
    footprint.extend(end);
    int x0 = std::clamp(int((footprint.min().x() - origin.x()) / cell), 0, nx - 1),
        x1 = std::clamp(int((footprint.max().x() - origin.x()) / cell), 0, nx - 1),
        y0 = std::clamp(int((footprint.min().y() - origin.y()) / cell), 0, ny - 1),
        y1 = std::clamp(int((footprint.max().y() - origin.y()) / cell), 0, ny - 1);
    std::vector<int> candidates;
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
            const auto &list = cells[size_t(y) * nx + x];
            candidates.insert(candidates.end(), list.begin(), list.end());
        }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    std::vector<double> hits;
    for (int id : candidates) {
        const auto &t = mesh->triangles[id];
        const Vec3 &v = mesh->vertices[t[0]];
        Vec3 a = mesh->vertices[t[1]] - v, b = mesh->vertices[t[2]] - v, h = testerRay.cross(b);
        double det = a.dot(h);
        if (std::abs(det) < 1e-12)
            continue;
        Vec3 s = point - v;
        double u = s.dot(h) / det;
        if (u < 0 || u > 1)
            continue;
        Vec3 q = s.cross(a);
        double w = testerRay.dot(q) / det;
        if (w < 0 || u + w > 1)
            continue;
        double d = b.dot(q) / det;
        if (d > 1e-9)
            hits.push_back(d);
    }
    std::sort(hits.begin(), hits.end());
    hits.erase(
        std::unique(hits.begin(), hits.end(), [](double a, double b) { return std::abs(a - b) < 1e-7; }),
        hits.end());
    return hits.size() % 2 != 0;
}
void weld_mesh_vertices(Mesh &m, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0)
        throw std::runtime_error("Invalid vertex weld tolerance");
    using Key = std::array<long long, 3>;
    std::map<Key, std::vector<int>> ids;
    std::vector<Vec3> v;
    std::vector<int> remap;
    Vec3 origin = m.vertices.empty() ? Vec3::Zero().eval() : m.vertices.front();
    for (const Vec3 &p : m.vertices) {
        if (!p.allFinite())
            throw std::runtime_error("Non-finite mesh coordinates");
        Vec3 scaled = (p - origin) / tolerance;
        // Keep neighbour-key arithmetic away from integer overflow.
        if (!scaled.allFinite() ||
            scaled.cwiseAbs().maxCoeff() >= double(std::numeric_limits<long long>::max()) / 2)
            throw std::runtime_error("Mesh extent too large for seam stitching");
        Key key{static_cast<long long>(std::floor(scaled.x())),
                static_cast<long long>(std::floor(scaled.y())),
                static_cast<long long>(std::floor(scaled.z()))};
        int match = -1;
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z) {
                    auto it = ids.find(Key{key[0] + x, key[1] + y, key[2] + z});
                    if (it != ids.end())
                        for (int id : it->second)
                            if ((p - v[id]).squaredNorm() <= tolerance * tolerance &&
                                (match < 0 || id < match))
                                match = id;
                }
        if (match < 0) {
            match = int(v.size());
            ids[key].push_back(match);
            v.push_back(p);
        }
        remap.push_back(match);
    }
    for (auto &t : m.triangles)
        for (int j = 0; j < 3; ++j) {
            if (t[j] < 0 || t[j] >= int(remap.size()))
                throw std::runtime_error("Mesh index out of range");
            t[j] = remap[t[j]];
        }
    m.vertices = std::move(v);
    m.triangles.erase(
        std::remove_if(m.triangles.begin(), m.triangles.end(),
                       [](const auto &t) { return t[0] == t[1] || t[1] == t[2] || t[0] == t[2]; }),
        m.triangles.end());
}
static Mesh parse_3mf(const QString &filename) {
    int error = 0;
    zip_t *z = zip_open(filename.toUtf8().constData(), ZIP_RDONLY, &error);
    if (!z)
        throw std::runtime_error("Cannot open 3MF archive");
    // Production-extension files (Orca, Bambu, PrusaSlicer) keep objects in separate
    // 3D/Objects/*.model parts referenced from the root model by p:path.
    std::map<QString, QByteArray> parts;
    QByteArray relationships;
    quint64 total = 0;
    for (zip_int64_t i = 0; i < zip_get_num_entries(z, 0); ++i) {
        QString name = QString::fromUtf8(zip_get_name(z, i, 0));
        bool model = name.endsWith(".model", Qt::CaseInsensitive);
        if (!model && name.compare("_rels/.rels", Qt::CaseInsensitive) != 0)
            continue;
        zip_stat_t st;
        if (zip_stat_index(z, i, 0, &st) < 0 || st.size > 256 * 1024 * 1024 ||
            (total += st.size) > 1024ull * 1024 * 1024) {
            zip_close(z);
            throw std::runtime_error("3MF model too large");
        }
        QByteArray bytes(qsizetype(st.size), Qt::Uninitialized);
        auto f = zip_fopen_index(z, i, 0);
        if (!f) {
            zip_close(z);
            throw std::runtime_error("Cannot read 3MF model part");
        }
        auto count = zip_fread(f, bytes.data(), st.size);
        zip_fclose(f);
        if (count != zip_int64_t(st.size)) {
            zip_close(z);
            throw std::runtime_error("Truncated 3MF");
        }
        if (model)
            parts[name.toLower()] = bytes;
        else
            relationships = bytes;
    }
    zip_close(z);
    if (parts.empty())
        throw std::runtime_error("3MF archive contains no model");
    auto normalize = [](QString path) {
        while (path.startsWith('/'))
            path.remove(0, 1);
        return path.toLower();
    };
    QString root = "3d/3dmodel.model";
    QXmlStreamReader rels(relationships);
    while (!rels.atEnd())
        if (rels.readNext() == QXmlStreamReader::StartElement && rels.name() == QLatin1String("Relationship") &&
            rels.attributes().value("Type").toString().endsWith("/3dmodel"))
            root = normalize(rels.attributes().value("Target").toString());
    if (!parts.count(root))
        root = parts.begin()->first;
    using ObjectKey = std::pair<QString, int>;
    struct Instance {
        ObjectKey id;
        Eigen::Matrix4d transform;
    };
    std::map<ObjectKey, Mesh> objects;
    std::map<ObjectKey, std::vector<Instance>> children;
    std::vector<Instance> build;
    double scale = 1;
    auto matrix = [](const QString &s) {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        auto a = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (a.isEmpty())
            return t;
        if (a.size() != 12)
            throw std::runtime_error("Invalid 3MF transform");
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 3; ++col)
                t(col, row) = a[row * 3 + col].toDouble();
        return t;
    };
    const QString production = "http://schemas.microsoft.com/3dmanufacturing/production/2015/06";
    auto referenced = [&](const QXmlStreamAttributes &a, const QString &part) {
        QString path = a.value(production, "path").toString();
        if (path.isEmpty())
            path = a.value("p:path").toString();
        return path.isEmpty() ? part : normalize(path);
    };
    for (const auto &[part, xml] : parts) {
        QXmlStreamReader r(xml);
        std::optional<ObjectKey> current;
        while (!r.atEnd()) {
            r.readNext();
            if (r.isEndElement() && r.name() == QLatin1String("object"))
                current.reset();
            if (!r.isStartElement())
                continue;
            auto a = r.attributes();
            auto n = r.name();
            if (n == QLatin1String("model") && part == root) {
                QString unit = a.value("unit").toString();
                if (unit == "inch")
                    scale = 25.4;
                else if (unit == "foot")
                    scale = 304.8;
                else if (unit == "centimeter")
                    scale = 10;
                else if (unit == "meter")
                    scale = 1000;
                else if (unit == "micron")
                    scale = .001;
            } else if (n == QLatin1String("object")) {
                current = ObjectKey{part, a.value("id").toInt()};
                objects[*current].name = a.value("name").toString();
            } else if (n == QLatin1String("vertex") || n == QLatin1String("triangle") ||
                       n == QLatin1String("component")) {
                if (!current)
                    throw std::runtime_error("3MF geometry outside an object");
                if (n == QLatin1String("vertex"))
                    objects[*current].vertices.emplace_back(a.value("x").toDouble(), a.value("y").toDouble(),
                                                            a.value("z").toDouble());
                else if (n == QLatin1String("triangle"))
                    objects[*current].triangles.emplace_back(a.value("v1").toInt(), a.value("v2").toInt(),
                                                             a.value("v3").toInt());
                else
                    children[*current].push_back({{referenced(a, part), a.value("objectid").toInt()},
                                                  matrix(a.value("transform").toString())});
            } else if (n == QLatin1String("item") && part == root)
                build.push_back({{referenced(a, part), a.value("objectid").toInt()},
                                 matrix(a.value("transform").toString())});
        }
        if (r.hasError())
            throw std::runtime_error(r.errorString().toStdString());
    }
    Mesh out;
    out.name = filename;
    std::function<void(const ObjectKey &, Eigen::Matrix4d, std::set<ObjectKey>)> add =
        [&](const ObjectKey &id, Eigen::Matrix4d t, std::set<ObjectKey> seen) {
        if (!seen.insert(id).second)
            throw std::runtime_error("Cyclic 3MF assembly");
        if (!objects.count(id))
            throw std::runtime_error("Missing 3MF object");
        auto &m = objects[id];
        int offset = int(out.vertices.size());
        for (const auto &v : m.vertices)
            out.vertices.push_back((t * Eigen::Vector4d(v.x(), v.y(), v.z(), 1)).head<3>() * scale);
        for (const auto &face : m.triangles)
            out.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
        for (const auto &child : children[id])
            add(child.id, t * child.transform, seen);
    };
    for (auto &item : build)
        add(item.id, item.transform, {});
    return out;
}
Mesh load_mesh(const QString &filename) {
    Mesh m;
    m.name = filename;
    if (filename.endsWith(".step", Qt::CaseInsensitive) || filename.endsWith(".stp", Qt::CaseInsensitive)) {
        auto t = load_step_assembly(filename);
        for (auto &c : t.components) {
            int offset = int(m.vertices.size());
            m.vertices.insert(m.vertices.end(), c.mesh.vertices.begin(), c.mesh.vertices.end());
            for (auto f : c.mesh.triangles)
                m.triangles.push_back(f + Eigen::Vector3i::Constant(offset));
        }
    } else if (filename.endsWith(".3mf", Qt::CaseInsensitive))
        m = parse_3mf(filename);
    else {
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly))
            throw std::runtime_error("Cannot read mesh");
        QByteArray data = file.readAll();
        if (filename.endsWith(".obj", Qt::CaseInsensitive)) {
            std::istringstream stream(data.toStdString());
            std::string line;
            while (std::getline(stream, line)) {
                std::istringstream row(line);
                std::string token;
                row >> token;
                if (token == "v") {
                    double x, y, z;
                    if (!(row >> x >> y >> z))
                        throw std::runtime_error("Malformed OBJ vertex");
                    m.vertices.emplace_back(x, y, z);
                } else if (token == "f") {
                    std::vector<int> face;
                    while (row >> token) {
                        int id = std::stoi(token.substr(0, token.find('/')));
                        face.push_back(id < 0 ? int(m.vertices.size()) + id : id - 1);
                    }
                    for (size_t i = 1; i + 1 < face.size(); ++i)
                        m.triangles.emplace_back(face[0], face[i], face[i + 1]);
                }
            }
        } else {
            quint32 count = 0;
            if (data.size() >= 84)
                std::memcpy(&count, data.constData() + 80, 4);
            if (data.size() >= 84 && quint64(data.size()) == 84 + quint64(count) * 50) {
                for (quint32 i = 0; i < count; ++i) {
                    int offset = int(m.vertices.size());
                    for (int j = 0; j < 3; ++j) {
                        float a[3];
                        std::memcpy(a, data.constData() + 84 + i * 50 + 12 + j * 12, 12);
                        m.vertices.emplace_back(a[0], a[1], a[2]);
                    }
                    m.triangles.emplace_back(offset, offset + 1, offset + 2);
                }
            } else {
                std::istringstream stream(data.toStdString());
                std::string token;
                while (stream >> token)
                    if (token == "vertex") {
                        double x, y, z;
                        if (!(stream >> x >> y >> z))
                            throw std::runtime_error("Malformed ASCII STL");
                        m.vertices.emplace_back(x, y, z);
                        if (m.vertices.size() % 3 == 0) {
                            int id = int(m.vertices.size());
                            m.triangles.emplace_back(id - 3, id - 2, id - 1);
                        }
                    }
            }
        }
    }
    // STEP seams share exact OCCT nodes. STL/OBJ/3MF exporters round shared vertices
    // independently (single-precision STL is ~1e-5 mm at 100 mm), so weld those more loosely.
    bool step = filename.endsWith(".step", Qt::CaseInsensitive) || filename.endsWith(".stp", Qt::CaseInsensitive);
    weld_mesh_vertices(m, step ? 1e-7 : 1e-5);
    if (m.empty())
        throw std::runtime_error("Mesh contains no valid triangles");
    return m;
}
VolumeMesh tetrahedralize(const Mesh &source, double maxVolume) {
    // Projects saved by older STEP importers contain independent face vertices.
    // Normalize a private copy so painting IDs and saved validation fingerprints stay intact.
    Mesh mesh = source;
    weld_mesh_vertices(mesh);
    if (mesh.empty())
        throw std::runtime_error("Volume slicing requires a non-empty mesh");
    if (!std::isfinite(maxVolume) || maxVolume <= 0)
        throw std::runtime_error("Invalid maximum tetrahedron volume");
    std::map<std::pair<int, int>, int> edges;
    for (auto t : mesh.triangles)
        for (int k = 0; k < 3; ++k) {
            int a = t[k], b = t[(k + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++edges[{a, b}];
        }
    size_t boundary = 0, nonManifold = 0;
    for (auto [edge, count] : edges) {
        boundary += count == 1;
        nonManifold += count > 2;
    }
    if (boundary || nonManifold)
        throw std::runtime_error(
            "Volume slicing requires a closed manifold mesh: " + std::to_string(boundary) +
            " boundary edges and " + std::to_string(nonManifold) +
            " non-manifold edges remain after seam stitching. Repair the source CAD solid or mesh.");
    tetgenio in, out;
    in.numberofpoints = int(mesh.vertices.size());
    in.pointlist = new REAL[in.numberofpoints * 3];
    for (int i = 0; i < in.numberofpoints; ++i)
        for (int j = 0; j < 3; ++j)
            in.pointlist[3 * i + j] = mesh.vertices[i][j];
    in.numberoffacets = int(mesh.triangles.size());
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    for (int i = 0; i < in.numberoffacets; ++i) {
        auto &f = in.facetlist[i];
        tetgenio::init(&f);
        f.numberofpolygons = 1;
        f.polygonlist = new tetgenio::polygon[1];
        tetgenio::init(f.polygonlist);
        f.polygonlist[0].numberofvertices = 3;
        f.polygonlist[0].vertexlist = new int[3];
        for (int j = 0; j < 3; ++j)
            f.polygonlist[0].vertexlist[j] = mesh.triangles[i][j];
    }
    std::string options = "pq1.4a" + std::to_string(maxVolume) + "Q";
    std::vector<char> args(options.begin(), options.end());
    args.push_back(0);
    try {
        ::tetrahedralize(args.data(), &in, &out);
    } catch (int code) {
        throw std::runtime_error("Tetrahedral meshing failed (code " + std::to_string(code) + ")");
    }
    VolumeMesh result;
    InsideTester inside(mesh);
    for (int i = 0; i < out.numberofpoints; ++i)
        result.vertices.emplace_back(out.pointlist[3 * i], out.pointlist[3 * i + 1],
                                     out.pointlist[3 * i + 2]);
    for (int i = 0; i < out.numberoftetrahedra; ++i) {
        Tetrahedron t;
        for (int j = 0; j < 4; ++j)
            t.nodes.push_back(out.tetrahedronlist[i * out.numberofcorners + j]);
        Vec3 center = Vec3::Zero();
        for (int node : t.nodes)
            center += result.vertices[node] / 4;
        if (inside.contains(center))
            result.cells.push_back(t);
    }
    std::vector<Vec3> retained;
    std::map<int, int> remap;
    for (auto &cell : result.cells)
        for (int &node : cell.nodes) {
            auto it = remap.find(node);
            if (it == remap.end()) {
                int id = int(retained.size());
                retained.push_back(result.vertices[node]);
                remap[node] = id;
                node = id;
            } else
                node = it->second;
        }
    result.vertices = std::move(retained);
    if (result.cells.empty())
        throw std::runtime_error("Tetrahedralization produced no material volume");
    return result;
}
Mesh volume_surface(const VolumeMesh &mesh) {
    Mesh out;
    out.vertices = mesh.vertices;
    std::map<std::array<int, 3>, std::pair<int, Eigen::Vector3i>> faces;
    const int local[][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    for (const auto &t : mesh.cells)
        for (auto &f : local) {
            Eigen::Vector3i tri(t.nodes[f[0]], t.nodes[f[1]], t.nodes[f[2]]);
            std::array<int, 3> key = {tri[0], tri[1], tri[2]};
            std::sort(key.begin(), key.end());
            auto &item = faces[key];
            ++item.first;
            item.second = tri;
        }
    for (auto &[k, v] : faces)
        if (v.first == 1)
            out.triangles.push_back(v.second);
    return out;
}
bool barycentric(const VolumeMesh &mesh, const Tetrahedron &cell, const Vec3 &p, Eigen::Vector4d &weights) {
    if (cell.nodes.size() < 4)
        return false;
    const Vec3 &a = mesh.vertices[cell.nodes[0]];
    Mat3 d;
    for (int j = 0; j < 3; ++j)
        d.col(j) = mesh.vertices[cell.nodes[j + 1]] - a;
    if (std::abs(d.determinant()) < 1e-14)
        return false;
    Vec3 w = d.inverse() * (p - a);
    if (cell.nodes.size() == 10) {
        auto position = [&](const Vec3 &local) {
            Eigen::Vector4d q;
            q << 1 - local.sum(), local;
            Vec3 out = Vec3::Zero();
            const int edges[][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
            for (int i = 0; i < 4; ++i)
                out += q[i] * (2 * q[i] - 1) * mesh.vertices[cell.nodes[i]];
            for (int i = 0; i < 6; ++i)
                out += 4 * q[edges[i][0]] * q[edges[i][1]] * mesh.vertices[cell.nodes[i + 4]];
            return out;
        };
        for (int iteration = 0; iteration < 20; ++iteration) {
            Vec3 error = position(w) - p;
            if (error.norm() < 1e-9)
                break;
            Mat3 jacobian;
            for (int i = 0; i < 3; ++i) {
                Vec3 shift = Vec3::Zero();
                shift[i] = 1e-6;
                jacobian.col(i) = (position(w + shift) - position(w - shift)) / (2e-6);
            }
            if (std::abs(jacobian.determinant()) < 1e-14)
                return false;
            w -= jacobian.inverse() * error;
            if (!w.allFinite())
                return false;
        }
        if ((position(w) - p).norm() > 1e-6)
            return false;
    }
    weights << 1 - w.sum(), w;
    return weights.minCoeff() >= -1e-7 && weights.maxCoeff() <= 1 + 1e-7;
}
} // namespace fiveaxis
