#include "project.h"
#include "geometry.h"
#include "plate.h"
#include "process.h"
#include "profile.h"
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <algorithm>
#include <stdexcept>
#include <zip.h>

namespace fiveaxis {
static QJsonArray vec(const Vec3 &v) {
    return {v.x(), v.y(), v.z()};
}
static Vec3 vec(const QJsonValue &v) {
    auto a = v.toArray();
    if (a.size() != 3)
        throw std::runtime_error("Invalid project vector");
    Vec3 p(a[0].toDouble(), a[1].toDouble(), a[2].toDouble());
    if (!p.allFinite())
        throw std::runtime_error("Invalid project coordinate");
    return p;
}
static QJsonArray matrix(const Transform &t) {
    QJsonArray a;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            a.append(t.matrix()(i, j));
    return a;
}
static Transform transform(const QJsonValue &v) {
    auto a = v.toArray();
    if (a.size() != 16)
        throw std::runtime_error("Invalid project transform");
    Transform t;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            t.matrix()(i, j) = a[4 * i + j].toDouble();
    if (!t.matrix().allFinite() || (t.linear().transpose() * t.linear() - Mat3::Identity()).norm() > 1e-6 ||
        std::abs(t.linear().determinant() - 1) > 1e-6 ||
        (t.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6)
        throw std::runtime_error("Non-rigid project transform");
    return t;
}
static QJsonObject mesh_json(const Mesh &m) {
    QJsonArray v, f;
    for (auto &x : m.vertices)
        v.append(vec(x));
    for (auto &x : m.triangles)
        f.append(QJsonArray{x[0], x[1], x[2]});
    return {{"name", m.name}, {"vertices", v}, {"triangles", f}, {"error", m.error}};
}
static Mesh read_mesh(const QJsonObject &o) {
    Mesh m;
    m.name = o["name"].toString();
    m.error = o["error"].toDouble();
    for (auto x : o["vertices"].toArray())
        m.vertices.push_back(vec(x));
    for (auto x : o["triangles"].toArray()) {
        auto f = x.toArray();
        if (f.size() != 3)
            throw std::runtime_error("Invalid project triangle");
        Eigen::Vector3i t(f[0].toInt(-1), f[1].toInt(-1), f[2].toInt(-1));
        if (t.minCoeff() < 0 || t.maxCoeff() >= int(m.vertices.size()))
            throw std::runtime_error("Project triangle index out of range");
        m.triangles.push_back(t);
    }
    return m;
}
static QJsonObject volume_json(const VolumeMesh &m) {
    QJsonArray v, c;
    for (auto &x : m.vertices)
        v.append(vec(x));
    for (auto &x : m.cells) {
        QJsonArray n;
        for (int i : x.nodes)
            n.append(i);
        c.append(n);
    }
    return {{"vertices", v}, {"cells", c}};
}
static VolumeMesh read_volume(const QJsonObject &o) {
    VolumeMesh m;
    for (auto x : o["vertices"].toArray())
        m.vertices.push_back(vec(x));
    for (auto x : o["cells"].toArray()) {
        Tetrahedron t;
        for (auto n : x.toArray()) {
            int i = n.toInt(-1);
            if (i < 0 || i >= int(m.vertices.size()))
                throw std::runtime_error("Project tetrahedron index out of range");
            t.nodes.push_back(i);
        }
        if (t.nodes.size() != 4 && t.nodes.size() != 10)
            throw std::runtime_error("Invalid tetrahedron");
        m.cells.push_back(std::move(t));
    }
    return m;
}
static QJsonObject joint_json(const JointState &q) {
    return {{"xyz", vec(q.xyz)}, {"a", q.a}, {"b", q.b}};
}
static JointState read_joint(const QJsonObject &o) {
    JointState q;
    q.xyz = vec(o["xyz"]);
    q.a = o["a"].toDouble();
    q.b = o["b"].toDouble();
    return q;
}
static QJsonObject tool_json(const ToolheadAssembly &tool) {
    QJsonArray components;
    for (auto &c : tool.components)
        components.append(QJsonObject{{"id", c.id},
                                      {"name", c.name},
                                      {"mesh", mesh_json(c.mesh)},
                                      {"link", int(c.link)},
                                      {"visualOnly", c.visualOnly},
                                      {"nozzle", c.nozzle},
                                      {"renderMesh", mesh_json(c.renderMesh)},
                                      {"assemblyPlacement", matrix(c.assemblyPlacement)}});
    QJsonArray interfaces;
    for (const auto &a : tool.interfaces)
        interfaces.append(QJsonObject{{"first", a.first},
                                      {"second", a.second},
                                      {"center", vec(a.center)},
                                      {"size", vec(a.size)},
                                      {"regionFrame", matrix(a.regionFrame)}});
    return QJsonObject{{"sourceName", tool.sourceName},
                       {"interfaces", interfaces},
                       {"sourceUnits", tool.sourceUnits},
                       {"sourceHash", tool.sourceHash},
                       {"mount", matrix(tool.mount)},
                       {"tcp", vec(tool.tcp)},
                       {"axis", vec(tool.axis)},
                       {"contactRadius", tool.contactRadius},
                       {"contactHeight", tool.contactHeight},
                       {"components", components}};
}
static ToolheadAssembly read_toolhead(const QJsonObject &t) {
    ToolheadAssembly tool;
    if (t.isEmpty())
        return tool;
    tool.sourceName = t["sourceName"].toString();
    tool.sourceUnits = t["sourceUnits"].toString();
    for (auto item : t["interfaces"].toArray()) {
        auto a = item.toObject();
        ToolheadAssembly::InterfaceAllowance rule;
        rule.first = a["first"].toString();
        rule.second = a["second"].toString();
        rule.center = vec(a["center"]);
        rule.size = vec(a["size"]);
        if (a.contains("regionFrame"))
            rule.regionFrame = transform(a["regionFrame"]);
        if (rule.size.minCoeff() <= 0)
            throw std::runtime_error("Invalid interface box");
        tool.interfaces.push_back(rule);
    }
    tool.sourceHash = t["sourceHash"].toString();
    tool.mount = transform(t["mount"]);
    tool.tcp = vec(t["tcp"]);
    tool.axis = vec(t["axis"]);
    tool.contactRadius = t["contactRadius"].toDouble(.55);
    tool.contactHeight = t["contactHeight"].toDouble(.75);
    if (tool.axis.norm() < 1e-6)
        throw std::runtime_error("Invalid nozzle axis");
    for (auto x : t["components"].toArray()) {
        auto c = x.toObject();
        Component a;
        a.id = c["id"].toString();
        a.name = c["name"].toString();
        a.mesh = read_mesh(c["mesh"].toObject());
        a.link = Link(c["link"].toInt());
        a.visualOnly = c["visualOnly"].toBool();
        a.nozzle = c["nozzle"].toBool();
        if (c.contains("renderMesh"))
            a.renderMesh = read_mesh(c["renderMesh"].toObject());
        if (c.contains("assemblyPlacement"))
            a.assemblyPlacement = transform(c["assemblyPlacement"]);
        tool.components.push_back(a);
    }
    return tool;
}
QJsonObject project_json(const Project &p) {
    QJsonObject m{{"layout", int(p.machine.layout)},
                  {"controller", int(p.machine.controller)},
                  {"bedPivot", vec(p.machine.bedPivot)},
                  {"headPivotA", vec(p.machine.headPivotA)},
                  {"headPivotB", vec(p.machine.headPivotB)},
                  {"minXYZ", vec(p.machine.minXYZ)},
                  {"maxXYZ", vec(p.machine.maxXYZ)},
                  {"minA", p.machine.minA},
                  {"maxA", p.machine.maxA},
                  {"minB", p.machine.minB},
                  {"maxB", p.machine.maxB},
                  {"linearSpeed", p.machine.linearSpeed},
                  {"linearAccel", p.machine.linearAccel},
                  {"angularSpeed", p.machine.angularSpeed},
                  {"angularAccel", p.machine.angularAccel},
                  {"clearance", p.machine.clearance},
                  {"chordTolerance", p.machine.chordTolerance},
                  {"angularTolerance", p.machine.angularTolerance},
                  {"filamentDiameter", p.machine.filamentDiameter},
                  {"maxFlow", p.machine.maxFlow},
                  {"klipperExtension", p.machine.klipperExtension},
                  {"stockExtrusionRatio", p.machine.stockExtrusionRatio},
                  {"klipperRevision", p.machine.klipperRevision}};
    QJsonObject s = process_json(p.process);
    QJsonArray components, fixtures, path, tensors, scalar, fixed, forces, events, modeled;
    for (auto &f : p.fixtures)
        fixtures.append(QJsonObject{{"name", f.name},
                                    {"mesh", mesh_json(f.mesh)},
                                    {"movesWithBed", f.movesWithBed},
                                    {"sourceStep", QString::fromLatin1(f.sourceStep.toBase64())}});
    for (auto &x : p.path) {
        QJsonObject entry{{"position", vec(x.position)},
                          {"direction", vec(x.direction)},
                          {"joint", joint_json(x.joint)},
                          {"role", int(x.role)},
                          {"time", x.time},
                          {"deltaE", x.deltaE},
                          {"width", x.width},
                          {"height", x.height},
                          {"speed", x.speed},
                          {"layer", x.layer},
                          {"region", x.region}};
        if (x.surfaceNormal)
            entry["surfaceNormal"] = vec(*x.surfaceNormal);
        entry["objectId"] = x.objectId;
        entry["toolId"] = x.toolId;
        entry["materialId"] = x.materialId;
        entry["event"] = int(x.event);
        entry["argument"] = x.argument;
        entry["dwell"] = x.dwell;
        entry["contactComponent"] = x.contactComponent;
        entry["contactFixture"] = x.contactFixture;
        entry["contactCenter"] = vec(x.contactCenter);
        entry["contactSize"] = vec(x.contactSize);
        if (x.carriage)
            entry["carriage"] = joint_json(*x.carriage);
        path.append(entry);
    }
    for (auto &t : p.stress.tensors) {
        QJsonArray a;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                a.append(t(i, j));
        tensors.append(a);
    }
    for (double v : p.stress.scalar)
        scalar.append(v);
    for (int i : p.loads.fixedNodes)
        fixed.append(i);
    for (auto &[i, f] : p.loads.nodalForces)
        forces.append(QJsonObject{{"node", i}, {"force", vec(f)}});
    for (auto &e : p.report.events)
        events.append(QJsonObject{{"status", int(e.status)},
                                  {"first", e.first},
                                  {"second", e.second},
                                  {"reason", e.reason},
                                  {"move", e.move},
                                  {"time", e.time},
                                  {"clearance", e.clearance},
                                  {"contact", vec(e.contact)}});
    for (auto &c : p.report.modeledComponents)
        modeled.append(c);
    auto tool = tool_json(p.toolhead);
    QJsonObject stress{{"mesh", volume_json(p.stress.mesh)},
                       {"tensors", tensors},
                       {"scalar", scalar},
                       {"pointData", p.stress.pointData},
                       {"directional", p.stress.directional},
                       {"source", p.stress.source},
                       {"step", p.stress.step}};
    QJsonObject loads{{"fixed", fixed},
                      {"forces", forces},
                      {"young", p.loads.material.young},
                      {"poisson", p.loads.material.poisson},
                      {"transverseYoung", p.loads.material.transverseYoung}};
    QJsonObject report{{"status", int(p.report.status)},
                       {"fingerprint", p.report.fingerprint},
                       {"minimumClearance", p.report.minimumClearance},
                       {"checkedIntervals", qint64(p.report.checkedIntervals)},
                       {"events", events},
                       {"modeled", modeled}};
    QJsonObject comparison{{"available", p.comparison.available},
                           {"planarCompliance", p.comparison.planarCompliance},
                           {"curvedCompliance", p.comparison.curvedCompliance},
                           {"alignment", p.comparison.alignment}};
    QJsonArray unsupported, dependencies;
    for (int i : p.deposition.unsupportedMoves)
        unsupported.append(i);
    for (auto [a, b] : p.deposition.dependencies)
        dependencies.append(QJsonArray{a, b});
    QJsonObject deposition{{"checked", p.deposition.checked},
                           {"coverage", p.deposition.coverage},
                           {"unsupported", unsupported},
                           {"dependencies", dependencies}};
    QJsonObject result{
        {"schema", 2},
        {"validationEngine", 4},
        {"engineRevision", !p.machine.rotary_axes()  ? "0.3.3-cartesian-nonplanar-1"
                           : p.process.profileNormal ? "0.3.2-profile-normal-1"
                                                     : "0.3.0-workspace-process-1"},
        {"deposition", deposition},
        {"plateJob", plate_json(p)},
        {"part", mesh_json(p.part)},
        {"partPlacement", matrix(p.partPlacement)},
        {"volume", volume_json(p.volume)},
        {"toolhead", tool},
        {"dummyToolhead", tool_json(p.dummyToolhead)},
        {"useDummyToolhead", p.useDummyToolhead},
        {"machine", m},
        {"process", s},
        {"filament",
         QJsonObject{{"name", p.filamentName}, {"density", p.filamentDensity}, {"cost", p.filamentCost}}},
        {"stress", stress},
        {"loads", loads},
        {"comparison", comparison},
        {"fixtures", fixtures},
        {"path", path},
        {"pathProcessFingerprint", p.pathProcessFingerprint.isEmpty() && !p.plateEnabled
                                       ? process_fingerprint(p.process)
                                       : p.pathProcessFingerprint},
        {"report", report},
        {"diagnostics", QJsonArray::fromStringList(p.diagnostics)},
        {"depositionValid", p.depositionValid}};
    if (!p.profileGuide.empty()) {
        QJsonArray guide;
        for (const auto &station : p.profileGuide)
            guide.append(QJsonObject{{"center", vec(station.center)}, {"normal", vec(station.normal)}});
        result["profileGuide"] = guide;
    }
    return result;
}
Project project_from_json(const QJsonObject &j) {
    if (j["schema"].toInt() != 1 && j["schema"].toInt() != 2)
        throw std::runtime_error("Unsupported project schema");
    Project p;
    for (auto entry : j["profileGuide"].toArray()) {
        auto station = entry.toObject();
        p.profileGuide.push_back({vec(station["center"]), vec(station["normal"])});
    }
    if (!p.profileGuide.empty())
        validate_profile_guide(p.profileGuide);
    auto deposition = j["deposition"].toObject();
    p.deposition.checked = deposition["checked"].toBool();
    p.deposition.coverage = deposition["coverage"].toDouble();
    for (auto i : deposition["unsupported"].toArray())
        p.deposition.unsupportedMoves.push_back(i.toInt());
    for (auto i : deposition["dependencies"].toArray()) {
        auto a = i.toArray();
        if (a.size() != 2)
            throw std::runtime_error("Invalid dependency");
        p.deposition.dependencies.emplace_back(a[0].toInt(), a[1].toInt());
    }
    if (j.contains("partPlacement"))
        p.partPlacement = transform(j["partPlacement"]);
    p.part = read_mesh(j["part"].toObject());
    p.volume = read_volume(j["volume"].toObject());
    auto m = j["machine"].toObject();
    p.machine.layout = Layout(m["layout"].toInt());
    p.machine.controller = Controller(m["controller"].toInt());
    if (int(p.machine.layout) < 0 || int(p.machine.layout) > 2 || int(p.machine.controller) < 0 ||
        int(p.machine.controller) > 2)
        throw std::runtime_error("Unsupported saved machine layout or controller");
    p.machine.bedPivot = vec(m["bedPivot"]);
    p.machine.headPivotA = vec(m["headPivotA"]);
    p.machine.headPivotB = vec(m["headPivotB"]);
    p.machine.minXYZ = vec(m["minXYZ"]);
    p.machine.maxXYZ = vec(m["maxXYZ"]);
    auto number = [&](const char *key, double &v) {
        if (m.contains(key))
            v = m[key].toDouble();
    };
    number("minA", p.machine.minA);
    number("maxA", p.machine.maxA);
    number("minB", p.machine.minB);
    number("maxB", p.machine.maxB);
    number("linearSpeed", p.machine.linearSpeed);
    number("linearAccel", p.machine.linearAccel);
    number("angularSpeed", p.machine.angularSpeed);
    number("angularAccel", p.machine.angularAccel);
    number("clearance", p.machine.clearance);
    number("chordTolerance", p.machine.chordTolerance);
    number("angularTolerance", p.machine.angularTolerance);
    number("filamentDiameter", p.machine.filamentDiameter);
    number("maxFlow", p.machine.maxFlow);
    number("stockExtrusionRatio", p.machine.stockExtrusionRatio);
    p.machine.klipperExtension = m["klipperExtension"].toBool();
    p.machine.klipperRevision = m["klipperRevision"].toString();
    if (p.machine.linearSpeed <= 0 || p.machine.angularSpeed <= 0 || p.machine.linearAccel <= 0 ||
        p.machine.angularAccel <= 0 || p.machine.clearance < 0 || p.machine.filamentDiameter <= 0 ||
        p.machine.maxFlow <= 0)
        throw std::runtime_error("Invalid machine configuration");
    p.process = read_process(j["process"].toObject());
    auto filament = j["filament"].toObject();
    p.filamentName = filament["name"].toString("Generic PLA");
    p.filamentDensity = filament["density"].toDouble(1.24);
    p.filamentCost = filament["cost"].toDouble(25);
    if (!std::isfinite(p.filamentDensity) || p.filamentDensity <= 0 || !std::isfinite(p.filamentCost) ||
        p.filamentCost < 0)
        throw std::runtime_error("Invalid filament properties");
    p.pathProcessFingerprint = j["pathProcessFingerprint"].toString(process_fingerprint(p.process));
    p.toolhead = read_toolhead(j["toolhead"].toObject());
    p.useDummyToolhead = j["useDummyToolhead"].toBool();
    p.dummyToolhead = j.contains("dummyToolhead") ? read_toolhead(j["dummyToolhead"].toObject())
                                                  : make_dummy_toolhead(p.process.nozzleDiameter);
    for (auto x : j["fixtures"].toArray()) {
        auto a = x.toObject();
        p.fixtures.push_back({a["name"].toString(), read_mesh(a["mesh"].toObject()),
                              a["movesWithBed"].toBool(),
                              QByteArray::fromBase64(a["sourceStep"].toString().toLatin1())});
    }
    for (auto x : j["path"].toArray()) {
        auto a = x.toObject();
        Sample v;
        v.position = vec(a["position"]);
        v.direction = vec(a["direction"]);
        if (a.contains("surfaceNormal")) {
            v.surfaceNormal = vec(a["surfaceNormal"]);
            if (std::abs(v.surfaceNormal->norm() - 1) > 1e-6)
                throw std::runtime_error("Invalid saved surface normal");
        }
        v.joint = read_joint(a["joint"].toObject());
        int role = a["role"].toInt();
        if (role < 0 || role >= role_count)
            throw std::runtime_error("Invalid path role");
        v.role = Role(role);
        v.time = a["time"].toDouble();
        v.deltaE = a["deltaE"].toDouble();
        v.width = a["width"].toDouble();
        v.height = a["height"].toDouble();
        v.speed = a["speed"].toDouble();
        v.layer = a["layer"].toInt();
        v.region = a["region"].toInt();
        v.objectId = a["objectId"].toString();
        v.toolId = a["toolId"].toString();
        v.materialId = a["materialId"].toString();
        v.event = EventKind(a["event"].toInt());
        v.argument = a["argument"].toString();
        v.dwell = a["dwell"].toDouble();
        v.contactComponent = a["contactComponent"].toString();
        v.contactFixture = a["contactFixture"].toString();
        if (a.contains("contactCenter"))
            v.contactCenter = vec(a["contactCenter"]);
        if (a.contains("contactSize"))
            v.contactSize = vec(a["contactSize"]);
        if (a.contains("carriage"))
            v.carriage = read_joint(a["carriage"].toObject());
        if (!p.path.empty() && v.time < p.path.back().time)
            throw std::runtime_error("Nonmonotone project timeline");
        p.path.push_back(v);
    }
    auto st = j["stress"].toObject();
    p.stress.mesh = read_volume(st["mesh"].toObject());
    p.stress.pointData = st["pointData"].toBool();
    p.stress.directional = st["directional"].toBool();
    p.stress.source = st["source"].toString();
    p.stress.step = st["step"].toString();
    for (auto x : st["tensors"].toArray()) {
        auto a = x.toArray();
        if (a.size() != 9)
            throw std::runtime_error("Invalid tensor");
        Mat3 v;
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                v(i, k) = a[3 * i + k].toDouble();
        p.stress.tensors.push_back(v);
    }
    for (auto x : st["scalar"].toArray())
        p.stress.scalar.push_back(x.toDouble());
    if (p.stress.directional && p.stress.tensors.size() != (p.stress.pointData ? p.stress.mesh.vertices.size()
                                                                               : p.stress.mesh.cells.size()))
        throw std::runtime_error("Incomplete saved stress field");
    auto loads = j["loads"].toObject();
    p.loads.material.young = loads["young"].toDouble(2200);
    p.loads.material.poisson = loads["poisson"].toDouble(.35);
    p.loads.material.transverseYoung = loads["transverseYoung"].toDouble(1200);
    for (auto x : loads["fixed"].toArray())
        p.loads.fixedNodes.push_back(x.toInt());
    for (auto x : loads["forces"].toArray()) {
        auto a = x.toObject();
        p.loads.nodalForces.emplace_back(a["node"].toInt(), vec(a["force"]));
    }
    p.depositionValid = j["depositionValid"].toBool();
    auto r = j["report"].toObject();
    p.report.status = Status(r["status"].toInt(2));
    p.report.fingerprint = r["fingerprint"].toString();
    p.report.minimumClearance = r["minimumClearance"].toDouble();
    p.report.checkedIntervals = r["checkedIntervals"].toInteger();
    for (auto x : r["events"].toArray()) {
        auto e = x.toObject();
        p.report.events.push_back({Status(e["status"].toInt()), e["first"].toString(), e["second"].toString(),
                                   e["reason"].toString(), e["move"].toInt(), e["time"].toDouble(),
                                   e["clearance"].toDouble(), vec(e["contact"])});
    }
    for (auto x : r["modeled"].toArray())
        p.report.modeledComponents.push_back(x.toString());
    for (auto x : j["diagnostics"].toArray())
        p.diagnostics << x.toString();
    auto comp = j["comparison"].toObject();
    p.comparison = {comp["available"].toBool(), comp["planarCompliance"].toDouble(),
                    comp["curvedCompliance"].toDouble(), comp["alignment"].toDouble()};
    if (j["schema"].toInt() == 1) {
        migrate_single_project(p);
        p.invalidate();
        p.diagnostics << "Legacy schema 1 imported; validate the migrated job again.";
    } else
        read_plate(p, j["plateJob"].toObject());
    return p;
}
void save_project(const Project &p, const QString &filename) {
    int error = 0;
    zip_t *z = zip_open(filename.toUtf8(), ZIP_CREATE | ZIP_TRUNCATE, &error);
    if (!z)
        throw std::runtime_error("Cannot create project archive");
    QByteArray manifest = QJsonDocument(project_json(p)).toJson();
    auto add = [&](const char *name, const QByteArray &bytes) {
        auto source = zip_source_buffer(z, bytes.constData(), bytes.size(), 0);
        if (!source || zip_file_add(z, name, source, ZIP_FL_OVERWRITE) < 0) {
            if (source)
                zip_source_free(source);
            zip_discard(z);
            throw std::runtime_error("Cannot write project asset");
        }
    };
    add("manifest.json", manifest);
    if (!p.plateEnabled && !p.toolhead.sourceStep.isEmpty())
        add("assets/toolhead.step", p.toolhead.sourceStep);
    if (zip_close(z) != 0) {
        zip_discard(z);
        throw std::runtime_error("Cannot finish project archive");
    }
}
Project load_project(const QString &filename) {
    int error = 0;
    zip_t *z = zip_open(filename.toUtf8(), ZIP_RDONLY, &error);
    if (!z)
        throw std::runtime_error("Cannot open project archive");
    auto read = [&](const char *name) {
        zip_stat_t st;
        if (zip_stat(z, name, 0, &st) < 0)
            return QByteArray{};
        if (st.size > 512 * 1024 * 1024) {
            zip_close(z);
            throw std::runtime_error("Project asset exceeds size limit");
        }
        auto f = zip_fopen(z, name, 0);
        if (!f) {
            zip_close(z);
            throw std::runtime_error("Missing project asset");
        }
        QByteArray b;
        b.resize(st.size);
        auto count = zip_fread(f, b.data(), st.size);
        zip_fclose(f);
        if (count != zip_int64_t(st.size)) {
            zip_close(z);
            throw std::runtime_error("Truncated project asset");
        }
        return b;
    };
    QByteArray manifest = read("manifest.json"), step = read("assets/toolhead.step");
    zip_close(z);
    QJsonParseError parse;
    auto document = QJsonDocument::fromJson(manifest, &parse);
    if (parse.error != QJsonParseError::NoError)
        throw std::runtime_error("Invalid project manifest");
    Project p = project_from_json(document.object());
    if (!step.isEmpty())
        p.toolhead.sourceStep = step;
    if (document.object()["schema"].toInt() == 1 && !p.tools.empty())
        p.tools[0].assembly.sourceStep = step;
    if (!step.isEmpty() &&
        QString::fromLatin1(QCryptographicHash::hash(step, QCryptographicHash::Sha256).toHex()) !=
            p.toolhead.sourceHash)
        throw std::runtime_error("Source STEP does not match the cached geometry hash");
    return p;
}

QJsonObject plate_json(const Project &p) {
    QJsonArray objects, materials, tools, purges, order;
    QJsonObject assets;
    auto cad = [&](const ToolheadAssembly &assembly) {
        auto value = tool_json(assembly);
        if (!assembly.sourceStep.isEmpty()) {
            QString hash = QString::fromLatin1(
                QCryptographicHash::hash(assembly.sourceStep, QCryptographicHash::Sha256).toHex());
            value["sourceHash"] = hash;
            assets[hash] = QString::fromLatin1(assembly.sourceStep.toBase64());
        }
        return value;
    };
    for (const auto &o : p.objects) {
        Project local;
        local.part = o.mesh;
        local.volume = o.volume;
        local.process = o.process;
        local.stress = o.stress;
        local.loads = o.loads;
        local.profileGuide = o.guide;
        auto data = project_json(local);
        data.remove("plateJob");
        QJsonArray assignments;
        for (const auto &a : o.assignments) {
            QJsonArray faces;
            for (int face : a.faces)
                faces.append(face);
            assignments.append(QJsonObject{{"id", a.id},
                                           {"kind", int(a.kind)},
                                           {"material", a.materialId},
                                           {"tool", a.toolId},
                                           {"volume", mesh_json(a.volume)},
                                           {"faces", faces},
                                           {"role", int(a.role)},
                                           {"priority", a.priority}});
        }
        objects.append(QJsonObject{{"id", o.id},
                                   {"name", o.name},
                                   {"material", o.materialId},
                                   {"tool", o.toolId},
                                   {"placement", matrix(o.placement)},
                                   {"structuralFingerprint", o.structuralFingerprint},
                                   {"inheritProcess", o.inheritProcess},
                                   {"data", data},
                                   {"assignments", assignments}});
    }
    for (const auto &m : p.materials) {
        QJsonObject value{{"id", m.id},
                          {"name", m.name},
                          {"color", m.color},
                          {"diameter", m.diameter},
                          {"temperature", m.temperature},
                          {"standby", m.standby},
                          {"minTemperature", m.minTemperature},
                          {"maxTemperature", m.maxTemperature},
                          {"bedMin", m.bedMin},
                          {"bedMax", m.bedMax},
                          {"fan", m.fan},
                          {"maxFlow", m.maxFlow},
                          {"density", m.density},
                          {"cost", m.cost}};
        if (m.mechanical)
            value["mechanical"] = QJsonObject{{"young", m.mechanical->young},
                                              {"poisson", m.mechanical->poisson},
                                              {"transverseYoung", m.mechanical->transverseYoung}};
        materials.append(value);
    }
    auto recipe = [&](const std::vector<RecipeStep> &steps) {
        QJsonArray result;
        for (const auto &step : steps)
            result.append(QJsonObject{{"kind", int(step.kind)},
                                      {"carriage", joint_json(step.carriage)},
                                      {"argument", step.argument},
                                      {"seconds", step.seconds},
                                      {"component", step.contactComponent},
                                      {"fixture", step.contactFixture},
                                      {"center", vec(step.contactCenter)},
                                      {"size", vec(step.contactSize)}});
        return result;
    };
    for (const auto &t : p.tools) {
        QJsonArray routes;
        for (const auto &route : t.routes)
            routes.append(QJsonObject{{"material", route.materialId},
                                      {"slot", route.slot},
                                      {"command", route.selectCommand},
                                      {"confirmed", route.confirmed}});
        auto &n = t.nozzle;
        QJsonObject nozzle{{"name", n.name},
                           {"diameter", n.diameter},
                           {"component", n.componentId},
                           {"geometryHash", n.geometryHash},
                           {"minHeight", n.minHeight},
                           {"maxHeight", n.maxHeight},
                           {"minWidth", n.minWidth},
                           {"maxWidth", n.maxWidth},
                           {"maxFlow", n.maxFlow},
                           {"protrusion", n.protrusion},
                           {"calibrated", n.calibrated},
                           {"dummy", n.dummyNozzle},
                           {"separate", cad(n.separateNozzle)},
                           {"mount", matrix(n.nozzleMount)}};
        QJsonObject change{{"release", recipe(t.change.release)}, {"pickup", recipe(t.change.pickup)},
                           {"retract", t.change.retract},         {"load", t.change.load},
                           {"unload", t.change.unload},           {"seconds", t.change.changeSeconds},
                           {"calibrated", t.change.calibrated}};
        tools.append(QJsonObject{{"id", t.id},
                                 {"name", t.name},
                                 {"extruder", t.extruder},
                                 {"heater", t.heater},
                                 {"number", t.controllerTool},
                                 {"duetDrive", t.duetDrive},
                                 {"duetHeater", t.duetHeater},
                                 {"assembly", cad(t.assembly)},
                                 {"dummy", cad(t.dummy)},
                                 {"useDummy", t.useDummy},
                                 {"detachable", t.detachable},
                                 {"nozzle", nozzle},
                                 {"dock", matrix(t.dock)},
                                 {"change", change},
                                 {"routes", routes}});
    }
    for (const auto &v : p.plate.purge)
        purges.append(QJsonObject{{"from", v.from}, {"to", v.to}, {"volume", v.volume}});
    for (const auto &id : p.plate.order)
        order.append(id);
    const auto &q = p.plate;
    QJsonObject settings{{"sequence", int(q.sequence)},
                         {"automaticOrder", q.automaticOrder},
                         {"automaticTools", q.automaticTools},
                         {"purgeTower", q.purgeTower},
                         {"searchBudget", q.searchBudget},
                         {"supportInterfaceLayers", q.supportInterfaceLayers},
                         {"bedTemperature", q.bedTemperature},
                         {"towerWidth", q.towerWidth},
                         {"purgeCapacity", q.purgeCapacity},
                         {"purgePosition", vec(q.purgePosition)},
                         {"supportMaterial", q.supportMaterial},
                         {"interfaceMaterial", q.interfaceMaterial},
                         {"order", order},
                         {"purge", purges}};
    return {{"enabled", p.plateEnabled},
            {"objects", objects},
            {"materials", materials},
            {"tools", tools},
            {"carriage", cad(p.carriageAssembly)},
            {"settings", settings},
            {"assets", assets},
            {"compute", QJsonObject{{"threads", p.compute.threads},
                                    {"backend", int(p.compute.backend)},
                                    {"device", p.compute.device},
                                    {"memoryFraction", p.compute.memoryFraction}}},
            {"computation", QJsonObject{{"backend", p.computation.backend},
                                        {"device", p.computation.device},
                                        {"precision", p.computation.precision},
                                        {"fallback", p.computation.fallback},
                                        {"milliseconds", p.computation.milliseconds},
                                        {"cpuMilliseconds", p.computation.cpuMilliseconds},
                                        {"gpuMilliseconds", p.computation.gpuMilliseconds},
                                        {"batches", qint64(p.computation.batches)}}}};
}
void read_plate(Project &p, const QJsonObject &j) {
    if (j.isEmpty())
        return;
    p.plateEnabled = j["enabled"].toBool();
    auto assets = j["assets"].toObject();
    auto cad = [&](const QJsonValue &value) {
        auto a = read_toolhead(value.toObject());
        if (assets.contains(a.sourceHash)) {
            a.sourceStep = QByteArray::fromBase64(assets[a.sourceHash].toString().toLatin1());
            auto hash = QString::fromLatin1(
                QCryptographicHash::hash(a.sourceStep, QCryptographicHash::Sha256).toHex());
            if (hash != a.sourceHash)
                throw std::runtime_error("Corrupt embedded tool CAD asset");
        }
        return a;
    };
    for (auto entry : j["objects"].toArray()) {
        auto value = entry.toObject();
        auto data = value["data"].toObject();
        data.remove("plateJob");
        data["schema"] = 2;
        auto local = project_from_json(data);
        PlateObject o;
        o.id = value["id"].toString();
        o.name = value["name"].toString();
        o.materialId = value["material"].toString();
        o.toolId = value["tool"].toString();
        o.placement = transform(value["placement"]);
        o.mesh = local.part;
        o.volume = local.volume;
        o.process = local.process;
        o.stress = local.stress;
        o.loads = local.loads;
        o.guide = local.profileGuide;
        o.structuralFingerprint = value["structuralFingerprint"].toString();
        o.inheritProcess = value["inheritProcess"].toBool(true);
        for (auto item : value["assignments"].toArray()) {
            auto v = item.toObject();
            MaterialAssignment a;
            a.id = v["id"].toString();
            a.kind = AssignmentKind(v["kind"].toInt());
            a.materialId = v["material"].toString();
            a.toolId = v["tool"].toString();
            a.volume = read_mesh(v["volume"].toObject());
            a.role = Role(v["role"].toInt());
            a.priority = v["priority"].toInt();
            for (auto face : v["faces"].toArray()) {
                int i = face.toInt(-1);
                if (i < 0 || i >= int(o.mesh.triangles.size()))
                    throw std::runtime_error("Paint face is outside the object mesh");
                a.faces.push_back(i);
            }
            o.assignments.push_back(a);
        }
        p.objects.push_back(std::move(o));
    }
    for (auto entry : j["materials"].toArray()) {
        auto v = entry.toObject();
        MaterialProfile m;
        m.id = v["id"].toString();
        m.name = v["name"].toString();
        m.color = v["color"].toString();
        m.diameter = v["diameter"].toDouble();
        m.temperature = v["temperature"].toDouble();
        m.standby = v["standby"].toDouble();
        m.minTemperature = v["minTemperature"].toDouble();
        m.maxTemperature = v["maxTemperature"].toDouble();
        m.bedMin = v["bedMin"].toDouble();
        m.bedMax = v["bedMax"].toDouble();
        m.fan = v["fan"].toDouble();
        m.maxFlow = v["maxFlow"].toDouble();
        m.density = v["density"].toDouble();
        m.cost = v["cost"].toDouble();
        if (v.contains("mechanical")) {
            auto a = v["mechanical"].toObject();
            m.mechanical =
                Material{a["young"].toDouble(), a["poisson"].toDouble(), a["transverseYoung"].toDouble()};
        }
        p.materials.push_back(m);
    }
    auto recipe = [&](const QJsonValue &value) {
        std::vector<RecipeStep> steps;
        for (auto entry : value.toArray()) {
            auto v = entry.toObject();
            RecipeStep step;
            step.kind = EventKind(v["kind"].toInt());
            step.carriage = read_joint(v["carriage"].toObject());
            step.argument = v["argument"].toString();
            step.seconds = v["seconds"].toDouble();
            step.contactComponent = v["component"].toString();
            step.contactFixture = v["fixture"].toString();
            step.contactCenter = vec(v["center"]);
            step.contactSize = vec(v["size"]);
            steps.push_back(step);
        }
        return steps;
    };
    for (auto entry : j["tools"].toArray()) {
        auto v = entry.toObject();
        ToolDefinition t;
        t.id = v["id"].toString();
        t.name = v["name"].toString();
        t.extruder = v["extruder"].toString();
        t.heater = v["heater"].toString();
        t.controllerTool = v["number"].toInt();
        t.duetDrive = v["duetDrive"].toInt(t.controllerTool);
        t.duetHeater = v["duetHeater"].toInt(t.controllerTool);
        t.assembly = cad(v["assembly"]);
        t.dummy = cad(v["dummy"]);
        t.useDummy = v["useDummy"].toBool();
        t.detachable = v["detachable"].toBool();
        t.dock = transform(v["dock"]);
        auto n = v["nozzle"].toObject();
        t.nozzle.name = n["name"].toString();
        t.nozzle.diameter = n["diameter"].toDouble();
        t.nozzle.componentId = n["component"].toString();
        t.nozzle.geometryHash = n["geometryHash"].toString();
        t.nozzle.minHeight = n["minHeight"].toDouble();
        t.nozzle.maxHeight = n["maxHeight"].toDouble();
        t.nozzle.minWidth = n["minWidth"].toDouble();
        t.nozzle.maxWidth = n["maxWidth"].toDouble();
        t.nozzle.maxFlow = n["maxFlow"].toDouble();
        t.nozzle.protrusion = n["protrusion"].toDouble();
        t.nozzle.calibrated = n["calibrated"].toBool();
        t.nozzle.dummyNozzle = n["dummy"].toBool();
        t.nozzle.separateNozzle = cad(n["separate"]);
        t.nozzle.nozzleMount = transform(n["mount"]);
        auto c = v["change"].toObject();
        t.change.release = recipe(c["release"]);
        t.change.pickup = recipe(c["pickup"]);
        t.change.retract = c["retract"].toDouble();
        t.change.load = c["load"].toDouble();
        t.change.unload = c["unload"].toDouble();
        t.change.changeSeconds = c["seconds"].toDouble();
        t.change.calibrated = c["calibrated"].toBool();
        for (auto entry : v["routes"].toArray()) {
            auto r = entry.toObject();
            t.routes.push_back({r["material"].toString(), r["slot"].toString(), r["command"].toString(),
                                r["confirmed"].toBool()});
        }
        p.tools.push_back(std::move(t));
    }
    p.carriageAssembly = cad(j["carriage"]);
    auto q = j["settings"].toObject();
    p.plate.sequence = PrintSequence(q["sequence"].toInt());
    p.plate.automaticOrder = q["automaticOrder"].toBool(true);
    p.plate.automaticTools = q["automaticTools"].toBool(true);
    p.plate.purgeTower = q["purgeTower"].toBool(true);
    p.plate.searchBudget = q["searchBudget"].toInt(32);
    p.plate.supportInterfaceLayers = q["supportInterfaceLayers"].toInt(2);
    p.plate.bedTemperature = q["bedTemperature"].toDouble(60);
    p.plate.towerWidth = q["towerWidth"].toDouble(12);
    p.plate.purgeCapacity = q["purgeCapacity"].toDouble();
    p.plate.purgePosition = vec(q["purgePosition"]);
    p.plate.supportMaterial = q["supportMaterial"].toString();
    p.plate.interfaceMaterial = q["interfaceMaterial"].toString();
    for (auto id : q["order"].toArray())
        p.plate.order.push_back(id.toString());
    for (auto entry : q["purge"].toArray()) {
        auto v = entry.toObject();
        p.plate.purge.push_back({v["from"].toString(), v["to"].toString(), v["volume"].toDouble()});
    }
    auto c = j["compute"].toObject();
    p.compute.threads = c["threads"].toInt();
    p.compute.backend = ComputeBackend(c["backend"].toInt());
    p.compute.device = c["device"].toString();
    p.compute.memoryFraction = c["memoryFraction"].toDouble(.6);
    auto r = j["computation"].toObject();
    p.computation.backend = r["backend"].toString("CPU");
    p.computation.device = r["device"].toString();
    p.computation.precision = r["precision"].toString("FP64 CPU");
    p.computation.fallback = r["fallback"].toString();
    p.computation.milliseconds = r["milliseconds"].toDouble();
    p.computation.cpuMilliseconds = r["cpuMilliseconds"].toDouble();
    p.computation.gpuMilliseconds = r["gpuMilliseconds"].toDouble();
    p.computation.batches = r["batches"].toInteger();
    rebuild_job_events(p);
}

Project demo_project() {
    Project p;
    p.part = box_mesh(Vec3(0, 0, 3), Vec3(20, 20, 6), "Research block");
    p.fixtures.push_back({"Bed", box_mesh(Vec3(0, 0, -2), Vec3(100, 100, 4), "Bed"), true});
    p.toolhead.components.push_back(
        {"nozzle", "Nozzle", cylinder_mesh(Vec3(0, 0, 1), .4, 2, "Nozzle"), Link::Nozzle, false, true});
    p.toolhead.components.push_back({"break", "Heat break", cylinder_mesh(Vec3(0, 0, 5), 1, 6, "Heat break"),
                                     Link::Nozzle, false, false});
    p.toolhead.components.push_back({"heater", "Heater block",
                                     box_mesh(Vec3(0, 0, 12), Vec3(10, 8, 8), "Heater block"), Link::Nozzle,
                                     false, false});
    p.toolhead.components.push_back({"fan", "Fan shroud",
                                     box_mesh(Vec3(11, 0, 18), Vec3(8, 12, 12), "Fan shroud"), Link::Nozzle,
                                     false, false});
    p.dummyToolhead = std::move(p.toolhead);
    p.dummyToolhead.sourceName = "Dummy demonstration toolhead";
    p.dummyToolhead.sourceUnits = "synthetic millimetres";
    p.useDummyToolhead = true;
    p.process.nozzleDiameter = .8;
    p.process.layerHeight = .5;
    p.process.width = .8;
    p.process.infill = .25;
    p.machine.clearance = .25;
    p.dummyToolhead.contactHeight = .5;
    p.dummyToolhead.contactRadius = .65;
    Sample start;
    start.position = Vec3(-9, -9, 10);
    start.joint.xyz = start.position;
    p.path.push_back(start);
    double area = bead_area(.8, .5), filament = pi * std::pow(1.75 / 2, 2);
    for (int layer = 0; layer < 12; ++layer) {
        double z = (layer + 1) * .5;
        std::array<Vec3, 5> corners = {Vec3(-9, -9, z), Vec3(9, -9, z), Vec3(9, 9, z), Vec3(-9, 9, z),
                                       Vec3(-9, -9, z)};
        Sample travel;
        travel.position = corners[0];
        travel.joint.xyz = travel.position;
        travel.width = .8;
        travel.height = .5;
        travel.layer = layer;
        p.path.push_back(travel);
        for (int edge = 0; edge < 4; ++edge)
            for (int j = 1; j <= 18; ++j) {
                Sample s;
                s.position = corners[edge] + (corners[edge + 1] - corners[edge]) * double(j) / 18;
                s.direction = Vec3::UnitZ();
                s.joint.xyz = s.position;
                s.deltaE = (s.position - p.path.back().position).norm() * area / filament;
                s.width = .8;
                s.height = .5;
                s.role = Role::Wall;
                s.layer = layer;
                p.path.push_back(s);
            }
    }
    p.depositionValid = true;
    time_path(p);
    p.diagnostics << "Synthetic machine and material: geometric research demonstration.";
    return p;
}
} // namespace fiveaxis
