#include "core/geometry.h"
#include "core/plate.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <QTemporaryDir>
#include <QtTest/QtTest>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <limits>
#include <map>
using namespace fiveaxis;

static void write_solid(const QString &file, int kind) {
    TopoDS_Shape shape;
    if (kind == 0)
        shape = BRepPrimAPI_MakeBox(4, 4, 3).Shape();
    else if (kind == 1)
        shape = BRepPrimAPI_MakeCylinder(2, 4).Shape();
    else if (kind == 2)
        shape = BRepPrimAPI_MakeTorus(6, 1.5, pi * .65).Shape();
    else {
        auto box = BRepPrimAPI_MakeBox(4, 4, 3).Shape();
        shape = TopExp_Explorer(box, TopAbs_FACE).Current();
    }
    STEPControl_Writer writer;
    if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone ||
        writer.Write(file.toUtf8().constData()) != IFSelect_RetDone)
        throw std::runtime_error("Cannot write STEP solid fixture");
}
static bool closed(const Mesh &mesh) {
    std::map<std::pair<int, int>, int> edges;
    for (auto tri : mesh.triangles)
        for (int k = 0; k < 3; ++k) {
            int a = tri[k], b = tri[(k + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++edges[{a, b}];
        }
    return !edges.empty() &&
           std::all_of(edges.begin(), edges.end(), [](auto &edge) { return edge.second == 2; });
}
static Mesh face_vertices(const Mesh &indexed) {
    Mesh mesh;
    mesh.name = indexed.name;
    for (auto face : indexed.triangles) {
        int start = int(mesh.vertices.size());
        for (int index : face)
            mesh.vertices.push_back(indexed.vertices[index]);
        mesh.triangles.emplace_back(start, start + 1, start + 2);
    }
    return mesh;
}
class StepMeshTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
    void solid_seams_data() {
        QTest::addColumn<int>("kind");
        QTest::addColumn<bool>("fine");
        for (int kind = 0; kind < 3; ++kind)
            for (bool fine : {false, true})
                QTest::newRow(qPrintable(QString("%1-%2").arg(kind).arg(fine ? "fine" : "coarse")))
                    << kind << fine;
    }
    void solid_seams() {
        QFETCH(int, kind);
        QFETCH(bool, fine);
        QTemporaryDir dir;
        auto file = dir.filePath("solid.step");
        write_solid(file, kind);
        StepImportOptions options;
        options.linearDeflection = fine ? .02 : .2;
        options.angularDeflection = fine ? .0872664626 : .6;
        auto assembly = load_step_assembly(file, options);
        QCOMPARE(assembly.components.size(), size_t(1));
        const auto &component = assembly.components.front();
        // The desktop import uses these meshes directly, bypassing load_mesh().
        QVERIFY2(closed(component.mesh), "STEP face seams remain open in the collision/part mesh");
        QVERIFY2(closed(component.renderMesh), "STEP face seams remain open in the rendering mesh");
        QVERIFY(!tetrahedralize(component.mesh, 1).cells.empty());
        auto cached = load_step_assembly(file, options);
        QVERIFY(closed(cached.components.front().mesh));
        QCOMPARE(cached.components.front().mesh.vertices.size(), component.mesh.vertices.size());
        QVERIFY(closed(load_mesh(file)));
    }
    void legacy_project_seams() {
        QTemporaryDir dir;
        Project p;
        p.part = face_vertices(box_mesh(Vec3(0, 0, 2), Vec3(4, 4, 4)));
        QVERIFY(!closed(p.part));
        auto file = dir.filePath("old-step.5axis");
        save_project(p, file);
        auto restored = load_project(file);
        auto before = restored.fingerprint();
        QVERIFY(!tetrahedralize(restored.part, 1).cells.empty());
        QCOMPARE(restored.fingerprint(), before);
        QCOMPARE(restored.part.vertices.size(), p.part.vertices.size());
    }
    void genuine_open_mesh() {
        auto mesh = box_mesh(Vec3::Zero(), Vec3(4, 4, 4));
        mesh.triangles.erase(mesh.triangles.begin());
        QVERIFY_EXCEPTION_THROWN(tetrahedralize(mesh), std::runtime_error);
        auto gap = face_vertices(box_mesh(Vec3::Zero(), Vec3(4, 4, 4)));
        // Much smaller than STEP tessellation deflection, but still a real gap.
        gap.vertices[0].z() += .0001;
        QVERIFY_EXCEPTION_THROWN(tetrahedralize(gap), std::runtime_error);
    }
    void open_step_surface() {
        QTemporaryDir dir;
        auto file = dir.filePath("open.step");
        write_solid(file, 3);
        auto mesh = load_step_assembly(file).components.front().mesh;
        QVERIFY(!closed(mesh));
        try {
            tetrahedralize(mesh);
            QFAIL("An open CAD face must not become a printable solid");
        } catch (const std::runtime_error &e) {
            QVERIFY(QString(e.what()).contains("4 boundary edges"));
        }
    }
    void assemblies_data() {
        QTest::addColumn<bool>("inches");
        QTest::newRow("mm") << false;
        QTest::newRow("inches") << true;
    }
    void assemblies() {
        QFETCH(bool, inches);
        QTemporaryDir dir;
        auto file = dir.filePath("nested.step");
        write_demo_step(file, true, inches, true);
        StepImportOptions options;
        options.splitCompounds = true;
        auto assembly = load_step_assembly(file, options);
        for (const auto &component : assembly.components) {
            QVERIFY(closed(component.mesh));
            QVERIFY(closed(component.renderMesh));
        }
    }
    void seam_precision() {
        auto mesh = face_vertices(box_mesh(Vec3::Zero(), Vec3(4, 4, 4)));
        for (size_t i = 0; i < mesh.vertices.size(); ++i)
            mesh.vertices[i].x() += i == 0 ? 5.0001e-8 : 4.9999e-8;
        weld_mesh_vertices(mesh);
        QCOMPARE(mesh.vertices.size(), size_t(8));
        QCOMPARE(mesh.triangles.size(), size_t(12));
        QVERIFY(closed(mesh));
        auto large = face_vertices(box_mesh(Vec3(1e8, -1e8, 1e8), Vec3(4, 4, 4)));
        weld_mesh_vertices(large);
        QVERIFY(closed(large));
        QCOMPARE(large.vertices.size(), size_t(8));
    }
    void invalid_mesh_inputs() {
        auto mesh = box_mesh(Vec3::Zero(), Vec3(4, 4, 4));
        auto nonfinite = mesh;
        nonfinite.vertices[0].x() = std::numeric_limits<double>::quiet_NaN();
        QVERIFY_EXCEPTION_THROWN(tetrahedralize(nonfinite), std::runtime_error);
        mesh.triangles[0][0] = -1;
        QVERIFY_EXCEPTION_THROWN(tetrahedralize(mesh), std::runtime_error);
        QVERIFY_EXCEPTION_THROWN(tetrahedralize(Mesh{}), std::runtime_error);
    }
    void native_mesh_slicing_data() {
        QTest::addColumn<int>("layout");
        QTest::newRow("XYZ") << int(Layout::Cartesian3Axis);
        QTest::newRow("rotary-bed") << int(Layout::RotaryBed);
        QTest::newRow("tilting-head") << int(Layout::TiltingHead);
    }
    void native_mesh_slicing() {
        QFETCH(int, layout);
        QTemporaryDir dir;
        auto file = dir.filePath("printable.step");
        write_solid(file, 1);
        auto fixture = qEnvironmentVariable("FIVEAXIS_STEP_FIXTURE");
        if (!fixture.isEmpty() && layout == int(Layout::RotaryBed))
            QVERIFY(QFile::copy(file, fixture));
        Project p;
        p.machine.layout = Layout(layout);
        p.process.curved = false;
        p.process.layerHeight = .4;
        p.process.width = .6;
        p.process.nozzleDiameter = .6;
        p.useDummyToolhead = true;
        p.dummyToolhead = make_dummy_toolhead(.6);
        p.fixtures.push_back({"Bed", box_mesh(Vec3(0, 0, -2), Vec3(100, 100, 4)), true});
        p.part = load_step_assembly(file).components.front().mesh;
        try {
            slice_project(p);
            QVERIFY(!p.path.empty());
            QVERIFY(!p.volume.cells.empty());
            enable_plate(p);
            slice_plate(p);
            QVERIFY(!p.path.empty());
            QVERIFY(!p.events.empty());
            if (!fixture.isEmpty() && layout == int(Layout::RotaryBed))
                save_project(p, fixture + ".5axis");
        } catch (const std::exception &e) {
            QFAIL(e.what());
        }
    }
};
QTEST_GUILESS_MAIN(StepMeshTests)
#include "step_mesh_tests.moc"
