#include "step_import.h"
#include "project.h"
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Poly_Triangulation.hxx>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Ax2.hxx>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace fiveaxis {
static std::mutex stepMutex;
ToolheadAssembly load_step_assembly(const QString &filename, double deflection, std::atomic_bool *cancel) {
    StepImportOptions options; options.linearDeflection=deflection;
    return load_step_assembly(filename,options,cancel);
}
ToolheadAssembly load_step_assembly(const QString &filename, const StepImportOptions &options, std::atomic_bool *cancel) {
    std::lock_guard<std::mutex> lock(stepMutex);
    QFile input(filename);
    if (!input.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot read STEP file");
    ToolheadAssembly out;
    out.sourceName = filename;
    out.sourceStep = input.readAll();
    out.sourceHash =
        QString::fromLatin1(QCryptographicHash::hash(out.sourceStep, QCryptographicHash::Sha256).toHex());
    double deflection=options.linearDeflection;
    if (deflection <= 0 || !std::isfinite(deflection) || !std::isfinite(options.angularDeflection) ||
        options.angularDeflection<=0 || options.angularDeflection>pi || !std::isfinite(options.renderDeflection) || options.renderDeflection<=0)
        throw std::runtime_error("Invalid STEP tessellation tolerance");
    QString cacheRoot = QCoreApplication::applicationDirPath() + "/cache/step";
    QString cached =
        cacheRoot + "/" + out.sourceHash + "-" + QString::number(deflection, 'g', 12) + "-"+
        QString::number(options.angularDeflection,'g',12)+"-"+QString::number(options.renderDeflection,'g',12)+
        (options.splitCompounds?"-split":"-whole")+"-v3.5axis";
    if (QFile::exists(cached))
        try {
            auto stored = load_project(cached);
            if (stored.toolhead.sourceHash == out.sourceHash) {
                stored.toolhead.sourceName = filename;
                return stored.toolhead;
            }
        } catch (...) {
        }
    auto app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);
    struct Close {
        Handle(XCAFApp_Application) a;
        Handle(TDocStd_Document) d;
        ~Close() {
            a->Close(d);
        }
    } close{app, doc};
    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    reader.SetColorMode(true);
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    std::istringstream source(out.sourceStep.toStdString());
    if (reader.ReadStream(filename.toUtf8().constData(), source) != IFSelect_RetDone || !reader.Transfer(doc))
        throw std::runtime_error("STEP translation failed");
    TColStd_SequenceOfAsciiString lengthUnits, angleUnits, solidUnits;
    reader.ChangeReader().FileUnits(lengthUnits, angleUnits, solidUnits);
    for (int i = 1; i <= lengthUnits.Length(); ++i)
        out.sourceUnits += QString::fromLatin1(lengthUnits.Value(i).ToCString()) + " ";
    auto shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence roots;
    shapes->GetFreeShapes(roots);
    std::function<void(TDF_Label, TopLoc_Location, QString, int)> visit =
        [&](TDF_Label instance, TopLoc_Location parent, QString path, int depth) {
            if (cancel && cancel->load())
                throw std::runtime_error("STEP import cancelled");
            if (depth > 64)
                throw std::runtime_error("STEP assembly nesting too deep");
            TDF_Label definition = instance;
            if (shapes->IsReference(instance))
                shapes->GetReferredShape(instance, definition);
            TopLoc_Location placement = parent * shapes->GetLocation(instance);
            Handle(TDataStd_Name) n;
            QString name = "Component";
            if (definition.FindAttribute(TDataStd_Name::GetID(), n))
                name = QString::fromUtf16(reinterpret_cast<const char16_t *>(n->Get().ToExtString()));
            QString id = path + "/" + QString::number(instance.Tag());
            TDF_LabelSequence children;
            if (shapes->GetComponents(definition, children)) {
                for (int j = 1; j <= children.Length(); ++j)
                    visit(children.Value(j), placement, id, depth + 1);
                return;
            }
            TopoDS_Shape shape = shapes->GetShape(definition);
            std::vector<TopoDS_Shape> pieces;
            if(options.splitCompounds && (shape.ShapeType()==TopAbs_COMPOUND || shape.ShapeType()==TopAbs_COMPSOLID))
                for(TopExp_Explorer ex(shape,TopAbs_SOLID);ex.More();ex.Next()) pieces.push_back(ex.Current());
            if(pieces.empty()) pieces.push_back(shape);
            for(size_t piece=0;piece<pieces.size();++piece) {
            auto part=pieces[piece];
            Component comp;
            comp.id = id+(pieces.size()>1?"/solid/"+QString::number(piece):QString{});
            comp.name = name+(pieces.size()>1?" · solid "+QString::number(piece+1):QString{});
            comp.mesh.name = name;
            comp.mesh.error = deflection;
            comp.nozzle = name.contains("nozzle", Qt::CaseInsensitive);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j)
                    comp.assemblyPlacement.matrix()(i, j) = placement.Transformation().Value(i + 1, j + 1);
            auto tessellate = [&](double tolerance) {
                BRepMesh_IncrementalMesh mesher(part, tolerance, false, options.angularDeflection, true);
                if (!mesher.IsDone())
                    throw std::runtime_error("STEP tessellation failed");
                Mesh mesh;
                mesh.name = name;
                mesh.error = tolerance;
                for (TopExp_Explorer ex(part, TopAbs_FACE); ex.More(); ex.Next()) {
                    TopLoc_Location loc;
                    auto face = TopoDS::Face(ex.Current());
                    auto tri = BRep_Tool::Triangulation(face, loc);
                    if (tri.IsNull())
                        throw std::runtime_error("STEP face has no tessellation");
                    int offset = int(mesh.vertices.size());
                    gp_Trsf tr = (placement * loc).Transformation();
                    for (int j = 1; j <= tri->NbNodes(); ++j) {
                        gp_Pnt p = tri->Node(j).Transformed(tr);
                        mesh.vertices.emplace_back(p.X(), p.Y(), p.Z());
                    }
                    for (int j = 1; j <= tri->NbTriangles(); ++j) {
                        int a, b, c;
                        tri->Triangle(j).Get(a, b, c);
                        if (face.Orientation() == TopAbs_REVERSED)
                            std::swap(b, c);
                        mesh.triangles.emplace_back(offset + a - 1, offset + b - 1, offset + c - 1);
                    }
                }
                return mesh;
            };
            comp.renderMesh = tessellate(std::max(options.renderDeflection, deflection));
            comp.mesh = tessellate(deflection);
            if (!comp.mesh.empty())
                out.components.push_back(std::move(comp));
            }
        };
    for (int i = 1; i <= roots.Length(); ++i)
        visit(roots.Value(i), TopLoc_Location{}, "root", 0);
    if (out.components.empty())
        throw std::runtime_error("STEP contains no tessellatable components");
    if (QDir().mkpath(cacheRoot))
        try {
            Project stored;
            stored.toolhead = out;
            save_project(stored, cached);
        } catch (...) {
        }
    return out;
}
void write_demo_step(const QString &filename, bool nested, bool inches, bool compound) {
    std::lock_guard<std::mutex> lock(stepMutex);
    auto app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("BinXCAF", doc);
    auto st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    auto root = st->NewShape();
    TDataStd_Name::Set(root, "Reference toolhead");
    auto add = [&](const TopoDS_Shape &s, const char *name, const TDF_Label &parent) {
        auto d = st->AddShape(s, false);
        TDataStd_Name::Set(d, name);
        st->AddComponent(parent, d, TopLoc_Location{});
        return d;
    };
    add(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0.4, 2).Shape(), "Nozzle", root);
    add(BRepPrimAPI_MakeBox(gp_Pnt(-5, -4, 8), 10, 8, 8).Shape(), "Heater block", root);
    add(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 2), gp_Dir(0, 0, 1)), 1, 8).Shape(), "Heat break", root);
    TopoDS_Shape fanShape=BRepPrimAPI_MakeBox(gp_Pnt(7,-6,12),8,12,12).Shape();
    if(compound) {TopoDS_Compound solids;BRep_Builder builder;builder.MakeCompound(solids);builder.Add(solids,fanShape);builder.Add(solids,BRepPrimAPI_MakeBox(gp_Pnt(7,8,12),8,3,12).Shape());fanShape=solids;}
    auto fan = add(fanShape, "Fan shroud", root);
    if (nested) {
        auto sub = st->NewShape();
        TDataStd_Name::Set(sub, "Repeated carriage");
        gp_Trsf tr;
        tr.SetTranslation(gp_Vec(20, 0, 0));
        st->AddComponent(sub, fan, TopLoc_Location(tr));
        st->AddComponent(root, sub, TopLoc_Location{});
    }
    st->UpdateAssemblies();
    Interface_Static::SetCVal("write.step.unit", inches ? "INCH" : "MM");
    STEPCAFControl_Writer writer;
    writer.Transfer(doc, STEPControl_AsIs);
    if (writer.Write(filename.toUtf8().constData()) != IFSelect_RetDone) {
        app->Close(doc);
        throw std::runtime_error("Cannot write STEP fixture");
    }
    app->Close(doc);
}
} // namespace fiveaxis
