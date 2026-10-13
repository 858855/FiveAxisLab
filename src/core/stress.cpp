#include "stress.h"
#include "geometry.h"
#include "slicing.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <QFile>
#include <algorithm>
#include <set>
#include <optional>
#include <stdexcept>
#include <vtkCell.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkErrorCode.h>
#include <vtkFieldData.h>
#include <vtkInformation.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridReader.h>

namespace fiveaxis {
static vtkSmartPointer<vtkUnstructuredGrid> read_vtu(const QString &filename) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot open VTU results");
    auto bytes = file.readAll();
    vtkNew<vtkXMLUnstructuredGridReader> reader;
    reader->ReadFromInputStringOn();
    reader->SetBinaryInputString(bytes.constData(), bytes.size());
    reader->Update();
    if (reader->GetErrorCode() != vtkErrorCode::NoError || !reader->GetOutput() ||
        reader->GetOutput()->GetNumberOfPoints() == 0)
        throw std::runtime_error("Cannot read VTU results");
    auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->DeepCopy(reader->GetOutput());
    return grid;
}
VtuInfo inspect_vtu(const QString &filename) {
    auto g = read_vtu(filename);
    VtuInfo info;
    info.points = g->GetNumberOfPoints();
    info.cells = g->GetNumberOfCells();
    for (int i = 0; i < g->GetPointData()->GetNumberOfArrays(); ++i)
        if (g->GetPointData()->GetArrayName(i))
            info.pointArrays << g->GetPointData()->GetArrayName(i);
    for (int i = 0; i < g->GetCellData()->GetNumberOfArrays(); ++i)
        if (g->GetCellData()->GetArrayName(i))
            info.cellArrays << g->GetCellData()->GetArrayName(i);
    return info;
}
static VolumeMesh vtk_mesh(vtkUnstructuredGrid *g, const VtuOptions &o) {
    VolumeMesh m;
    for (vtkIdType i = 0; i < g->GetNumberOfPoints(); ++i) {
        double a[3];
        g->GetPoint(i, a);
        Vec3 p = o.alignment * (Vec3(a[0], a[1], a[2]) * o.lengthScale);
        if (!p.allFinite())
            throw std::runtime_error("VTU has invalid coordinates");
        m.vertices.push_back(p);
    }
    for (vtkIdType i = 0; i < g->GetNumberOfCells(); ++i) {
        auto c = g->GetCell(i);
        if (c->GetCellType() != VTK_TETRA && c->GetCellType() != VTK_QUADRATIC_TETRA)
            throw std::runtime_error("Fusion import requires tetrahedral VTU cells (4 or 10 nodes)");
        Tetrahedron t;
        for (int j = 0; j < c->GetNumberOfPoints(); ++j)
            t.nodes.push_back(int(c->GetPointId(j)));
        m.cells.push_back(std::move(t));
    }
    return m;
}
static bool same_mesh(const VolumeMesh &a, const VolumeMesh &b) {
    if (a.vertices.size() != b.vertices.size() || a.cells.size() != b.cells.size())
        return false;
    for (size_t i = 0; i < a.vertices.size(); ++i)
        if ((a.vertices[i] - b.vertices[i]).norm() > 1e-7)
            return false;
    for (size_t i = 0; i < a.cells.size(); ++i)
        if (a.cells[i].nodes != b.cells[i].nodes)
            return false;
    return true;
}
static std::optional<double> result_time(vtkUnstructuredGrid* grid) {
    if(grid->GetInformation()->Has(vtkDataObject::DATA_TIME_STEP()))return grid->GetInformation()->Get(vtkDataObject::DATA_TIME_STEP());
    for(const char* name:{"TimeValue","Time","TIME_STEP"})if(auto array=grid->GetFieldData()->GetArray(name)) {
        if(array->GetNumberOfTuples()!=1||array->GetNumberOfComponents()!=1||!std::isfinite(array->GetComponent(0,0)))
            throw std::runtime_error("Invalid FEM result step metadata");
        return array->GetComponent(0,0);
    }return {};
}
StressField import_vtu(const QStringList &filenames, const VtuOptions &o) {
    if ((filenames.size() != 1 && filenames.size() != 6) || o.resultStep.trimmed().isEmpty() ||
        o.lengthScale <= 0 || o.stressScale <= 0 || !std::isfinite(o.lengthScale) ||
        !std::isfinite(o.stressScale))
        throw std::runtime_error("Invalid FEM import units or files");
    if (std::abs(o.alignment.linear().determinant() - 1) > 1e-6 ||
        (o.alignment.linear().transpose() * o.alignment.linear() - Mat3::Identity()).norm() > 1e-6)
        throw std::runtime_error("FEM alignment must be a rigid transform; set units separately");
    StressField out;
    out.source = filenames.join("; ");
    out.pointData = o.pointData;
    out.step = o.resultStep;
    std::vector<vtkSmartPointer<vtkUnstructuredGrid>> grids;
    std::optional<double> time;
    for (auto &file : filenames) {
        auto g = read_vtu(file);
        auto currentTime=result_time(g);
        if(grids.empty())time=currentTime;
        else if(time.has_value()!=currentTime.has_value()||(time&&std::abs(*time-*currentTime)>1e-12*(1+std::abs(*time))))
            throw std::runtime_error("FEM component files represent different result steps");
        auto mesh = vtk_mesh(g, o);
        if (grids.empty())
            out.mesh = std::move(mesh);
        else if (!same_mesh(out.mesh, mesh))
            throw std::runtime_error("FEM component files have different meshes or node ordering");
        grids.push_back(g);
    }
    if(time){bool numeric;double requested=o.resultStep.toDouble(&numeric);
        if(numeric&&std::abs(requested-*time)>1e-12*(1+std::abs(*time)))throw std::runtime_error("Selected FEM step does not match VTU metadata");
        out.step+=" @ VTU time "+QString::number(*time,'g',17);
    }
    size_t count = out.pointData ? out.mesh.vertices.size() : out.mesh.cells.size();
    out.tensors.assign(count, Mat3::Zero());
    auto data = [&](size_t index) -> vtkDataSetAttributes * {
        return out.pointData ? static_cast<vtkDataSetAttributes *>(grids[index]->GetPointData())
                             : static_cast<vtkDataSetAttributes *>(grids[index]->GetCellData());
    };
    auto read = [&](vtkDataArray *a, size_t i, int c) {
        if (!a || a->GetNumberOfTuples() != vtkIdType(count))
            throw std::runtime_error("FEM result association or coverage mismatch");
        double v = a->GetComponent(i, c) * o.stressScale;
        if (!std::isfinite(v))
            throw std::runtime_error("FEM result has non-finite stress");
        return v;
    };
    vtkDataArray *tensor = nullptr;
    if (grids.size() == 1) {
        if (!o.array.isEmpty())
            tensor = data(0)->GetArray(o.array.toUtf8());
        else
            for (int i = 0; i < data(0)->GetNumberOfArrays(); ++i) {
                auto a = data(0)->GetArray(i);
                if (a && (a->GetNumberOfComponents() == 6 || a->GetNumberOfComponents() == 9)) {
                    tensor = a;
                    break;
                }
            }
    }
    if (tensor && (tensor->GetNumberOfComponents() == 6 || tensor->GetNumberOfComponents() == 9)) {
        for (size_t i = 0; i < count; ++i) {
            Mat3 t;
            if (tensor->GetNumberOfComponents() == 9) {
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        t(r, c) = read(tensor, i, 3 * r + c);
                if ((t - t.transpose()).norm() > 1e-6 * std::max(1.0, t.norm()))
                    throw std::runtime_error("Stress tensor is not symmetric");
            } else {
                t << read(tensor, i, 0), read(tensor, i, 3), read(tensor, i, 5), read(tensor, i, 3),
                    read(tensor, i, 1), read(tensor, i, 4), read(tensor, i, 5), read(tensor, i, 4),
                    read(tensor, i, 2);
            }
            out.tensors[i] = o.alignment.linear() * t * o.alignment.linear().transpose();
        }
        out.directional = true;
    } else {
        std::array<vtkDataArray *, 6> arrays{};
        bool complete = true;
        for (int c = 0; c < 6; ++c) {
            size_t file = grids.size() == 6 ? c : 0;
            QString name = o.components[c];
            if (!name.isEmpty())
                arrays[c] = data(file)->GetArray(name.toUtf8());
            else if (grids.size() == 6 && data(file)->GetNumberOfArrays() == 1)
                arrays[c] = data(file)->GetArray(0);
            complete &= arrays[c] != nullptr && arrays[c]->GetNumberOfComponents() == 1;
        }
        if (complete) {
            for (size_t i = 0; i < count; ++i) {
                Mat3 t;
                t << read(arrays[0], i, 0), read(arrays[3], i, 0), read(arrays[5], i, 0),
                    read(arrays[3], i, 0), read(arrays[1], i, 0), read(arrays[4], i, 0),
                    read(arrays[5], i, 0), read(arrays[4], i, 0), read(arrays[2], i, 0);
                out.tensors[i] = o.alignment.linear() * t * o.alignment.linear().transpose();
            }
            out.directional = true;
        } else if (grids.size() == 1) {
            vtkDataArray *a = o.array.isEmpty() ? data(0)->GetArray(0) : data(0)->GetArray(o.array.toUtf8());
            if (!a || a->GetNumberOfComponents() != 1)
                throw std::runtime_error("Select a stress tensor array or provide all six stress components");
            for (size_t i = 0; i < count; ++i)
                out.scalar.push_back(read(a, i, 0));
            out.tensors.clear();
        } else
            throw std::runtime_error(
                "Provide exactly six matching scalar stress components in XX YY ZZ XY YZ ZX order");
    }
    return out;
}
double StressField::scalar_at(const Vec3 &p, bool *covered) const {
    if (directional) {
        auto t = sample(p, covered);
        auto deviator = t - Mat3::Identity() * t.trace() / 3;
        return std::sqrt(1.5 * deviator.squaredNorm());
    }
    if (covered)
        *covered = false;
    for (size_t i = 0; i < mesh.cells.size(); ++i) {
        auto &cell = mesh.cells[i];
        Eigen::Vector4d w;
        if (!barycentric(mesh, cell, p, w))
            continue;
        if (covered)
            *covered = true;
        if (!pointData)
            return scalar.at(i);
        double v = 0;
        if (cell.nodes.size() == 10) {
            const int edges[][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
            for (int j = 0; j < 4; ++j)
                v += w[j] * (2 * w[j] - 1) * scalar.at(cell.nodes[j]);
            for (int j = 0; j < 6; ++j)
                v += 4 * w[edges[j][0]] * w[edges[j][1]] * scalar.at(cell.nodes[j + 4]);
        } else
            for (int j = 0; j < 4; ++j)
                v += w[j] * scalar.at(cell.nodes[j]);
        return v;
    }
    return 0;
}
Mat3 StressField::sample(const Vec3 &p, bool *covered) const {
    if (covered)
        *covered = false;
    if (!directional)
        return Mat3::Zero();
    for (size_t i = 0; i < mesh.cells.size(); ++i) {
        const auto &cell = mesh.cells[i];
        Eigen::AlignedBox3d b;
        for (int n : cell.nodes)
            b.extend(mesh.vertices[n]);
        b.min().array() -= 1e-7;
        b.max().array() += 1e-7;
        if (!b.contains(p))
            continue;
        Eigen::Vector4d w;
        if (!barycentric(mesh, cell, p, w))
            continue;
        if (covered)
            *covered = true;
        if (!pointData)
            return tensors.at(i);
        Mat3 t = Mat3::Zero();
        if (cell.nodes.size() == 10) {
            const int e[][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
            for (int j = 0; j < 4; ++j)
                t += w[j] * (2 * w[j] - 1) * tensors.at(cell.nodes[j]);
            for (int j = 0; j < 6; ++j)
                t += 4 * w[e[j][0]] * w[e[j][1]] * tensors.at(cell.nodes[4 + j]);
        } else
            for (int j = 0; j < 4; ++j)
                t += w[j] * tensors.at(cell.nodes[j]);
        return t;
    }
    return Mat3::Zero();
}
Vec3 principal_tensile_direction(const Mat3 &tensor, double *confidence) {
    Eigen::SelfAdjointEigenSolver<Mat3> solver(tensor);
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("Invalid stress eigensystem");
    auto values = solver.eigenvalues();
    double scale = std::max(values.cwiseAbs().maxCoeff(), 1e-9);
    if (confidence)
        *confidence = std::max(0.0, values[2] - values[1]) / scale * std::max(0.0, values[2]) / scale;
    return solver.eigenvectors().col(2);
}
static Eigen::Matrix<double, 6, 6> orthotropic_stiffness(const Material &material, const Mat3 &frame) {
    double E = material.young, T = material.transverseYoung, nu = material.poisson;
    if (T <= 0)
        throw std::runtime_error("Transverse modulus must be positive");
    Eigen::Matrix<double, 6, 6> S = Eigen::Matrix<double, 6, 6>::Zero();
    S(0, 0) = 1 / E;
    S(1, 1) = S(2, 2) = 1 / T;
    S(0, 1) = S(1, 0) = S(0, 2) = S(2, 0) = -nu / E;
    S(1, 2) = S(2, 1) = -nu / T;
    S(3, 3) = S(5, 5) = 2 * (1 + nu) / std::sqrt(E * T);
    S(4, 4) = 2 * (1 + nu) / T;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(S);
    if (eigen.eigenvalues().minCoeff() <= 0)
        throw std::runtime_error("Material constants do not define a positive elastic compliance");
    auto local = S.inverse().eval();
    Eigen::Matrix<double, 6, 6> global;
    for (int column = 0; column < 6; ++column) {
        Mat3 strain = Mat3::Zero();
        if (column < 3)
            strain(column, column) = 1;
        else {
            const int pair[][2] = {{0, 1}, {1, 2}, {2, 0}};
            int a = pair[column - 3][0], b = pair[column - 3][1];
            strain(a, b) = strain(b, a) = .5;
        }
        Mat3 e = frame.transpose() * strain * frame;
        Eigen::Matrix<double, 6, 1> ev;
        ev << e(0, 0), e(1, 1), e(2, 2), 2 * e(0, 1), 2 * e(1, 2), 2 * e(2, 0);
        Eigen::Matrix<double, 6, 1> sv = local * ev;
        Mat3 stress;
        stress << sv[0], sv[3], sv[5], sv[3], sv[1], sv[4], sv[5], sv[4], sv[2];
        stress = frame * stress * frame.transpose();
        global.col(column) << stress(0, 0), stress(1, 1), stress(2, 2), stress(0, 1), stress(1, 2),
            stress(2, 0);
    }
    return (global + global.transpose()) / 2;
}
FEAResult solve_fea(const VolumeMesh &mesh, const LoadCase &loads, const std::vector<Mat3> &materialFrames) {
    if (mesh.cells.empty() || loads.fixedNodes.empty() || loads.nodalForces.empty())
        throw std::runtime_error("FEA requires a volume mesh, fixed nodes, and forces");
    double E = loads.material.young, nu = loads.material.poisson;
    if (E <= 0 || nu <= -1 || nu >= .499)
        throw std::runtime_error("Invalid elastic material");
    int n = int(mesh.vertices.size()), dofs = 3 * n;
    std::set<int> fixed;
    for (int i : loads.fixedNodes) {
        if (i < 0 || i >= n)
            throw std::runtime_error("Fixture node out of range");
        for (int j = 0; j < 3; ++j)
            fixed.insert(3 * i + j);
    }
    Eigen::VectorXd forces = Eigen::VectorXd::Zero(dofs);
    for (auto &[i, f] : loads.nodalForces) {
        if (i < 0 || i >= n || !f.allFinite())
            throw std::runtime_error("Invalid load");
        for (int j = 0; j < 3; ++j)
            if (!fixed.count(3 * i + j))
                forces[3 * i + j] += f[j];
    }
    double lambda = E * nu / ((1 + nu) * (1 - 2 * nu)), mu = E / (2 * (1 + nu));
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    D.topLeftCorner<3, 3>().setConstant(lambda);
    D.diagonal().head<3>().array() += 2 * mu;
    D.diagonal().tail<3>().setConstant(mu);
    if (!materialFrames.empty() && materialFrames.size() != mesh.cells.size())
        throw std::runtime_error("Material orientation field does not cover the volume");
    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<Eigen::Matrix<double, 6, 12>> Bs;
    std::vector<Eigen::Matrix<double, 6, 6>> Ds;
    for (auto &t : mesh.cells) {
        if (t.nodes.size() != 4)
            throw std::runtime_error(
                "Built-in FEA requires linear tetrahedra; imported quadratic results remain supported");
        Eigen::Matrix4d M;
        for (int j = 0; j < 4; ++j) {
            auto v = mesh.vertices[t.nodes[j]];
            M.row(j) << 1, v.transpose();
        }
        double volume = std::abs(M.determinant()) / 6;
        if (volume < 1e-12)
            throw std::runtime_error("Degenerate FEA tetrahedron");
        auto inv = M.inverse().eval();
        Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
        for (int j = 0; j < 4; ++j) {
            Vec3 g = inv.col(j).tail<3>();
            B(0, 3 * j) = g.x();
            B(1, 3 * j + 1) = g.y();
            B(2, 3 * j + 2) = g.z();
            B(3, 3 * j) = g.y();
            B(3, 3 * j + 1) = g.x();
            B(4, 3 * j + 1) = g.z();
            B(4, 3 * j + 2) = g.y();
            B(5, 3 * j) = g.z();
            B(5, 3 * j + 2) = g.x();
        }
        auto cellD =
            materialFrames.empty() ? D : orthotropic_stiffness(loads.material, materialFrames[Bs.size()]);
        Ds.push_back(cellD);
        Bs.push_back(B);
        Eigen::Matrix<double, 12, 12> K = B.transpose() * cellD * B * volume;
        for (int a = 0; a < 12; ++a)
            for (int b = 0; b < 12; ++b) {
                int row = 3 * t.nodes[a / 3] + a % 3, col = 3 * t.nodes[b / 3] + b % 3;
                if (!fixed.count(row) && !fixed.count(col))
                    triplets.emplace_back(row, col, K(a, b));
            }
    }
    for (int i : fixed)
        triplets.emplace_back(i, i, 1);
    Eigen::SparseMatrix<double> K(dofs, dofs);
    K.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);
    if (solver.info() != Eigen::Success || solver.vectorD().minCoeff() <= 0)
        throw std::runtime_error("FEA is underconstrained or singular");
    Eigen::VectorXd u = solver.solve(forces);
    if (solver.info() != Eigen::Success || !u.allFinite() ||
        (K * u - forces).norm() > 1e-6 * std::max(1.0, forces.norm()))
        throw std::runtime_error("FEA failed residual validation");
    FEAResult out;
    out.compliance = forces.dot(u);
    out.stress.mesh = mesh;
    out.stress.directional = true;
    out.stress.pointData = false;
    out.stress.source = "Built-in linear elastic FEA";
    out.stress.step = "static";
    for (int i = 0; i < n; ++i)
        out.displacement.push_back(u.segment<3>(3 * i));
    for (size_t i = 0; i < mesh.cells.size(); ++i) {
        Eigen::Matrix<double, 12, 1> ue;
        for (int j = 0; j < 4; ++j)
            ue.segment<3>(3 * j) = out.displacement[mesh.cells[i].nodes[j]];
        Eigen::Matrix<double, 6, 1> s = Ds[i] * Bs[i] * ue;
        Mat3 t;
        t << s[0], s[3], s[5], s[3], s[1], s[4], s[5], s[4], s[2];
        out.stress.tensors.push_back(t);
    }
    return out;
}
StructuralComparison compare_strength(const Project &p) {
    if (p.path.empty() || p.volume.cells.empty())
        throw std::runtime_error("Slice a volume before comparing structural predictions");
    Project baseline = p;
    baseline.process.curved = false;
    slice_project(baseline);
    auto frames = [&](const Project &job) {
        std::vector<Mat3> result;
        for (auto &cell : p.volume.cells) {
            Vec3 center = Vec3::Zero();
            for (int j = 0; j < 4; ++j)
                center += p.volume.vertices[cell.nodes[j]] / 4;
            double nearest = 1e100;
            Vec3 tangent = Vec3::UnitX(), normal = Vec3::UnitZ();
            for (size_t i = 1; i < job.path.size(); ++i) {
                auto &b = job.path[i];
                if (b.deltaE <= 0 || b.role == Role::Support)
                    continue;
                Vec3 delta = b.position - job.path[i - 1].position;
                if (delta.norm() < 1e-8)
                    continue;
                double d = ((b.position + job.path[i - 1].position) / 2 - center).squaredNorm();
                if (d < nearest) {
                    nearest = d;
                    tangent = delta.normalized();
                    normal = b.direction;
                }
            }
            Vec3 side = normal.cross(tangent);
            if (side.norm() < 1e-7)
                side = tangent.unitOrthogonal();
            side.normalize();
            Mat3 R;
            R.col(0) = tangent;
            R.col(1) = side;
            R.col(2) = tangent.cross(side).normalized();
            result.push_back(R);
        }
        return result;
    };
    StructuralComparison result;
    result.planarCompliance = solve_fea(p.volume, p.loads, frames(baseline)).compliance;
    result.curvedCompliance = solve_fea(p.volume, p.loads, frames(p)).compliance;
    result.alignment = stress_alignment(p);
    result.available = true;
    return result;
}
} // namespace fiveaxis
