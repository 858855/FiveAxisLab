#pragma once
#include "types.h"
#include <QStringList>
namespace fiveaxis {
struct VtuInfo {
    QStringList pointArrays, cellArrays;
    size_t points = 0, cells = 0;
};
struct VtuOptions {
    double lengthScale = 1, stressScale = 1;
    Transform alignment = Transform::Identity();
    QString array;
    bool pointData = true;
    std::array<QString, 6> components;
    QString resultStep = "static";
};
VtuInfo inspect_vtu(const QString &filename);
StressField import_vtu(const QStringList &filenames, const VtuOptions &options);
FEAResult solve_fea(const VolumeMesh &mesh, const LoadCase &loads,
                    const std::vector<Mat3> &materialFrames = {});
StructuralComparison compare_strength(const Project &p);
Vec3 principal_tensile_direction(const Mat3 &stress, double *confidence = nullptr);
} // namespace fiveaxis
