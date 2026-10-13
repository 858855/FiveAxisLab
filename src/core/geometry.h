#pragma once
#include "types.h"
namespace fiveaxis {
Mesh box_mesh(const Vec3 &center, const Vec3 &size, const QString &name = "Box");
Mesh cylinder_mesh(const Vec3 &center, double radius, double height, const QString &name = "Cylinder",
                   int segments = 32);
Mesh load_mesh(const QString &filename);
VolumeMesh tetrahedralize(const Mesh &mesh, double maxVolume = 5);
Mesh volume_surface(const VolumeMesh &mesh);
bool contains(const Mesh &mesh, const Vec3 &point);
bool barycentric(const VolumeMesh &mesh, const Tetrahedron &cell, const Vec3 &p, Eigen::Vector4d &weights);
double bead_area(double width, double height);
double bead_spacing(double width, double height);
double default_line_width(double nozzle, bool topOrSupport = false);
QString orca_revision();
} // namespace fiveaxis
