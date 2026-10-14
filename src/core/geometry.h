#pragma once
#include "types.h"
namespace fiveaxis {
Mesh box_mesh(const Vec3 &center, const Vec3 &size, const QString &name = "Box");
Mesh cylinder_mesh(const Vec3 &center, double radius, double height, const QString &name = "Cylinder",
                   int segments = 32);
Mesh load_mesh(const QString &filename);
// Join coincident vertices within the tolerance (1e-7 mm for CAD face seams), without filling boundary holes.
void weld_mesh_vertices(Mesh &mesh, double tolerance = 1e-7);
VolumeMesh tetrahedralize(const Mesh &mesh, double maxVolume = 5);
Mesh volume_surface(const VolumeMesh &mesh);
bool contains(const Mesh &mesh, const Vec3 &point);
// Point-in-solid queries against one closed mesh, accelerated by an XY triangle grid.
// The mesh must outlive the tester.
class InsideTester {
  public:
    explicit InsideTester(const Mesh &mesh);
    bool contains(const Vec3 &point) const;

  private:
    const Mesh *mesh;
    Eigen::AlignedBox3d box;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    double cell = 1;
    int nx = 1, ny = 1;
    std::vector<std::vector<int>> cells;
};
bool barycentric(const VolumeMesh &mesh, const Tetrahedron &cell, const Vec3 &p, Eigen::Vector4d &weights);
double bead_area(double width, double height);
double bead_spacing(double width, double height);
double default_line_width(double nozzle, bool topOrSupport = false);
QString orca_revision();
} // namespace fiveaxis
