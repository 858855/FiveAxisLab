// Adapted from OrcaSlicer libslic3r Flow.hpp / Flow.cpp (AGPL-3.0).
// Exact upstream revision and original sources are in vendor/orca-reference.
#include "geometry.h"
#include <stdexcept>
namespace fiveaxis {
double bead_area(double width, double height) {
    if (height <= 0 || width < height)
        throw std::runtime_error("Extrusion width must be at least the layer height");
    return height * (width - height * (1.0 - pi / 4.0));
}
double bead_spacing(double width, double height) {
    return width - height * (1.0 - pi / 4.0);
}
double default_line_width(double nozzle, bool topOrSupport) {
    return topOrSupport ? nozzle : 1.125 * nozzle;
}
QString orca_revision() {
    return "f0e9cdc470f3eba904a9a6cc2109e3793522b103";
}
} // namespace fiveaxis
