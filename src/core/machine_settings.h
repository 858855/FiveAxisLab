#pragma once
#include "types.h"
namespace fiveaxis {
struct PrinterOption {
    const char *key, *label, *section, *unit;
    double initial, minimum, maximum;
    bool advanced = true;
};
const std::vector<PrinterOption> &motion_options();
const std::vector<PrinterOption> &extruder_options();
double printer_number(const QJsonObject &settings, const char *key, double fallback);
void validate_printer_settings(const MachineProfile &machine);
void validate_extruder_settings(const QJsonObject &settings);
double printer_move_seconds(const MachineProfile &machine, const Vec3 &delta, double extrusion,
                            const QJsonObject &extruder);
std::vector<Vec3> bed_outline(const QJsonObject &settings);
Mesh printer_bed_mesh(const QJsonObject &settings);
bool printable_point(const MachineProfile &machine, const Vec3 &point, double margin = 0);
void apply_printer_path(Project &project);
void check_printer_envelope(const Project &project);
QString printer_controller_template(const MachineProfile &machine);
bool relative_extrusion(const MachineProfile &machine);
QString printer_path_signature(const Project &project);
Vec3 preferred_plate_position(const MachineProfile &machine);
} // namespace fiveaxis
