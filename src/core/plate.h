#pragma once
#include "types.h"
namespace fiveaxis {
void enable_plate(Project &p);
void synchronize_plate_preview(Project &p);
void validate_plate_setup(const Project &p);
void slice_plate(Project &p, std::atomic_bool *cancel = nullptr, std::function<void(double)> progress = {});
CollisionReport validate_plate_motion(const Project &p, std::atomic_bool *cancel = nullptr,
                                      std::function<void(double)> progress = {});
bool repair_plate_motion(Project &p, std::atomic_bool *cancel = nullptr);
QString export_plate_gcode(const Project &p);
Project replay_plate_gcode(const Project &p, const QString &code);
const ToolDefinition *find_tool(const Project &p, const QString &id);
const MaterialProfile *find_material(const Project &p, const QString &id);
ToolheadAssembly installed_assembly(const ToolDefinition &tool);
ToolheadAssembly assembly_at(const Project &p, const QString &toolId);
JointState sample_axes(const Project &p, const Sample &s);
JointState sample_motion(const Project &p, const Sample &a, const Sample &b, double fraction);
void time_plate(Project &p);
void rebuild_job_events(Project &p);
SimulationState simulate_plate(const Project &p, double time, bool rebuild = true);
QJsonObject plate_json(const Project &p);
void read_plate(Project &p, const QJsonObject &json);
void migrate_single_project(Project &p);
FEAResult solve_plate_fea(Project &p, size_t object, std::atomic_bool *cancel = nullptr);
QString structural_signature(const Project &p, size_t object);
QString plate_generation_signature(const Project &p);
Project plate_demo(PrintSequence sequence = PrintSequence::Objects);
} // namespace fiveaxis
