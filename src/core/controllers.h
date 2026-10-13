#pragma once
#include "types.h"
namespace fiveaxis {
QString export_gcode(const Project &p);
Project replay_gcode(const Project &context, const QString &gcode);
QString controller_template(const MachineProfile &machine);
double feed_metric(const MachineProfile &m, const JointState &a, const JointState &b, double deltaE);
} // namespace fiveaxis
