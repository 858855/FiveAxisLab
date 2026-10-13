#pragma once
#include "types.h"
namespace fiveaxis {
void validate_profile_guide(const std::vector<ProfileStation> &guide);
double profile_length(const std::vector<ProfileStation> &guide);
ProfileStation profile_section(const std::vector<ProfileStation> &guide, double distance);
ProfileStation profile_layer(const Project &p, int layer);
std::vector<ProfileStation> import_profile_guide(const QString &file, const Transform &placement);
int profile_motion_violation(const Project &p);
Project spiral_profile_project(Layout layout);
}
