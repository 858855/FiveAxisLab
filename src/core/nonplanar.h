#pragma once
#include "types.h"
namespace fiveaxis {
std::vector<double> conformal_field(const Project &project);
int cartesian_motion_violation(const Project &project);
Project nonplanar_cartesian_project();
}
