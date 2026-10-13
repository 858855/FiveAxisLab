#pragma once
#include "types.h"
namespace fiveaxis {
void slice_project(Project &p, std::atomic_bool *cancel = nullptr, std::function<void(double)> progress = {});
void regenerate_joints(Project &p);
void refine_joint_motion(Project &p);
void check_deposition(Project &p);
double stress_alignment(const Project &p);
} // namespace fiveaxis
