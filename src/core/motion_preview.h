#pragma once
#include "types.h"
#include <array>
namespace fiveaxis {
// Hold the same point in workpiece coordinates while demonstrating the rotary joints.
JointState rotary_preview_pose(const MachineProfile &machine,const Vec3 &partAnchor,double a,double b);
// Right-handed frames: origin is the calibrated joint pivot, local Z is its world rotation axis.
std::array<Transform,2> rotary_axis_frames(const MachineProfile &machine,const ToolheadAssembly &toolhead,
                                          const JointState &joint);
Project axis_motion_demo(Layout layout);
} // namespace fiveaxis
