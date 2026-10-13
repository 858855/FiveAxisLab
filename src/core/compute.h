#pragma once
#include "types.h"
namespace fiveaxis {
int compute_threads(const ComputeSettings &settings);
void parallel_for(size_t count, const ComputeSettings &settings, const std::function<void(size_t)> &task,
                  std::atomic_bool *cancel = nullptr);
QStringList compute_devices();
std::vector<Vec3> compute_workload(const std::vector<double> &input, uint32_t workload,
                                   const Transform &transform, const ComputeSettings &settings,
                                   ComputeReport &report, std::atomic_bool *cancel = nullptr);
std::vector<Vec3> compute_transform(const std::vector<Vec3> &points, const Transform &transform,
                                    const ComputeSettings &settings, ComputeReport &report,
                                    std::atomic_bool *cancel = nullptr);
} // namespace fiveaxis
