#pragma once
#include "types.h"
namespace fiveaxis {
ToolheadAssembly load_step_assembly(const QString &filename, double deflection = 0.02,
                                    std::atomic_bool *cancel = nullptr);
void write_demo_step(const QString &filename, bool nested = false, bool inches = false);
} // namespace fiveaxis
