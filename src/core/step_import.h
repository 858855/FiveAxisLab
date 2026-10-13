#pragma once
#include "types.h"
namespace fiveaxis {
struct StepImportOptions {
    double linearDeflection = .02;
    double angularDeflection = .0872664626; // radians
    double renderDeflection = .1;
    bool splitCompounds = false;
};
ToolheadAssembly load_step_assembly(const QString &filename, const StepImportOptions &options,
                                    std::atomic_bool *cancel = nullptr);
ToolheadAssembly load_step_assembly(const QString &filename, double deflection = 0.02,
                                    std::atomic_bool *cancel = nullptr);
void write_demo_step(const QString &filename, bool nested = false, bool inches = false, bool compound = false);
} // namespace fiveaxis
