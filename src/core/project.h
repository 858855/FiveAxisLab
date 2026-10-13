#pragma once
#include "types.h"
namespace fiveaxis {
void save_project(const Project &p, const QString &filename);
Project load_project(const QString &filename);
} // namespace fiveaxis
