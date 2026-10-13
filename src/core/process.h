#pragma once
#include "types.h"
namespace fiveaxis {
QJsonObject process_json(const ProcessSettings &s);
QString process_fingerprint(const ProcessSettings &s);
ProcessSettings read_process(const QJsonObject &json);
ProcessSettings import_orca_process(const QJsonObject &json, const ProcessSettings &base, QStringList &notes);
void validate_process(const ProcessSettings &s);
double extrusion_width(const ProcessSettings &s, Role role, int layer);
double extrusion_height(const ProcessSettings &s, int layer);
double process_speed(const ProcessSettings &s, Role role, int layer);
int effective_walls(const ProcessSettings &s, int layer);
} // namespace fiveaxis
