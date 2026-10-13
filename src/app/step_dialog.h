#pragma once
#include "core/step_import.h"
#include <QWidget>
bool import_step_dialog(QWidget *parent,const QString &file,fiveaxis::ToolheadAssembly &assembly,bool toolhead);
