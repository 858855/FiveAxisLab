#pragma once
#include "core/types.h"
#include <QWidget>
bool edit_machine(fiveaxis::Project &project, QWidget *parent);
bool edit_toolhead(fiveaxis::Project &project, QWidget *parent, const QString &toolId = {});
