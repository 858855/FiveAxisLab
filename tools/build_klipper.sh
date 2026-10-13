#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$TASK_ROOT/build/klipper-python"
mkdir -p "$TASK_ROOT/build/klipper-mcu"
cp "$TASK_ROOT/vendor/klipper/test/configs/linuxprocess.config" "$TASK_ROOT/build/klipper-linux.config"
cd "$TASK_ROOT/vendor/klipper"
make KCONFIG_CONFIG="$TASK_ROOT/build/klipper-linux.config" OUT="$TASK_ROOT/build/klipper-mcu/" olddefconfig
make KCONFIG_CONFIG="$TASK_ROOT/build/klipper-linux.config" OUT="$TASK_ROOT/build/klipper-mcu/" -j4
