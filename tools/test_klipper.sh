#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$TASK_ROOT/build/klipper-python"
cp "$TASK_ROOT/controllers/klipper/fiveaxis_flow.py" "$TASK_ROOT/vendor/klipper/klippy/extras/fiveaxis_flow.py"
: > "$TASK_ROOT/build/klipper-batch.log"
python3 "$TASK_ROOT/vendor/klipper/klippy/klippy.py" "$TASK_ROOT/tests/klipper-batch.cfg" -i "$TASK_ROOT/tests/klipper-batch.gcode" -o "$TASK_ROOT/build/klipper-batch.serial" -d "$TASK_ROOT/build/klipper-mcu/klipper.dict" -l "$TASK_ROOT/build/klipper-batch.log"
python3 "$TASK_ROOT/vendor/klipper/klippy/parsedump.py" "$TASK_ROOT/build/klipper-mcu/klipper.dict" "$TASK_ROOT/build/klipper-batch.serial" > "$TASK_ROOT/build/klipper-batch.txt"
if grep -E 'Internal error|Error loading|Traceback|Unknown command|Move exceeds' "$TASK_ROOT/build/klipper-batch.log"; then
    exit 1
fi
grep -q 'queue_step' "$TASK_ROOT/build/klipper-batch.txt"
echo 'Native Klipper batch generated synchronized step commands.'
