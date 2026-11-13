#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export ENABLE_CAMERA_PITCH_SERVO=false
export SERVO_HARDWARE_AUTO_ARM=0
exec "$SCRIPT_DIR/start_visual_intercept.sh" horizontal_only
