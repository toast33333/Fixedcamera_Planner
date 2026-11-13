#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export ENABLE_CAMERA_PITCH_SERVO=${ENABLE_CAMERA_PITCH_SERVO:-true}
export SERVO_HARDWARE_AUTO_ARM=${SERVO_HARDWARE_AUTO_ARM:-1}
exec "$SCRIPT_DIR/start_visual_intercept.sh" full
