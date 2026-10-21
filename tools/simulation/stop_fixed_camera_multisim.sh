#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
if [[ -f "$REPO_DIR/devel/setup.bash" ]]; then
  DEFAULT_WS_DIR=$REPO_DIR
else
  DEFAULT_WS_DIR=$(cd "$REPO_DIR/../.." && pwd)
fi
WS_DIR=${WS_DIR:-$DEFAULT_WS_DIR}
RUN_ROOT=${RUN_ROOT:-$WS_DIR/../fixedcamera_multisim_results}
pid_file="$RUN_ROOT/active.pid"

if [[ ! -f "$pid_file" ]]; then
  echo "No managed fixed-camera multisim run is active."
  exit 0
fi

run_pid=$(tr -cd '0-9' < "$pid_file")
if [[ -z "$run_pid" ]]; then
  echo "The active PID file is invalid: $pid_file"
  exit 1
fi

if kill -0 "$run_pid" 2>/dev/null; then
  kill -TERM "$run_pid"
  echo "Stop requested for fixed-camera multisim PID $run_pid."
else
  echo "The recorded run is no longer active."
  rm -f "$pid_file"
fi
