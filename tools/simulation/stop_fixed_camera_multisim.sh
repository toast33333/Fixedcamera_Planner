#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT=/home/toast/fixedcamera_multisim_results
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
