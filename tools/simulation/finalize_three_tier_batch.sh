#!/usr/bin/env bash
# Rebuild reporting only, then record the median-like field case separately.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BATCH_DIR=${1:?usage: finalize_three_tier_batch.sh BATCH_DIR [BATCH_PID]}
BATCH_PID=${2:-}
if [[ -n "$BATCH_PID" ]]; then
  while kill -0 "$BATCH_PID" 2>/dev/null; do
    sleep 10
  done
fi

python3 - "$BATCH_DIR" <<'PY'
import csv
import sys
from pathlib import Path
root = Path(sys.argv[1])
with (root / "cases.csv").open(newline="", encoding="utf-8-sig") as stream:
    cases = list(csv.DictReader(stream))
missing = [row["CASE_NAME"] for row in cases
           if not (root / row["CASE_NAME"] / "summary.json").is_file()]
if missing:
    raise SystemExit("Cannot finalize: missing summaries " + ", ".join(missing))
print("All %d formal summaries present" % len(cases), flush=True)
PY

for run_dir in "$BATCH_DIR"/*/; do
  [[ -s "$run_dir/experiment.csv" ]] || continue
  python3 "$SCRIPT_DIR/analyze_radar_multisim_result.py" "$run_dir" \
    > "$run_dir/analyzer.log" 2>&1
done
python3 "$SCRIPT_DIR/aggregate_radar_multisim.py" "$BATCH_DIR"
python3 "$SCRIPT_DIR/summarize_full_chain_demo.py" "$BATCH_DIR"
python3 "$SCRIPT_DIR/audit_three_tier_batch.py" "$BATCH_DIR" \
  --output "$BATCH_DIR/run_audit.json"
if [[ "${REPORT_ONLY:-0}" == "1" ]]; then
  exit 0
fi
python3 "$SCRIPT_DIR/select_representative_video_case.py" "$BATCH_DIR" \
  --profile field --json > "$BATCH_DIR/video_selection.json"
SELECTED_CASE=$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["ranking"][0]["case_name"])' \
  "$BATCH_DIR/video_selection.json")
VIDEO_BATCH_ID="$(basename "$BATCH_DIR")_video"
VIDEO_RUN_DIR="$(dirname "$BATCH_DIR")/$VIDEO_BATCH_ID/$SELECTED_CASE"
if [[ -e "$VIDEO_RUN_DIR" ]]; then
  echo "Video run already exists: $VIDEO_RUN_DIR; preserving it." >&2
  exit 2
fi
if [[ -f "$SCRIPT_DIR/start_radar_approach_multisim.video_staged.sh" ]]; then
  cp "$SCRIPT_DIR/start_radar_approach_multisim.video_staged.sh" \
    "$SCRIPT_DIR/start_radar_approach_multisim.sh"
fi
echo "Recording separate run: $SELECTED_CASE"
env BATCH_ID="$VIDEO_BATCH_ID" CASE_FILTER="$SELECTED_CASE" \
  SHOW_GUI=1 SHOW_RQT=0 DEMO_GUI=1 DEMO_VISUAL_MARKERS=1 RECORD_VIDEO=1 \
  RUN_WALL_TIMEOUT_S=200 python3 "$SCRIPT_DIR/batch_full_chain_demo.py" \
  "$BATCH_DIR/cases.csv"
python3 "$SCRIPT_DIR/make_client_demo_video.py" "$VIDEO_RUN_DIR" \
  "$VIDEO_RUN_DIR/client_demo_mpeg4.mp4"
gst-launch-1.0 -e filesrc location="$VIDEO_RUN_DIR/client_demo_mpeg4.mp4" \
  ! qtdemux ! decodebin ! videoconvert \
  ! x264enc speed-preset=fast bitrate=5000 key-int-max=48 \
  ! video/x-h264,profile=high ! h264parse ! mp4mux \
  ! filesink location="$VIDEO_RUN_DIR/client_demo_h264.mp4"
python3 "$SCRIPT_DIR/audit_three_tier_batch.py" \
  "$(dirname "$VIDEO_RUN_DIR")" \
  --output "$(dirname "$VIDEO_RUN_DIR")/run_audit.json"
echo "VIDEO_COMPLETE=$VIDEO_RUN_DIR/client_demo_h264.mp4"
