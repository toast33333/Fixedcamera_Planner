#!/usr/bin/env python3
"""Read per-run telemetry and logs for a frozen three-tier batch."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import summarize_full_chain_demo as client


def floats(rows, field):
    return client.finite(row.get(field) for row in rows)


def audit(root):
    output = []
    for record in client.load_records(str(root)):
        directory = root / record["case_dir"]
        csv_path = directory / "experiment.csv"
        rows = []
        if csv_path.is_file():
            with csv_path.open(newline="", encoding="utf-8-sig") as stream:
                rows = list(csv.DictReader(stream))
        chase = [row for row in rows if row.get("phase") == "CHASE"]
        all_times = floats(rows, "ros_time_s")
        exit_path = directory / "roslaunch_exit_code.txt"
        last_window = []
        if chase:
            end_time = float(chase[-1]["ros_time_s"])
            last_window = [row for row in chase
                           if float(row["ros_time_s"]) >= end_time - 10.0]
        closing_slope = None
        if len(last_window) >= 2:
            times = [float(row["ros_time_s"]) for row in last_window]
            distances = [float(row["distance_m"]) for row in last_window]
            t_mean = sum(times) / len(times)
            d_mean = sum(distances) / len(distances)
            variance = sum((t - t_mean) ** 2 for t in times)
            if variance > 0:
                closing_slope = sum((t - t_mean) * (d - d_mean)
                                    for t, d in zip(times, distances)) / variance
        vz = floats(chase, "target_vz")
        z = floats(chase, "target_z")
        handover = next((row for row in chase
                         if row.get("handover_complete") == "1"), None)
        state_samples = {}
        for row in chase:
            state = row.get("radar_state", "unknown")
            state_samples[state] = state_samples.get(state, 0) + 1
        text = ""
        log_path = directory / "roslaunch.log"
        if log_path.is_file():
            text = log_path.read_text(encoding="utf-8", errors="replace")
        source_path = directory / "controller_source_sha256.txt"
        hashes = (source_path.read_text(encoding="utf-8").splitlines()
                  if source_path.is_file() else [])
        config_path = directory / "radar_approach_handover.yaml"
        item = {
            "case_name": record["case_name"],
            "profile": record["error_profile"],
            "group": record["test_group"],
            "primary_pass": record["primary_validation_pass"],
            "failure_reason": record["primary_validation_reason"],
            "analyzer_reason": record.get("reason"),
            "success": record.get("success"),
            "handover": record.get("handover_occurred"),
            "capture_time_s": record.get("capture_time_from_chase_s"),
            "capture_distance_m": record.get("capture_distance_at_success_m"),
            "minimum_chase_distance_m": record.get("minimum_distance_m"),
            "final_10s_distance_slope_mps": closing_slope,
            "minimum_altitude_m": record["minimum_interceptor_altitude_m"],
            "fov_max": record.get("post_handover_max_fov_ratio"),
            "tracking_percent": record.get(
                "post_handover_tracking_availability_percent"),
            "radar_rmse_m": record.get("radar_prediction_rmse_m"),
            "handover_true_distance_m": (
                float(handover["distance_m"]) if handover else None),
            "chase_target_vz_rms_mps": (
                math.sqrt(sum(v * v for v in vz) / len(vz)) if vz else None),
            "chase_target_vz_peak_mps": max(map(abs, vz)) if vz else None,
            "chase_target_z_span_m": max(z) - min(z) if z else None,
            "chase_state_samples": state_samples,
            "radar_drops": client.maximum(floats(rows, "radar_dropped_count")),
            "visual_drops": client.maximum(floats(rows, "visual_dropped_count")),
            "time_jump_warnings": text.count("Time jump detected"),
            "csv_time_reversals": sum(after < before for before, after in
                                       zip(all_times, all_times[1:])),
            "roslaunch_exit_code": (exit_path.read_text().strip()
                                    if exit_path.is_file() else None),
            "radar_rejection_log_entries": text.count("Radar sample rejected"),
            "acquire_timeout_log_entries": text.count("Radar acquisition timed out"),
            "source_hashes": hashes,
            "radar_config_sha256": (hashlib.sha256(config_path.read_bytes()).hexdigest()
                                     if config_path.is_file() else None),
        }
        output.append(item)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rendered = json.dumps(audit(args.batch_dir), ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
        print("Wrote " + str(args.output))
    else:
        print(rendered)


if __name__ == "__main__":
    main()
