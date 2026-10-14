#!/usr/bin/env python3
"""Aggregate and rank one fixed-camera FOV tuning batch."""

import csv
import json
import math
import os
import sys


FIELDS = [
    "case_name", "success", "minimum_distance_m",
    "post_handover_tracking_availability_percent",
    "post_handover_max_fov_ratio", "post_handover_p95_fov_ratio",
    "post_handover_fov_exceedance_percent",
    "settled_post_handover_max_fov_ratio",
    "settled_post_handover_fov_exceedance_percent",
    "post_handover_fov_guard_active_percent",
    "fov_correction_linear_rms_mps",
    "fov_correction_linear_ratio_percent",
    "fov_correction_vertical_rms_mps",
    "fov_correction_yaw_rms_rps",
    "fov_correction_yaw_ratio_percent",
    "handover_time_from_chase_s", "chase_duration_s",
    "fov_guard_center_area_ratio", "fov_guard_yaw_gain_radps",
    "fov_guard_vertical_gain_mps", "fov_guard_shape_max",
    "fov_barrier_gain", "fov_barrier_max", "fov_slowdown_start_ratio",
    "fov_min_forward_speed_ratio", "fov_emergency_yaw_ratio",
    "k_z_ff", "n_nav_z", "max_vz", "k_yaw", "k_yaw_d", "max_yaw_rate",
    "controller_fov_horizontal_limit_deg", "controller_fov_vertical_limit_deg",
    "multisim_launch_file", "visual_forward_speed_mps",
    "tuning_score",
]


def finite(value, fallback):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def score(row):
    """Lower is better; leaving the FOV dominates PN disturbance."""
    return (
        (0.0 if row.get("success") else 1000.0) +
        20.0 * finite(row.get("post_handover_fov_exceedance_percent"), 100.0) +
        200.0 * max(0.0, finite(row.get("post_handover_p95_fov_ratio"), 2.0) - 0.85) +
        2.0 * finite(row.get("fov_correction_linear_ratio_percent"), 100.0) +
        0.5 * finite(row.get("fov_correction_yaw_ratio_percent"), 100.0)
    )


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: aggregate_fov_tuning.py BATCH_DIRECTORY")
    batch_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
    rows = []
    for name in sorted(os.listdir(batch_dir)):
        run_dir = os.path.join(batch_dir, name)
        summary_path = os.path.join(run_dir, "summary.json")
        params_path = os.path.join(run_dir, "case_params.json")
        if not os.path.isfile(summary_path):
            continue
        with open(summary_path, encoding="utf-8") as stream:
            row = json.load(stream)
        if os.path.isfile(params_path):
            with open(params_path, encoding="utf-8") as stream:
                row.update(json.load(stream))
        row["tuning_score"] = score(row)
        rows.append(row)
    rows.sort(key=lambda item: item["tuning_score"])
    with open(os.path.join(batch_dir, "fov_tuning_summary.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(batch_dir, "fov_tuning_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(rows, stream, ensure_ascii=False, indent=2, allow_nan=True)
    print("rank case success maxFOV p95FOV exceed% linearImpact% score")
    for index, row in enumerate(rows, 1):
        print("%d %s %s %.3f %.3f %.1f %.1f %.1f" % (
            index, row.get("case_name"), row.get("success"),
            finite(row.get("post_handover_max_fov_ratio"), float("nan")),
            finite(row.get("post_handover_p95_fov_ratio"), float("nan")),
            finite(row.get("post_handover_fov_exceedance_percent"), float("nan")),
            finite(row.get("fov_correction_linear_ratio_percent"), float("nan")),
            row["tuning_score"]))


if __name__ == "__main__":
    main()
