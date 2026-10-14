#!/usr/bin/env python3
"""Aggregate and rank direct visual-interception (情况_只拦截) cases."""

import csv
import json
import math
import os
import sys


FIELDS = [
    "case_name", "scenario_name", "disturbance_model", "disturbance_seed",
    "success", "visual_only_pass", "hard_pass", "hard_fail_reasons",
    "pre_chase_tracking_samples",
    "initial_chase_tracking", "initial_chase_fov_ratio",
    "capture_time_from_chase_s", "capture_time_fraction",
    "capture_distance_at_success_m", "minimum_distance_m",
    "post_handover_tracking_availability_percent",
    "max_continuous_tracking_loss_s", "post_handover_max_fov_ratio",
    "post_handover_p95_fov_ratio", "settled_post_handover_max_fov_ratio",
    "post_handover_fov_exceedance_percent",
    "fov_correction_linear_rms_mps", "fov_correction_linear_ratio_percent",
    "fov_correction_yaw_rms_rps", "fov_correction_yaw_ratio_percent",
    "interceptor_disturbance_rms_mps", "target_disturbance_rms_mps",
    "visual_disturbance_rms_px", "wind_enabled", "wind_velocity_mean_mps",
    "visual_forward_speed_mps", "forward_speed_accel_limit_mps2",
    "yaw_enable_speed_ratio", "fov_guard_center_area_ratio",
    "fov_guard_yaw_gain_radps", "fov_guard_vertical_gain_mps",
    "fov_guard_shape_max", "fov_barrier_gain", "fov_barrier_max",
    "fov_slowdown_start_ratio", "fov_min_forward_speed_ratio",
    "fov_emergency_yaw_ratio", "controller_fov_horizontal_limit_deg",
    "controller_fov_vertical_limit_deg", "max_vz", "tuning_score",
]


def finite(value, fallback):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def normalized(value, best, limit):
    if limit <= best:
        return 0.0
    return max(0.0, min(1.0, (finite(value, limit) - best) / (limit - best)))


def evaluate(row):
    disturbed = str(row.get("disturbance_model", "none")) != "none" or bool(
        finite(row.get("wind_enabled"), 0.0))
    tracking_limit = 99.5 if disturbed else 99.999
    reasons = []
    if not row.get("visual_only_pass"):
        reasons.append("visual_only_pass=false")
    if finite(row.get("capture_time_fraction"), 99.0) > 0.95:
        reasons.append("capture_time_fraction>0.95")
    if finite(row.get("post_handover_max_fov_ratio"), 99.0) > 0.99:
        reasons.append("max_fov>0.99")
    if finite(row.get("post_handover_p95_fov_ratio"), 99.0) > 0.85:
        reasons.append("p95_fov>0.85")
    if finite(row.get("settled_post_handover_max_fov_ratio"), 99.0) > 0.90:
        reasons.append("settled_fov>0.90")
    if finite(row.get("post_handover_tracking_availability_percent"), 0.0) < tracking_limit:
        reasons.append("tracking_below_limit")
    if finite(row.get("fov_correction_linear_ratio_percent"), 99.0) > 5.0:
        reasons.append("linear_correction>5%")
    if finite(row.get("fov_correction_linear_rms_mps"), 99.0) > 0.20:
        reasons.append("linear_correction>0.20mps")

    fov_component = (
        0.45 * normalized(row.get("post_handover_max_fov_ratio"), 0.85, 0.99) +
        0.25 * normalized(row.get("post_handover_p95_fov_ratio"), 0.60, 0.85) +
        0.20 * normalized(row.get("settled_post_handover_max_fov_ratio"), 0.70, 0.90) +
        0.10 * normalized(
            100.0 - finite(row.get("post_handover_tracking_availability_percent"), 0.0),
            0.0, 0.5))
    convergence_component = normalized(row.get("capture_time_fraction"), 0.75, 0.95)
    linear_component = normalized(
        row.get("fov_correction_linear_ratio_percent"), 1.0, 3.0)
    yaw_ratio = finite(row.get("fov_correction_yaw_ratio_percent"), float("nan"))
    if math.isfinite(yaw_ratio):
        yaw_component = normalized(yaw_ratio, 1.0, 3.0)
    else:
        yaw_component = normalized(row.get("fov_correction_yaw_rms_rps"), 0.01, 0.02)
    pn_component = 0.70 * linear_component + 0.30 * yaw_component
    row["hard_pass"] = not reasons
    row["hard_fail_reasons"] = ";".join(reasons)
    row["tuning_score"] = (
        (0.0 if row["hard_pass"] else 1000.0) +
        60.0 * fov_component + 25.0 * convergence_component +
        15.0 * pn_component)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: aggregate_intercept_only.py BATCH_DIRECTORY")
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
        evaluate(row)
        rows.append(row)
    rows.sort(key=lambda item: item["tuning_score"])
    for suffix in ("csv", "json"):
        output = os.path.join(batch_dir, "intercept_only_summary." + suffix)
        if suffix == "csv":
            with open(output, "w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
        else:
            with open(output, "w", encoding="utf-8") as stream:
                json.dump(rows, stream, ensure_ascii=False, indent=2, allow_nan=True)
    print("rank case disturbed pass timeFrac maxFOV p95FOV settled tracking score")
    for index, row in enumerate(rows, 1):
        print("%d %s %s %s %.3f %.3f %.3f %.3f %.2f %.1f" % (
            index, row.get("case_name"), row.get("disturbance_model"),
            row.get("hard_pass"), finite(row.get("capture_time_fraction"), float("nan")),
            finite(row.get("post_handover_max_fov_ratio"), float("nan")),
            finite(row.get("post_handover_p95_fov_ratio"), float("nan")),
            finite(row.get("settled_post_handover_max_fov_ratio"), float("nan")),
            finite(row.get("post_handover_tracking_availability_percent"), float("nan")),
            row["tuning_score"]))


if __name__ == "__main__":
    main()
