#!/usr/bin/env python3
"""Analyze one radar-approach-to-IBVS PX4/Gazebo run."""

import csv
import json
import math
import os
import statistics
import sys


def number(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")
    return value


def finite(values):
    return [value for value in values if math.isfinite(value)]


def mean_or_nan(values):
    values = finite(values)
    return statistics.mean(values) if values else float("nan")


def max_or_nan(values):
    values = finite(values)
    return max(values) if values else float("nan")


def min_or_nan(values):
    values = finite(values)
    return min(values) if values else float("nan")


def rms_or_nan(values):
    values = finite(values)
    return math.sqrt(statistics.mean(value * value for value in values)) if values else float("nan")


def maximum_continuous_false_duration(rows, key):
    longest = 0.0
    loss_start = None
    last_time = None
    for row in rows:
        now = number(row, "ros_time_s")
        if not math.isfinite(now):
            continue
        last_time = now
        is_true = bool(int(float(row.get(key, 0))))
        if not is_true and loss_start is None:
            loss_start = now
        elif is_true and loss_start is not None:
            longest = max(longest, now - loss_start)
            loss_start = None
    if loss_start is not None and last_time is not None:
        longest = max(longest, last_time - loss_start)
    return longest


def percentile_or_nan(values, percentile):
    values = sorted(finite(values))
    if not values:
        return float("nan")
    position = (len(values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def vector_error(row, prefix, truth_prefix="target"):
    values = [
        number(row, prefix + "_x") - number(row, truth_prefix + "_x"),
        number(row, prefix + "_y") - number(row, truth_prefix + "_y"),
        number(row, prefix + "_z") - number(row, truth_prefix + "_z"),
    ]
    if not all(math.isfinite(value) for value in values):
        return float("nan")
    return math.sqrt(sum(value * value for value in values))


def terminal_error(row):
    values = [
        number(row, "handover_x") - number(row, "interceptor_x"),
        number(row, "handover_y") - number(row, "interceptor_y"),
        number(row, "handover_z") - number(row, "interceptor_z"),
    ]
    if not all(math.isfinite(value) for value in values):
        return float("nan")
    return math.sqrt(sum(value * value for value in values))


def horizontal_speed(row, prefix):
    vx = number(row, prefix + "_vx")
    vy = number(row, prefix + "_vy")
    return math.hypot(vx, vy) if math.isfinite(vx) and math.isfinite(vy) else float("nan")


def linear_magnitude(row, prefix):
    values = [number(row, prefix + "_vx"), number(row, prefix + "_vy"),
              number(row, prefix + "_vz")]
    if not all(math.isfinite(value) for value in values):
        return float("nan")
    return math.sqrt(sum(value * value for value in values))


def classify_failure(summary, params):
    if summary["success"]:
        if summary.get("approach_only", False):
            return "进入交接距离区间并满足视觉区域及姿态稳定条件"
        return "进入 %.1f m 拦截判定半径" % params.get("capture_distance_m", 5.0)
    if not summary["chase_started"]:
        return "雷达滤波未完成初始化，未进入抵近阶段"
    if summary["handover_occurred"] is False:
        if summary["radar_timeout_fraction"] > 0.05:
            return "雷达长时间断更触发 RADAR_TIMEOUT，抵近指令被置零"
        if abs(params.get("target_climb_rate_mps", 0.0)) >= 2.0:
            return "目标垂向速度达到或超过抵近控制器 2 m/s 的垂向限幅"
        if params.get("target_speed_mps", 0.0) >= 12.0:
            return "目标水平速度达到或超过抵近控制器 12 m/s 的水平限幅，无法闭合距离"
        legacy_pose_gate = str(params.get(
            "use_legacy_pose_handover_gate", True)).lower() in ("1", "true")
        position_tolerance = float(params.get("handover_position_tolerance_m", 2.5))
        if (legacy_pose_gate and
                summary["minimum_terminal_error_m"] > position_tolerance):
            return "未进入 %.1f m 交接位置容差，视觉控制未接管" % position_tolerance
        if summary["tracking_availability_percent"] < 20.0:
            return "抵近末端未形成稳定视觉检测，未满足连续区域交接条件"
        if legacy_pose_gate:
            return "位置、航向或连续区域检测条件未能同时满足，未发生视觉交接"
        return "时限内未同时满足交接区间、视觉区域、雷达估计健康及姿态稳定条件"
    if (summary["post_handover_tracking_availability_percent"] < 70.0 or
            summary["post_handover_max_fov_ratio"] > 1.0):
        return "视觉交接后目标频繁越出可用视场，未在时限内闭合到拦截半径"
    if summary["mean_target_horizontal_speed_mps"] >= summary["mean_interceptor_horizontal_speed_mps"]:
        return "视觉交接后测试机平均速度不高于靶机，缺少有效闭合速度"
    return "已完成视觉交接，但在设定追击时限内未进入拦截半径"


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_radar_multisim_result.py RUN_DIRECTORY")
    run_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
    csv_path = os.path.join(run_dir, "experiment.csv")
    with open(csv_path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("No samples found in " + csv_path)

    params_path = os.path.join(run_dir, "case_params.json")
    params = {}
    if os.path.exists(params_path):
        with open(params_path, encoding="utf-8") as stream:
            params = json.load(stream)

    active = [row for row in rows if row["phase"] in ("RADAR_ACQUIRE", "CHASE")]
    chase = [row for row in rows if row["phase"] == "CHASE"]
    t0 = number(active[0] if active else rows[0], "ros_time_s")
    chase_t0 = number(chase[0], "ros_time_s") if chase else float("nan")
    pre_chase = [
        row for row in rows
        if chase and number(row, "ros_time_s") < chase_t0]
    pre_chase_tracking_samples = sum(
        int(float(row.get("tracking", 0))) for row in pre_chase)
    visual_only = bool(chase) and all(
        row.get("radar_state") == "DISABLED" for row in chase)
    handover_rows = [row for row in rows if int(float(row.get("handover_complete", 0))) == 1]
    approach_only = str(params.get("stop_on_handover", "false")).lower() in (
        "1", "true", "yes")
    handover_hold_rows = [row for row in rows if row["phase"] == "HANDOVER_HOLD"]
    post_handover = (chase if visual_only else [
        row for row in chase
        if int(float(row.get("handover_complete", 0))) == 1])
    success = bool(handover_hold_rows) if approach_only else any(
        row["phase"] == "SUCCESS_HOLD" for row in rows)
    success_rows = handover_hold_rows if approach_only else [
        row for row in rows if row["phase"] == "SUCCESS_HOLD"]

    # The approach node deliberately stops updating its predicted-target
    # diagnostic after visual handover.  Restrict radar-estimator accuracy to
    # the interval in which radar still owns the guidance loop; otherwise the
    # last valid prediction is compared with a target that keeps moving and a
    # successful visual intercept appears to have a growing radar error.
    radar_evaluation_rows = [
        row for row in active
        if int(float(row.get("handover_complete", 0))) == 0]

    distances = finite(number(row, "distance_m") for row in (chase or rows))
    prediction_errors = finite(
        vector_error(row, "predicted_target") for row in radar_evaluation_rows)
    terminal_errors = finite(terminal_error(row) for row in active)
    fov_ratios = finite(
        max(number(row, "horizontal_fov_ratio"), number(row, "vertical_fov_ratio"))
        for row in chase)
    tracking = [int(float(row.get("tracking", 0))) for row in chase]
    post_handover_tracking = [
        int(float(row.get("tracking", 0))) for row in post_handover]
    post_handover_fov_ratios = finite(
        max(number(row, "horizontal_fov_ratio"), number(row, "vertical_fov_ratio"))
        for row in post_handover)
    post_handover_t0 = (
        number(post_handover[0], "ros_time_s") if post_handover else float("nan"))
    settled_post_handover = [
        row for row in post_handover
        if number(row, "ros_time_s") - post_handover_t0 >= 1.0]
    settled_post_handover_fov_ratios = finite(
        max(number(row, "horizontal_fov_ratio"), number(row, "vertical_fov_ratio"))
        for row in settled_post_handover)
    nominal_linear = finite(linear_magnitude(row, "nominal_pn") for row in post_handover)
    correction_linear = finite(
        linear_magnitude(row, "fov_correction") for row in post_handover)
    nominal_yaw = finite(number(row, "nominal_pn_yaw_rate") for row in post_handover)
    correction_yaw = finite(
        number(row, "fov_correction_yaw_rate") for row in post_handover)
    correction_vertical = finite(
        number(row, "fov_correction_vz") for row in post_handover)
    guard_shapes = finite(
        number(row, "fov_guard_shape") for row in post_handover
        if "fov_guard_shape" in row)
    nominal_linear_rms = rms_or_nan(nominal_linear)
    correction_linear_rms = rms_or_nan(correction_linear)
    nominal_yaw_rms = rms_or_nan(nominal_yaw)
    correction_yaw_rms = rms_or_nan(correction_yaw)
    radar_timeout_rows = [row for row in active if row.get("radar_state") == "RADAR_TIMEOUT"]
    initial_chase_fov_ratio = (
        max(number(chase[0], "horizontal_fov_ratio"),
            number(chase[0], "vertical_fov_ratio")) if chase else float("nan"))
    capture_time_from_chase = (
        number(success_rows[0], "ros_time_s") - chase_t0
        if success_rows and chase else float("nan"))
    capture_distance_at_success = (
        number(success_rows[0], "distance_m") if success_rows else float("nan"))
    interceptor_disturbance_rms = rms_or_nan(
        linear_magnitude(row, "interceptor_disturbance") for row in chase)
    target_disturbance_rms = rms_or_nan(
        linear_magnitude(row, "target_disturbance") for row in chase)
    visual_disturbance_rms = rms_or_nan(
        math.hypot(number(row, "visual_disturbance_u_px"),
                   number(row, "visual_disturbance_v_px")) for row in chase)
    radar_event_rows = []
    previous_radar_samples = -1
    for row in active:
        sample_count = int(float(row.get("radar_samples", 0)))
        if sample_count > previous_radar_samples:
            if sample_count > 0:
                radar_event_rows.append(row)
            previous_radar_samples = sample_count
    radar_bias_rms = rms_or_nan(
        math.sqrt(number(row, "radar_bias_x_m") ** 2 +
                  number(row, "radar_bias_y_m") ** 2 +
                  number(row, "radar_bias_z_m") ** 2)
        for row in radar_event_rows)
    radar_measurement_error_rms = rms_or_nan(
        number(row, "radar_measurement_error_m") for row in radar_event_rows)
    visual_bias_rms = rms_or_nan(
        math.hypot(number(row, "visual_bias_u_px"),
                   number(row, "visual_bias_v_px")) for row in chase)
    radar_delays = finite(number(row, "radar_delay_s") for row in radar_event_rows)
    visual_delays = finite(number(row, "visual_delay_s") for row in chase)
    guidance_delays = finite(number(row, "guidance_delay_s") for row in chase)

    summary = {
        "case_name": params.get("case_name", os.path.basename(run_dir)),
        "expected_result": params.get("expected_result", "unspecified"),
        "approach_only": approach_only,
        "success": success,
        "chase_started": bool(chase),
        "handover_occurred": bool(handover_rows) or visual_only,
        "visual_only": visual_only,
        "pre_chase_tracking_samples": pre_chase_tracking_samples,
        "initial_chase_tracking": (
            bool(int(float(chase[0].get("tracking", 0)))) if chase else False),
        "initial_chase_fov_ratio": initial_chase_fov_ratio,
        "initial_chase_distance_m": number(chase[0], "distance_m") if chase else float("nan"),
        "minimum_distance_m": min_or_nan(distances),
        "final_distance_m": number((chase or rows)[-1], "distance_m"),
        "chase_duration_s": (
            number(chase[-1], "ros_time_s") - chase_t0 if chase else 0.0),
        "capture_time_from_chase_s": capture_time_from_chase,
        "capture_distance_at_success_m": capture_distance_at_success,
        "capture_time_fraction": (
            capture_time_from_chase / float(params.get("chase_timeout_s", 45.0))
            if math.isfinite(capture_time_from_chase) and
            float(params.get("chase_timeout_s", 45.0)) > 0.0 else float("nan")),
        "handover_time_from_chase_s": (
            number(handover_rows[0], "ros_time_s") - chase_t0
            if handover_rows and chase else (0.0 if visual_only else float("nan"))),
        "tracking_availability_percent": (
            100.0 * statistics.mean(tracking) if tracking else 0.0),
        "max_continuous_tracking_loss_s": maximum_continuous_false_duration(
            chase, "tracking"),
        "post_handover_max_continuous_tracking_loss_s":
            maximum_continuous_false_duration(post_handover, "tracking"),
        "max_fov_ratio": max_or_nan(fov_ratios),
        "post_handover_tracking_availability_percent": (
            100.0 * statistics.mean(post_handover_tracking)
            if post_handover_tracking else 0.0),
        "post_handover_max_fov_ratio": max_or_nan(post_handover_fov_ratios),
        "post_handover_p95_fov_ratio": percentile_or_nan(
            post_handover_fov_ratios, 95.0),
        "post_handover_fov_exceedance_percent": (
            100.0 * sum(value >= 1.0 for value in post_handover_fov_ratios) /
            len(post_handover_fov_ratios)
            if post_handover_fov_ratios else 0.0),
        "settled_post_handover_max_fov_ratio": max_or_nan(
            settled_post_handover_fov_ratios),
        "settled_post_handover_fov_exceedance_percent": (
            100.0 * sum(value >= 1.0 for value in settled_post_handover_fov_ratios) /
            len(settled_post_handover_fov_ratios)
            if settled_post_handover_fov_ratios else 0.0),
        "post_handover_fov_guard_active_percent": (
            100.0 * sum(value > 1e-9 for value in guard_shapes) /
            len(guard_shapes)
            if guard_shapes else 0.0),
        "fov_correction_linear_rms_mps": correction_linear_rms,
        "nominal_pn_linear_rms_mps": nominal_linear_rms,
        "fov_correction_linear_ratio_percent": (
            100.0 * correction_linear_rms / nominal_linear_rms
            if math.isfinite(correction_linear_rms) and
            math.isfinite(nominal_linear_rms) and nominal_linear_rms > 1e-6
            else float("nan")),
        "fov_correction_vertical_rms_mps": rms_or_nan(correction_vertical),
        "fov_correction_yaw_rms_rps": correction_yaw_rms,
        "nominal_pn_yaw_rms_rps": nominal_yaw_rms,
        "fov_correction_yaw_ratio_percent": (
            100.0 * correction_yaw_rms / nominal_yaw_rms
            if math.isfinite(correction_yaw_rms) and
            math.isfinite(nominal_yaw_rms) and nominal_yaw_rms > 1e-6
            else float("nan")),
        "radar_prediction_rmse_m": (
            math.sqrt(statistics.mean(value * value for value in prediction_errors))
            if prediction_errors else float("nan")),
        "radar_prediction_max_error_m": max_or_nan(prediction_errors),
        "minimum_terminal_error_m": min_or_nan(terminal_errors),
        "radar_timeout_fraction": (
            len(radar_timeout_rows) / float(len(active)) if active else 0.0),
        "mean_target_horizontal_speed_mps": mean_or_nan(
            horizontal_speed(row, "target") for row in chase),
        "mean_interceptor_horizontal_speed_mps": mean_or_nan(
            horizontal_speed(row, "interceptor") for row in chase),
        "maximum_interceptor_horizontal_speed_mps": max_or_nan(
            horizontal_speed(row, "interceptor") for row in chase),
        "radar_samples": max(int(float(row.get("radar_samples", 0))) for row in rows),
        "radar_bypassed": visual_only and
            max(int(float(row.get("radar_samples", 0))) for row in rows) == 0,
        "disturbance_model": params.get("disturbance_model", "none"),
        "interceptor_disturbance_rms_mps": interceptor_disturbance_rms,
        "target_disturbance_rms_mps": target_disturbance_rms,
        "visual_disturbance_rms_px": visual_disturbance_rms,
        "radar_bias_rms_m": radar_bias_rms,
        "radar_measurement_error_rms_m": radar_measurement_error_rms,
        "radar_delay_mean_s": mean_or_nan(radar_delays),
        "radar_delay_max_s": max_or_nan(radar_delays),
        "radar_dropped_count": max(
            int(float(row.get("radar_dropped_count", 0))) for row in rows),
        "visual_bias_rms_px": visual_bias_rms,
        "visual_delay_mean_s": mean_or_nan(visual_delays),
        "visual_delay_p95_s": percentile_or_nan(visual_delays, 95.0),
        "visual_dropped_count": max(
            int(float(row.get("visual_dropped_count", 0))) for row in rows),
        "guidance_delay_mean_s": mean_or_nan(guidance_delays),
        "guidance_delay_p95_s": percentile_or_nan(guidance_delays, 95.0),
        "guidance_dropped_count": max(
            int(float(row.get("guidance_dropped_count", 0))) for row in rows),
        "analysis_start_time_s": t0,
    }
    summary["visual_only_pass"] = bool(
        summary["visual_only"] and summary["success"] and
        summary["pre_chase_tracking_samples"] == 0 and
        summary["initial_chase_tracking"] and summary["radar_bypassed"] and
        math.isfinite(summary["initial_chase_fov_ratio"]) and
        summary["initial_chase_fov_ratio"] < 1.0 and
        summary["post_handover_fov_exceedance_percent"] == 0.0 and
        summary["post_handover_max_continuous_tracking_loss_s"] <= 0.300001)
    summary["reason"] = classify_failure(summary, params)
    summary["full_chain_pass"] = bool(
        not summary["visual_only"] and summary["success"] and
        summary["handover_occurred"] and
        summary["post_handover_tracking_availability_percent"] >= 99.0 and
        summary["post_handover_max_continuous_tracking_loss_s"] <= 0.300001 and
        math.isfinite(summary["post_handover_max_fov_ratio"]) and
        summary["post_handover_max_fov_ratio"] < 1.0 and
        summary["post_handover_fov_exceedance_percent"] == 0.0)

    with open(os.path.join(run_dir, "summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=True)

    lines = [
        ("PX4/Gazebo radar approach-only summary" if approach_only else
         ("PX4/Gazebo direct visual-interception summary" if visual_only else
          "PX4/Gazebo radar approach -> fixed-camera IBVS summary")),
        "case: %s" % summary["case_name"],
        "result: %s" % ("SUCCESS" if success else "FAILURE"),
        "reason: %s" % summary["reason"],
        "handover occurred: %s" % summary["handover_occurred"],
        "visual-only / radar bypassed / visual-only pass: %s / %s / %s" % (
            summary["visual_only"], summary["radar_bypassed"],
            summary["visual_only_pass"]),
        "pre-CHASE tracking samples: %d" %
        summary["pre_chase_tracking_samples"],
        "initial CHASE tracking / FOV ratio: %s / %.3f" % (
            summary["initial_chase_tracking"], summary["initial_chase_fov_ratio"]),
        "CHASE initial/minimum/final distance: %.2f / %.2f / %.2f m" % (
            summary["initial_chase_distance_m"], summary["minimum_distance_m"],
            summary["final_distance_m"]),
        "chase duration: %.2f s" % summary["chase_duration_s"],
        "capture time / chase-budget fraction / success distance: %.2f s / %.3f / %.3f m" % (
            summary["capture_time_from_chase_s"], summary["capture_time_fraction"],
            summary["capture_distance_at_success_m"]),
        "handover time from chase: %.2f s" % summary["handover_time_from_chase_s"],
        "minimum terminal-position error: %.2f m" % summary["minimum_terminal_error_m"],
        "radar prediction RMSE/max: %.2f / %.2f m" % (
            summary["radar_prediction_rmse_m"],
            summary["radar_prediction_max_error_m"]),
        "tracking availability during chase: %.1f %%" %
        summary["tracking_availability_percent"],
        "tracking availability after handover: %.1f %%" %
        summary["post_handover_tracking_availability_percent"],
        "maximum continuous tracking loss: %.3f s" %
        summary["max_continuous_tracking_loss_s"],
        "maximum continuous tracking loss after handover: %.3f s" %
        summary["post_handover_max_continuous_tracking_loss_s"],
        "maximum normalized FOV usage: %.3f" % summary["max_fov_ratio"],
        "maximum normalized FOV after handover: %.3f" %
        summary["post_handover_max_fov_ratio"],
        "post-handover FOV p95 / exceedance: %.3f / %.1f %%" % (
            summary["post_handover_p95_fov_ratio"],
            summary["post_handover_fov_exceedance_percent"]),
        "settled FOV max / exceedance after 1 s: %.3f / %.1f %%" % (
            summary["settled_post_handover_max_fov_ratio"],
            summary["settled_post_handover_fov_exceedance_percent"]),
        "FOV correction linear RMS / nominal PN: %.3f m/s / %.1f %%" % (
            summary["fov_correction_linear_rms_mps"],
            summary["fov_correction_linear_ratio_percent"]),
        "FOV correction yaw RMS / nominal PN: %.3f rad/s / %.1f %%" % (
            summary["fov_correction_yaw_rms_rps"],
            summary["fov_correction_yaw_ratio_percent"]),
        "mean target/interceptor horizontal speed: %.2f / %.2f m/s" % (
            summary["mean_target_horizontal_speed_mps"],
            summary["mean_interceptor_horizontal_speed_mps"]),
        "disturbance model / interceptor / target / visual RMS: %s / %.3f m/s / %.3f m/s / %.3f px" % (
            summary["disturbance_model"],
            summary["interceptor_disturbance_rms_mps"],
            summary["target_disturbance_rms_mps"],
            summary["visual_disturbance_rms_px"]),
        "radar measurement/bias RMS, delay mean/max, drops: %.3f / %.3f m, %.3f / %.3f s, %d" % (
            summary["radar_measurement_error_rms_m"],
            summary["radar_bias_rms_m"], summary["radar_delay_mean_s"],
            summary["radar_delay_max_s"], summary["radar_dropped_count"]),
        "visual bias RMS, delay mean/p95, drops: %.3f px, %.3f / %.3f s, %d" % (
            summary["visual_bias_rms_px"], summary["visual_delay_mean_s"],
            summary["visual_delay_p95_s"], summary["visual_dropped_count"]),
        "guidance delay mean/p95, drops, full-chain pass: %.3f / %.3f s, %d, %s" % (
            summary["guidance_delay_mean_s"],
            summary["guidance_delay_p95_s"],
            summary["guidance_dropped_count"], summary["full_chain_pass"]),
    ]
    text = "\n".join(lines) + "\n"
    with open(os.path.join(run_dir, "summary.txt"), "w", encoding="utf-8") as stream:
        stream.write(text)
    print(text, end="")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plot_rows = list(active or rows)
        if active and success_rows:
            plot_rows.append(success_rows[0])
        times = [number(row, "ros_time_s") - t0 for row in plot_rows]
        range_values = [number(row, "distance_m") for row in plot_rows]
        terminal_values = [terminal_error(row)
                           if int(float(row.get("handover_complete", 0))) == 0
                           else float("nan") for row in plot_rows]
        prediction_values = [vector_error(row, "predicted_target")
                             if int(float(row.get("handover_complete", 0))) == 0
                             else float("nan") for row in plot_rows]
        fov_values = [max(number(row, "horizontal_fov_ratio"),
                          number(row, "vertical_fov_ratio")) for row in plot_rows]
        target_speeds = [horizontal_speed(row, "target") for row in plot_rows]
        interceptor_speeds = [horizontal_speed(row, "interceptor") for row in plot_rows]

        panel_count = 3 if visual_only else 4
        figure, axes = plt.subplots(
            panel_count, 1,
            figsize=(10, 8.5 if visual_only else 11), sharex=True)
        axes[0].plot(times, range_values, color="black", label="separation")
        axes[0].axhline(params.get("capture_distance_m", 5.0), color="green",
                        linestyle="--", label="capture threshold")
        if success_rows:
            axes[0].scatter(
                [number(success_rows[0], "ros_time_s") - t0],
                [capture_distance_at_success], color="green", zorder=3,
                label="success decision frame")
        axes[0].set_ylabel("range (m)")
        axes[0].legend(loc="best")
        if visual_only:
            speed_axis = axes[1]
            fov_axis = axes[2]
        else:
            axes[1].plot(times, terminal_values, label="terminal pose error")
            axes[1].plot(times, prediction_values, label="radar prediction error")
            if str(params.get("use_legacy_pose_handover_gate", True)).lower() in ("1", "true"):
                axes[1].axhline(
                    float(params.get("handover_position_tolerance_m", 2.5)),
                    color="gray", linestyle="--", label="handover tolerance")
            axes[1].set_ylabel("error (m)")
            axes[1].legend(loc="best")
            speed_axis = axes[2]
            fov_axis = axes[3]
        speed_axis.plot(times, target_speeds, label="target")
        speed_axis.plot(times, interceptor_speeds, label="interceptor")
        speed_axis.set_ylabel("horizontal speed (m/s)")
        speed_axis.legend(loc="best")
        fov_axis.plot(times, fov_values, color="purple", label="FOV ratio")
        fov_axis.axhline(1.0, color="red", linestyle="--", label="FOV boundary")
        fov_axis.set_ylabel("normalized FOV")
        fov_axis.set_xlabel(
            "time from CHASE start (s)" if visual_only else
            "time from radar acquisition (s)")
        fov_axis.legend(loc="best")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            if handover_rows:
                axis.axvline(number(handover_rows[0], "ros_time_s") - t0,
                             color="teal", linestyle=":", alpha=0.7)
        figure.suptitle("%s: %s" % (
            summary["case_name"], "SUCCESS" if success else "FAILURE"))
        figure.tight_layout(rect=(0, 0, 1, 0.97))
        figure.savefig(os.path.join(run_dir, "result_plot.png"), dpi=150)
        plt.close(figure)
    except Exception as exc:
        print("Plot generation skipped: " + str(exc))


if __name__ == "__main__":
    main()
