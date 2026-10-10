#!/usr/bin/env python3
"""Create compact client-facing tables for full-chain validation batches."""

import csv
import json
import math
import os
import statistics
import sys


GROUP_LABELS = {
    "tail": "名义尾追",
    "tail_slow": "低速尾追",
    "tail_fast": "高速尾追",
    "oblique45": "45°斜横穿",
    "cross90": "90°横穿",
    "climb08": "0.8 m/s持续爬升",
}

PROFILE_LABELS = {
    "clean": "基线（其余扰动关闭）",
    "mild": "综合温和误差",
    "ideal": "理想条件",
    "field": "接近实际条件",
    "harsh": "严苛条件",
}

PROFILE_ORDER = {
    "ideal": 0,
    "field": 1,
    "harsh": 2,
    "clean": 3,
    "mild": 4,
}

GROUP_ORDER = {
    "tail_slow": 0,
    "tail_fast": 1,
    "oblique45": 2,
    "cross90": 3,
    "tail": 4,
    "climb08": 5,
}

TIER_REQUIREMENTS = {
    "ideal": 100.0,
    "field": 90.0,
}

MINIMUM_SAFE_ALTITUDE_M = 2.0
SAFETY_PHASES = {
    "RADAR_ACQUIRE", "CHASE", "HANDOVER_HOLD", "SUCCESS_HOLD",
    "TIMEOUT_HOLD",
}
CRITICAL_CSV_FIELDS = (
    "distance_m",
    "interceptor_x", "interceptor_y", "interceptor_z",
    "target_x", "target_y", "target_z",
    "guidance_vx", "guidance_vy", "guidance_vz", "guidance_yaw_rate",
    "interceptor_vx", "interceptor_vy", "interceptor_vz",
    "target_vx", "target_vy", "target_vz",
)


def setting(record, key, default=None):
    """Return a recorded setting, preferring case_params over summaries."""
    params = record.get("params") or {}
    value = params.get(key)
    if value is None or value == "":
        value = record.get(key, default)
    return default if value is None or value == "" else value


def settings(records, key):
    values = []
    for record in records:
        value = setting(record, key)
        if value is None:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            pass
        if value not in values:
            values.append(value)
    return values


def fmt_settings(records, key, digits=2, default="未记录"):
    values = settings(records, key)
    if not values:
        return default
    rendered = [fmt(value, digits) for value in values]
    return rendered[0] if len(rendered) == 1 else " / ".join(rendered)


def radar_noise_text(records):
    values = settings(records, "radar_noise_std_m")
    if not values:
        return "三轴白噪声标准差未记录"
    rendered = []
    for value in values:
        try:
            per_axis = float(value)
        except (TypeError, ValueError):
            rendered.append(str(value))
            continue
        rendered.append("每轴σ=%s m（等效三维RMS≈%s m）" % (
            fmt(per_axis, 2), fmt(math.sqrt(3.0) * per_axis, 2)))
    return "；".join(rendered)


def finite(values):
    output = []
    for value in values:
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            output.append(value)
    return output


def median(values):
    values = finite(values)
    return statistics.median(values) if values else float("nan")


def maximum(values):
    values = finite(values)
    return max(values) if values else float("nan")


def minimum(values):
    values = finite(values)
    return min(values) if values else float("nan")


def fmt(value, digits=2):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(value):
        return "—"
    return ("%." + str(digits) + "f") % value


def profile_sort_key(profile):
    return (PROFILE_ORDER.get(profile, 100), str(profile))


def group_sort_key(group):
    return (GROUP_ORDER.get(group, 100), str(group))


def finite_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def inspect_run_health(case_dir, summary):
    """Read safety evidence without treating expected diagnostic NaNs as faults.

    Predicted-target, handover and image coordinates are legitimately NaN before
    their corresponding subsystem becomes active. Numeric health is therefore
    checked only on vehicle state and applied guidance fields during flight.
    """
    csv_path = os.path.join(case_dir, "experiment.csv")
    minimum_altitude = float("nan")
    target_vertical_speeds = []
    target_altitudes = []
    numeric_anomaly = False
    safety_data_available = False
    rows = []

    if os.path.isfile(csv_path):
        with open(csv_path, newline="", encoding="utf-8", errors="replace") as stream:
            rows = [row for row in csv.DictReader(stream)
                    if row.get("phase") in SAFETY_PHASES]
        safety_data_available = bool(rows)
        altitudes = []
        for row in rows:
            for key in CRITICAL_CSV_FIELDS:
                if not finite_number(row.get(key)):
                    numeric_anomaly = True
            if finite_number(row.get("interceptor_z")):
                altitudes.append(float(row["interceptor_z"]))
            if finite_number(row.get("target_vz")):
                target_vertical_speeds.append(float(row["target_vz"]))
            if finite_number(row.get("target_z")):
                target_altitudes.append(float(row["target_z"]))
        if altitudes:
            minimum_altitude = min(altitudes)
        else:
            numeric_anomaly = True
    else:
        # Future/alternate analyzers may save these derived fields directly.
        # Archived summaries without either field remain reportable but cannot
        # satisfy the safety-evidenced validation gate.
        recorded_altitude = summary.get("minimum_interceptor_altitude_m")
        recorded_anomaly = summary.get(
            "numeric_anomaly_detected", summary.get("numeric_anomaly"))
        if finite_number(recorded_altitude) and recorded_anomaly is not None:
            minimum_altitude = float(recorded_altitude)
            numeric_anomaly = bool(recorded_anomaly)
            safety_data_available = True
        else:
            numeric_anomaly = True

    # A run that never hands over has no post-handover FOV statistic. That
    # expected missing diagnostic is a failed handover, not a numeric fault.
    required_summary_fields = []
    if safety_data_available and any(row.get("phase") == "CHASE" for row in rows):
        required_summary_fields.append("minimum_distance_m")
    if summary.get("success"):
        required_summary_fields.extend([
            "capture_time_from_chase_s", "capture_distance_at_success_m",
        ])
    if summary.get("handover_occurred"):
        required_summary_fields.extend([
            "handover_time_from_chase_s", "post_handover_max_fov_ratio",
        ])
    if any(not finite_number(summary.get(key)) for key in required_summary_fields):
        numeric_anomaly = True

    return {
        "minimum_interceptor_altitude_m": minimum_altitude,
        "target_vertical_speed_rms_mps": (
            math.sqrt(sum(value * value for value in target_vertical_speeds) /
                      len(target_vertical_speeds))
            if target_vertical_speeds else float("nan")),
        "target_vertical_speed_peak_mps": (
            max(abs(value) for value in target_vertical_speeds)
            if target_vertical_speeds else float("nan")),
        "target_altitude_span_m": (
            max(target_altitudes) - min(target_altitudes)
            if target_altitudes else float("nan")),
        "numeric_anomaly_detected": numeric_anomaly,
        "safety_data_available": safety_data_available,
    }


def primary_validation_pass(record):
    fov = record.get("post_handover_max_fov_ratio")
    altitude = record.get("minimum_interceptor_altitude_m")
    capture_distance = record.get("capture_distance_at_success_m")
    capture_threshold = setting(record, "capture_distance_m", 5.0)
    return bool(
        record.get("success") and
        finite_number(capture_distance) and
        finite_number(capture_threshold) and
        float(capture_distance) <= float(capture_threshold) and
        record.get("handover_occurred") and
        finite_number(fov) and float(fov) < 1.0 and
        record.get("safety_data_available") and
        finite_number(altitude) and float(altitude) >= MINIMUM_SAFE_ALTITUDE_M and
        not record.get("numeric_anomaly_detected"))


def primary_failure_reason(record):
    if not record.get("success"):
        return "未进入拦截判定半径"
    capture_distance = record.get("capture_distance_at_success_m")
    capture_threshold = setting(record, "capture_distance_m", 5.0)
    if (not finite_number(capture_distance) or
            not finite_number(capture_threshold) or
            float(capture_distance) > float(capture_threshold)):
        return "成功判定距离缺失或超过设定门限"
    if not record.get("handover_occurred"):
        return "未完成雷达—视觉交接"
    fov = record.get("post_handover_max_fov_ratio")
    if not finite_number(fov) or float(fov) >= 1.0:
        return "交接后越出物理视野或FOV数据无效"
    if not record.get("safety_data_available"):
        return "缺少安全高度或数值健康性证据"
    altitude = record.get("minimum_interceptor_altitude_m")
    if not finite_number(altitude) or float(altitude) < MINIMUM_SAFE_ALTITUDE_M:
        return "飞行阶段最低高度低于%.1f m" % MINIMUM_SAFE_ALTITUDE_M
    if record.get("numeric_anomaly_detected"):
        return "关键状态或控制量存在NaN/Inf/缺失"
    return "满足主通过判据"


def load_records(batch_dir):
    records = []
    loaded_case_names = set()
    for name in sorted(os.listdir(batch_dir)):
        case_dir = os.path.join(batch_dir, name)
        summary_path = os.path.join(case_dir, "summary.json")
        params_path = os.path.join(case_dir, "case_params.json")
        if not os.path.isfile(summary_path):
            continue
        with open(summary_path, encoding="utf-8") as stream:
            summary = json.load(stream)
        params = {}
        if os.path.isfile(params_path):
            with open(params_path, encoding="utf-8") as stream:
                params = json.load(stream)
        summary["test_group"] = params.get(
            "test_group", summary.get("test_group", "unspecified"))
        summary["error_profile"] = params.get(
            "error_profile", summary.get("error_profile", "clean"))
        summary["params"] = params
        summary["case_dir"] = name
        summary.update(inspect_run_health(case_dir, summary))
        summary["primary_validation_pass"] = primary_validation_pass(summary)
        summary["primary_validation_reason"] = primary_failure_reason(summary)
        records.append(summary)
        loaded_case_names.add(str(summary["case_name"]))

    # A launcher crash may leave a log but no summary. Count that attempted
    # case as a failed run instead of silently improving the reported rate.
    # CASE_FILTER smoke tests are not penalized because skipped rows have no
    # launcher log.
    cases_path = os.path.join(batch_dir, "cases.csv")
    if os.path.isfile(cases_path):
        with open(cases_path, newline="", encoding="utf-8-sig") as stream:
            expected_rows = list(csv.DictReader(stream))
        for row in expected_rows:
            case_name = str(row.get("CASE_NAME", "")).strip()
            launcher_log = os.path.join(
                batch_dir, case_name + "_launcher.log")
            if (not case_name or case_name in loaded_case_names or
                    not os.path.isfile(launcher_log)):
                continue
            params = {
                str(key).lower(): value for key, value in row.items()
                if key and value not in (None, "")
            }
            incomplete = {
                "case_name": case_name,
                "test_group": params.get("test_group", "unspecified"),
                "error_profile": params.get("error_profile", "clean"),
                "params": params,
                "case_dir": case_name,
                "success": False,
                "handover_occurred": False,
                "full_chain_pass": False,
                "minimum_distance_m": float("nan"),
                "handover_time_from_chase_s": float("nan"),
                "capture_time_from_chase_s": float("nan"),
                "capture_distance_at_success_m": float("nan"),
                "radar_prediction_rmse_m": float("nan"),
                "post_handover_tracking_availability_percent": 0.0,
                "post_handover_max_fov_ratio": float("nan"),
                "post_handover_p95_fov_ratio": float("nan"),
                "radar_delay_mean_s": float("nan"),
                "visual_delay_mean_s": float("nan"),
                "guidance_delay_mean_s": float("nan"),
                "minimum_interceptor_altitude_m": float("nan"),
                "numeric_anomaly_detected": True,
                "safety_data_available": False,
                "primary_validation_pass": False,
                "primary_validation_reason": "运行未生成summary.json",
            }
            records.append(incomplete)
    return sorted(records, key=lambda record: str(record.get("case_name", "")))


def summarize_selection(selected, group=None, profile=None):
    runs = len(selected)
    passes = sum(bool(record.get("primary_validation_pass"))
                 for record in selected)
    row = {}
    if group is not None:
        row["test_group"] = group
    if profile is not None:
        row["error_profile"] = profile
    row.update({
        "runs": runs,
        "passes": passes,
        "pass_rate_percent": 100.0 * passes / runs if runs else float("nan"),
        "handovers": sum(bool(record.get("handover_occurred"))
                         for record in selected),
        "legacy_full_chain_passes": sum(
            bool(record.get("full_chain_pass")) for record in selected),
        "numeric_anomaly_runs": sum(
            bool(record.get("numeric_anomaly_detected")) for record in selected),
        "unsafe_altitude_runs": sum(
            finite_number(record.get("minimum_interceptor_altitude_m")) and
            float(record["minimum_interceptor_altitude_m"]) <
            MINIMUM_SAFE_ALTITUDE_M for record in selected),
        "minimum_altitude_worst_m": minimum(
            record.get("minimum_interceptor_altitude_m") for record in selected),
        "target_vertical_speed_rms_worst_mps": maximum(
            record.get("target_vertical_speed_rms_mps") for record in selected),
        "target_vertical_speed_peak_worst_mps": maximum(
            record.get("target_vertical_speed_peak_mps") for record in selected),
        "target_altitude_span_worst_m": maximum(
            record.get("target_altitude_span_m") for record in selected),
        "radar_prediction_rmse_median_m": median(
            record.get("radar_prediction_rmse_m") for record in selected),
        "radar_prediction_rmse_worst_m": maximum(
            record.get("radar_prediction_rmse_m") for record in selected),
        "handover_time_median_s": median(
            record.get("handover_time_from_chase_s") for record in selected),
        "capture_time_median_s": median(
            record.get("capture_time_from_chase_s") for record in selected),
        "capture_time_worst_s": maximum(
            record.get("capture_time_from_chase_s") for record in selected),
        "capture_distance_worst_m": maximum(
            record.get("capture_distance_at_success_m") for record in selected
            if record.get("success")),
        "minimum_distance_worst_m": maximum(
            record.get("minimum_distance_m") for record in selected),
        "tracking_worst_percent": minimum(
            record.get("post_handover_tracking_availability_percent")
            for record in selected),
        "fov_max_worst": maximum(
            record.get("post_handover_max_fov_ratio") for record in selected),
        "fov_p95_worst": maximum(
            record.get("post_handover_p95_fov_ratio") for record in selected),
        "radar_delay_mean_s": median(
            record.get("radar_delay_mean_s") for record in selected),
        "visual_delay_mean_s": median(
            record.get("visual_delay_mean_s") for record in selected),
        "guidance_delay_mean_s": median(
            record.get("guidance_delay_mean_s") for record in selected),
    })
    return row


def aggregate(records):
    """Aggregate every profile/group pair present; no names are hard-coded."""
    pairs = sorted(
        {(record["error_profile"], record["test_group"]) for record in records},
        key=lambda item: (profile_sort_key(item[0]), group_sort_key(item[1])))
    output = []
    for profile, group in pairs:
        selected = [record for record in records
                    if record["test_group"] == group and
                    record["error_profile"] == profile]
        output.append(summarize_selection(selected, group, profile))
    return output


def aggregate_tiers(records):
    output = []
    for profile in sorted(
            {record["error_profile"] for record in records},
            key=profile_sort_key):
        selected = [record for record in records
                    if record["error_profile"] == profile]
        row = summarize_selection(selected, profile=profile)
        requirement = TIER_REQUIREMENTS.get(profile)
        row["required_pass_rate_percent"] = (
            requirement if requirement is not None else "")
        row["tier_requirement_met"] = (
            row["pass_rate_percent"] >= requirement
            if requirement is not None else "observation_only")
        output.append(row)
    return output


def write_rows(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_csv(batch_dir, rows):
    write_rows(os.path.join(batch_dir, "client_result_table.csv"), rows)


def write_tier_csv(batch_dir, rows):
    write_rows(os.path.join(batch_dir, "client_tier_result_table.csv"), rows)


def tier_conclusion(row):
    profile = row["error_profile"]
    requirement = TIER_REQUIREMENTS.get(profile)
    if requirement is None:
        return "仅观察边界" if profile == "harsh" else "未设置档级门槛"
    return "达标" if row["pass_rate_percent"] >= requirement else "未达标"


def write_markdown(batch_dir, records, rows, tier_rows=None):
    tier_rows = tier_rows if tier_rows is not None else aggregate_tiers(records)
    radar_period = fmt_settings(records, "radar_period_sec", 2)
    radar_timeout = fmt_settings(records, "radar_timeout_sec", 2)
    radar_bootstrap = fmt_settings(records, "radar_bootstrap_samples", 0)
    filter_alpha = fmt_settings(records, "filter_alpha", 2)
    filter_beta = fmt_settings(records, "filter_beta", 2)
    measurement_std = fmt_settings(records, "radar_measurement_std_m", 2)
    process_accel_std = fmt_settings(
        records, "radar_process_accel_std_mps2", 2)
    radar_innovation = fmt_settings(records, "max_radar_innovation_m", 1)
    max_target_speed = fmt_settings(records, "max_target_speed_mps", 1)
    radar_noise = radar_noise_text(records)
    lines = [
        "# 雷达抵近—固定相机拦截完整链路验证（简表）",
        "",
        "> 口径：双PX4 SITL + Gazebo Classic + 实际ROS制导控制节点，属于软件在环验证。",
        "> 视觉检测由真值几何投影生成，但接口保持 `/object_kcf` + `kcf_msgs/Bbox` 不变。",
        "",
        "## 表1  各档实验条件",
        "",
        "| 档位 | 雷达白噪声 | 雷达延迟/丢包 | 视觉延迟/丢帧 | 导航位置噪声 | 水平风 |",
        "|---|---|---|---|---|---|",
    ]
    for profile in sorted(
            {record["error_profile"] for record in records},
            key=profile_sort_key):
        selected = [record for record in records
                    if record["error_profile"] == profile]
        lines.append(
            "| {profile} | {noise} | {radar_delay}±{radar_jitter} s / {radar_drop} | "
            "{visual_delay}±{visual_jitter} s / {visual_drop} | {nav_noise} m | {wind} m/s |".format(
                profile=PROFILE_LABELS.get(profile, profile),
                noise=radar_noise_text(selected),
                radar_delay=fmt_settings(selected, "radar_delay_mean_sec", 2),
                radar_jitter=fmt_settings(
                    selected, "radar_delay_jitter_std_sec", 2),
                radar_drop=fmt_settings(
                    selected, "radar_dropout_probability", 3),
                visual_delay=fmt_settings(selected, "visual_delay_mean_sec", 2),
                visual_jitter=fmt_settings(
                    selected, "visual_delay_jitter_std_sec", 2),
                visual_drop=fmt_settings(
                    selected, "visual_dropout_probability", 3),
                nav_noise=fmt_settings(
                    selected, "nav_pose_position_white_noise_std_m", 2),
                wind=fmt_settings(selected, "wind_velocity_mean_mps", 1),
            ))
    lines.extend([
        "",
        "雷达噪声总览：%s；名义更新周期=%s s，超时=%s s。" % (
            radar_noise, radar_period, radar_timeout),
        "雷达抵近关键鲁棒参数：5帧稳健初始化样本数=%s，CV Kalman测量标准差=%s m，"
        "过程加速度标准差=%s m/s²，创新绝对保护=%s m，目标速度上限=%s m/s。" % (
            radar_bootstrap, measurement_std, process_accel_std,
            radar_innovation, max_target_speed),
    ])
    if any(record["error_profile"] in ("clean", "mild") for record in records):
        lines.append(
            "历史α-β配置（如有记录）：初始化样本数=%s，α-β滤波α/β=%s/%s，"
            "创新限幅=%s m。" % (
                radar_bootstrap, filter_alpha, filter_beta, radar_innovation))
    lines.extend([
        "",
        "## 表2  按档通过率",
        "",
        "| 档位 | 通过 | 通过率 | 要求 | 交接 | 最低高度最差值 m | 靶机垂速RMS最差值 m/s | 数值异常次数 | 结论 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ])
    for row in tier_rows:
        requirement = row["required_pass_rate_percent"]
        lines.append(
            "| {profile} | {passes}/{runs} | {rate}% | {required} | "
            "{handovers}/{runs} | {altitude} | {target_vz} | {anomalies} | {conclusion} |".format(
                profile=PROFILE_LABELS.get(
                    row["error_profile"], row["error_profile"]),
                passes=row["passes"], runs=row["runs"],
                rate=fmt(row["pass_rate_percent"], 1),
                required=("—" if requirement == "" else
                          "≥%s%%" % fmt(requirement, 0)),
                handovers=row["handovers"],
                altitude=fmt(row["minimum_altitude_worst_m"]),
                target_vz=fmt(row["target_vertical_speed_rms_worst_mps"], 3),
                anomalies=row["numeric_anomaly_runs"],
                conclusion=tier_conclusion(row)))
    lines.extend([
        "",
        "## 表3  分工况结果",
        "",
        "| 工况 | 档位 | 主判据通过 | 通过率 | 雷达预测RMSE 中位/最坏 m | "
        "交接时间中位 s | 拦截时间中位/最坏 s | 成功判定距离最坏 m | 最差跟踪率 | 最坏FOV max/P95 |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in rows:
        lines.append(
            "| {group} | {profile} | {passes}/{runs} | {rate}% | "
            "{rmse_med}/{rmse_max} | {handover} | {capture_med}/{capture_max} | "
            "{capture_distance} | {tracking}% | {fov_max}/{fov_p95} |".format(
                group=GROUP_LABELS.get(row["test_group"], row["test_group"]),
                profile=PROFILE_LABELS.get(
                    row["error_profile"], row["error_profile"]),
                passes=row["passes"], runs=row["runs"],
                rate=fmt(row["pass_rate_percent"], 1),
                rmse_med=fmt(row["radar_prediction_rmse_median_m"]),
                rmse_max=fmt(row["radar_prediction_rmse_worst_m"]),
                handover=fmt(row["handover_time_median_s"]),
                capture_med=fmt(row["capture_time_median_s"]),
                capture_max=fmt(row["capture_time_worst_s"]),
                capture_distance=fmt(row["capture_distance_worst_m"], 3),
                tracking=fmt(row["tracking_worst_percent"], 1),
                fov_max=fmt(row["fov_max_worst"], 3),
                fov_p95=fmt(row["fov_p95_worst"], 3)))

    evaluated = [row for row in tier_rows
                 if row["error_profile"] in TIER_REQUIREMENTS]
    acceptance_met = bool(evaluated) and all(
        row["pass_rate_percent"] >= TIER_REQUIREMENTS[row["error_profile"]]
        for row in evaluated)
    lines.extend([
        "",
        "主通过判据：成功判定首帧的两机三维距离不大于各工况的"
        " `capture_distance_m`（本矩阵为5 m）、完成雷达—视觉交接、交接后最大FOV占用<1.0、"
        "飞行阶段最低高度≥%.1f m，并且关键状态和控制量没有NaN/Inf/缺失。" %
        MINIMUM_SAFE_ALTITUDE_M,
        "跟踪率和连续短时丢帧单独报告，不作为本轮主通过判据，也不据此修改 `is_tracking` 逻辑。",
        "",
        "验收结论：%s" % (
            "已达到本批次中有定义的档级门槛。" if acceptance_met else
            "至少一个有定义的档级门槛尚未达到。"),
        "",
    ])
    with open(os.path.join(batch_dir, "client_summary.md"), "w",
              encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: summarize_full_chain_demo.py BATCH_DIRECTORY")
    batch_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
    records = load_records(batch_dir)
    if not records:
        raise SystemExit("No completed cases under " + batch_dir)
    rows = aggregate(records)
    tier_rows = aggregate_tiers(records)
    write_csv(batch_dir, rows)
    write_tier_csv(batch_dir, tier_rows)
    write_markdown(batch_dir, records, rows, tier_rows)
    print("Wrote client_summary.md, client_result_table.csv and "
          "client_tier_result_table.csv")


if __name__ == "__main__":
    main()
