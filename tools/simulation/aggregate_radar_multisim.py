#!/usr/bin/env python3
"""Build a comparison table and Chinese report for a radar multisim batch."""

import csv
import datetime as dt
import json
import math
import os
import sys


SUMMARY_FIELDS = [
    "case_name", "expected_result", "success", "handover_occurred",
    "initial_chase_distance_m", "minimum_distance_m", "final_distance_m",
    "chase_duration_s", "handover_time_from_chase_s",
    "minimum_terminal_error_m", "radar_prediction_rmse_m",
    "radar_prediction_max_error_m", "tracking_availability_percent",
    "max_fov_ratio", "post_handover_tracking_availability_percent",
    "post_handover_max_fov_ratio", "radar_timeout_fraction",
    "mean_target_horizontal_speed_mps",
    "mean_interceptor_horizontal_speed_mps",
    "maximum_interceptor_horizontal_speed_mps", "radar_samples", "reason",
    "full_chain_pass", "post_handover_p95_fov_ratio",
    "max_continuous_tracking_loss_s", "capture_time_from_chase_s",
    "capture_distance_at_success_m",
    "post_handover_max_continuous_tracking_loss_s",
    "radar_measurement_error_rms_m", "radar_bias_rms_m",
    "radar_delay_mean_s", "radar_delay_max_s", "radar_dropped_count",
    "visual_bias_rms_px", "visual_delay_mean_s", "visual_delay_p95_s",
    "visual_dropped_count", "guidance_delay_mean_s",
    "guidance_delay_p95_s", "guidance_dropped_count",
    "radar_noise_std_m", "radar_period_sec", "radar_timeout_sec",
    "radar_bootstrap_samples", "filter_alpha", "filter_beta",
    "max_radar_innovation_m", "max_target_speed_mps",
    "max_target_horizontal_speed_mps", "max_target_vertical_speed_mps",
    "radar_measurement_std_m", "radar_process_accel_std_mps2",
    "max_handover_opening_speed_mps",
]


RADAR_CONFIG_FIELDS = {
    "radar_noise_std_m", "radar_period_sec", "radar_timeout_sec",
    "radar_bootstrap_samples", "filter_alpha", "filter_beta",
    "max_radar_innovation_m", "max_target_speed_mps",
    "max_target_horizontal_speed_mps", "max_target_vertical_speed_mps",
    "radar_measurement_std_m", "radar_process_accel_std_mps2",
    "max_handover_opening_speed_mps",
}


def setting(record, key, default=None):
    """Read a setting from case_params, falling back to old summary files."""
    params = record.get("params") or {}
    value = params.get(key)
    if value is None or value == "":
        value = record.get(key, default)
    return default if value is None or value == "" else value


def fmt(value, digits=2):
    if isinstance(value, bool):
        return "是" if value else "否"
    try:
        value = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(value):
        return "—"
    return ("%." + str(digits) + "f") % value


def fmt_setting(record, key, digits=2, default="未记录"):
    """Format an optional recorded setting without leaking ``None`` to reports."""
    value = setting(record, key)
    if value is None or value == "":
        return default
    return fmt(value, digits)


def load_batch(batch_dir):
    records = []
    for name in sorted(os.listdir(batch_dir)):
        case_dir = os.path.join(batch_dir, name)
        summary_path = os.path.join(case_dir, "summary.json")
        if not os.path.isfile(summary_path):
            continue
        with open(summary_path, encoding="utf-8") as stream:
            summary = json.load(stream)
        params_path = os.path.join(case_dir, "case_params.json")
        params = {}
        if os.path.isfile(params_path):
            with open(params_path, encoding="utf-8") as stream:
                params = json.load(stream)
        summary["params"] = params
        summary["case_dir"] = name
        records.append(summary)
    return records


def write_csv_summary(batch_dir, records):
    path = os.path.join(batch_dir, "batch_summary.csv")
    with open(path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for record in records:
            writer.writerow({
                key: (setting(record, key) if key in RADAR_CONFIG_FIELDS else
                      record.get(key))
                for key in SUMMARY_FIELDS
            })


def build_report(batch_dir, records):
    successes = sum(1 for record in records if record.get("success"))
    handed_over = sum(1 for record in records if record.get("handover_occurred"))
    generated = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "# PX4/Gazebo 雷达抵近—固定相机拦截多工况仿真报告",
        "",
        "生成时间：%s" % generated,
        "",
        "## 结论摘要",
        "",
        "本批次共运行 **%d** 个双 PX4/Gazebo 工况，其中 **%d** 个进入 5 m 拦截判定半径，"
        "**%d** 个完成雷达抵近到固定相机 IBVS 的控制权交接。" %
        (len(records), successes, handed_over),
        "",
        "这里的“拦截成功”定义为两机三维距离不大于 5 m；Gazebo 中未建模碰撞毁伤。"
        "视觉输入由真实相对位姿投影生成，但消息类型、话题和控制器入口保持为"
        " `/object_kcf` + `kcf_msgs/Bbox`，因此本报告验证的是制导与控制链路，"
        "不是目标检测算法本身的识别率。",
        "",
        "## 工况与结果",
        "",
        "| 工况 | 预期 | 结果 | 交接 | 成功判定距离 m | CHASE最小距离 m | 交接时间 s | 雷达预测 RMSE m | 视觉可用率 | 最大 FOV 比 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for record in records:
        lines.append(
            "| {name} | {expected} | {result} | {handover} | {capture_distance} | {distance} | {handover_time} | "
            "{radar_rmse} | {tracking}% | {fov} |".format(
                name=record["case_name"],
                expected=record.get("expected_result", "—"),
                result="成功" if record.get("success") else "失败",
                handover="是" if record.get("handover_occurred") else "否",
                capture_distance=fmt(record.get("capture_distance_at_success_m")),
                distance=fmt(record.get("minimum_distance_m")),
                handover_time=fmt(record.get("handover_time_from_chase_s")),
                radar_rmse=fmt(record.get("radar_prediction_rmse_m")),
                tracking=fmt(record.get("post_handover_tracking_availability_percent"), 1),
                fov=fmt(record.get("post_handover_max_fov_ratio"), 3),
            ))

    lines.extend(["", "## 逐工况判读", ""])
    for record in records:
        radar_noise_std = setting(record, "radar_noise_std_m")
        try:
            radar_noise_3d_rms = math.sqrt(3.0) * float(radar_noise_std)
        except (TypeError, ValueError):
            radar_noise_3d_rms = None
        if setting(record, "radar_measurement_std_m") is not None:
            filter_description = "CV Kalman滤波，测量标准差 %s m，过程加速度标准差 %s m/s²" % (
                fmt_setting(record, "radar_measurement_std_m"),
                fmt_setting(record, "radar_process_accel_std_mps2"))
        else:
            filter_description = "α-β滤波 α/β=%s/%s" % (
                fmt_setting(record, "filter_alpha"),
                fmt_setting(record, "filter_beta"))
        lines.extend([
            "### %s — %s" % (
                record["case_name"], "成功" if record.get("success") else "失败"),
            "",
            "- 条件：靶机 %s m/s，航向 %s°，转弯率 %s°/s，爬升率 %s m/s；"
            "雷达周期 %s s，位置白噪声每轴σ=%s m（等效三维RMS≈%s m），"
            "断更时长 %s s。" % (
                fmt_setting(record, "target_speed_mps", 1),
                fmt_setting(record, "target_heading_deg", 1),
                fmt_setting(record, "target_turn_rate_degps", 1),
                fmt_setting(record, "target_climb_rate_mps", 1),
                fmt_setting(record, "radar_period_sec"),
                fmt_setting(record, "radar_noise_std_m"),
                (fmt(radar_noise_3d_rms) if radar_noise_3d_rms is not None
                 else "未记录"),
                fmt_setting(record, "radar_dropout_duration_sec")),
            "- 雷达鲁棒参数：初始化样本数 %s，%s，"
            "创新保护阈值 %s m，目标速度上限 %s m/s。" % (
                fmt_setting(record, "radar_bootstrap_samples", 0),
                filter_description,
                fmt_setting(record, "max_radar_innovation_m", 1),
                fmt_setting(record, "max_target_speed_mps", 1)),
            "- 结果：CHASE阶段最小距离 %s m，成功判定首帧距离 %s m，"
            "末端位置误差最小值 %s m，雷达预测 RMSE %s m，"
            "视觉跟踪可用率 %s%%。" % (
                fmt(record.get("minimum_distance_m")),
                fmt(record.get("capture_distance_at_success_m")),
                fmt(record.get("minimum_terminal_error_m")),
                fmt(record.get("radar_prediction_rmse_m")),
                fmt(record.get("post_handover_tracking_availability_percent"), 1)),
            "- 判读：%s。" % record.get("reason", "未生成判读"),
            "- [结果曲线](%s/result_plot.png)" % record["case_dir"],
            "",
        ])

    failures = [record for record in records if not record.get("success")]
    lines.extend(["## 失败边界归纳", ""])
    if failures:
        for record in failures:
            lines.append("- **%s**：%s。" % (record["case_name"], record["reason"]))
    else:
        lines.append("本批次没有失败工况；需要继续提高靶机机动强度或传感器中断时间来确定边界。")

    lines.extend([
        "",
        "## 适用范围与限制",
        "",
        "- 雷达输入由 Gazebo 真值生成，可配置白噪声、有界随机游走偏置、采样周期抖动、传输延迟和随机丢包；尚未覆盖坐标标定误差与消息乱序。",
        "- 视觉检测采用几何投影，在距离和视场内即认为可检测；没有验证真实检测器在模糊、遮挡和光照变化下的性能。",
        "- 靶机由速度指令驱动，机体动力学由 PX4 SITL 与 Gazebo 执行；展示批次可配置物理风、等效跟踪残差和命令传输延迟，但不等同于真实网络拥塞模型。",
        "- 成功半径为 5 m，结果用于证明代码链路和控制思路可闭合，不等同于实飞安全结论。",
        "",
        "## 可复现性",
        "",
        "每个工况目录保存参数 JSON、原始 CSV、ROS bag、节点日志、单工况摘要和曲线。"
        "批次输入为 `cases.csv`，汇总数据为 `batch_summary.csv`。",
        "",
    ])
    with open(os.path.join(batch_dir, "report.md"), "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def plot_comparison(batch_dir, records):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np

        names = [record["case_name"] for record in records]
        x = np.arange(len(names))
        colors = ["#2a9d8f" if record.get("success") else "#e76f51" for record in records]
        # The phase is switched to SUCCESS_HOLD before the threshold-crossing
        # row is written, so minimum_distance_m intentionally describes CHASE
        # only.  Plot the decision-frame distance for successes to avoid bars
        # above the capture threshold being mistaken for contradictory results.
        decision_range = [
            (record.get("capture_distance_at_success_m", float("nan"))
             if record.get("success") else
             record.get("minimum_distance_m", float("nan")))
            for record in records
        ]
        radar_rmse = [record.get("radar_prediction_rmse_m", float("nan")) for record in records]
        tracking = [record.get("post_handover_tracking_availability_percent", float("nan")) for record in records]

        figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].bar(x, decision_range, color=colors)
        axes[0].axhline(5.0, color="black", linestyle="--", label="capture threshold")
        axes[0].set_ylabel("success decision / closest CHASE range (m)")
        axes[0].legend(loc="best")
        axes[1].bar(x, radar_rmse, color="#457b9d")
        axes[1].set_ylabel("radar prediction RMSE (m)")
        axes[2].bar(x, tracking, color="#6a4c93")
        axes[2].set_ylabel("visual availability (%)")
        axes[2].set_xticks(x)
        axes[2].set_xticklabels(names, rotation=35, ha="right")
        for axis in axes:
            axis.grid(True, axis="y", alpha=0.3)
        figure.tight_layout()
        figure.savefig(os.path.join(batch_dir, "batch_comparison.png"), dpi=160)
        plt.close(figure)
    except Exception as exc:
        print("Comparison plot skipped: " + str(exc))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: aggregate_radar_multisim.py BATCH_DIRECTORY")
    batch_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
    records = load_batch(batch_dir)
    if not records:
        raise SystemExit("No summary.json files found under " + batch_dir)
    write_csv_summary(batch_dir, records)
    build_report(batch_dir, records)
    plot_comparison(batch_dir, records)
    print("Wrote report for %d cases to %s" % (len(records), batch_dir))


if __name__ == "__main__":
    main()
