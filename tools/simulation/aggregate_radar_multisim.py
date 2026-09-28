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
]


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
            writer.writerow({key: record.get(key) for key in SUMMARY_FIELDS})


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
        "| 工况 | 预期 | 结果 | 交接 | 最小距离 m | 交接时间 s | 雷达预测 RMSE m | 视觉可用率 | 最大 FOV 比 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for record in records:
        lines.append(
            "| {name} | {expected} | {result} | {handover} | {distance} | {handover_time} | "
            "{radar_rmse} | {tracking}% | {fov} |".format(
                name=record["case_name"],
                expected=record.get("expected_result", "—"),
                result="成功" if record.get("success") else "失败",
                handover="是" if record.get("handover_occurred") else "否",
                distance=fmt(record.get("minimum_distance_m")),
                handover_time=fmt(record.get("handover_time_from_chase_s")),
                radar_rmse=fmt(record.get("radar_prediction_rmse_m")),
                tracking=fmt(record.get("post_handover_tracking_availability_percent"), 1),
                fov=fmt(record.get("post_handover_max_fov_ratio"), 3),
            ))

    lines.extend(["", "## 逐工况判读", ""])
    for record in records:
        params = record.get("params", {})
        lines.extend([
            "### %s — %s" % (
                record["case_name"], "成功" if record.get("success") else "失败"),
            "",
            "- 条件：靶机 %.1f m/s，航向 %.1f°，转弯率 %.1f°/s，爬升率 %.1f m/s；"
            "雷达周期 %.1f s，位置噪声标准差 %.1f m，断更时长 %.1f s。" % (
                params.get("target_speed_mps", float("nan")),
                params.get("target_heading_deg", float("nan")),
                params.get("target_turn_rate_degps", float("nan")),
                params.get("target_climb_rate_mps", float("nan")),
                params.get("radar_period_sec", float("nan")),
                params.get("radar_noise_std_m", float("nan")),
                params.get("radar_dropout_duration_sec", float("nan"))),
            "- 结果：最小距离 %s m，末端位置误差最小值 %s m，雷达预测 RMSE %s m，"
            "视觉跟踪可用率 %s%%。" % (
                fmt(record.get("minimum_distance_m")),
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
        "- 雷达输入是 Gazebo 真值按设定周期采样后叠加白噪声，尚未加入系统偏差、时间戳乱序和坐标标定误差。",
        "- 视觉检测采用几何投影，在距离和视场内即认为可检测；没有验证真实检测器在模糊、遮挡和光照变化下的性能。",
        "- 靶机由速度指令驱动，机体动力学仍由 PX4 SITL 与 Gazebo 执行；未模拟风场和通信网络拥塞。",
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
        minimum_range = [record.get("minimum_distance_m", float("nan")) for record in records]
        radar_rmse = [record.get("radar_prediction_rmse_m", float("nan")) for record in records]
        tracking = [record.get("post_handover_tracking_availability_percent", float("nan")) for record in records]

        figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        axes[0].bar(x, minimum_range, color=colors)
        axes[0].axhline(5.0, color="black", linestyle="--", label="capture threshold")
        axes[0].set_ylabel("minimum range (m)")
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
