#!/usr/bin/env python3
"""Summarize phase timing from completed PX4/Gazebo experiment logs.

This utility is deliberately read-only with respect to controller behavior.  It
uses the recorded state transitions and distance samples to separate startup,
approach, handover, and post-handover time.
"""

import argparse
import csv
import json
import math
import os
import statistics


PRE_CHASE_PHASES = (
    "WAITING_FOR_LINKS",
    "SETPOINT_WARMUP",
    "TAKEOFF",
    "STABILIZE",
    "RADAR_ACQUIRE",
)


def number(row, key):
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")


def first_time(rows, predicate, not_before=-math.inf):
    for row in rows:
        now = number(row, "ros_time_s")
        if math.isfinite(now) and now >= not_before and predicate(row):
            return now
    return float("nan")


def elapsed(end, start):
    if not math.isfinite(end) or not math.isfinite(start):
        return float("nan")
    return end - start


def phase_durations(rows):
    """Integrate sample-to-sample time by the phase owning each interval."""
    durations = {}
    for current, following in zip(rows, rows[1:]):
        start = number(current, "ros_time_s")
        stop = number(following, "ros_time_s")
        if not math.isfinite(start) or not math.isfinite(stop) or stop < start:
            continue
        phase = current.get("phase", "")
        durations[phase] = durations.get(phase, 0.0) + stop - start
    return durations


def truthy(row, key):
    try:
        return bool(int(float(row.get(key, 0))))
    except (TypeError, ValueError):
        return False


def analyze_rows(rows, params=None, case_name=""):
    params = params or {}
    durations = phase_durations(rows)
    log_start = first_time(rows, lambda unused: True)
    chase_start = first_time(rows, lambda row: row.get("phase") == "CHASE")
    detection_range = float(params.get("detection_range_m", 30.0))
    visual_range_time = first_time(
        rows,
        lambda row: number(row, "distance_m") <= detection_range,
        chase_start,
    )
    first_tracking = first_time(
        rows, lambda row: truthy(row, "tracking"), chase_start)
    handover = first_time(
        rows, lambda row: truthy(row, "handover_complete"), chase_start)
    approach_only = str(params.get("stop_on_handover", "false")).lower() in (
        "1", "true", "yes")
    success_phase = "HANDOVER_HOLD" if approach_only else "SUCCESS_HOLD"
    success = first_time(
        rows, lambda row: row.get("phase") == success_phase, chase_start)

    result = {
        "case_name": params.get("case_name", case_name),
        "error_profile": params.get("error_profile", "unknown"),
        "test_group": params.get("test_group", "unknown"),
        "success": math.isfinite(success),
        "detection_range_m": detection_range,
        "log_start_to_chase_s": elapsed(chase_start, log_start),
        "chase_to_detection_range_s": elapsed(visual_range_time, chase_start),
        "detection_range_to_first_tracking_s": elapsed(
            first_tracking, visual_range_time),
        "first_tracking_to_handover_s": elapsed(handover, first_tracking),
        "detection_range_to_handover_s": elapsed(handover, visual_range_time),
        "handover_to_success_s": elapsed(success, handover),
        "chase_to_success_s": elapsed(success, chase_start),
    }
    for phase in PRE_CHASE_PHASES:
        result[phase.lower() + "_s"] = durations.get(phase, 0.0)
    return result


def load_case(case_dir):
    csv_path = os.path.join(case_dir, "experiment.csv")
    with open(csv_path, newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    params = {}
    params_path = os.path.join(case_dir, "case_params.json")
    if os.path.exists(params_path):
        with open(params_path, encoding="utf-8") as stream:
            params = json.load(stream)
    return analyze_rows(rows, params, os.path.basename(case_dir))


def finite_values(records, key):
    return [float(record[key]) for record in records
            if isinstance(record.get(key), (int, float)) and
            math.isfinite(float(record[key]))]


def format_seconds(value):
    return "--" if not math.isfinite(value) else "%.2f" % value


def write_csv(path, records):
    if not records:
        return
    with open(path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        for record in records:
            writer.writerow({
                key: "" if isinstance(value, float) and math.isnan(value)
                else value
                for key, value in record.items()
            })


def write_markdown(path, records):
    metrics = (
        ("log_start_to_chase_s", "日志开始至进入 CHASE"),
        ("radar_acquire_s", "雷达初始化"),
        ("chase_to_detection_range_s", "CHASE 至进入相机范围"),
        ("detection_range_to_handover_s", "进入相机范围至交接"),
        ("handover_to_success_s", "交接至成功"),
        ("chase_to_success_s", "CHASE 至成功"),
    )
    profiles = sorted({record["error_profile"] for record in records})
    lines = [
        "# 仿真实验阶段耗时",
        "",
        "全部数值由已有实验日志计算；本工具不修改控制器或交接参数。",
        "",
        "| 档位 | 指标 | 有效样本 | 中位数（秒） | 最小值（秒） | 最大值（秒） |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for profile in profiles:
        selected = [record for record in records
                    if record["error_profile"] == profile]
        for key, label in metrics:
            values = finite_values(selected, key)
            lines.append(
                "| %s | %s | %d | %s | %s | %s |" % (
                    profile, label, len(values),
                    format_seconds(statistics.median(values)) if values else "--",
                    format_seconds(min(values)) if values else "--",
                    format_seconds(max(values)) if values else "--"))
    lines.extend((
        "",
        "“相机范围”使用每个工况保存的 `detection_range_m`；未保存该参数时按30米计算。"
        "未发生的事件在CSV中留空，并从对应的汇总统计中排除。",
        "",
    ))
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("batch_directory")
    parser.add_argument("--csv-name", default="phase_timing.csv")
    parser.add_argument("--report-name", default="phase_timing.md")
    args = parser.parse_args()

    batch_dir = os.path.abspath(os.path.expanduser(args.batch_directory))
    case_dirs = sorted(
        entry.path for entry in os.scandir(batch_dir)
        if entry.is_dir() and os.path.exists(
            os.path.join(entry.path, "experiment.csv")))
    records = [load_case(case_dir) for case_dir in case_dirs]
    if not records:
        raise SystemExit("No case directories with experiment.csv found")
    write_csv(os.path.join(batch_dir, args.csv_name), records)
    write_markdown(os.path.join(batch_dir, args.report_name), records)
    print("Analyzed %d cases" % len(records))


if __name__ == "__main__":
    main()
