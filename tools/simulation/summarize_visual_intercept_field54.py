#!/usr/bin/env python3
"""Create the compact user-facing table for the 54-run visual-only batch."""

import csv
import json
import math
import os
import sys


COLUMNS = [
    "序号",
    "靶机速度(m/s)",
    "测试机默认前飞速度(m/s)",
    "起始水平相对距离(m)",
    "起始竖直相对距离(m)",
    "是否成功撞击",
    "撞击所用时间(s)",
]


def number(value, digits=1):
    value = float(value)
    return f"{value:.{digits}f}"


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: summarize_visual_intercept_field54.py BATCH_DIR OUTPUT_CSV")
    batch_dir = os.path.abspath(sys.argv[1])
    output_csv = os.path.abspath(sys.argv[2])

    with open(os.path.join(batch_dir, "cases.csv"), encoding="utf-8-sig",
              newline="") as stream:
        cases = list(csv.DictReader(stream))

    output_rows = []
    failures = []
    for index, case in enumerate(cases, 1):
        case_name = case["CASE_NAME"]
        summary_path = os.path.join(batch_dir, case_name, "summary.json")
        with open(summary_path, encoding="utf-8") as stream:
            summary = json.load(stream)

        success = bool(summary.get("success"))
        capture_time = summary.get("capture_time_from_chase_s")
        if success and isinstance(capture_time, (int, float)) and math.isfinite(capture_time):
            capture_text = number(capture_time, 2)
        else:
            capture_text = "—"

        vertical_distance = (
            float(case["TARGET_ALTITUDE_M"]) -
            float(case["INTERCEPTOR_ALTITUDE_M"]))
        output_rows.append({
            "序号": index,
            "靶机速度(m/s)": number(case["TARGET_SPEED_MPS"]),
            "测试机默认前飞速度(m/s)": number(
                case["VISUAL_FORWARD_SPEED_MPS"]),
            "起始水平相对距离(m)": number(case["TARGET_SPAWN_X"]),
            "起始竖直相对距离(m)": number(vertical_distance),
            "是否成功撞击": "成功" if success else "失败",
            "撞击所用时间(s)": capture_text,
        })
        if not success:
            failures.append((case_name, summary))

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    with open(output_csv, "w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(output_rows)

    success_count = len(output_rows) - len(failures)
    success_rate = 100.0 * success_count / len(output_rows)
    print(
        f"completed={len(output_rows)} success={success_count} "
        f"success_rate={success_rate:.2f}%")

    # The user requested a failure-reason TXT only when the overall result is
    # not above 90%.  Do not create one for a passing batch.
    if success_rate <= 90.0:
        failure_path = os.path.splitext(output_csv)[0] + "_失败原因.txt"
        with open(failure_path, "w", encoding="utf-8") as stream:
            stream.write(f"成功率：{success_rate:.2f}%\n\n")
            for case_name, summary in failures:
                stream.write(
                    f"{case_name}: {summary.get('reason', '未记录原因')}\n")
        print("failure report: " + failure_path)


if __name__ == "__main__":
    main()
