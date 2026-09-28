#!/usr/bin/env python3
"""Run a CSV batch for the direct visual-interception scenario."""

import csv
import datetime
import os
import shutil
import subprocess
import sys


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cases_file = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else
                                 os.path.join(script_dir, "intercept_only_tuning_cases.csv"))
    batch_id = os.environ.get(
        "BATCH_ID", datetime.datetime.now().strftime("%Y%m%d_%H%M%S") +
        "_intercept_only")
    run_root = "/home/toast/fixedcamera_radar_multisim_results"
    batch_dir = os.path.join(run_root, batch_id)
    os.makedirs(batch_dir, exist_ok=True)
    shutil.copyfile(cases_file, os.path.join(batch_dir, "cases.csv"))

    with open(cases_file, newline="", encoding="utf-8-sig") as stream:
        cases = list(csv.DictReader(stream))
    for row in cases:
        case_name = row["CASE_NAME"].strip()
        if not case_name:
            continue
        env = os.environ.copy()
        env["BATCH_ID"] = batch_id
        env["SCENARIO_NAME"] = "情况_只拦截"
        env["SHOW_GUI"] = "0"
        env["RUN_WALL_TIMEOUT_S"] = env.get("RUN_WALL_TIMEOUT_S", "110")
        for key, value in row.items():
            if key and value is not None and value.strip() != "":
                env[key.strip()] = value.strip()
        log_path = os.path.join(batch_dir, case_name + "_launcher.log")
        print("=== Running %s ===" % case_name, flush=True)
        with open(log_path, "w", encoding="utf-8") as log:
            subprocess.run(
                [os.path.join(script_dir, "start_intercept_only_multisim.sh")],
                env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
        with open(log_path, encoding="utf-8", errors="replace") as log:
            tail = log.readlines()[-8:]
        sys.stdout.write("".join(tail))
        sys.stdout.flush()

    subprocess.run(
        [sys.executable, os.path.join(script_dir, "aggregate_intercept_only.py"),
         batch_dir], check=True)
    with open(os.path.join(run_root, "latest_intercept_only_batch.txt"),
              "w", encoding="utf-8") as stream:
        stream.write(batch_dir + "\n")
    print("Intercept-only batch complete: " + batch_dir)


if __name__ == "__main__":
    main()
