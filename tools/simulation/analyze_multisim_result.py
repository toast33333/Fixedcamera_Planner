#!/usr/bin/env python3
import csv
import math
import os
import statistics
import sys


def as_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_multisim_result.py RUN_DIRECTORY")
    run_dir = os.path.abspath(os.path.expanduser(sys.argv[1]))
    csv_path = os.path.join(run_dir, "experiment.csv")
    with open(csv_path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    chase = [row for row in rows if row["phase"] in ("CHASE", "SUCCESS_HOLD")]
    chase_only = [row for row in rows if row["phase"] == "CHASE"]
    if not chase_only:
        raise SystemExit("No CHASE samples found in " + csv_path)

    distances = [as_float(row, "distance_m") for row in chase]
    distances = [value for value in distances if math.isfinite(value)]
    tracking_values = [int(row["tracking"]) for row in chase_only]
    fov_ratios = [max(as_float(row, "horizontal_fov_ratio"),
                      as_float(row, "vertical_fov_ratio")) for row in chase_only]
    fov_ratios = [value for value in fov_ratios if math.isfinite(value)]
    start_time = as_float(chase_only[0], "ros_time_s")
    end_time = as_float(chase_only[-1], "ros_time_s")
    duration = max(0.0, end_time - start_time)
    success = any(row["phase"] == "SUCCESS_HOLD" for row in rows)

    lines = [
        "PX4 fixed-camera multisim summary",
        "result: " + ("SUCCESS (capture radius reached)" if success else "NO SUCCESS FLAG"),
        "chase duration: %.2f s" % duration,
        "initial distance: %.2f m" % as_float(chase_only[0], "distance_m"),
        "minimum distance: %.2f m" % min(distances),
        "tracking availability during chase: %.1f %%" %
        (100.0 * statistics.mean(tracking_values)),
        "maximum normalized FOV usage: %.3f" % max(fov_ratios),
        "target commanded speed: 2.0 m/s",
        "interceptor nominal IBVS forward speed: 4.5 m/s",
    ]
    summary_path = os.path.join(run_dir, "summary.txt")
    with open(summary_path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    print("\n".join(lines))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times = [as_float(row, "ros_time_s") - start_time for row in chase_only]
        chase_distances = [as_float(row, "distance_m") for row in chase_only]
        u_offsets = [as_float(row, "pixel_u_offset") for row in chase_only]
        v_offsets = [as_float(row, "pixel_v_offset") for row in chase_only]
        ratios = [max(as_float(row, "horizontal_fov_ratio"),
                      as_float(row, "vertical_fov_ratio")) for row in chase_only]

        figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        axes[0].plot(times, chase_distances, color="black", linewidth=1.8)
        axes[0].axhline(7.5, color="gray", linestyle="--", label="capture radius")
        axes[0].set_ylabel("distance (m)")
        axes[0].legend(loc="best")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(times, u_offsets, label="u offset")
        axes[1].plot(times, v_offsets, label="v offset")
        axes[1].set_ylabel("pixel offset")
        axes[1].legend(loc="best")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(times, ratios, color="black")
        axes[2].axhline(1.0, color="red", linestyle="--", label="FOV boundary")
        axes[2].set_ylabel("FOV ratio")
        axes[2].set_xlabel("chase time (s)")
        axes[2].legend(loc="best")
        axes[2].grid(True, alpha=0.3)
        figure.tight_layout()
        figure.savefig(os.path.join(run_dir, "result_plot.png"), dpi=150)
    except Exception as exc:
        print("Plot generation skipped: " + str(exc))


if __name__ == "__main__":
    main()
