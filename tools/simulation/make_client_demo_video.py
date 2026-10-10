#!/usr/bin/env python3
"""Build a client-facing full-chain demonstration video from one simulation run.

The source recording is synchronized with ``experiment.csv`` using sampled
wall-to-simulation timestamps in ``video_clock.csv`` when available, with the
single ``video_start_ros_time_s.txt`` timestamp retained for archived videos.
The clock map accounts for simulation speed changes. Recorder startup latency can be calibrated
with ``--sync-offset-sec``.  The resulting 1920x1080 composition keeps the
Gazebo view prominent and adds only telemetry that can be traced back to the
run artefacts.

Typical use::

    python3 tools/simulation/make_client_demo_video.py RUN_DIR client_demo.mp4

OpenCV's broadly available ``mp4v`` encoder is the default.  Select another
four-character codec with ``--codec`` when the target OpenCV build supports it.
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover - depends on the runtime image
    raise SystemExit("OpenCV and NumPy are required: %s" % exc)


BG = (20, 25, 34)
PANEL = (30, 37, 49)
PANEL_2 = (38, 46, 60)
WHITE = (238, 243, 248)
MUTED = (155, 167, 181)
CYAN = (231, 188, 54)
BLUE = (230, 139, 65)
GREEN = (115, 207, 91)
AMBER = (55, 180, 245)
RED = (91, 91, 239)
GRID = (66, 75, 89)


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def number(row, key, default=float("nan")):
    try:
        value = float(row.get(key, default))
    except (AttributeError, TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def integer(row, key, default=0):
    value = number(row, key, float(default))
    return int(round(value)) if finite(value) else default


def load_json(path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def find_input(run_dir, explicit, candidates, suffix=None):
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_absolute():
            path = run_dir / path
        if not path.is_file():
            raise FileNotFoundError(str(path))
        return path
    for name in candidates:
        path = run_dir / name
        if path.is_file():
            return path
    if suffix:
        matches = sorted(run_dir.glob("*" + suffix))
        if matches:
            return matches[0]
    raise FileNotFoundError(
        "none of %s exists in %s" % (", ".join(candidates), run_dir))


def load_rows(csv_path):
    rows = []
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        for raw in csv.DictReader(stream):
            timestamp = number(raw, "ros_time_s")
            if finite(timestamp):
                raw["_time"] = timestamp
                rows.append(raw)
    rows.sort(key=lambda row: row["_time"])
    if not rows:
        raise ValueError("telemetry CSV has no finite ros_time_s rows: %s" % csv_path)
    return rows


def read_epoch(path):
    with path.open("r", encoding="utf-8") as stream:
        value = float(stream.read().strip())
    if not math.isfinite(value):
        raise ValueError("non-finite video start epoch in %s" % path)
    return value


def resolve_video_start_time(run_dir, explicit, ros_time_file, epoch_file):
    """Resolve frame-zero time in the same ROS/simulation clock as the CSV.

    ``video_start_epoch_s.txt`` is retained by the launcher as useful forensic
    wall-clock metadata, but Unix epoch seconds cannot be compared with Gazebo
    simulation time.  Requiring a ROS timestamp (or an explicit override)
    avoids silently producing a zero-frame video with mismatched clocks.
    """
    if explicit is not None:
        start_time = float(explicit)
        if not math.isfinite(start_time):
            raise ValueError("--video-start-ros-time must be finite")
        return start_time

    ros_time_path = Path(ros_time_file).expanduser()
    if not ros_time_path.is_absolute():
        ros_time_path = run_dir / ros_time_path
    if ros_time_path.is_file():
        return read_epoch(ros_time_path)

    epoch_path = Path(epoch_file).expanduser()
    if not epoch_path.is_absolute():
        epoch_path = run_dir / epoch_path
    if epoch_path.is_file():
        raise ValueError(
            "%s is Unix wall-clock time and cannot synchronize ROS simulation "
            "telemetry; provide %s or --video-start-ros-time" %
            (epoch_path, ros_time_path))
    raise FileNotFoundError(
        "missing ROS video timestamp %s; provide --video-start-ros-time" %
        ros_time_path)


def load_video_clock(run_dir, epoch_file):
    """Return frame-zero wall epoch and sampled wall-to-simulation mapping."""
    path = run_dir / "video_clock.csv"
    if not path.is_file():
        return None
    wall, simulation = [], []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            wall_time = number(row, "wall_time_s")
            sim_time = number(row, "ros_time_s")
            if not finite(wall_time) or not finite(sim_time):
                continue
            if wall and (wall_time <= wall[-1] or sim_time < simulation[-1]):
                raise ValueError("video_clock.csv contains a clock reversal")
            wall.append(wall_time)
            simulation.append(sim_time)
    if len(wall) < 2:
        raise ValueError("video_clock.csv has fewer than two usable samples")
    epoch_path = Path(epoch_file)
    if not epoch_path.is_absolute():
        epoch_path = run_dir / epoch_path
    return read_epoch(epoch_path), wall, simulation


def simulation_time_at_wall(timestamp, wall, simulation):
    index = bisect.bisect_left(wall, timestamp)
    if index <= 0:
        return simulation[0]
    if index >= len(wall):
        return simulation[-1]
    fraction = (timestamp - wall[index - 1]) / (wall[index] - wall[index - 1])
    return simulation[index - 1] + fraction * (
        simulation[index] - simulation[index - 1])


def text(frame, value, point, scale=0.55, color=WHITE, thickness=1,
         font=cv2.FONT_HERSHEY_SIMPLEX):
    cv2.putText(frame, str(value), point, font, scale, color, thickness,
                cv2.LINE_AA)


def right_text(frame, value, right_x, y, scale=0.55, color=WHITE,
               thickness=1):
    size = cv2.getTextSize(str(value), cv2.FONT_HERSHEY_SIMPLEX, scale,
                           thickness)[0]
    text(frame, value, (right_x - size[0], y), scale, color, thickness)


def rounded_rect(frame, p1, p2, color, radius=12, thickness=-1):
    # Filled rounded rectangles are used throughout; retain a simple outline
    # fallback for callers that request one.
    x1, y1 = p1
    x2, y2 = p2
    radius = max(1, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))
    if thickness >= 0:
        cv2.rectangle(frame, p1, p2, color, thickness, cv2.LINE_AA)
        return
    cv2.rectangle(frame, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(frame, (x1, y1 + radius), (x2, y2 - radius), color, -1)
    cv2.circle(frame, (x1 + radius, y1 + radius), radius, color, -1,
               cv2.LINE_AA)
    cv2.circle(frame, (x2 - radius, y1 + radius), radius, color, -1,
               cv2.LINE_AA)
    cv2.circle(frame, (x1 + radius, y2 - radius), radius, color, -1,
               cv2.LINE_AA)
    cv2.circle(frame, (x2 - radius, y2 - radius), radius, color, -1,
               cv2.LINE_AA)


def fit_image(image, width, height):
    if image is None or image.size == 0:
        return np.full((height, width, 3), (8, 10, 14), dtype=np.uint8)
    source_h, source_w = image.shape[:2]
    scale = min(width / float(source_w), height / float(source_h))
    resized_w = max(1, int(round(source_w * scale)))
    resized_h = max(1, int(round(source_h * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=interpolation)
    output = np.full((height, width, 3), (8, 10, 14), dtype=np.uint8)
    x = (width - resized_w) // 2
    y = (height - resized_h) // 2
    output[y:y + resized_h, x:x + resized_w] = resized
    return output


def status_color(active, failure=False):
    if failure:
        return RED
    return GREEN if active else GRID


def phase_label(row):
    phase = str(row.get("phase", "UNKNOWN")).strip().upper()
    handover = integer(row, "handover_complete") > 0
    labels = {
        "WAITING_FOR_LINKS": "Link initialization",
        "SETPOINT_WARMUP": "Offboard warm-up",
        "TAKEOFF": "Dual-UAV takeoff",
        "STABILIZE": "Formation stabilization",
        "RADAR_ACQUIRE": "Radar acquisition",
        "HANDOVER_HOLD": "Visual handover verified",
        "SUCCESS_HOLD": "Successful intercept",
        "TIMEOUT_HOLD": "Intercept timeout",
    }
    if phase == "CHASE":
        return "Visual PN interception" if handover else "Radar-guided closing"
    return labels.get(phase, phase.replace("_", " ").title())


def draw_header(canvas, row, summary, params, elapsed):
    text(canvas, "RADAR-TO-VISUAL INTERCEPTION", (24, 35), 0.78, WHITE, 2)
    text(canvas, "PX4/Gazebo dual-UAV software-in-loop", (24, 61), 0.47,
         MUTED, 1)
    case = summary.get("case_name") or params.get("case_name") or "simulation run"
    right_text(canvas, str(case), 1892, 33, 0.55, WHITE, 1)
    right_text(canvas, "T+%05.1f s" % max(0.0, elapsed), 1892, 60, 0.50,
               CYAN, 1)
    cv2.line(canvas, (20, 72), (1900, 72), GRID, 1, cv2.LINE_AA)


def crop_gazebo_ui(source, target_aspect):
    """Remove desktop/Gazebo chrome, then center-crop to the panel aspect."""
    source_h, source_w = source.shape[:2]
    top = min(int(round(source_h * 0.075)), max(source_h - 2, 0))
    bottom = max(top + 1, source_h - int(round(source_h * 0.040)))
    cropped = source[top:bottom, :]
    crop_h, crop_w = cropped.shape[:2]
    wanted_w = int(round(crop_h * target_aspect))
    if 0 < wanted_w < crop_w:
        left = (crop_w - wanted_w) // 2
        cropped = cropped[:, left:left + wanted_w]
    return cropped


def draw_main_view(canvas, source):
    x, y, w, h = 20, 84, 1320, 742
    rounded_rect(canvas, (x, y), (x + w, y + h), PANEL, 12)
    source = crop_gazebo_ui(source, (w - 8) / float(h - 8))
    view = fit_image(source, w - 8, h - 8)
    canvas[y + 4:y + h - 4, x + 4:x + w - 4] = view
    rounded_rect(canvas, (x + 16, y + 14), (x + 232, y + 47),
                 (12, 17, 24), 8)
    text(canvas, "GAZEBO WORLD VIEW", (x + 29, y + 37), 0.48, WHITE, 1)
    cv2.circle(canvas, (x + w - 286, y + 31), 7, BLUE, -1, cv2.LINE_AA)
    text(canvas, "INTERCEPTOR", (x + w - 273, y + 37), 0.36, WHITE, 1)
    cv2.circle(canvas, (x + w - 143, y + 31), 7, RED, -1, cv2.LINE_AA)
    text(canvas, "TARGET", (x + w - 130, y + 37), 0.36, WHITE, 1)


def draw_progress(canvas, row, x, y, width):
    phase = str(row.get("phase", "")).upper()
    radar_reached = phase in ("RADAR_ACQUIRE", "CHASE", "HANDOVER_HOLD",
                              "SUCCESS_HOLD", "TIMEOUT_HOLD")
    handover = integer(row, "handover_complete") > 0 or phase in (
        "HANDOVER_HOLD", "SUCCESS_HOLD")
    intercepted = phase == "SUCCESS_HOLD"
    items = [("RADAR", radar_reached), ("HANDOVER", handover),
             ("INTERCEPT", intercepted)]
    centers = [x + 50, x + width // 2, x + width - 50]
    for index in range(2):
        cv2.line(canvas, (centers[index] + 13, y),
                 (centers[index + 1] - 13, y),
                 GREEN if items[index + 1][1] else GRID, 3, cv2.LINE_AA)
    for (label, active), center in zip(items, centers):
        cv2.circle(canvas, (center, y), 11, GREEN if active else GRID, -1,
                   cv2.LINE_AA)
        if active:
            cv2.circle(canvas, (center, y), 4, WHITE, -1, cv2.LINE_AA)
        size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)[0]
        text(canvas, label, (center - size[0] // 2, y + 30), 0.36,
             WHITE if active else MUTED, 1)


def draw_gauge(canvas, x, y, width, value):
    value = value if finite(value) else 0.0
    color = GREEN if value < 0.75 else (AMBER if value < 1.0 else RED)
    cv2.rectangle(canvas, (x, y), (x + width, y + 12), GRID, -1)
    cv2.rectangle(canvas, (x, y),
                  (x + int(round(width * min(max(value, 0.0), 1.2) / 1.2)),
                   y + 12), color, -1)
    boundary_x = x + int(round(width / 1.2))
    cv2.line(canvas, (boundary_x, y - 4), (boundary_x, y + 16), WHITE, 1,
             cv2.LINE_AA)


def metric_line(canvas, label, value, x, y, width, color=WHITE):
    text(canvas, label, (x, y), 0.40, MUTED, 1)
    right_text(canvas, value, x + width, y, 0.43, color, 1)


def draw_fixed_camera(canvas, row, x, y, width, height):
    rounded_rect(canvas, (x, y), (x + width, y + height), (12, 17, 23), 8)
    # A subdued grid makes angular motion easy to read without implying that
    # the synthetic projection is a photorealistic camera stream.
    for fraction in (0.25, 0.5, 0.75):
        gx = int(round(x + width * fraction))
        gy = int(round(y + height * fraction))
        cv2.line(canvas, (gx, y + 1), (gx, y + height - 1), GRID, 1)
        cv2.line(canvas, (x + 1, gy), (x + width - 1, gy), GRID, 1)
    cx, cy = x + width // 2, y + height // 2
    cv2.line(canvas, (cx - 12, cy), (cx + 12, cy), MUTED, 1, cv2.LINE_AA)
    cv2.line(canvas, (cx, cy - 12), (cx, cy + 12), MUTED, 1, cv2.LINE_AA)

    u = number(row, "pixel_u_offset")
    v = number(row, "pixel_v_offset")
    tracking = integer(row, "tracking") > 0
    if finite(u) and finite(v):
        px = int(round(cx + u * width / 960.0))
        py = int(round(cy - v * height / 540.0))
        px = min(max(px, x + 5), x + width - 5)
        py = min(max(py, y + 5), y + height - 5)
        marker_color = GREEN if tracking else RED
        cv2.circle(canvas, (px, py), 13, marker_color, 2, cv2.LINE_AA)
        cv2.line(canvas, (px - 18, py), (px + 18, py), marker_color, 1,
                 cv2.LINE_AA)
        cv2.line(canvas, (px, py - 18), (px, py + 18), marker_color, 1,
                 cv2.LINE_AA)
    if not tracking:
        right_text(canvas, "NO VISUAL TRACK", x + width - 10, y + 22, 0.35,
                   RED, 1)
    text(canvas, "FIXED CAMERA / SYNTHETIC TARGET PROJECTION",
         (x + 10, y + height - 11), 0.34, MUTED, 1)


def draw_side_panel(canvas, row, summary, params):
    x, y, w, h = 1360, 84, 540, 742
    rounded_rect(canvas, (x, y), (x + w, y + h), PANEL, 12)
    text(canvas, "MISSION TELEMETRY", (x + 20, y + 32), 0.58, WHITE, 1)

    phase = str(row.get("phase", "")).upper()
    phase_color = RED if phase == "TIMEOUT_HOLD" else (
        GREEN if phase == "SUCCESS_HOLD" else CYAN)
    text(canvas, phase_label(row), (x + 20, y + 67), 0.58, phase_color, 2)
    draw_progress(canvas, row, x + 20, y + 99, w - 40)

    distance = number(row, "distance_m")
    text(canvas, "SEPARATION", (x + 20, y + 154), 0.38, MUTED, 1)
    distance_text = "--.- m" if not finite(distance) else "%.1f m" % distance
    text(canvas, distance_text, (x + 20, y + 198), 1.20, WHITE, 2)

    handover = integer(row, "handover_complete") > 0
    tracking = integer(row, "tracking") > 0
    rounded_rect(canvas, (x + 260, y + 145), (x + 374, y + 180),
                 status_color(handover), 8)
    rounded_rect(canvas, (x + 385, y + 145), (x + 500, y + 180),
                 status_color(tracking), 8)
    text(canvas, "HANDOVER", (x + 274, y + 168), 0.37, WHITE, 1)
    text(canvas, "TRACKING", (x + 401, y + 168), 0.37, WHITE, 1)
    text(canvas, "radar samples %d" % integer(row, "radar_samples"),
         (x + 262, y + 202), 0.40, MUTED, 1)

    h_ratio = number(row, "horizontal_fov_ratio")
    v_ratio = number(row, "vertical_fov_ratio")
    values = [value for value in (h_ratio, v_ratio) if finite(value)]
    fov_ratio = max(values) if values else float("nan")
    text(canvas, "NORMALIZED FOV OCCUPANCY", (x + 20, y + 237), 0.38,
         MUTED, 1)
    right_text(canvas, "--" if not finite(fov_ratio) else "%.2f" % fov_ratio,
               x + w - 20, y + 237, 0.50,
               GREEN if finite(fov_ratio) and fov_ratio < 0.75 else AMBER, 1)
    draw_gauge(canvas, x + 20, y + 249, w - 40, fov_ratio)

    cv2.line(canvas, (x + 20, y + 280), (x + w - 20, y + 280), GRID, 1)
    radar_error = number(row, "radar_measurement_error_m")
    radar_bias = math.sqrt(sum(number(row, key, 0.0) ** 2 for key in
                         ("radar_bias_x_m", "radar_bias_y_m", "radar_bias_z_m")))
    visual_bias = math.hypot(number(row, "visual_bias_u_px", 0.0),
                             number(row, "visual_bias_v_px", 0.0))
    radar_delay = number(row, "radar_delay_s")
    visual_delay = number(row, "visual_delay_s")
    command_delay = number(row, "guidance_delay_s")
    metric_line(canvas, "radar error / bias",
                "%s / %.2f m" % (
                    "--" if not finite(radar_error) else "%.2f m" % radar_error,
                    radar_bias), x + 20, y + 305, w - 40)
    metric_line(canvas, "radar / visual latency",
                "%s / %s" % (
                    "--" if not finite(radar_delay) else "%.0f ms" % (radar_delay * 1000),
                    "--" if not finite(visual_delay) else "%.0f ms" % (visual_delay * 1000)),
                x + 20, y + 329, w - 40)
    metric_line(canvas, "command latency / visual drift",
                "%s / %.2f px" % (
                    "--" if not finite(command_delay) else "%.0f ms" % (command_delay * 1000),
                    visual_bias), x + 20, y + 353, w - 40)
    metric_line(canvas, "dropped radar / visual / command",
                "%d / %d / %d" % (
                    integer(row, "radar_dropped_count"),
                    integer(row, "visual_dropped_count"),
                    integer(row, "guidance_dropped_count")),
                x + 20, y + 377, w - 40)

    profile = str(params.get("disturbance_model", "unknown")).replace("_", " ")
    wind = number(params, "wind_velocity_mean_mps", 0.0)
    metric_line(canvas, "error profile",
                "%s; wind %.1f m/s" % (profile, wind),
                x + 20, y + 401, w - 40, CYAN)

    draw_fixed_camera(canvas, row, x + 20, y + 430, w - 40, 281)


def draw_chart(canvas, rows, times, now, x, y, width, height, key, title,
               line_color, fixed_max=None, threshold=None, units=""):
    rounded_rect(canvas, (x, y), (x + width, y + height), PANEL, 10)
    text(canvas, title, (x + 14, y + 23), 0.42, WHITE, 1)
    left, top = x + 50, y + 36
    right, bottom = x + width - 16, y + height - 27
    cv2.rectangle(canvas, (left, top), (right, bottom), GRID, 1)
    start_index = bisect.bisect_left(times, now - 12.0)
    end_index = bisect.bisect_right(times, now)
    window = rows[start_index:end_index]
    points = []
    values = []
    for row in window:
        if isinstance(key, tuple):
            candidates = [number(row, item) for item in key]
            candidates = [value for value in candidates if finite(value)]
            value = max(candidates) if candidates else float("nan")
        else:
            value = number(row, key)
        if finite(value):
            values.append(value)
            points.append((row["_time"], value))
    if not points:
        text(canvas, "waiting for telemetry", (left + 15, top + 35), 0.40,
             MUTED, 1)
        return
    y_min = 0.0
    if fixed_max is None:
        y_max = max(max(values) * 1.10, 1.0)
    else:
        y_max = max(float(fixed_max), max(values) * 1.05)
    time_min = max(times[0], now - 12.0)
    time_span = max(now - time_min, 0.2)

    if threshold is not None and y_min <= threshold <= y_max:
        threshold_y = int(round(bottom - (threshold - y_min) /
                                (y_max - y_min) * (bottom - top)))
        cv2.line(canvas, (left, threshold_y), (right, threshold_y), RED, 1,
                 cv2.LINE_AA)
    pixel_points = []
    for timestamp, value in points:
        px = int(round(left + (timestamp - time_min) / time_span * (right - left)))
        py = int(round(bottom - (value - y_min) / (y_max - y_min) *
                       (bottom - top)))
        pixel_points.append((min(max(px, left), right), min(max(py, top), bottom)))
    if len(pixel_points) >= 2:
        cv2.polylines(canvas, [np.asarray(pixel_points, dtype=np.int32)], False,
                      line_color, 2, cv2.LINE_AA)
    else:
        cv2.circle(canvas, pixel_points[0], 2, line_color, -1)
    text(canvas, "%.1f%s" % (y_max, units), (x + 5, top + 5), 0.30, MUTED, 1)
    text(canvas, "0", (x + 28, bottom + 4), 0.30, MUTED, 1)
    right_text(canvas, "last 12 s", right, y + height - 8, 0.31, MUTED, 1)
    right_text(canvas, "%.2f%s" % (values[-1], units), right, y + 23, 0.42,
               line_color, 1)


def draw_footer(canvas, rows, times, now, params):
    capture_distance = number(params, "capture_distance_m", 5.0)
    draw_chart(canvas, rows, times, now, 20, 842, 925, 213, "distance_m",
               "SEPARATION HISTORY", BLUE, threshold=capture_distance,
               units="m")
    current = nearest_row(rows, times, now)
    handover_visible = integer(current, "handover_complete") > 0 or str(
        current.get("phase", "")).upper() in ("HANDOVER_HOLD", "SUCCESS_HOLD")
    fov_rows = rows
    fov_times = times
    fov_title = "FOV OCCUPANCY HISTORY"
    if handover_visible:
        first_handover = next((index for index, row in enumerate(rows)
                               if integer(row, "handover_complete") > 0), None)
        if first_handover is not None:
            visual_intercept_rows = [
                row for row in rows[first_handover:]
                if str(row.get("phase", "")).upper() == "CHASE"]
            if visual_intercept_rows:
                fov_rows = visual_intercept_rows
                fov_times = [row["_time"] for row in fov_rows]
            fov_title = "FOV DURING VISUAL INTERCEPTION"
    draw_chart(canvas, fov_rows, fov_times, now, 960, 842, 940, 213,
               ("horizontal_fov_ratio", "vertical_fov_ratio"),
               fov_title, CYAN, fixed_max=1.10,
               threshold=1.0)
    text(canvas,
         "Synthetic visual projection; production /object_kcf interface unchanged.",
         (38, 1046), 0.34, MUTED, 1)


def nearest_row(rows, times, timestamp):
    index = bisect.bisect_left(times, timestamp)
    if index <= 0:
        return rows[0]
    if index >= len(rows):
        return rows[-1]
    before, after = rows[index - 1], rows[index]
    return before if timestamp - before["_time"] <= after["_time"] - timestamp else after


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", help="single simulation run directory")
    parser.add_argument("output", help="output MP4 path")
    parser.add_argument("--input-video", help="input video (default gazebo_raw.mp4)")
    parser.add_argument("--csv", dest="csv_path",
                        help="telemetry CSV (default full_chain.csv/experiment.csv)")
    parser.add_argument("--epoch-file", default="video_start_epoch_s.txt",
                        help="wall-clock diagnostic file (not a simulation-time fallback)")
    parser.add_argument("--ros-time-file", default="video_start_ros_time_s.txt",
                        help="video frame-zero ROS/simulation time file")
    parser.add_argument("--video-start-ros-time", type=float,
                        help="explicit frame-zero ROS/simulation time")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=0.0,
                        help="output FPS (0 keeps source FPS)")
    parser.add_argument("--codec", default="mp4v",
                        help="OpenCV fourcc (default mp4v)")
    parser.add_argument("--sync-offset-sec", type=float, default=0.0,
                        help="additive correction to frame-zero ROS/simulation time")
    parser.add_argument("--lead-seconds", type=float, default=0.5,
                        help="video retained before first telemetry row")
    parser.add_argument("--tail-seconds", type=float, default=1.5,
                        help="video retained after last telemetry row")
    parser.add_argument("--success-hold-seconds", type=float, default=0.05,
                        help="retain this much after SUCCESS_HOLD starts (-1 keeps all)")
    parser.add_argument("--end-freeze-seconds", type=float, default=2.0,
                        help="explicitly hold the final success frame for this duration")
    parser.add_argument("--no-trim", action="store_true",
                        help="retain the complete raw recording")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if args.width != 1920 or args.height != 1080:
        raise SystemExit("this client layout currently requires --width 1920 --height 1080")
    if len(args.codec) != 4:
        raise SystemExit("--codec must contain exactly four characters")

    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise SystemExit("run directory does not exist: %s" % run_dir)
    try:
        video_path = find_input(run_dir, args.input_video,
                                ("gazebo_raw.mp4", "raw.mp4"), ".mp4")
        csv_path = find_input(run_dir, args.csv_path,
                              ("full_chain.csv", "experiment.csv"), ".csv")
        rows = load_rows(csv_path)
        start_time = resolve_video_start_time(
            run_dir, args.video_start_ros_time, args.ros_time_file,
            args.epoch_file)
        clock_map = load_video_clock(run_dir, args.epoch_file)
        start_time += args.sync_offset_sec
    except (FileNotFoundError, OSError, ValueError) as exc:
        raise SystemExit(str(exc))

    summary = load_json(run_dir / "summary.json")
    params = load_json(run_dir / "case_params.json")
    times = [row["_time"] for row in rows]

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit("cannot open input video: %s" % video_path)
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    if not finite(source_fps) or source_fps <= 0.0:
        source_fps = 24.0
    output_fps = args.fps if args.fps > 0.0 else source_fps
    frame_count = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))

    output_path = Path(args.output).expanduser()
    if not output_path.is_absolute():
        output_path = Path.cwd() / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*args.codec), output_fps,
        (args.width, args.height))
    if not writer.isOpened():
        capture.release()
        raise SystemExit("cannot open output encoder %s for %s" %
                         (args.codec, output_path))

    first_keep = -float("inf") if args.no_trim else times[0] - max(args.lead_seconds, 0.0)
    last_keep = float("inf") if args.no_trim else times[-1] + max(args.tail_seconds, 0.0)
    if not args.no_trim and args.success_hold_seconds >= 0.0:
        success_row = next((row for row in rows
                            if row.get("phase") == "SUCCESS_HOLD"), None)
        if success_row is not None:
            last_keep = min(last_keep,
                            success_row["_time"] + args.success_hold_seconds)
    telemetry_start = times[0]
    read_count = 0
    write_count = 0
    next_progress = 0
    last_canvas = None
    last_phase = None
    try:
        while True:
            ok, source = capture.read()
            if not ok:
                break
            if clock_map is not None and args.video_start_ros_time is None:
                wall_start, wall_times, simulation_times = clock_map
                frame_time = simulation_time_at_wall(
                    wall_start + read_count / source_fps,
                    wall_times, simulation_times) + args.sync_offset_sec
            else:
                frame_time = start_time + read_count / source_fps
            read_count += 1
            if frame_time < first_keep:
                continue
            if frame_time > last_keep:
                break

            row = nearest_row(rows, times, frame_time)
            canvas = np.full((args.height, args.width, 3), BG, dtype=np.uint8)
            draw_header(canvas, row, summary, params, frame_time - telemetry_start)
            draw_main_view(canvas, source)
            draw_side_panel(canvas, row, summary, params)
            draw_footer(canvas, rows, times, frame_time, params)
            writer.write(canvas)
            write_count += 1
            last_canvas = canvas
            last_phase = row.get("phase")

            if frame_count > 0:
                percent = int(100.0 * read_count / frame_count)
                if percent >= next_progress:
                    print("rendering: %d%%" % min(percent, 100), flush=True)
                    next_progress += 10
        if (last_canvas is not None and last_phase == "SUCCESS_HOLD" and
                not args.no_trim and args.end_freeze_seconds > 0):
            rounded_rect(last_canvas, (350, 766), (1028, 806), (12, 17, 24), 8)
            text(last_canvas, "SUCCESS FRAME HELD - capture radius reached",
                 (370, 794), 0.58, GREEN, 1)
            for _ in range(int(round(output_fps * args.end_freeze_seconds))):
                writer.write(last_canvas)
                write_count += 1
    finally:
        capture.release()
        writer.release()

    if write_count == 0:
        try:
            output_path.unlink()
        except OSError:
            pass
        raise SystemExit(
            "no synchronized frames were written; provide video_start_ros_time_s.txt "
            "or use --video-start-ros-time/--sync-offset-sec")
    print("wrote %d frames at %.3f fps: %s" %
          (write_count, output_fps, output_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
