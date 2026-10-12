#!/usr/bin/env python3
"""Combine three Gazebo recordings into one synchronized 2x2 comparison."""

import argparse
import csv
import json
import math
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit("OpenCV and NumPy are required: %s" % exc)


PANEL_W = 960
PANEL_H = 540
CANVAS_W = 1920
CANVAS_H = 1080
BACKGROUND = (23, 27, 34)
WHITE = (242, 244, 247)
MUTED = (170, 179, 190)
GREEN = (105, 205, 115)
AMBER = (70, 190, 245)


def resolve_video(value):
    path = Path(value).expanduser().resolve()
    if path.is_dir():
        path = path / "gazebo_raw.mp4"
    if not path.is_file():
        raise SystemExit("Video not found: %s" % path)
    return path


def read_case_params(video_path):
    path = video_path.parent / "case_params.json"
    if not path.is_file():
        return {}
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def read_run_result(video_path):
    csv_path = video_path.parent / "experiment.csv"
    if not csv_path.is_file():
        return "RESULT UNKNOWN", MUTED
    try:
        with csv_path.open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        success = any(row.get("phase") == "SUCCESS_HOLD" for row in rows)
        return ("SUCCESS", GREEN) if success else ("NOT CAPTURED", AMBER)
    except OSError:
        return "RESULT UNKNOWN", MUTED


class VideoStream:
    def __init__(self, path):
        self.path = path
        self.capture = cv2.VideoCapture(str(path))
        if not self.capture.isOpened():
            raise SystemExit("Cannot open video: %s" % path)
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(self.fps) or self.fps <= 0.0:
            self.fps = 24.0
        self.frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.duration = self.frame_count / self.fps if self.frame_count else 0.0
        self.index = -1
        self.last_frame = None

    def frame_at(self, seconds):
        target = min(max(int(seconds * self.fps), 0), max(self.frame_count - 1, 0))
        while self.index < target:
            ok, frame = self.capture.read()
            if not ok:
                break
            self.last_frame = frame
            self.index += 1
        if self.last_frame is None:
            ok, frame = self.capture.read()
            if not ok:
                raise SystemExit("No decodable frames in %s" % self.path)
            self.last_frame = frame
            self.index = 0
        return self.last_frame

    def close(self):
        self.capture.release()


def fit_panel(frame):
    height, width = frame.shape[:2]
    scale = min(PANEL_W / float(width), PANEL_H / float(height))
    resized = cv2.resize(
        frame,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=cv2.INTER_AREA,
    )
    panel = np.full((PANEL_H, PANEL_W, 3), BACKGROUND, dtype=np.uint8)
    y0 = (PANEL_H - resized.shape[0]) // 2
    x0 = (PANEL_W - resized.shape[1]) // 2
    panel[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    return panel


def add_label(panel, title, subtitle, result, color):
    overlay = panel.copy()
    cv2.rectangle(overlay, (0, 0), (PANEL_W, 82), (12, 15, 20), -1)
    cv2.addWeighted(overlay, 0.78, panel, 0.22, 0.0, panel)
    cv2.putText(panel, title, (24, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.82, WHITE, 2, cv2.LINE_AA)
    cv2.putText(panel, subtitle, (24, 66), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, MUTED, 1, cv2.LINE_AA)
    size = cv2.getTextSize(result, cv2.FONT_HERSHEY_SIMPLEX, 0.64, 2)[0]
    cv2.putText(panel, result, (PANEL_W - size[0] - 24, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.64, color, 2, cv2.LINE_AA)


def add_info_panel(canvas, params, seconds):
    x0, y0 = PANEL_W, PANEL_H
    cv2.rectangle(canvas, (x0, y0), (CANVAS_W, CANVAS_H), BACKGROUND, -1)
    target_speed = params.get("target_speed_mps", 9.0)
    forward_speed = params.get("visual_forward_speed_mps", 11.5)
    lines = [
        ("DIRECT VISUAL CONTROL", 0.90, WHITE, 2),
        ("Three runs, same initial geometry", 0.58, MUTED, 1),
        ("Target speed: %.1f m/s" % float(target_speed), 0.66, WHITE, 2),
        ("Commanded forward speed: %.1f m/s" % float(forward_speed), 0.66, WHITE, 2),
        ("No radar-approach stage", 0.62, WHITE, 1),
        ("Target motion: level and straight", 0.62, WHITE, 1),
        ("Elapsed video time: %5.1f s" % seconds, 0.62, GREEN, 2),
    ]
    y = y0 + 80
    for text, scale, color, thickness in lines:
        cv2.putText(canvas, text, (x0 + 55, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)
        y += 58 if scale >= 0.65 else 48
    cv2.line(canvas, (x0 + 55, y + 5), (CANVAS_W - 55, y + 5),
             (75, 83, 94), 1, cv2.LINE_AA)
    y += 55
    cv2.putText(canvas, "FULL: lateral/yaw + vertical correction", (x0 + 55, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, MUTED, 1, cv2.LINE_AA)
    cv2.putText(canvas, "HORIZONTAL: lateral/yaw correction only", (x0 + 55, y + 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, MUTED, 1, cv2.LINE_AA)
    cv2.putText(canvas, "VERTICAL: vertical correction only", (x0 + 55, y + 76),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, MUTED, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("full", help="Full-control run directory or Gazebo video")
    parser.add_argument("horizontal", help="Horizontal-only run directory or Gazebo video")
    parser.add_argument("vertical", help="Vertical-only run directory or Gazebo video")
    parser.add_argument("output", help="Output MP4 path")
    parser.add_argument("--fps", type=float, default=24.0)
    args = parser.parse_args()

    paths = [resolve_video(args.full), resolve_video(args.horizontal),
             resolve_video(args.vertical)]
    streams = [VideoStream(path) for path in paths]
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), args.fps,
        (CANVAS_W, CANVAS_H),
    )
    if not writer.isOpened():
        raise SystemExit("Cannot create output video: %s" % output_path)

    labels = [
        ("FULL CONTROL", "Horizontal and vertical correction"),
        ("HORIZONTAL ONLY", "Vertical velocity command = 0"),
        ("VERTICAL ONLY", "Lateral and yaw commands = 0"),
    ]
    results = [read_run_result(path) for path in paths]
    params = read_case_params(paths[0])
    duration = max(stream.duration for stream in streams)
    frame_total = max(1, int(math.ceil(duration * args.fps)))

    try:
        for output_index in range(frame_total):
            seconds = output_index / args.fps
            canvas = np.full((CANVAS_H, CANVAS_W, 3), BACKGROUND, dtype=np.uint8)
            panels = []
            for stream, label, result in zip(streams, labels, results):
                panel = fit_panel(stream.frame_at(seconds))
                add_label(panel, label[0], label[1], result[0], result[1])
                panels.append(panel)
            canvas[0:PANEL_H, 0:PANEL_W] = panels[0]
            canvas[0:PANEL_H, PANEL_W:CANVAS_W] = panels[1]
            canvas[PANEL_H:CANVAS_H, 0:PANEL_W] = panels[2]
            add_info_panel(canvas, params, seconds)
            cv2.line(canvas, (PANEL_W, 0), (PANEL_W, CANVAS_H),
                     (75, 83, 94), 2)
            cv2.line(canvas, (0, PANEL_H), (CANVAS_W, PANEL_H),
                     (75, 83, 94), 2)
            writer.write(canvas)
            if output_index % max(1, int(args.fps * 10)) == 0:
                print("rendered %.1f / %.1f s" % (seconds, duration), flush=True)
    finally:
        writer.release()
        for stream in streams:
            stream.close()

    print(str(output_path))


if __name__ == "__main__":
    main()
