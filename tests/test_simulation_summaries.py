#!/usr/bin/env python3
"""Regression tests for client-facing radar batch summaries."""

import csv
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "tools" / "simulation" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CLIENT = load_script("summarize_full_chain_demo")
AGGREGATE = load_script("aggregate_radar_multisim")
SELECTOR = load_script("select_representative_video_case")
ANALYZER = load_script("analyze_radar_multisim_result")


def sample_summary():
    return {
        "case_name": "radar10_tail_combined_s101",
        "expected_result": "success",
        "success": True,
        "handover_occurred": True,
        "full_chain_pass": True,
        "minimum_distance_m": 4.2,
        "minimum_terminal_error_m": 1.2,
        "handover_time_from_chase_s": 9.0,
        "capture_time_from_chase_s": 22.0,
        "capture_distance_at_success_m": 4.9,
        "radar_prediction_rmse_m": 8.0,
        "post_handover_tracking_availability_percent": 100.0,
        "post_handover_max_fov_ratio": 0.8,
        "post_handover_p95_fov_ratio": 0.7,
        "radar_delay_mean_s": 0.15,
        "visual_delay_mean_s": 0.08,
        "guidance_delay_mean_s": 0.04,
        "reason": "intercepted",
    }


def sample_params():
    return {
        "test_group": "tail",
        "error_profile": "mild",
        "target_speed_mps": 2.0,
        "target_heading_deg": 0.0,
        "target_turn_rate_degps": 0.0,
        "target_climb_rate_mps": 0.0,
        "radar_noise_std_m": 10.0 / math.sqrt(3.0),
        "radar_period_sec": 3.0,
        "radar_timeout_sec": 7.0,
        "capture_distance_m": 5.0,
        "radar_bootstrap_samples": 2,
        "filter_alpha": 0.70,
        "filter_beta": 0.20,
        "max_radar_innovation_m": 80.0,
        "max_target_speed_mps": 25.0,
        "radar_period_jitter_std_sec": 0.10,
        "radar_delay_mean_sec": 0.15,
        "radar_delay_jitter_std_sec": 0.03,
        "radar_dropout_probability": 0.05,
        "radar_bias_random_walk_xy_m_sqrts": 0.04,
        "radar_bias_random_walk_z_m_sqrts": 0.06,
        "radar_bias_limit_xy_m": 0.60,
        "radar_bias_limit_z_m": 1.00,
        "radar_dropout_duration_sec": 0.0,
    }


def write_case(batch, include_params=True, summary=None,
               summary_overrides=None, params=None):
    case = batch / "radar10_tail_combined_s101"
    case.mkdir()
    summary = sample_summary() if summary is None else dict(summary)
    if summary_overrides:
        summary.update(summary_overrides)
    (case / "summary.json").write_text(
        json.dumps(summary), encoding="utf-8")
    if include_params:
        (case / "case_params.json").write_text(
            json.dumps(sample_params() if params is None else params),
            encoding="utf-8")
    return case


class SimulationSummaryTests(unittest.TestCase):

    def test_disabled_legacy_pose_gate_is_not_reported_as_failure_cause(self):
        summary = {
            "success": False, "chase_started": True,
            "handover_occurred": False, "radar_timeout_fraction": 0.0,
            "minimum_terminal_error_m": 10.0,
            "tracking_availability_percent": 50.0,
        }
        params = {"use_legacy_pose_handover_gate": False,
                  "handover_position_tolerance_m": 2.5}
        reason = ANALYZER.classify_failure(summary, params)
        self.assertNotIn("交接位置容差", reason)
        self.assertIn("雷达估计健康", reason)

    def test_three_tier_matrix_is_frozen_and_horizontal(self):
        matrix = ROOT / "tools" / "simulation" / (
            "radar_approach_three_tier_cases.csv")
        with matrix.open(newline="", encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))

        self.assertEqual(len(rows), 36)
        for profile in ("ideal", "field", "harsh"):
            selected = [row for row in rows
                        if row["ERROR_PROFILE"] == profile]
            self.assertEqual(len(selected), 12)
        self.assertEqual(
            {row["TEST_GROUP"] for row in rows},
            {"tail_slow", "tail_fast", "oblique45", "cross90"})
        self.assertEqual(
            {row["DISTURBANCE_SEED"] for row in rows},
            {"104729", "130363", "155921"})
        self.assertTrue(all(row["TARGET_CLIMB_RATE_MPS"] == "0"
                            for row in rows))
        self.assertTrue(all(row["TARGET_DISTURBANCE_STD_Z_MPS"] == "0"
                            for row in rows))
        self.assertTrue(all(row["WIND_DIRECTION_Z"] == "0"
                            for row in rows))

    def test_client_summary_uses_recorded_radar_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            write_case(batch)
            records = CLIENT.load_records(str(batch))
            rows = CLIENT.aggregate(records)
            CLIENT.write_markdown(str(batch), records, rows)
            report = (batch / "client_summary.md").read_text(
                encoding="utf-8")

            self.assertIn("每轴σ=5.77 m（等效三维RMS≈10.00 m）", report)
            self.assertIn("初始化样本数=2", report)
            self.assertIn("α-β滤波α/β=0.70/0.20", report)
            self.assertIn("创新限幅=80.0 m", report)

    def test_aggregate_csv_exports_recorded_radar_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            write_case(batch)
            records = AGGREGATE.load_batch(str(batch))
            AGGREGATE.write_csv_summary(str(batch), records)
            AGGREGATE.build_report(str(batch), records)

            with (batch / "batch_summary.csv").open(
                    newline="", encoding="utf-8-sig") as stream:
                row = next(csv.DictReader(stream))
            self.assertTrue(math.isclose(
                float(row["radar_noise_std_m"]), 10.0 / math.sqrt(3.0)))
            self.assertEqual(row["radar_bootstrap_samples"], "2")
            self.assertEqual(row["filter_alpha"], "0.7")
            self.assertEqual(row["capture_distance_at_success_m"], "4.9")
            report = (batch / "report.md").read_text(encoding="utf-8")
            self.assertIn(
                "位置白噪声每轴σ=5.77 m（等效三维RMS≈10.00 m）", report)
            self.assertIn("成功判定首帧距离 4.90 m", report)

    def test_client_summary_falls_back_to_archived_summary_without_params(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            fallback = sample_params()
            fallback.update({"test_group": "tail", "error_profile": "mild"})
            write_case(batch, include_params=False, summary_overrides=fallback)

            records = CLIENT.load_records(str(batch))
            self.assertEqual(len(records), 1)
            self.assertEqual(
                CLIENT.setting(records[0], "radar_noise_std_m"),
                fallback["radar_noise_std_m"])
            CLIENT.write_markdown(
                str(batch), records, CLIENT.aggregate(records))
            report = (batch / "client_summary.md").read_text(
                encoding="utf-8")
            self.assertIn("等效三维RMS≈10.00 m", report)

    def test_aggregate_report_marks_missing_legacy_settings_as_unrecorded(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            legacy_params = sample_params()
            for key in (
                    "radar_bootstrap_samples", "filter_alpha", "filter_beta",
                    "max_radar_innovation_m", "max_target_speed_mps"):
                legacy_params.pop(key)
            legacy_summary = sample_summary()
            for key in (
                    "full_chain_pass", "capture_time_from_chase_s",
                    "post_handover_p95_fov_ratio"):
                legacy_summary.pop(key)
            write_case(
                batch, summary=legacy_summary,
                params=legacy_params)

            records = AGGREGATE.load_batch(str(batch))
            AGGREGATE.build_report(str(batch), records)
            report = (batch / "report.md").read_text(encoding="utf-8")
            robust_line = next(
                line for line in report.splitlines()
                if line.startswith("- 雷达鲁棒参数："))
            self.assertNotIn("None", robust_line)
            self.assertIn("初始化样本数 未记录", robust_line)
            self.assertIn("α/β=未记录/未记录", robust_line)

    def test_primary_gate_uses_success_decision_distance_not_chase_minimum(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            case = write_case(batch, summary_overrides={
                # The threshold-crossing row is logged after the phase changes
                # from CHASE to SUCCESS_HOLD, so this value may remain >5 m.
                "minimum_distance_m": 5.08,
                "capture_distance_at_success_m": 4.97,
            })
            (case / "experiment.csv").write_text(
                "phase,distance_m,interceptor_x,interceptor_y,interceptor_z,"
                "target_x,target_y,target_z,guidance_vx,guidance_vy,guidance_vz,"
                "guidance_yaw_rate,interceptor_vx,interceptor_vy,interceptor_vz,"
                "target_vx,target_vy,target_vz\n"
                "SUCCESS_HOLD,4.97,0,0,5,1,1,9,0,0,0,0,0,0,0,2,0,0\n",
                encoding="utf-8")

            record = CLIENT.load_records(str(batch))[0]
            self.assertTrue(record["primary_validation_pass"])

            record["capture_distance_at_success_m"] = 5.01
            self.assertFalse(CLIENT.primary_validation_pass(record))
            self.assertIn("超过设定门限", CLIENT.primary_failure_reason(record))

    def test_representative_selector_uses_primary_validation_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            case = write_case(batch)
            header = (
                "phase,distance_m,interceptor_x,interceptor_y,interceptor_z,"
                "target_x,target_y,target_z,guidance_vx,guidance_vy,guidance_vz,"
                "guidance_yaw_rate,interceptor_vx,interceptor_vy,interceptor_vz,"
                "target_vx,target_vy,target_vz\n")
            unsafe = (
                "SUCCESS_HOLD,4.9,0,0,1.5,1,1,9,0,0,0,0,0,0,0,2,0,0\n")
            (case / "experiment.csv").write_text(
                header + unsafe, encoding="utf-8")
            self.assertEqual(
                SELECTOR.load_successful_runs(batch, "mild"), [])

            safe = (
                "SUCCESS_HOLD,4.9,0,0,5,1,1,9,0,0,0,0,0,0,0,2,0,0\n")
            (case / "experiment.csv").write_text(header + safe, encoding="utf-8")
            selected = SELECTOR.load_successful_runs(batch, "mild")
            self.assertEqual([item["case_name"] for item in selected],
                             ["radar10_tail_combined_s101"])
            self.assertEqual(selected[0]["minimum_interceptor_altitude_m"], 5.0)

    def test_failed_handover_missing_fov_is_not_a_numeric_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            batch = Path(directory)
            case = write_case(batch, summary_overrides={
                "success": False, "handover_occurred": False,
                "post_handover_max_fov_ratio": float("nan"),
                "capture_time_from_chase_s": float("nan"),
                "handover_time_from_chase_s": float("nan"),
            })
            (case / "experiment.csv").write_text(
                "phase,distance_m,interceptor_x,interceptor_y,interceptor_z,"
                "target_x,target_y,target_z,guidance_vx,guidance_vy,guidance_vz,"
                "guidance_yaw_rate,interceptor_vx,interceptor_vy,interceptor_vz,"
                "target_vx,target_vy,target_vz\n"
                "CHASE,20,0,0,5,20,0,9,0,0,0,0,0,0,0,2,0,0\n",
                encoding="utf-8")
            record = CLIENT.load_records(str(batch))[0]
            self.assertFalse(record["primary_validation_pass"])
            self.assertFalse(record["numeric_anomaly_detected"])

    def test_video_sync_rejects_unix_epoch_as_ros_time(self):
        # Import lazily because OpenCV availability is an optional runtime
        # dependency of the video renderer, not of the summary scripts.
        try:
            video = load_script("make_client_demo_video")
        except SystemExit as exc:
            self.skipTest(str(exc))
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "video_start_epoch_s.txt").write_text(
                "1791590400.0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Unix wall-clock"):
                video.resolve_video_start_time(
                    run_dir, None, "video_start_ros_time_s.txt",
                    "video_start_epoch_s.txt")

            (run_dir / "video_start_ros_time_s.txt").write_text(
                "123.5\n", encoding="utf-8")
            self.assertEqual(
                video.resolve_video_start_time(
                    run_dir, None, "video_start_ros_time_s.txt",
                    "video_start_epoch_s.txt"),
                123.5)

    def test_analyzer_labels_chase_budget_and_success_distance(self):
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "case_params.json").write_text(json.dumps({
                "case_name": "distance_boundary",
                "capture_distance_m": 5.0,
                "chase_timeout_s": 60.0,
            }), encoding="utf-8")
            (run_dir / "experiment.csv").write_text(
                "ros_time_s,phase,distance_m,handover_complete,radar_state,"
                "tracking,radar_samples,horizontal_fov_ratio,vertical_fov_ratio\n"
                "10.0,CHASE,5.08,1,APPROACH,1,5,0.5,0.4\n"
                "10.1,SUCCESS_HOLD,4.97,1,APPROACH,1,5,0.5,0.4\n",
                encoding="utf-8")
            completed = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "simulation" /
                                     "analyze_radar_multisim_result.py"),
                 str(run_dir)],
                check=False, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            report = (run_dir / "summary.txt").read_text(encoding="utf-8")
            self.assertIn("CHASE initial/minimum/final distance", report)
            self.assertIn("chase-budget fraction", report)
            self.assertNotIn("timeout fraction", report)
            summary = json.loads(
                (run_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["minimum_distance_m"], 5.08)
            self.assertEqual(summary["capture_distance_at_success_m"], 4.97)

    def test_video_clock_mapping_tracks_simulation_slowdown(self):
        try:
            video = load_script("make_client_demo_video")
        except SystemExit as exc:
            self.skipTest(str(exc))
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "video_start_epoch_s.txt").write_text(
                "1000.0\n", encoding="utf-8")
            (run_dir / "video_clock.csv").write_text(
                "wall_time_s,ros_time_s\n"
                "1000.0,20.0\n1002.0,21.0\n1004.0,23.0\n",
                encoding="utf-8")
            start, wall, simulation = video.load_video_clock(
                run_dir, "video_start_epoch_s.txt")
            self.assertEqual(start, 1000.0)
            self.assertEqual(video.simulation_time_at_wall(
                1001.0, wall, simulation), 20.5)
            self.assertEqual(video.simulation_time_at_wall(
                1003.0, wall, simulation), 22.0)
            (run_dir / "video_clock.csv").write_text(
                "wall_time_s,ros_time_s\n1000.0,20.0\n1002.0,19.0\n",
                encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "clock reversal"):
                video.load_video_clock(run_dir, "video_start_epoch_s.txt")


if __name__ == "__main__":
    unittest.main()
