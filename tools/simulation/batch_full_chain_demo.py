#!/usr/bin/env python3
"""Run the client-demo radar-to-visual full-chain experiment matrix."""

import csv
import datetime
import os
import shutil
import subprocess
import sys


COMMON_ENV = {
    "EXPECTED_RESULT": "success",
    "INTERCEPTOR_ALTITUDE_M": "4",
    "RADAR_PERIOD_SEC": "3",
    "RADAR_TIMEOUT_SEC": "9.5",
    "RADAR_DROPOUT_START_SEC": "-1",
    "RADAR_DROPOUT_DURATION_SEC": "0",
    "RADAR_ACQUIRE_DURATION_SEC": "9.7",
    "RADAR_ACQUIRE_TIMEOUT_SEC": "24",
    "RADAR_MEASUREMENT_STD_M": "5.773502692",
    "RADAR_BOOTSTRAP_SAMPLES": "4",
    "MAX_TARGET_HORIZONTAL_SPEED_MPS": "7",
    "MAX_TARGET_VERTICAL_SPEED_MPS": "0",
    "TERMINAL_CAMERA_RANGE_M": "25.0",
    "POSITION_GAIN_XY": "0.40",
    "POSITION_GAIN_Z": "0.35",
    "TARGET_VELOCITY_FEEDFORWARD_GAIN_Z": "0",
    "ENFORCE_MINIMUM_HANDOVER_ALTITUDE": "true",
    "MINIMUM_HANDOVER_ALTITUDE_M": "3",
    "REQUIRED_CENTERED_DETECTIONS": "10",
    "USE_BRAKING_SPEED_PROFILE": "true",
    "APPROACH_BRAKING_DECELERATION_MPS2": "2.0",
    "APPROACH_BRAKING_POSITION_BUFFER_M": "2.5",
    "USE_LEGACY_POSE_HANDOVER_GATE": "true",
    "HANDOVER_POSITION_TOLERANCE_M": "5.0",
    "HANDOVER_YAW_TOLERANCE_DEG": "20.0",
    "HANDOVER_MIN_ESTIMATED_RANGE_M": "18.0",
    "HANDOVER_MAX_ESTIMATED_RANGE_M": "38.0",
    "MAX_HANDOVER_CLOSING_SPEED_MPS": "2.0",
    "MAX_HANDOVER_OPENING_SPEED_MPS": "0.5",
    "CAPTURE_DISTANCE_M": "5",
    "HANDOVER_BLEND_DURATION_SEC": "2.0",
    "DETECTION_RANGE_M": "30",
    "VISUAL_FORWARD_SPEED_MPS": "8",
    "FORWARD_SPEED_ACCEL_LIMIT_MPS2": "1.5",
    "GATE_DETECTION_UNTIL_CHASE": "true",
    "FOV_BARRIER_GAIN": "0.20",
    "FOV_BARRIER_MAX": "4.0",
    "FOV_SLOWDOWN_START_RATIO": "0.99",
    "FOV_MIN_FORWARD_SPEED_RATIO": "1.0",
    "FOV_EMERGENCY_YAW_RATIO": "0.85",
    "K_Z_FF": "1.0",
    "N_NAV_Z": "2.0",
    "MAX_VZ": "2.0",
    "K_YAW": "1.5",
    "K_YAW_D": "0.3",
    "MAX_YAW_RATE": "1.0",
    "YAW_ENABLE_SPEED_RATIO": "0.1",
    "CONTROLLER_FOV_HORIZONTAL_LIMIT_DEG": "23.94",
    "CONTROLLER_FOV_VERTICAL_LIMIT_DEG": "18.54",
}

IDEAL_ENV = {
    "DISTURBANCE_MODEL": "none",
    "DISTURBANCE_TIME_CONSTANT_S": "2",
    "DISTURBANCE_CLIP_SIGMA": "3",
    "RADAR_NOISE_STD_M": "0",
    "RADAR_PERIOD_JITTER_STD_SEC": "0",
    "RADAR_DELAY_MEAN_SEC": "0",
    "RADAR_DELAY_JITTER_STD_SEC": "0",
    "RADAR_DROPOUT_PROBABILITY": "0",
    "RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS": "0",
    "RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS": "0",
    "RADAR_BIAS_LIMIT_XY_M": "0",
    "RADAR_BIAS_LIMIT_Z_M": "0",
    "INTERCEPTOR_DISTURBANCE_STD_XY_MPS": "0",
    "INTERCEPTOR_DISTURBANCE_STD_Z_MPS": "0",
    "TARGET_DISTURBANCE_STD_XY_MPS": "0",
    "TARGET_DISTURBANCE_STD_Z_MPS": "0",
    "VISUAL_DISTURBANCE_STD_PX": "0",
    "VISUAL_BIAS_RANDOM_WALK_PX_SQRTS": "0",
    "VISUAL_BIAS_LIMIT_PX": "0",
    "VISUAL_DELAY_MEAN_SEC": "0",
    "VISUAL_DELAY_JITTER_STD_SEC": "0",
    "VISUAL_DROPOUT_PROBABILITY": "0",
    "GUIDANCE_DELAY_MEAN_SEC": "0",
    "GUIDANCE_DELAY_JITTER_STD_SEC": "0",
    "GUIDANCE_DROPOUT_PROBABILITY": "0",
    "CONTROLLER_IMAGE_DELAY_SEC": "0",
    "NAV_POSE_POSITION_WHITE_NOISE_STD_M": "0",
    "NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS": "0",
    "NAV_POSE_POSITION_BIAS_LIMIT_M": "0",
    "NAV_ATTITUDE_WHITE_NOISE_STD_DEG": "0",
    "NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS": "0",
    "NAV_ATTITUDE_BIAS_LIMIT_DEG": "0",
    "NAV_VELOCITY_WHITE_NOISE_STD_MPS": "0",
    "NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS": "0",
    "NAV_VELOCITY_BIAS_LIMIT_MPS": "0",
    "NAV_GYRO_WHITE_NOISE_STD_RADPS": "0",
    "NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS": "0",
    "NAV_GYRO_BIAS_LIMIT_RADPS": "0",
    "NAV_ACCEL_WHITE_NOISE_STD_MPS2": "0",
    "NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS": "0",
    "NAV_ACCEL_BIAS_LIMIT_MPS2": "0",
    "NAV_POSE_DELAY_MEAN_SEC": "0",
    "NAV_POSE_DELAY_JITTER_STD_SEC": "0",
    "NAV_POSE_DROPOUT_PROBABILITY": "0",
    "NAV_VELOCITY_DELAY_MEAN_SEC": "0",
    "NAV_VELOCITY_DELAY_JITTER_STD_SEC": "0",
    "NAV_VELOCITY_DROPOUT_PROBABILITY": "0",
    "NAV_IMU_DELAY_MEAN_SEC": "0",
    "NAV_IMU_DELAY_JITTER_STD_SEC": "0",
    "NAV_IMU_DROPOUT_PROBABILITY": "0",
    "WIND_ENABLED": "0",
    "WIND_VELOCITY_MEAN_MPS": "0",
    "WIND_VELOCITY_MAX_MPS": "0",
    "WIND_VELOCITY_VARIANCE": "0",
    "WIND_DIRECTION_X": "0",
    "WIND_DIRECTION_Y": "1",
    "WIND_DIRECTION_Z": "0",
}

# Backward-compatible name for archived two-profile matrices.
CLEAN_ENV = IDEAL_ENV

MILD_ENV = {
    "DISTURBANCE_MODEL": "gauss_markov",
    "DISTURBANCE_TIME_CONSTANT_S": "2",
    "DISTURBANCE_CLIP_SIGMA": "3",
    "RADAR_NOISE_STD_M": "1.0",
    "RADAR_PERIOD_JITTER_STD_SEC": "0.10",
    "RADAR_DELAY_MEAN_SEC": "0.15",
    "RADAR_DELAY_JITTER_STD_SEC": "0.03",
    "RADAR_DROPOUT_PROBABILITY": "0.05",
    "RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS": "0.04",
    "RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS": "0.06",
    "RADAR_BIAS_LIMIT_XY_M": "0.60",
    "RADAR_BIAS_LIMIT_Z_M": "1.00",
    "INTERCEPTOR_DISTURBANCE_STD_XY_MPS": "0.20",
    "INTERCEPTOR_DISTURBANCE_STD_Z_MPS": "0.10",
    "TARGET_DISTURBANCE_STD_XY_MPS": "0.15",
    "TARGET_DISTURBANCE_STD_Z_MPS": "0.08",
    "VISUAL_DISTURBANCE_STD_PX": "1.0",
    "VISUAL_BIAS_RANDOM_WALK_PX_SQRTS": "0.08",
    "VISUAL_BIAS_LIMIT_PX": "1.50",
    "VISUAL_DELAY_MEAN_SEC": "0.08",
    "VISUAL_DELAY_JITTER_STD_SEC": "0.02",
    "VISUAL_DROPOUT_PROBABILITY": "0.02",
    "GUIDANCE_DELAY_MEAN_SEC": "0.04",
    "GUIDANCE_DELAY_JITTER_STD_SEC": "0.01",
    "GUIDANCE_DROPOUT_PROBABILITY": "0",
    "CONTROLLER_IMAGE_DELAY_SEC": "0.10",
    "NAV_POSE_POSITION_WHITE_NOISE_STD_M": "0.20",
    "NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS": "0.01",
    "NAV_POSE_POSITION_BIAS_LIMIT_M": "0.30",
    "NAV_ATTITUDE_WHITE_NOISE_STD_DEG": "0.15",
    "NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS": "0.01",
    "NAV_ATTITUDE_BIAS_LIMIT_DEG": "0.30",
    "NAV_VELOCITY_WHITE_NOISE_STD_MPS": "0.05",
    "NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS": "0.005",
    "NAV_VELOCITY_BIAS_LIMIT_MPS": "0.10",
    "NAV_GYRO_WHITE_NOISE_STD_RADPS": "0.002",
    "NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS": "0.0002",
    "NAV_GYRO_BIAS_LIMIT_RADPS": "0.005",
    "NAV_ACCEL_WHITE_NOISE_STD_MPS2": "0.05",
    "NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS": "0.005",
    "NAV_ACCEL_BIAS_LIMIT_MPS2": "0.10",
    "NAV_POSE_DELAY_MEAN_SEC": "0.04",
    "NAV_POSE_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_POSE_DROPOUT_PROBABILITY": "0.005",
    "NAV_VELOCITY_DELAY_MEAN_SEC": "0.04",
    "NAV_VELOCITY_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_VELOCITY_DROPOUT_PROBABILITY": "0.005",
    "NAV_IMU_DELAY_MEAN_SEC": "0.04",
    "NAV_IMU_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_IMU_DROPOUT_PROBABILITY": "0.005",
    "WIND_ENABLED": "1",
    "WIND_VELOCITY_MEAN_MPS": "1.0",
    "WIND_VELOCITY_MAX_MPS": "3.0",
    "WIND_VELOCITY_VARIANCE": "0",
    "WIND_DIRECTION_X": "0",
    "WIND_DIRECTION_Y": "1",
    "WIND_DIRECTION_Z": "0",
}


FIELD_ENV = {
    "DISTURBANCE_MODEL": "gauss_markov",
    "DISTURBANCE_TIME_CONSTANT_S": "2",
    "DISTURBANCE_CLIP_SIGMA": "3",
    # The 10 m radar specification is modeled as three-dimensional RMS.
    # RADAR_NOISE_STD_M is the independent standard deviation on each axis.
    "RADAR_NOISE_STD_M": "5.773502692",
    "RADAR_PERIOD_JITTER_STD_SEC": "0.10",
    "RADAR_DELAY_MEAN_SEC": "0.15",
    "RADAR_DELAY_JITTER_STD_SEC": "0.03",
    "RADAR_DROPOUT_PROBABILITY": "0.05",
    "RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS": "0.04",
    "RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS": "0.06",
    "RADAR_BIAS_LIMIT_XY_M": "0.60",
    "RADAR_BIAS_LIMIT_Z_M": "1.00",
    "INTERCEPTOR_DISTURBANCE_STD_XY_MPS": "0.20",
    "INTERCEPTOR_DISTURBANCE_STD_Z_MPS": "0.10",
    "TARGET_DISTURBANCE_STD_XY_MPS": "0.15",
    "TARGET_DISTURBANCE_STD_Z_MPS": "0",
    "VISUAL_DISTURBANCE_STD_PX": "1.0",
    "VISUAL_BIAS_RANDOM_WALK_PX_SQRTS": "0.08",
    "VISUAL_BIAS_LIMIT_PX": "1.50",
    "VISUAL_DELAY_MEAN_SEC": "0.08",
    "VISUAL_DELAY_JITTER_STD_SEC": "0.02",
    "VISUAL_DROPOUT_PROBABILITY": "0.02",
    "GUIDANCE_DELAY_MEAN_SEC": "0.04",
    "GUIDANCE_DELAY_JITTER_STD_SEC": "0.01",
    "GUIDANCE_DROPOUT_PROBABILITY": "0",
    "CONTROLLER_IMAGE_DELAY_SEC": "0.10",
    "NAV_POSE_POSITION_WHITE_NOISE_STD_M": "0.20",
    "NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS": "0.01",
    "NAV_POSE_POSITION_BIAS_LIMIT_M": "0.30",
    "NAV_ATTITUDE_WHITE_NOISE_STD_DEG": "0.15",
    "NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS": "0.01",
    "NAV_ATTITUDE_BIAS_LIMIT_DEG": "0.30",
    "NAV_VELOCITY_WHITE_NOISE_STD_MPS": "0.05",
    "NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS": "0.005",
    "NAV_VELOCITY_BIAS_LIMIT_MPS": "0.10",
    "NAV_GYRO_WHITE_NOISE_STD_RADPS": "0.002",
    "NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS": "0.0002",
    "NAV_GYRO_BIAS_LIMIT_RADPS": "0.005",
    "NAV_ACCEL_WHITE_NOISE_STD_MPS2": "0.05",
    "NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS": "0.005",
    "NAV_ACCEL_BIAS_LIMIT_MPS2": "0.10",
    "NAV_POSE_DELAY_MEAN_SEC": "0.04",
    "NAV_POSE_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_POSE_DROPOUT_PROBABILITY": "0.005",
    "NAV_VELOCITY_DELAY_MEAN_SEC": "0.04",
    "NAV_VELOCITY_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_VELOCITY_DROPOUT_PROBABILITY": "0.005",
    "NAV_IMU_DELAY_MEAN_SEC": "0.04",
    "NAV_IMU_DELAY_JITTER_STD_SEC": "0.01",
    "NAV_IMU_DROPOUT_PROBABILITY": "0.005",
    "WIND_ENABLED": "1",
    "WIND_VELOCITY_MEAN_MPS": "1.0",
    "WIND_VELOCITY_MAX_MPS": "3.0",
    "WIND_VELOCITY_VARIANCE": "0",
    "WIND_DIRECTION_X": "0",
    "WIND_DIRECTION_Y": "1",
    "WIND_DIRECTION_Z": "0",
}


HARSH_ENV = {
    "DISTURBANCE_MODEL": "gauss_markov",
    "DISTURBANCE_TIME_CONSTANT_S": "1.5",
    "DISTURBANCE_CLIP_SIGMA": "3",
    # 15 m three-dimensional radar RMS -> 8.660254038 m per-axis sigma.
    "RADAR_NOISE_STD_M": "8.660254038",
    "RADAR_PERIOD_JITTER_STD_SEC": "0.25",
    "RADAR_DELAY_MEAN_SEC": "0.35",
    "RADAR_DELAY_JITTER_STD_SEC": "0.10",
    "RADAR_DROPOUT_PROBABILITY": "0.15",
    "RADAR_BIAS_RANDOM_WALK_XY_M_SQRTS": "0.10",
    "RADAR_BIAS_RANDOM_WALK_Z_M_SQRTS": "0.15",
    "RADAR_BIAS_LIMIT_XY_M": "2.0",
    "RADAR_BIAS_LIMIT_Z_M": "3.0",
    "INTERCEPTOR_DISTURBANCE_STD_XY_MPS": "0.45",
    "INTERCEPTOR_DISTURBANCE_STD_Z_MPS": "0.20",
    "TARGET_DISTURBANCE_STD_XY_MPS": "0.30",
    "TARGET_DISTURBANCE_STD_Z_MPS": "0",
    "VISUAL_DISTURBANCE_STD_PX": "3.0",
    "VISUAL_BIAS_RANDOM_WALK_PX_SQRTS": "0.20",
    "VISUAL_BIAS_LIMIT_PX": "4.0",
    "VISUAL_DELAY_MEAN_SEC": "0.15",
    "VISUAL_DELAY_JITTER_STD_SEC": "0.05",
    "VISUAL_DROPOUT_PROBABILITY": "0.08",
    "GUIDANCE_DELAY_MEAN_SEC": "0.10",
    "GUIDANCE_DELAY_JITTER_STD_SEC": "0.03",
    "GUIDANCE_DROPOUT_PROBABILITY": "0.02",
    "CONTROLLER_IMAGE_DELAY_SEC": "0.20",
    "NAV_POSE_POSITION_WHITE_NOISE_STD_M": "0.50",
    "NAV_POSE_POSITION_BIAS_RANDOM_WALK_M_SQRTS": "0.03",
    "NAV_POSE_POSITION_BIAS_LIMIT_M": "1.0",
    "NAV_ATTITUDE_WHITE_NOISE_STD_DEG": "0.50",
    "NAV_ATTITUDE_BIAS_RANDOM_WALK_DEG_SQRTS": "0.03",
    "NAV_ATTITUDE_BIAS_LIMIT_DEG": "1.0",
    "NAV_VELOCITY_WHITE_NOISE_STD_MPS": "0.15",
    "NAV_VELOCITY_BIAS_RANDOM_WALK_MPS_SQRTS": "0.015",
    "NAV_VELOCITY_BIAS_LIMIT_MPS": "0.30",
    "NAV_GYRO_WHITE_NOISE_STD_RADPS": "0.006",
    "NAV_GYRO_BIAS_RANDOM_WALK_RADPS_SQRTS": "0.0006",
    "NAV_GYRO_BIAS_LIMIT_RADPS": "0.015",
    "NAV_ACCEL_WHITE_NOISE_STD_MPS2": "0.15",
    "NAV_ACCEL_BIAS_RANDOM_WALK_MPS2_SQRTS": "0.015",
    "NAV_ACCEL_BIAS_LIMIT_MPS2": "0.30",
    "NAV_POSE_DELAY_MEAN_SEC": "0.10",
    "NAV_POSE_DELAY_JITTER_STD_SEC": "0.03",
    "NAV_POSE_DROPOUT_PROBABILITY": "0.02",
    "NAV_VELOCITY_DELAY_MEAN_SEC": "0.10",
    "NAV_VELOCITY_DELAY_JITTER_STD_SEC": "0.03",
    "NAV_VELOCITY_DROPOUT_PROBABILITY": "0.02",
    "NAV_IMU_DELAY_MEAN_SEC": "0.10",
    "NAV_IMU_DELAY_JITTER_STD_SEC": "0.03",
    "NAV_IMU_DROPOUT_PROBABILITY": "0.02",
    "WIND_ENABLED": "1",
    "WIND_VELOCITY_MEAN_MPS": "3.0",
    "WIND_VELOCITY_MAX_MPS": "5.0",
    "WIND_VELOCITY_VARIANCE": "0",
    "WIND_DIRECTION_X": "0",
    "WIND_DIRECTION_Y": "1",
    "WIND_DIRECTION_Z": "0",
}


PROFILE_ENVS = {
    "clean": CLEAN_ENV,
    "mild": MILD_ENV,
    "ideal": IDEAL_ENV,
    "field": FIELD_ENV,
    "harsh": HARSH_ENV,
}


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cases_file = os.path.abspath(
        sys.argv[1] if len(sys.argv) > 1 else
        os.path.join(script_dir, "full_chain_demo_cases.csv"))
    batch_id = os.environ.get(
        "BATCH_ID",
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S") +
        "_full_chain_demo")
    run_root = "/home/toast/fixedcamera_radar_multisim_results"
    batch_dir = os.path.join(run_root, batch_id)
    os.makedirs(batch_dir, exist_ok=True)
    shutil.copyfile(cases_file, os.path.join(batch_dir, "cases.csv"))

    with open(cases_file, newline="", encoding="utf-8-sig") as stream:
        cases = list(csv.DictReader(stream))
    case_filter = {
        name.strip() for name in os.environ.get("CASE_FILTER", "").split(",")
        if name.strip()
    }
    for row in cases:
        case_name = row.get("CASE_NAME", "").strip()
        if not case_name:
            continue
        if case_filter and case_name not in case_filter:
            continue
        env = os.environ.copy()
        env["BATCH_ID"] = batch_id
        env["NAV_PROXY_SEED"] = row.get(
            "DISTURBANCE_SEED", "20260928").strip() or "20260928"
        env["SCENARIO_NAME"] = "完整链路_展示验证"
        env["SHOW_GUI"] = os.environ.get("SHOW_GUI", "0")
        env["RUN_WALL_TIMEOUT_S"] = env.get("RUN_WALL_TIMEOUT_S", "140")
        env.update(COMMON_ENV)
        profile = row.get("ERROR_PROFILE", "clean").strip().lower()
        profile_env = PROFILE_ENVS.get(profile)
        if profile_env is None:
            raise ValueError("unknown ERROR_PROFILE: " + profile)
        env.update(profile_env)
        for key, value in row.items():
            if key and value is not None and value.strip() != "":
                env[key.strip()] = value.strip()
        log_path = os.path.join(batch_dir, case_name + "_launcher.log")
        print("=== Running %s ===" % case_name, flush=True)
        with open(log_path, "w", encoding="utf-8") as log:
            subprocess.run(
                ["bash", os.path.join(script_dir, "start_radar_approach_multisim.sh")],
                env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
        with open(log_path, encoding="utf-8", errors="replace") as log:
            sys.stdout.write("".join(log.readlines()[-10:]))
        sys.stdout.flush()

    subprocess.run(
        [sys.executable,
         os.path.join(script_dir, "aggregate_radar_multisim.py"), batch_dir],
        check=True)
    subprocess.run(
        [sys.executable,
         os.path.join(script_dir, "summarize_full_chain_demo.py"), batch_dir],
        check=True)
    with open(os.path.join(run_root, "latest_full_chain_demo_batch.txt"),
              "w", encoding="utf-8") as stream:
        stream.write(batch_dir + "\n")
    print("Full-chain demo batch complete: " + batch_dir)


if __name__ == "__main__":
    main()
