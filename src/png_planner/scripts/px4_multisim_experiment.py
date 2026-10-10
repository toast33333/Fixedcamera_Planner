#!/usr/bin/env python3
"""Two-PX4 fixed-camera pursuit platform-validation experiment.

The node deliberately keeps the production detector interface unchanged.  It
projects Gazebo ground truth into a synthetic camera image and publishes the
same kcf_msgs/Bbox message that the existing controller consumes.  It also
multiplexes takeoff commands and the selected guidance output so that only one
source ever drives the interceptor MAVROS velocity topic.  With
``radar_guidance_enabled`` it also publishes configurable 3 s radar samples and
records the approach-to-IBVS handover diagnostics without changing the visual
message interface.
"""

import csv
import copy
import heapq
import math
import os
import sys
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Vector3Stamped
from kcf_msgs.msg import Bbox
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gauss_markov_disturbance import BoundedRandomWalk, FirstOrderGaussMarkov


def clamp(value, low, high):
    return max(low, min(high, value))


def quat_rotate(q, vector):
    """Rotate vector by geometry_msgs/Quaternion q without extra dependencies."""
    qv = np.array([q.x, q.y, q.z], dtype=float)
    v = np.asarray(vector, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + q.w * t + np.cross(qv, t)


def world_to_body(q, vector):
    class InverseQuaternion:
        pass

    inverse = InverseQuaternion()
    inverse.x = -q.x
    inverse.y = -q.y
    inverse.z = -q.z
    inverse.w = q.w
    return quat_rotate(inverse, vector)


class FixedCameraMultisimExperiment:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.interceptor_model = rospy.get_param("~interceptor_model", "iris_1")
        self.target_model = rospy.get_param("~target_model", "iris_2")
        self.interceptor_ns = rospy.get_param("~interceptor_ns", "/interceptor/mavros")
        self.target_ns = rospy.get_param("~target_ns", "/target/mavros")

        self.fx = float(rospy.get_param("~fx", 640.0))
        self.fy = float(rospy.get_param("~fy", 640.0))
        self.cx = float(rospy.get_param("~cx", 480.0))
        self.cy = float(rospy.get_param("~cy", 270.0))
        self.image_width = int(rospy.get_param("~image_width", 960))
        self.image_height = int(rospy.get_param("~image_height", 540))
        self.mount_pitch_deg = float(rospy.get_param("~camera_mount_pitch_deg", -20.0))
        self.horizontal_limit_deg = float(rospy.get_param("~fov_horizontal_limit_deg", 26.6))
        self.vertical_limit_deg = float(rospy.get_param("~fov_vertical_limit_deg", 20.6))
        self.detection_range_m = float(rospy.get_param("~detection_range_m", 30.0))
        self.gate_detection_until_chase = bool(
            rospy.get_param("~gate_detection_until_chase", False))

        self.interceptor_altitude = float(rospy.get_param("~interceptor_altitude_m", 4.0))
        self.target_altitude = float(rospy.get_param("~target_altitude_m", 9.0))
        self.target_speed = float(rospy.get_param("~target_speed_mps", 2.0))
        self.target_heading_deg = float(rospy.get_param("~target_heading_deg", 0.0))
        self.target_turn_rate_degps = float(rospy.get_param("~target_turn_rate_degps", 0.0))
        self.target_climb_rate = float(rospy.get_param("~target_climb_rate_mps", 0.0))
        self.target_max_vertical_speed = float(
            rospy.get_param("~target_max_vertical_speed_mps", 4.0))
        self.capture_distance = float(rospy.get_param("~capture_distance_m", 7.5))
        self.chase_timeout = float(rospy.get_param("~chase_timeout_s", 45.0))
        self.takeoff_tolerance = float(rospy.get_param("~takeoff_tolerance_m", 0.45))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.results_dir = os.path.expanduser(rospy.get_param("~results_dir", "/tmp/fixed_camera_multisim"))
        self.stop_on_handover = bool(
            rospy.get_param("~stop_on_handover", False))

        self.radar_guidance_enabled = bool(
            rospy.get_param("~radar_guidance_enabled", False))
        self.guidance_command_topic = rospy.get_param(
            "~guidance_command_topic", "/fixed_camera_sim/ibvs_cmd")
        self.radar_topic = rospy.get_param(
            "~radar_topic", "/radar/target/position")
        self.radar_period = max(0.1, float(rospy.get_param("~radar_period_sec", 3.0)))
        self.radar_period_jitter = max(
            0.0, float(rospy.get_param("~radar_period_jitter_std_sec", 0.0)))
        self.radar_noise_std = max(
            0.0, float(rospy.get_param("~radar_noise_std_m", 0.0)))
        self.radar_delay_mean = max(
            0.0, float(rospy.get_param("~radar_delay_mean_sec", 0.0)))
        self.radar_delay_jitter = max(
            0.0, float(rospy.get_param("~radar_delay_jitter_std_sec", 0.0)))
        self.radar_dropout_probability = clamp(
            float(rospy.get_param("~radar_dropout_probability", 0.0)), 0.0, 1.0)
        radar_bias_rw_xy = max(0.0, float(rospy.get_param(
            "~radar_bias_random_walk_xy_m_sqrts", 0.0)))
        radar_bias_rw_z = max(0.0, float(rospy.get_param(
            "~radar_bias_random_walk_z_m_sqrts", radar_bias_rw_xy)))
        radar_bias_limit_xy = max(0.0, float(rospy.get_param(
            "~radar_bias_limit_xy_m", 0.0)))
        radar_bias_limit_z = max(0.0, float(rospy.get_param(
            "~radar_bias_limit_z_m", radar_bias_limit_xy)))
        self.radar_dropout_start = float(
            rospy.get_param("~radar_dropout_start_sec", -1.0))
        self.radar_dropout_duration = max(
            0.0, float(rospy.get_param("~radar_dropout_duration_sec", 0.0)))
        self.radar_acquire_duration = max(
            self.radar_period + 0.5,
            float(rospy.get_param("~radar_acquire_duration_sec", 6.5)))
        self.radar_acquire_timeout = max(
            self.radar_acquire_duration,
            float(rospy.get_param("~radar_acquire_timeout_sec", 15.0)))
        self.result_hold_duration = max(
            0.5, float(rospy.get_param("~result_hold_duration_sec", 2.0)))
        self.radar_seed = int(rospy.get_param("~radar_seed", 1))
        self.radar_rng = np.random.RandomState(self.radar_seed)
        self.radar_timing_rng = np.random.RandomState(
            (self.radar_seed + 1001) % (2 ** 32))
        self.radar_bias_process = BoundedRandomWalk(
            [radar_bias_rw_xy, radar_bias_rw_xy, radar_bias_rw_z],
            self.radar_seed + 1002,
            [radar_bias_limit_xy, radar_bias_limit_xy, radar_bias_limit_z])

        self.disturbance_model = str(
            rospy.get_param("~disturbance_model", "none")).strip().lower()
        if self.disturbance_model not in ("none", "gauss_markov"):
            rospy.logwarn(
                "[FixedCameraSim] Unknown disturbance model '%s'; using none.",
                self.disturbance_model)
            self.disturbance_model = "none"
        self.disturbance_seed = int(rospy.get_param("~disturbance_seed", 20260926))
        disturbance_tau = max(
            0.05, float(rospy.get_param("~disturbance_time_constant_s", 2.0)))
        disturbance_clip_sigma = max(
            0.0, float(rospy.get_param("~disturbance_clip_sigma", 3.0)))
        interceptor_xy_std = max(0.0, float(rospy.get_param(
            "~interceptor_disturbance_std_xy_mps", 0.0)))
        interceptor_z_std = max(0.0, float(rospy.get_param(
            "~interceptor_disturbance_std_z_mps", 0.0)))
        target_xy_std = max(0.0, float(rospy.get_param(
            "~target_disturbance_std_xy_mps", 0.0)))
        target_z_std = max(0.0, float(rospy.get_param(
            "~target_disturbance_std_z_mps", 0.0)))
        visual_std_px = max(
            0.0, float(rospy.get_param("~visual_disturbance_std_px", 0.0)))
        visual_bias_rw_px = max(0.0, float(rospy.get_param(
            "~visual_bias_random_walk_px_sqrts", 0.0)))
        visual_bias_limit_px = max(0.0, float(rospy.get_param(
            "~visual_bias_limit_px", 0.0)))
        self.visual_delay_mean = max(
            0.0, float(rospy.get_param("~visual_delay_mean_sec", 0.0)))
        self.visual_delay_jitter = max(
            0.0, float(rospy.get_param("~visual_delay_jitter_std_sec", 0.0)))
        self.visual_dropout_probability = clamp(
            float(rospy.get_param("~visual_dropout_probability", 0.0)), 0.0, 1.0)
        self.visual_tracking_timeout = max(
            0.02, float(rospy.get_param("~visual_tracking_timeout_sec", 0.30)))
        self.guidance_delay_mean = max(
            0.0, float(rospy.get_param("~guidance_delay_mean_sec", 0.0)))
        self.guidance_delay_jitter = max(
            0.0, float(rospy.get_param("~guidance_delay_jitter_std_sec", 0.0)))
        self.guidance_dropout_probability = clamp(
            float(rospy.get_param("~guidance_dropout_probability", 0.0)), 0.0, 1.0)
        if self.disturbance_model == "none":
            interceptor_xy_std = interceptor_z_std = 0.0
            target_xy_std = target_z_std = visual_std_px = 0.0
        self.interceptor_disturbance_process = FirstOrderGaussMarkov(
            [interceptor_xy_std, interceptor_xy_std, interceptor_z_std],
            disturbance_tau, self.disturbance_seed, disturbance_clip_sigma)
        self.target_disturbance_process = FirstOrderGaussMarkov(
            [target_xy_std, target_xy_std, target_z_std],
            disturbance_tau, self.disturbance_seed + 1, disturbance_clip_sigma)
        self.visual_disturbance_process = FirstOrderGaussMarkov(
            [visual_std_px, visual_std_px], disturbance_tau,
            self.disturbance_seed + 2, disturbance_clip_sigma)
        self.visual_bias_process = BoundedRandomWalk(
            [visual_bias_rw_px, visual_bias_rw_px],
            self.disturbance_seed + 3,
            [visual_bias_limit_px, visual_bias_limit_px])
        self.visual_timing_rng = np.random.RandomState(
            (self.disturbance_seed + 1003) % (2 ** 32))
        self.guidance_timing_rng = np.random.RandomState(
            (self.disturbance_seed + 1004) % (2 ** 32))
        self.interceptor_disturbance = np.zeros(3, dtype=float)
        self.target_disturbance = np.zeros(3, dtype=float)
        self.visual_disturbance = np.zeros(2, dtype=float)
        self.visual_bias = np.zeros(2, dtype=float)
        self.radar_bias = np.zeros(3, dtype=float)
        self.last_disturbance_update = rospy.Time(0)

        self.model_states = None
        self.interceptor_state = State()
        self.target_state = State()
        self.interceptor_pose = None
        self.target_pose = None
        self.ibvs_command = TwistStamped()
        self.nominal_pn_command = TwistStamped()
        self.fov_correction = TwistStamped()
        self.last_ibvs_command_time = rospy.Time(0)
        self.guidance_queue = []
        self.guidance_queue_sequence = 0
        self.guidance_last_due_time = 0.0
        self.guidance_last_delay = 0.0
        self.guidance_dropped_count = 0
        self.radar_state = "DISABLED" if not self.radar_guidance_enabled else "WAIT_RADAR"
        self.handover_complete = False
        self.predicted_target = None
        self.handover_pose = None
        self.estimated_target_velocity = TwistStamped()
        self.last_radar_capture_time = rospy.Time(0)
        self.next_radar_capture_time = rospy.Time(0)
        self.radar_queue = []
        self.radar_queue_sequence = 0
        self.radar_last_delay = 0.0
        self.radar_last_measurement_error = 0.0
        self.radar_dropped_count = 0
        self.radar_sample_count = 0
        self.motion_started = rospy.Time(0)

        self.phase = "WAITING_FOR_LINKS"
        self.phase_started = rospy.Time.now()
        self.chase_started = rospy.Time(0)
        self.last_service_attempt = rospy.Time(0)
        self.success_time = None
        self.tracking = False
        self.pixel_u = float("nan")
        self.pixel_v = float("nan")
        self.horizontal_ratio = float("nan")
        self.vertical_ratio = float("nan")
        self.distance = float("nan")
        self.horizontal_distance = float("nan")
        self.frame_count = 0
        self.visual_queue = []
        self.visual_queue_sequence = 0
        self.visual_last_due_time = 0.0
        self.visual_last_delay = 0.0
        self.visual_dropped_count = 0
        self.last_visual_delivery_time = rospy.Time(0)
        self.last_visual_visible = False

        os.makedirs(self.results_dir, exist_ok=True)
        self.csv_path = os.path.join(self.results_dir, "experiment.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "ros_time_s", "phase", "distance_m", "horizontal_distance_m",
            "pixel_u_offset", "pixel_v_offset", "horizontal_fov_ratio",
            "vertical_fov_ratio", "tracking", "interceptor_x", "interceptor_y",
            "interceptor_z", "target_x", "target_y", "target_z",
            "guidance_vx", "guidance_vy", "guidance_vz", "guidance_yaw_rate",
            "interceptor_vx", "interceptor_vy", "interceptor_vz",
            "target_vx", "target_vy", "target_vz", "radar_state",
            "handover_complete", "radar_samples", "predicted_target_x",
            "predicted_target_y", "predicted_target_z", "handover_x",
            "handover_y", "handover_z", "estimated_target_vx",
            "estimated_target_vy", "estimated_target_vz",
            "nominal_pn_vx", "nominal_pn_vy", "nominal_pn_vz",
            "nominal_pn_yaw_rate", "fov_correction_vx", "fov_correction_vy",
            "fov_correction_vz", "fov_correction_yaw_rate",
            "interceptor_disturbance_vx", "interceptor_disturbance_vy",
            "interceptor_disturbance_vz", "target_disturbance_vx",
            "target_disturbance_vy", "target_disturbance_vz",
            "visual_disturbance_u_px", "visual_disturbance_v_px",
            "radar_bias_x_m", "radar_bias_y_m", "radar_bias_z_m",
            "radar_measurement_error_m", "radar_delay_s",
            "radar_dropped_count", "visual_bias_u_px", "visual_bias_v_px",
            "visual_delay_s", "visual_dropped_count",
            "guidance_delay_s", "guidance_dropped_count"
        ])

        self.interceptor_cmd_pub = rospy.Publisher(
            self.interceptor_ns + "/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.target_cmd_pub = rospy.Publisher(
            self.target_ns + "/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.detection_pub = rospy.Publisher("/object_kcf", Bbox, queue_size=10)
        self.image_pub = rospy.Publisher("/fixed_camera_sim/image", Image, queue_size=2)
        self.status_pub = rospy.Publisher("/fixed_camera_sim/status", String, queue_size=5, latch=True)
        self.distance_pub = rospy.Publisher("/fixed_camera_sim/distance", Float32, queue_size=10)
        self.fov_ratio_pub = rospy.Publisher("/fixed_camera_sim/fov_ratio", Float32, queue_size=10)
        self.marker_pub = rospy.Publisher("/fixed_camera_sim/markers", MarkerArray, queue_size=2)
        self.interceptor_path_pub = rospy.Publisher("/fixed_camera_sim/interceptor_path", Path, queue_size=2)
        self.target_path_pub = rospy.Publisher("/fixed_camera_sim/target_path", Path, queue_size=2)
        self.radar_pub = rospy.Publisher(self.radar_topic, PoseStamped, queue_size=5)
        self.interceptor_disturbance_pub = rospy.Publisher(
            "/fixed_camera_sim/interceptor_velocity_disturbance",
            TwistStamped, queue_size=10)
        self.target_disturbance_pub = rospy.Publisher(
            "/fixed_camera_sim/target_velocity_disturbance",
            TwistStamped, queue_size=10)
        self.visual_disturbance_pub = rospy.Publisher(
            "/fixed_camera_sim/visual_disturbance", Vector3Stamped, queue_size=10)
        self.radar_bias_pub = rospy.Publisher(
            "/fixed_camera_sim/radar_bias", Vector3Stamped, queue_size=10)
        self.visual_bias_pub = rospy.Publisher(
            "/fixed_camera_sim/visual_bias", Vector3Stamped, queue_size=10)
        self.delay_state_pub = rospy.Publisher(
            "/fixed_camera_sim/delay_state", Vector3Stamped, queue_size=10)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber(self.interceptor_ns + "/state", State, self.interceptor_state_cb, queue_size=5)
        rospy.Subscriber(self.target_ns + "/state", State, self.target_state_cb, queue_size=5)
        rospy.Subscriber(self.interceptor_ns + "/local_position/pose", PoseStamped,
                         self.interceptor_pose_cb, queue_size=5)
        rospy.Subscriber(self.target_ns + "/local_position/pose", PoseStamped,
                         self.target_pose_cb, queue_size=5)
        rospy.Subscriber(self.guidance_command_topic, TwistStamped,
                         self.ibvs_command_cb, queue_size=5)
        rospy.Subscriber("/fixed_camera_ibvs/nominal_pn_cmd", TwistStamped,
                         self.nominal_pn_command_cb, queue_size=5)
        rospy.Subscriber("/fixed_camera_ibvs/fov_correction", TwistStamped,
                         self.fov_correction_cb, queue_size=5)
        rospy.Subscriber("/radar_approach/state", String,
                         self.radar_state_cb, queue_size=5)
        rospy.Subscriber("/radar_approach/handover_complete", Bool,
                         self.handover_complete_cb, queue_size=5)
        rospy.Subscriber("/radar_approach/predicted_target", PoseStamped,
                         self.predicted_target_cb, queue_size=5)
        rospy.Subscriber("/radar_approach/handover_pose", PoseStamped,
                         self.handover_pose_cb, queue_size=5)
        rospy.Subscriber("/radar_approach/estimated_target_velocity", TwistStamped,
                         self.estimated_target_velocity_cb, queue_size=5)

        self.interceptor_arm = rospy.ServiceProxy(self.interceptor_ns + "/cmd/arming", CommandBool)
        self.target_arm = rospy.ServiceProxy(self.target_ns + "/cmd/arming", CommandBool)
        self.interceptor_mode = rospy.ServiceProxy(self.interceptor_ns + "/set_mode", SetMode)
        self.target_mode = rospy.ServiceProxy(self.target_ns + "/set_mode", SetMode)

        self.interceptor_path = Path()
        self.interceptor_path.header.frame_id = "world"
        self.target_path = Path()
        self.target_path.header.frame_id = "world"

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "[FixedCameraSim] Experiment manager started; disturbance=%s seed=%d; "
            "radar delay=%.3f+/-%.3f s, visual delay=%.3f+/-%.3f s, "
            "guidance delay=%.3f+/-%.3f s; results: %s",
            self.disturbance_model, self.disturbance_seed,
            self.radar_delay_mean, self.radar_delay_jitter,
            self.visual_delay_mean, self.visual_delay_jitter,
            self.guidance_delay_mean, self.guidance_delay_jitter,
            self.results_dir)

    def model_states_cb(self, message):
        with self.lock:
            self.model_states = message

    def interceptor_state_cb(self, message):
        self.interceptor_state = message

    def target_state_cb(self, message):
        self.target_state = message

    def interceptor_pose_cb(self, message):
        self.interceptor_pose = message

    def target_pose_cb(self, message):
        self.target_pose = message

    def ibvs_command_cb(self, message):
        now = rospy.Time.now()
        if self.guidance_timing_rng.uniform() < self.guidance_dropout_probability:
            self.guidance_dropped_count += 1
            return
        if self.guidance_delay_mean <= 0.0 and self.guidance_delay_jitter <= 0.0:
            self.ibvs_command = copy.deepcopy(message)
            self.last_ibvs_command_time = now
            self.guidance_last_delay = 0.0
            return
        delay = self.sample_nonnegative_delay(
            self.guidance_delay_mean, self.guidance_delay_jitter,
            self.guidance_timing_rng)
        due_time = max(
            now.to_sec() + delay,
            self.guidance_last_due_time + 1.0e-6)
        self.guidance_last_due_time = due_time
        self.guidance_queue_sequence += 1
        heapq.heappush(
            self.guidance_queue,
            (due_time, self.guidance_queue_sequence, now.to_sec(),
             copy.deepcopy(message)))

    def nominal_pn_command_cb(self, message):
        self.nominal_pn_command = message

    def fov_correction_cb(self, message):
        self.fov_correction = message

    def radar_state_cb(self, message):
        self.radar_state = message.data

    def handover_complete_cb(self, message):
        self.handover_complete = bool(message.data)

    def predicted_target_cb(self, message):
        self.predicted_target = message

    def handover_pose_cb(self, message):
        self.handover_pose = message

    def estimated_target_velocity_cb(self, message):
        self.estimated_target_velocity = message

    def set_phase(self, phase):
        if phase == self.phase:
            return
        rospy.loginfo("[FixedCameraSim] Phase: %s -> %s", self.phase, phase)
        self.phase = phase
        self.phase_started = rospy.Time.now()
        if phase == "RADAR_ACQUIRE":
            self.motion_started = self.phase_started
            self.radar_bias_process.reset()
            self.radar_bias.fill(0.0)
            self.last_radar_capture_time = rospy.Time(0)
            self.next_radar_capture_time = self.phase_started
            self.radar_queue = []
        if phase == "CHASE":
            self.chase_started = self.phase_started
            if self.motion_started == rospy.Time(0):
                self.motion_started = self.phase_started
            self.interceptor_disturbance_process.reset()
            self.target_disturbance_process.reset()
            self.visual_disturbance_process.reset()
            self.visual_bias_process.reset()
            self.interceptor_disturbance.fill(0.0)
            self.target_disturbance.fill(0.0)
            self.visual_disturbance.fill(0.0)
            self.visual_bias.fill(0.0)
            self.last_disturbance_update = self.phase_started
            self.visual_queue = []
            self.last_visual_delivery_time = rospy.Time(0)
            self.last_visual_visible = False

    @staticmethod
    def sample_nonnegative_delay(mean, standard_deviation, rng):
        if standard_deviation <= 0.0:
            return max(0.0, mean)
        # Three-sigma clipping keeps a declared "mild" timing model bounded
        # while preserving deterministic jitter for A/B comparisons.
        sample = rng.normal(mean, standard_deviation)
        return clamp(sample, 0.0, mean + 3.0 * standard_deviation)

    def update_guidance_delivery(self, now):
        latest = None
        while self.guidance_queue and self.guidance_queue[0][0] <= now.to_sec():
            latest = heapq.heappop(self.guidance_queue)
        if latest is None:
            return
        _, _, received_time, message = latest
        self.ibvs_command = message
        self.last_ibvs_command_time = now
        self.guidance_last_delay = max(0.0, now.to_sec() - received_time)

    def get_models(self):
        with self.lock:
            states = self.model_states
            if states is None:
                return None
            try:
                i = states.name.index(self.interceptor_model)
                t = states.name.index(self.target_model)
            except ValueError:
                return None
            return states.pose[i], states.twist[i], states.pose[t], states.twist[t]

    def make_command(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        command = TwistStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "map"
        command.twist.linear.x = vx
        command.twist.linear.y = vy
        command.twist.linear.z = vz
        command.twist.angular.z = yaw_rate
        return command

    def update_disturbance(self, now):
        if self.phase != "CHASE":
            self.interceptor_disturbance.fill(0.0)
            self.target_disturbance.fill(0.0)
            self.visual_disturbance.fill(0.0)
            self.visual_bias.fill(0.0)
            self.last_disturbance_update = now
            return
        if self.last_disturbance_update == rospy.Time(0):
            self.last_disturbance_update = now
            return
        dt = clamp((now - self.last_disturbance_update).to_sec(), 0.0, 0.25)
        self.last_disturbance_update = now
        if self.disturbance_model == "gauss_markov":
            self.interceptor_disturbance = self.interceptor_disturbance_process.step(dt)
            self.target_disturbance = self.target_disturbance_process.step(dt)
            self.visual_disturbance = self.visual_disturbance_process.step(dt)
        else:
            self.interceptor_disturbance.fill(0.0)
            self.target_disturbance.fill(0.0)
            self.visual_disturbance.fill(0.0)
        self.visual_bias = self.visual_bias_process.step(dt)

    def add_velocity_disturbance(self, command, disturbance):
        return self.make_command(
            vx=command.twist.linear.x + float(disturbance[0]),
            vy=command.twist.linear.y + float(disturbance[1]),
            vz=command.twist.linear.z + float(disturbance[2]),
            yaw_rate=command.twist.angular.z)

    def publish_disturbance_diagnostics(self, now):
        interceptor = TwistStamped()
        interceptor.header.stamp = now
        interceptor.header.frame_id = "map"
        interceptor.twist.linear.x = float(self.interceptor_disturbance[0])
        interceptor.twist.linear.y = float(self.interceptor_disturbance[1])
        interceptor.twist.linear.z = float(self.interceptor_disturbance[2])
        self.interceptor_disturbance_pub.publish(interceptor)

        target = TwistStamped()
        target.header = interceptor.header
        target.twist.linear.x = float(self.target_disturbance[0])
        target.twist.linear.y = float(self.target_disturbance[1])
        target.twist.linear.z = float(self.target_disturbance[2])
        self.target_disturbance_pub.publish(target)

        visual = Vector3Stamped()
        visual.header.stamp = now
        visual.header.frame_id = "camera_optical"
        visual.vector.x = float(self.visual_disturbance[0])
        visual.vector.y = float(self.visual_disturbance[1])
        self.visual_disturbance_pub.publish(visual)

        radar_bias = Vector3Stamped()
        radar_bias.header.stamp = now
        radar_bias.header.frame_id = "map"
        radar_bias.vector.x = float(self.radar_bias[0])
        radar_bias.vector.y = float(self.radar_bias[1])
        radar_bias.vector.z = float(self.radar_bias[2])
        self.radar_bias_pub.publish(radar_bias)

        visual_bias = Vector3Stamped()
        visual_bias.header.stamp = now
        visual_bias.header.frame_id = "camera_optical"
        visual_bias.vector.x = float(self.visual_bias[0])
        visual_bias.vector.y = float(self.visual_bias[1])
        self.visual_bias_pub.publish(visual_bias)

        delays = Vector3Stamped()
        delays.header.stamp = now
        delays.header.frame_id = "radar_visual_guidance"
        delays.vector.x = self.radar_last_delay
        delays.vector.y = self.visual_last_delay
        delays.vector.z = self.guidance_last_delay
        self.delay_state_pub.publish(delays)

    def altitude_command(self, pose, desired_altitude, vx=0.0):
        current = pose.pose.position.z if pose is not None else 0.0
        vz = clamp(0.9 * (desired_altitude - current), -1.3, 1.3)
        return self.make_command(vx=vx, vz=vz)

    def target_motion_command(self, now):
        """Generate a world-frame target command for the configured case."""
        elapsed = max(0.0, (now - self.motion_started).to_sec())
        heading = math.radians(
            self.target_heading_deg + self.target_turn_rate_degps * elapsed)
        desired_altitude = self.target_altitude + self.target_climb_rate * elapsed
        current_altitude = (
            self.target_pose.pose.position.z if self.target_pose is not None else 0.0)
        vz = self.target_climb_rate + 0.9 * (desired_altitude - current_altitude)
        vz = clamp(vz, -self.target_max_vertical_speed, self.target_max_vertical_speed)
        return self.make_command(
            vx=self.target_speed * math.cos(heading),
            vy=self.target_speed * math.sin(heading),
            vz=vz)

    def radar_dropout_active(self, now):
        if self.radar_dropout_start < 0.0 or self.motion_started == rospy.Time(0):
            return False
        elapsed = (now - self.motion_started).to_sec()
        return (self.radar_dropout_start <= elapsed <
                self.radar_dropout_start + self.radar_dropout_duration)

    def maybe_capture_radar(self, target_pose, now):
        if not self.radar_guidance_enabled or self.phase not in (
                "RADAR_ACQUIRE", "CHASE", "SUCCESS_HOLD", "TIMEOUT_HOLD"):
            return
        if self.radar_dropout_active(now):
            return
        if (self.next_radar_capture_time != rospy.Time(0) and
                now < self.next_radar_capture_time):
            return

        period = self.radar_period
        if self.radar_period_jitter > 0.0:
            period += self.radar_timing_rng.normal(0.0, self.radar_period_jitter)
        period = max(0.1, period)
        self.next_radar_capture_time = now + rospy.Duration(period)
        bias_dt = (
            (now - self.last_radar_capture_time).to_sec()
            if self.last_radar_capture_time != rospy.Time(0) else 0.0)
        self.radar_bias = self.radar_bias_process.step(bias_dt)
        self.last_radar_capture_time = now
        if self.radar_timing_rng.uniform() < self.radar_dropout_probability:
            self.radar_dropped_count += 1
            return

        message = PoseStamped()
        message.header.stamp = now
        message.header.frame_id = "map"
        noise = self.radar_rng.normal(0.0, self.radar_noise_std, size=3)
        measurement_error = noise + self.radar_bias
        message.pose.position.x = target_pose.position.x + float(measurement_error[0])
        message.pose.position.y = target_pose.position.y + float(measurement_error[1])
        message.pose.position.z = target_pose.position.z + float(measurement_error[2])
        message.pose.orientation.w = 1.0
        delay = self.sample_nonnegative_delay(
            self.radar_delay_mean, self.radar_delay_jitter,
            self.radar_timing_rng)
        self.radar_queue_sequence += 1
        heapq.heappush(
            self.radar_queue,
            (now.to_sec() + delay, self.radar_queue_sequence,
             now.to_sec(), float(np.linalg.norm(measurement_error)), message))

    def publish_due_radar(self, now):
        while self.radar_queue and self.radar_queue[0][0] <= now.to_sec():
            _, _, capture_time, measurement_error, message = heapq.heappop(
                self.radar_queue)
            self.radar_pub.publish(message)
            self.radar_last_delay = max(0.0, now.to_sec() - capture_time)
            self.radar_last_measurement_error = measurement_error
            self.radar_sample_count += 1

    def request_offboard_and_arm(self):
        now = rospy.Time.now()
        if (now - self.last_service_attempt).to_sec() < 1.0:
            return
        self.last_service_attempt = now
        pairs = [
            ("interceptor", self.interceptor_state, self.interceptor_mode, self.interceptor_arm),
            ("target", self.target_state, self.target_mode, self.target_arm),
        ]
        for name, state, mode_service, arm_service in pairs:
            try:
                if state.mode != "OFFBOARD":
                    mode_service(base_mode=0, custom_mode="OFFBOARD")
                if not state.armed:
                    arm_service(True)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(2.0, "[FixedCameraSim] %s service call failed: %s", name, exc)

    def project_target(self, interceptor_pose, target_pose):
        delta_world = np.array([
            target_pose.position.x - interceptor_pose.position.x,
            target_pose.position.y - interceptor_pose.position.y,
            target_pose.position.z - interceptor_pose.position.z,
        ], dtype=float)
        distance = float(np.linalg.norm(delta_world))
        horizontal_distance = float(np.linalg.norm(delta_world[:2]))
        if distance < 1.0e-6:
            return distance, horizontal_distance, False, float("nan"), float("nan"), 99.0, 99.0

        body = world_to_body(interceptor_pose.orientation, delta_world / distance)
        angle = math.radians(-self.mount_pitch_deg)
        nominal_x = math.cos(angle) * body[0] + math.sin(angle) * body[2]
        nominal_y = body[1]
        nominal_z = -math.sin(angle) * body[0] + math.cos(angle) * body[2]
        camera_x = -nominal_y
        camera_y = -nominal_z
        camera_z = nominal_x

        horizontal_angle = math.atan2(camera_x, camera_z)
        vertical_angle = math.atan2(camera_y, camera_z)
        horizontal_ratio = abs(math.degrees(horizontal_angle)) / self.horizontal_limit_deg
        vertical_ratio = abs(math.degrees(vertical_angle)) / self.vertical_limit_deg
        visible = (camera_z > 0.0 and distance <= self.detection_range_m and
                   horizontal_ratio <= 1.0 and vertical_ratio <= 1.0)
        if camera_z <= 1.0e-6:
            return distance, horizontal_distance, False, float("nan"), float("nan"), horizontal_ratio, vertical_ratio

        # Existing controller uses centered offsets with positive v pointing up.
        u_offset = self.fx * camera_x / camera_z
        v_offset = -self.fy * camera_y / camera_z
        return distance, horizontal_distance, visible, u_offset, v_offset, horizontal_ratio, vertical_ratio

    def apply_visual_disturbance(self, projection):
        if self.phase != "CHASE":
            return projection
        distance, horizontal_distance, visible, u_offset, v_offset, h_ratio, v_ratio = projection
        if not visible or not (math.isfinite(u_offset) and math.isfinite(v_offset)):
            return projection
        u_offset += float(self.visual_disturbance[0] + self.visual_bias[0])
        v_offset += float(self.visual_disturbance[1] + self.visual_bias[1])
        measured_h_ratio = abs(math.degrees(math.atan2(u_offset, self.fx))) / self.horizontal_limit_deg
        measured_v_ratio = abs(math.degrees(math.atan2(v_offset, self.fy))) / self.vertical_limit_deg
        visible = visible and measured_h_ratio <= 1.0 and measured_v_ratio <= 1.0
        return (
            distance, horizontal_distance, visible,
            u_offset, v_offset, measured_h_ratio, measured_v_ratio)

    def enqueue_visual_measurement(self, projection, now):
        if self.visual_timing_rng.uniform() < self.visual_dropout_probability:
            self.visual_dropped_count += 1
            return
        delay = self.sample_nonnegative_delay(
            self.visual_delay_mean, self.visual_delay_jitter,
            self.visual_timing_rng)
        due_time = max(
            now.to_sec() + delay,
            self.visual_last_due_time + 1.0e-6)
        self.visual_last_due_time = due_time
        self.visual_queue_sequence += 1
        heapq.heappush(
            self.visual_queue,
            (due_time, self.visual_queue_sequence, now.to_sec(), projection))

    def publish_due_visual_measurement(self, now):
        latest = None
        while self.visual_queue and self.visual_queue[0][0] <= now.to_sec():
            latest = heapq.heappop(self.visual_queue)
        if latest is not None:
            _, _, capture_time, projection = latest
            self.visual_last_delay = max(0.0, now.to_sec() - capture_time)
            self.publish_detection_and_image(projection, now)
        if self.last_visual_delivery_time == rospy.Time(0):
            self.tracking = False
        elif (now - self.last_visual_delivery_time).to_sec() > self.visual_tracking_timeout:
            self.tracking = False

    def detection_enabled_for_control(self, now):
        if not self.gate_detection_until_chase:
            return True
        if self.phase in ("CHASE", "SUCCESS_HOLD", "TIMEOUT_HOLD"):
            return True
        # Enable the first measurement on the transition tick so the first
        # logged CHASE sample is a real visual-control sample, while avoiding
        # integrator accumulation throughout takeoff and stabilization.
        return (self.phase == "STABILIZE" and not self.radar_guidance_enabled and
                (now - self.phase_started).to_sec() >= 3.0)

    def publish_detection_and_image(self, projection, now):
        distance, horizontal_distance, raw_visible, u_offset, v_offset, h_ratio, v_ratio = projection
        visible = raw_visible and self.detection_enabled_for_control(now)
        self.tracking = visible
        self.last_visual_visible = visible
        self.last_visual_delivery_time = now
        self.pixel_u = u_offset
        self.pixel_v = v_offset

        box = Bbox()
        if visible:
            half_size = clamp(180.0 / max(distance, 1.0), 8.0, 28.0)
            box.x1 = u_offset - half_size
            box.x2 = u_offset + half_size
            box.y1 = v_offset - half_size
            box.y2 = v_offset + half_size
            box.score = 0.98
            box.class_name = "target_uav"
            box.track_id = 1
            box.is_tracking = True
        else:
            box.score = 0.0
            box.class_name = "target_uav"
            box.track_id = 1
            box.is_tracking = False
        self.detection_pub.publish(box)

        image = np.full((self.image_height, self.image_width, 3), 245, dtype=np.uint8)
        safe_half_width = int(round(self.fx * math.tan(math.radians(self.horizontal_limit_deg))))
        safe_half_height = int(round(self.fy * math.tan(math.radians(self.vertical_limit_deg))))
        top_left = (int(self.cx) - safe_half_width, int(self.cy) - safe_half_height)
        bottom_right = (int(self.cx) + safe_half_width, int(self.cy) + safe_half_height)
        cv2.rectangle(image, top_left, bottom_right, (30, 30, 30), 2)
        cv2.line(image, (int(self.cx) - 14, int(self.cy)), (int(self.cx) + 14, int(self.cy)), (120, 120, 120), 1)
        cv2.line(image, (int(self.cx), int(self.cy) - 14), (int(self.cx), int(self.cy) + 14), (120, 120, 120), 1)
        if raw_visible:
            pixel_x = int(round(self.cx + u_offset))
            pixel_y = int(round(self.cy - v_offset))
            cv2.circle(image, (pixel_x, pixel_y), 13, (0, 0, 220), 3)
            cv2.putText(image, "TARGET", (pixel_x + 18, pixel_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 170), 2, cv2.LINE_AA)
        cv2.putText(image, "Fixed camera: +20 deg | safe FOV: +/-26.6 x +/-20.6 deg",
                    (22, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(image, "Phase: %s   Range: %.1f m   Tracking: %s" %
                    (self.phase, distance, "YES" if visible else "NO"),
                    (22, self.image_height - 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (0, 120, 0) if visible else (0, 0, 180), 2, cv2.LINE_AA)
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_message.header.stamp = now
        image_message.header.frame_id = "camera_optical"
        self.image_pub.publish(image_message)

    def append_path_pose(self, path, pose):
        stamped = PoseStamped()
        stamped.header.stamp = rospy.Time.now()
        stamped.header.frame_id = "world"
        stamped.pose = pose
        path.header.stamp = stamped.header.stamp
        path.poses.append(stamped)
        if len(path.poses) > 1600:
            path.poses = path.poses[-1600:]

    def publish_visualization(self, interceptor_pose, target_pose):
        self.frame_count += 1
        if self.frame_count % 4 == 0:
            self.append_path_pose(self.interceptor_path, interceptor_pose)
            self.append_path_pose(self.target_path, target_pose)
            self.interceptor_path_pub.publish(self.interceptor_path)
            self.target_path_pub.publish(self.target_path)

        markers = MarkerArray()
        for marker_id, pose, name, color in [
                (0, interceptor_pose, "INTERCEPTOR", (0.10, 0.35, 0.95)),
                (1, target_pose, "TARGET", (0.90, 0.15, 0.10))]:
            sphere = Marker()
            sphere.header.frame_id = "world"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "aircraft"
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.2
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 1.0
            markers.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns = "labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose
            label.pose.position.z += 1.2
            label.scale.z = 0.75
            label.color.r = label.color.g = label.color.b = 0.05
            label.color.a = 1.0
            label.text = name
            markers.markers.append(label)

        line = Marker()
        line.header.frame_id = "world"
        line.header.stamp = rospy.Time.now()
        line.ns = "line_of_sight"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.12
        line.color.r = 0.15
        line.color.g = 0.15
        line.color.b = 0.15
        line.color.a = 0.85
        line.points = [Point(x=interceptor_pose.position.x, y=interceptor_pose.position.y,
                             z=interceptor_pose.position.z),
                       Point(x=target_pose.position.x, y=target_pose.position.y,
                             z=target_pose.position.z)]
        markers.markers.append(line)
        self.marker_pub.publish(markers)

    def links_ready(self, models):
        return (models is not None and self.interceptor_state.connected and
                self.target_state.connected and self.interceptor_pose is not None and
                self.target_pose is not None)

    def control_tick(self, _event):
        models = self.get_models()
        now = rospy.Time.now()
        self.update_disturbance(now)
        self.update_guidance_delivery(now)
        interceptor_command = self.make_command()
        target_command = self.make_command()

        if models is not None:
            interceptor_model_pose, _, target_model_pose, _ = models
            truth_projection = self.project_target(
                interceptor_model_pose, target_model_pose)
            self.distance = truth_projection[0]
            self.horizontal_distance = truth_projection[1]
            self.horizontal_ratio = truth_projection[5]
            self.vertical_ratio = truth_projection[6]
            sensor_projection = self.apply_visual_disturbance(truth_projection)
            self.enqueue_visual_measurement(sensor_projection, now)
            self.publish_due_visual_measurement(now)
            self.publish_visualization(interceptor_model_pose, target_model_pose)
            self.distance_pub.publish(Float32(data=self.distance))
            self.fov_ratio_pub.publish(Float32(data=max(
                self.horizontal_ratio, self.vertical_ratio)))
            self.maybe_capture_radar(target_model_pose, now)
            self.publish_due_radar(now)
        else:
            self.publish_due_visual_measurement(now)
            self.publish_due_radar(now)

        if self.phase == "WAITING_FOR_LINKS":
            if self.links_ready(models):
                self.set_phase("SETPOINT_WARMUP")

        elif self.phase == "SETPOINT_WARMUP":
            if (now - self.phase_started).to_sec() >= 3.0:
                self.request_offboard_and_arm()
                if (self.interceptor_state.armed and self.target_state.armed and
                        self.interceptor_state.mode == "OFFBOARD" and
                        self.target_state.mode == "OFFBOARD"):
                    self.set_phase("TAKEOFF")

        elif self.phase == "TAKEOFF":
            self.request_offboard_and_arm()
            interceptor_command = self.altitude_command(self.interceptor_pose, self.interceptor_altitude)
            target_command = self.altitude_command(self.target_pose, self.target_altitude)
            interceptor_error = abs(self.interceptor_pose.pose.position.z - self.interceptor_altitude)
            target_error = abs(self.target_pose.pose.position.z - self.target_altitude)
            if (interceptor_error <= self.takeoff_tolerance and
                    target_error <= self.takeoff_tolerance and
                    (now - self.phase_started).to_sec() >= 4.0):
                self.set_phase("STABILIZE")

        elif self.phase == "STABILIZE":
            self.request_offboard_and_arm()
            interceptor_command = self.altitude_command(self.interceptor_pose, self.interceptor_altitude)
            target_command = self.altitude_command(self.target_pose, self.target_altitude)
            if (now - self.phase_started).to_sec() >= 3.0:
                self.set_phase("RADAR_ACQUIRE" if self.radar_guidance_enabled else "CHASE")

        elif self.phase == "RADAR_ACQUIRE":
            self.request_offboard_and_arm()
            interceptor_command = self.altitude_command(
                self.interceptor_pose, self.interceptor_altitude)
            target_command = self.target_motion_command(now)
            elapsed = (now - self.phase_started).to_sec()
            if (elapsed >= self.radar_acquire_duration and
                    self.radar_state == "APPROACH"):
                self.set_phase("CHASE")
            elif elapsed >= self.radar_acquire_timeout:
                self.set_phase("TIMEOUT_HOLD")
                rospy.logwarn("[FixedCameraSim] Radar acquisition timed out")

        elif self.phase == "CHASE":
            self.request_offboard_and_arm()
            target_command = self.add_velocity_disturbance(
                self.target_motion_command(now), self.target_disturbance)
            if (now - self.last_ibvs_command_time).to_sec() < 0.35:
                interceptor_command = self.add_velocity_disturbance(
                    self.ibvs_command, self.interceptor_disturbance)
            else:
                interceptor_command = self.add_velocity_disturbance(
                    self.make_command(), self.interceptor_disturbance)

            if self.stop_on_handover and self.handover_complete:
                self.success_time = (now - self.chase_started).to_sec()
                self.set_phase("HANDOVER_HOLD")
                rospy.loginfo(
                    "[FixedCameraSim] APPROACH SUCCESS: visual handover conditions "
                    "reached in %.2f s at %.2f m",
                    self.success_time, self.distance)
            elif math.isfinite(self.distance) and self.distance <= self.capture_distance:
                self.success_time = (now - self.chase_started).to_sec()
                self.set_phase("SUCCESS_HOLD")
                rospy.loginfo("[FixedCameraSim] SUCCESS: capture radius reached in %.2f s at %.2f m",
                              self.success_time, self.distance)
            elif (now - self.chase_started).to_sec() > self.chase_timeout:
                self.set_phase("TIMEOUT_HOLD")
                rospy.logwarn("[FixedCameraSim] Chase timeout; minimum result is retained for analysis")

        elif self.phase in ("HANDOVER_HOLD", "SUCCESS_HOLD", "TIMEOUT_HOLD"):
            self.request_offboard_and_arm()
            interceptor_hold_altitude = self.interceptor_pose.pose.position.z
            target_hold_altitude = self.target_pose.pose.position.z
            interceptor_command = self.altitude_command(self.interceptor_pose, interceptor_hold_altitude)
            target_command = self.altitude_command(self.target_pose, target_hold_altitude)
            if (now - self.phase_started).to_sec() >= self.result_hold_duration:
                rospy.signal_shutdown("experiment complete: " + self.phase)

        self.interceptor_cmd_pub.publish(interceptor_command)
        self.target_cmd_pub.publish(target_command)
        self.publish_disturbance_diagnostics(now)
        self.publish_status()
        if models is not None:
            self.write_csv(models)

    def publish_status(self):
        status = ("phase=%s | distance=%.2f m | horizontal=%.2f m | target_visible=%s | "
                  "pixel=(%.1f, %.1f) | fov_ratio=(%.2f, %.2f) | "
                  "guidance=%s | handover=%s | radar_samples=%d | disturbance=%s | "
                  "delay_r/v/c=(%.3f/%.3f/%.3f)s | "
                  "interceptor=%s/%s | target=%s/%s" %
                  (self.phase, self.distance, self.horizontal_distance, self.tracking,
                   self.pixel_u, self.pixel_v, self.horizontal_ratio, self.vertical_ratio,
                   self.radar_state, self.handover_complete, self.radar_sample_count,
                   self.disturbance_model,
                   self.radar_last_delay, self.visual_last_delay,
                   self.guidance_last_delay,
                   self.interceptor_state.mode, self.interceptor_state.armed,
                   self.target_state.mode, self.target_state.armed))
        self.status_pub.publish(String(data=status))
        rospy.loginfo_throttle(2.0, "[FixedCameraSim] %s", status)

    def write_csv(self, models):
        interceptor_pose, interceptor_twist, target_pose, target_twist = models
        command = self.ibvs_command.twist

        def pose_values(message):
            if message is None:
                return (float("nan"),) * 3
            return (message.pose.position.x, message.pose.position.y,
                    message.pose.position.z)

        predicted = pose_values(self.predicted_target)
        handover = pose_values(self.handover_pose)
        estimated_velocity = self.estimated_target_velocity.twist.linear
        nominal = self.nominal_pn_command.twist
        correction = self.fov_correction.twist
        self.csv_writer.writerow([
            "%.6f" % rospy.Time.now().to_sec(), self.phase,
            "%.6f" % self.distance, "%.6f" % self.horizontal_distance,
            "%.6f" % self.pixel_u, "%.6f" % self.pixel_v,
            "%.6f" % self.horizontal_ratio, "%.6f" % self.vertical_ratio,
            int(self.tracking),
            "%.6f" % interceptor_pose.position.x, "%.6f" % interceptor_pose.position.y,
            "%.6f" % interceptor_pose.position.z, "%.6f" % target_pose.position.x,
            "%.6f" % target_pose.position.y, "%.6f" % target_pose.position.z,
            "%.6f" % command.linear.x, "%.6f" % command.linear.y,
            "%.6f" % command.linear.z, "%.6f" % command.angular.z,
            "%.6f" % interceptor_twist.linear.x,
            "%.6f" % interceptor_twist.linear.y,
            "%.6f" % interceptor_twist.linear.z,
            "%.6f" % target_twist.linear.x,
            "%.6f" % target_twist.linear.y,
            "%.6f" % target_twist.linear.z,
            self.radar_state, int(self.handover_complete), self.radar_sample_count,
            "%.6f" % predicted[0], "%.6f" % predicted[1], "%.6f" % predicted[2],
            "%.6f" % handover[0], "%.6f" % handover[1], "%.6f" % handover[2],
            "%.6f" % estimated_velocity.x, "%.6f" % estimated_velocity.y,
            "%.6f" % estimated_velocity.z,
            "%.6f" % nominal.linear.x, "%.6f" % nominal.linear.y,
            "%.6f" % nominal.linear.z, "%.6f" % nominal.angular.z,
            "%.6f" % correction.linear.x, "%.6f" % correction.linear.y,
            "%.6f" % correction.linear.z, "%.6f" % correction.angular.z,
            "%.6f" % self.interceptor_disturbance[0],
            "%.6f" % self.interceptor_disturbance[1],
            "%.6f" % self.interceptor_disturbance[2],
            "%.6f" % self.target_disturbance[0],
            "%.6f" % self.target_disturbance[1],
            "%.6f" % self.target_disturbance[2],
            "%.6f" % self.visual_disturbance[0],
            "%.6f" % self.visual_disturbance[1],
            "%.6f" % self.radar_bias[0],
            "%.6f" % self.radar_bias[1],
            "%.6f" % self.radar_bias[2],
            "%.6f" % self.radar_last_measurement_error,
            "%.6f" % self.radar_last_delay,
            self.radar_dropped_count,
            "%.6f" % self.visual_bias[0],
            "%.6f" % self.visual_bias[1],
            "%.6f" % self.visual_last_delay,
            self.visual_dropped_count,
            "%.6f" % self.guidance_last_delay,
            self.guidance_dropped_count,
        ])
        if self.frame_count % 20 == 0:
            self.csv_file.flush()

    def shutdown(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("px4_multisim_experiment")
    FixedCameraMultisimExperiment()
    rospy.spin()
