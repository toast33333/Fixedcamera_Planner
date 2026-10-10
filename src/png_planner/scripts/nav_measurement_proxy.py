#!/usr/bin/env python3
"""Inject navigation-measurement impairments without modifying MAVROS/PX4.

The proxy keeps each source message's header (especially the acquisition
timestamp) unchanged.  A delayed message is therefore published later while
still describing when the measurement was acquired.  This is important for
the fixed-camera controller's attitude-history interpolation.

All vector-valued parameters accept either one scalar (applied to x/y/z) or a
three-element list.  Random-walk densities use the customary ``unit/sqrt(s)``
convention and are protected by configurable absolute bounds.
"""

import copy
import heapq
import math
import os
import sys
import threading

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gauss_markov_disturbance import BoundedRandomWalk

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import Imu
except ImportError:  # Allow the deterministic model helpers to be unit-tested off ROS.
    rospy = None
    PoseStamped = None
    TwistStamped = None
    Imu = None


def vector3_parameter(value, name):
    """Return a non-negative three-vector from a scalar or length-three value."""
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.repeat(float(array), 3)
    if array.shape != (3,):
        raise ValueError("{} must be a scalar or a three-element vector".format(name))
    if not np.all(np.isfinite(array)):
        raise ValueError("{} must contain finite values".format(name))
    return np.maximum(array, 0.0)


def quaternion_multiply(left, right):
    """Hamilton product for quaternions stored as [x, y, z, w]."""
    lx, ly, lz, lw = np.asarray(left, dtype=float)
    rx, ry, rz, rw = np.asarray(right, dtype=float)
    return np.asarray([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def apply_rotation_vector(quaternion_xyzw, rotation_vector_rad):
    """Apply a body-frame rotation-vector error and return a unit quaternion."""
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12 or not np.isfinite(norm):
        return quaternion.copy()
    quaternion /= norm

    rotation = np.asarray(rotation_vector_rad, dtype=float)
    angle = float(np.linalg.norm(rotation))
    if angle <= 1.0e-12:
        delta = np.asarray([0.5 * rotation[0], 0.5 * rotation[1],
                            0.5 * rotation[2], 1.0])
    else:
        scale = math.sin(0.5 * angle) / angle
        delta = np.asarray([scale * rotation[0], scale * rotation[1],
                            scale * rotation[2], math.cos(0.5 * angle)])
    result = quaternion_multiply(quaternion, delta)
    result_norm = float(np.linalg.norm(result))
    return result / result_norm if result_norm > 1.0e-12 else quaternion


class VectorErrorModel:
    """White noise plus bounded cumulative bias for one three-vector."""

    def __init__(self, white_std, walk_density, bias_limit, seed):
        self.white_std = vector3_parameter(white_std, "white_std")
        density = vector3_parameter(walk_density, "walk_density")
        limit = vector3_parameter(bias_limit, "bias_limit")
        self.bias = BoundedRandomWalk(density, seed + 1, limit)
        self._rng = np.random.RandomState(int(seed) % (2 ** 32))
        self._last_stamp_s = None

    def sample_error(self, stamp_s):
        stamp_s = float(stamp_s)
        if self._last_stamp_s is None:
            dt_s = 0.0
        else:
            dt_s = max(0.0, stamp_s - self._last_stamp_s)
        if self._last_stamp_s is None or stamp_s >= self._last_stamp_s:
            self._last_stamp_s = stamp_s
        bias = self.bias.step(dt_s)
        white = self.white_std * self._rng.normal(0.0, 1.0, size=3)
        return bias + white


class StreamTransportModel:
    """Deterministic delay/dropout sampler with non-reordering due times."""

    def __init__(self, delay_mean_s, delay_jitter_std_s,
                 dropout_probability, seed):
        self.delay_mean_s = max(float(delay_mean_s), 0.0)
        self.delay_jitter_std_s = max(float(delay_jitter_std_s), 0.0)
        self.dropout_probability = min(
            max(float(dropout_probability), 0.0), 1.0)
        self._rng = np.random.RandomState(int(seed) % (2 ** 32))
        self._last_due_s = -float("inf")

    def should_drop(self):
        return bool(self._rng.uniform(0.0, 1.0) < self.dropout_probability)

    def due_time(self, receipt_time_s):
        jitter = self._rng.normal(0.0, self.delay_jitter_std_s)
        delay_s = max(0.0, self.delay_mean_s + jitter)
        due_s = max(float(receipt_time_s) + delay_s, self._last_due_s + 1.0e-9)
        self._last_due_s = due_s
        return due_s


class NavigationMeasurementProxy:
    def __init__(self):
        if rospy is None:
            raise RuntimeError("ROS Python modules are required to run this node")

        seed = int(rospy.get_param("~seed", 20260928))
        self._lock = threading.Lock()
        self._model_lock = threading.Lock()
        self._pending = []
        self._sequence = 0

        self._position_error = self._make_vector_model(
            "pose_position", "m", seed + 10)
        self._velocity_error = self._make_vector_model(
            "velocity", "mps", seed + 20)
        degrees_to_radians = math.pi / 180.0
        self._attitude_error = VectorErrorModel(
            vector3_parameter(rospy.get_param(
                "~attitude_white_noise_std_deg", 0.0),
                "attitude_white_noise_std_deg") * degrees_to_radians,
            vector3_parameter(rospy.get_param(
                "~attitude_bias_random_walk_deg_sqrts", 0.0),
                "attitude_bias_random_walk_deg_sqrts") * degrees_to_radians,
            vector3_parameter(rospy.get_param(
                "~attitude_bias_limit_deg", 0.0),
                "attitude_bias_limit_deg") * degrees_to_radians,
            seed + 30)
        self._gyro_error = self._make_vector_model(
            "gyro", "radps", seed + 40)
        self._accel_error = self._make_vector_model(
            "accel", "mps2", seed + 50)

        self._pose_transport = self._make_transport("pose", seed + 110)
        self._velocity_transport = self._make_transport("velocity", seed + 120)
        self._imu_transport = self._make_transport("imu", seed + 130)

        pose_input = rospy.get_param(
            "~pose_input_topic", "/interceptor/mavros/local_position/pose")
        velocity_input = rospy.get_param(
            "~velocity_input_topic",
            "/interceptor/mavros/local_position/velocity_local")
        imu_input = rospy.get_param(
            "~imu_input_topic", "/interceptor/mavros/imu/data")
        pose_output = rospy.get_param(
            "~pose_output_topic", "/fixed_camera_sim/nav/pose")
        velocity_output = rospy.get_param(
            "~velocity_output_topic", "/fixed_camera_sim/nav/velocity")
        imu_output = rospy.get_param(
            "~imu_output_topic", "/fixed_camera_sim/nav/imu")

        self._pose_pub = rospy.Publisher(pose_output, PoseStamped, queue_size=20)
        self._velocity_pub = rospy.Publisher(
            velocity_output, TwistStamped, queue_size=20)
        self._imu_pub = rospy.Publisher(imu_output, Imu, queue_size=100)
        self._pose_sub = rospy.Subscriber(
            pose_input, PoseStamped, self._pose_callback, queue_size=20)
        self._velocity_sub = rospy.Subscriber(
            velocity_input, TwistStamped, self._velocity_callback, queue_size=20)
        self._imu_sub = rospy.Subscriber(
            imu_input, Imu, self._imu_callback, queue_size=100)

        publish_rate_hz = max(float(rospy.get_param(
            "~publish_rate_hz", 500.0)), 20.0)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate_hz), self._publish_due)
        rospy.loginfo(
            "Navigation measurement proxy: pose=%s velocity=%s imu=%s; "
            "source headers are preserved", pose_output, velocity_output, imu_output)

    @staticmethod
    def _stamp_seconds(message):
        if not message.header.stamp.is_zero():
            return message.header.stamp.to_sec()
        return rospy.Time.now().to_sec()

    @staticmethod
    def _vector(message_vector):
        return np.asarray([message_vector.x, message_vector.y, message_vector.z])

    @staticmethod
    def _set_vector(message_vector, value):
        message_vector.x = float(value[0])
        message_vector.y = float(value[1])
        message_vector.z = float(value[2])

    @staticmethod
    def _set_quaternion(message_quaternion, value):
        message_quaternion.x = float(value[0])
        message_quaternion.y = float(value[1])
        message_quaternion.z = float(value[2])
        message_quaternion.w = float(value[3])

    def _make_vector_model(self, prefix, unit_suffix, seed):
        return VectorErrorModel(
            rospy.get_param("~{}_white_noise_std_{}".format(
                prefix, unit_suffix), 0.0),
            rospy.get_param("~{}_bias_random_walk_{}_sqrts".format(
                prefix, unit_suffix), 0.0),
            rospy.get_param("~{}_bias_limit_{}".format(
                prefix, unit_suffix), 0.0),
            seed)

    @staticmethod
    def _make_transport(prefix, seed):
        return StreamTransportModel(
            rospy.get_param("~{}_delay_mean_sec".format(prefix), 0.0),
            rospy.get_param("~{}_delay_jitter_std_sec".format(prefix), 0.0),
            rospy.get_param("~{}_dropout_probability".format(prefix), 0.0),
            seed)

    def _schedule(self, publisher, message, transport):
        if transport.should_drop():
            return
        now_s = rospy.Time.now().to_sec()
        due_s = transport.due_time(now_s)
        with self._lock:
            self._sequence += 1
            heapq.heappush(
                self._pending, (due_s, self._sequence, publisher, message))
        # Avoid adding one timer period in the zero-delay baseline.
        if due_s <= now_s + 1.0e-9:
            self._publish_due(None)

    def _pose_callback(self, source):
        message = copy.deepcopy(source)
        stamp_s = self._stamp_seconds(message)
        position = self._vector(message.pose.position)
        with self._model_lock:
            position_error = self._position_error.sample_error(stamp_s)
            attitude_error = self._attitude_error.sample_error(stamp_s)
        self._set_vector(
            message.pose.position,
            position + position_error)
        quaternion = np.asarray([
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w])
        self._set_quaternion(
            message.pose.orientation,
            apply_rotation_vector(quaternion, attitude_error))
        self._schedule(self._pose_pub, message, self._pose_transport)

    def _velocity_callback(self, source):
        message = copy.deepcopy(source)
        stamp_s = self._stamp_seconds(message)
        velocity = self._vector(message.twist.linear)
        with self._model_lock:
            velocity_error = self._velocity_error.sample_error(stamp_s)
        self._set_vector(
            message.twist.linear,
            velocity + velocity_error)
        self._schedule(self._velocity_pub, message, self._velocity_transport)

    def _imu_callback(self, source):
        message = copy.deepcopy(source)
        stamp_s = self._stamp_seconds(message)
        with self._model_lock:
            attitude_error = self._attitude_error.sample_error(stamp_s)
            gyro_error = self._gyro_error.sample_error(stamp_s)
            accel_error = self._accel_error.sample_error(stamp_s)
        quaternion = np.asarray([
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w])
        self._set_quaternion(
            message.orientation,
            apply_rotation_vector(quaternion, attitude_error))
        self._set_vector(
            message.angular_velocity,
            self._vector(message.angular_velocity) + gyro_error)
        self._set_vector(
            message.linear_acceleration,
            self._vector(message.linear_acceleration) + accel_error)
        self._schedule(self._imu_pub, message, self._imu_transport)

    def _publish_due(self, _event):
        due_messages = []
        now_s = rospy.Time.now().to_sec()
        with self._lock:
            while self._pending and self._pending[0][0] <= now_s:
                due_messages.append(heapq.heappop(self._pending))
        for _, _, publisher, message in due_messages:
            publisher.publish(message)


def main():
    if rospy is None:
        raise RuntimeError("ROS Python modules are required to run this node")
    rospy.init_node("nav_measurement_proxy")
    NavigationMeasurementProxy()
    rospy.spin()


if __name__ == "__main__":
    main()
