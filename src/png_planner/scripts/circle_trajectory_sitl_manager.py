#!/usr/bin/env python3
"""Prepare one PX4 SITL vehicle, then hand command ownership to the circle test."""

import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamSet, SetMode
from std_msgs.msg import Bool, String


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class CircleTrajectorySITLManager:
    def __init__(self):
        self.namespace = rospy.get_param("~mavros_namespace", "/interceptor/mavros").rstrip("/")
        self.pose_topic = rospy.get_param(
            "~pose_topic", self.namespace + "/local_position/pose")
        self.command_topic = rospy.get_param(
            "~command_topic", self.namespace + "/setpoint_velocity/cmd_vel")
        self.start_topic = rospy.get_param("~start_topic", "/circle_trajectory/start")
        self.finished_topic = rospy.get_param(
            "~finished_topic", "/circle_trajectory/finished")
        self.status_topic = rospy.get_param(
            "~status_topic", "/circle_trajectory/sitl_manager_status")
        self.target_altitude = float(rospy.get_param("~target_altitude_m", 5.0))
        self.takeoff_tolerance = float(rospy.get_param("~takeoff_tolerance_m", 0.35))
        self.warmup_duration = float(rospy.get_param("~warmup_duration_s", 3.0))
        self.stabilize_duration = float(rospy.get_param("~stabilize_duration_s", 3.0))
        self.xy_velocity_limit = float(rospy.get_param("~xy_velocity_limit_mps", 20.0))
        self.rate = max(10.0, float(rospy.get_param("~rate", 20.0)))

        self.lock = threading.Lock()
        self.state = State()
        self.pose = None
        self.phase = "WAITING_FOR_LINK"
        self.phase_started = rospy.Time.now()
        self.last_service_attempt = rospy.Time(0)
        self.limit_configured = False
        self.takeoff_reached_at = None
        self.finished = False

        self.command_pub = rospy.Publisher(self.command_topic, TwistStamped, queue_size=1)
        self.start_pub = rospy.Publisher(self.start_topic, Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.namespace + "/state", State, self.state_callback, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber(self.finished_topic, Bool, self.finished_callback, queue_size=1)

        self.arm_service = rospy.ServiceProxy(self.namespace + "/cmd/arming", CommandBool)
        self.mode_service = rospy.ServiceProxy(self.namespace + "/set_mode", SetMode)
        self.param_service = rospy.ServiceProxy(self.namespace + "/param/set", ParamSet)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_callback)
        rospy.on_shutdown(self.shutdown)
        self.publish_status()

    def state_callback(self, message):
        with self.lock:
            self.state = message

    def pose_callback(self, message):
        with self.lock:
            self.pose = message

    def finished_callback(self, message):
        if message.data:
            self.finished = True
            self.set_phase("COMPLETE")

    def set_phase(self, phase):
        if phase == self.phase:
            return
        rospy.loginfo("[CircleSITLManager] Phase %s -> %s", self.phase, phase)
        self.phase = phase
        self.phase_started = rospy.Time.now()
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=self.phase))

    def make_command(self, vz=0.0):
        command = TwistStamped()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "map"
        command.twist.linear.z = vz
        return command

    def publish_altitude_command(self, pose):
        error = self.target_altitude - pose.pose.position.z
        self.command_pub.publish(self.make_command(clamp(1.2 * error, -1.5, 1.5)))

    def request_offboard_and_arm(self):
        now = rospy.Time.now()
        if (now - self.last_service_attempt).to_sec() < 1.0:
            return
        self.last_service_attempt = now
        with self.lock:
            state = self.state
        try:
            if state.mode != "OFFBOARD":
                self.mode_service(base_mode=0, custom_mode="OFFBOARD")
            if not state.armed:
                self.arm_service(True)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "[CircleSITLManager] Mode/arm request failed: %s", error)

    def configure_velocity_limit(self):
        if self.limit_configured:
            return True
        value = ParamValue(integer=0, real=self.xy_velocity_limit)
        try:
            response = self.param_service(param_id="MPC_XY_VEL_MAX", value=value)
            self.limit_configured = bool(response.success)
            if self.limit_configured:
                rospy.loginfo("[CircleSITLManager] MPC_XY_VEL_MAX set to %.1f m/s",
                              self.xy_velocity_limit)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "[CircleSITLManager] Parameter update failed: %s", error)
        return self.limit_configured

    def snapshot(self):
        with self.lock:
            return self.state, self.pose

    def timer_callback(self, _event):
        state, pose = self.snapshot()
        now = rospy.Time.now()

        if self.phase == "WAITING_FOR_LINK":
            if state.connected and pose is not None and self.configure_velocity_limit():
                self.set_phase("SETPOINT_WARMUP")
            return

        if self.phase in ("COMPLETE", "RUNNING"):
            if self.phase == "RUNNING" and (not state.connected or not state.armed):
                rospy.logerr_throttle(2.0, "[CircleSITLManager] PX4 link/arming lost during test")
            return

        if pose is None:
            return

        self.publish_altitude_command(pose)
        if self.phase == "SETPOINT_WARMUP":
            if (now - self.phase_started).to_sec() >= self.warmup_duration:
                self.set_phase("ARM_AND_OFFBOARD")
            return

        if self.phase == "ARM_AND_OFFBOARD":
            self.request_offboard_and_arm()
            if state.armed and state.mode == "OFFBOARD":
                self.set_phase("TAKEOFF")
            return

        if self.phase == "TAKEOFF":
            self.request_offboard_and_arm()
            altitude_error = abs(pose.pose.position.z - self.target_altitude)
            if altitude_error <= self.takeoff_tolerance:
                if self.takeoff_reached_at is None:
                    self.takeoff_reached_at = now
                elif (now - self.takeoff_reached_at).to_sec() >= self.stabilize_duration:
                    self.set_phase("START_TEST")
            else:
                self.takeoff_reached_at = None
            return

        if self.phase == "START_TEST":
            # The latched start message transfers sole command ownership to the
            # circle node. The manager deliberately stops publishing afterward.
            self.start_pub.publish(Bool(data=True))
            self.set_phase("RUNNING")

    def shutdown(self):
        if self.phase not in ("RUNNING", "COMPLETE"):
            self.command_pub.publish(self.make_command())


if __name__ == "__main__":
    rospy.init_node("circle_trajectory_sitl_manager")
    CircleTrajectorySITLManager()
    rospy.spin()
