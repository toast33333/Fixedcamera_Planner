#!/usr/bin/env python3
"""Two-PX4 fixed-camera pursuit platform-validation experiment.

The node deliberately keeps the production detector interface unchanged.  It
projects Gazebo ground truth into a synthetic camera image and publishes the
same kcf_msgs/Bbox message that the existing controller consumes.  It also
multiplexes takeoff commands and the controller output so that only one source
ever drives the interceptor MAVROS velocity topic.
"""

import csv
import math
import os
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from kcf_msgs.msg import Bbox
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray


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

        self.interceptor_altitude = float(rospy.get_param("~interceptor_altitude_m", 4.0))
        self.target_altitude = float(rospy.get_param("~target_altitude_m", 9.0))
        self.target_speed = float(rospy.get_param("~target_speed_mps", 2.0))
        self.capture_distance = float(rospy.get_param("~capture_distance_m", 7.5))
        self.chase_timeout = float(rospy.get_param("~chase_timeout_s", 45.0))
        self.takeoff_tolerance = float(rospy.get_param("~takeoff_tolerance_m", 0.45))
        self.control_rate = float(rospy.get_param("~control_rate_hz", 20.0))
        self.results_dir = os.path.expanduser(rospy.get_param("~results_dir", "/tmp/fixed_camera_multisim"))

        self.model_states = None
        self.interceptor_state = State()
        self.target_state = State()
        self.interceptor_pose = None
        self.target_pose = None
        self.ibvs_command = TwistStamped()
        self.last_ibvs_command_time = rospy.Time(0)

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

        os.makedirs(self.results_dir, exist_ok=True)
        self.csv_path = os.path.join(self.results_dir, "experiment.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "ros_time_s", "phase", "distance_m", "horizontal_distance_m",
            "pixel_u_offset", "pixel_v_offset", "horizontal_fov_ratio",
            "vertical_fov_ratio", "tracking", "interceptor_x", "interceptor_y",
            "interceptor_z", "target_x", "target_y", "target_z",
            "ibvs_vx", "ibvs_vy", "ibvs_vz", "ibvs_yaw_rate"
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

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)
        rospy.Subscriber(self.interceptor_ns + "/state", State, self.interceptor_state_cb, queue_size=5)
        rospy.Subscriber(self.target_ns + "/state", State, self.target_state_cb, queue_size=5)
        rospy.Subscriber(self.interceptor_ns + "/local_position/pose", PoseStamped,
                         self.interceptor_pose_cb, queue_size=5)
        rospy.Subscriber(self.target_ns + "/local_position/pose", PoseStamped,
                         self.target_pose_cb, queue_size=5)
        rospy.Subscriber("/fixed_camera_sim/ibvs_cmd", TwistStamped, self.ibvs_command_cb, queue_size=5)

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
        rospy.loginfo("[FixedCameraSim] Experiment manager started; results: %s", self.results_dir)

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
        self.ibvs_command = message
        self.last_ibvs_command_time = rospy.Time.now()

    def set_phase(self, phase):
        if phase == self.phase:
            return
        rospy.loginfo("[FixedCameraSim] Phase: %s -> %s", self.phase, phase)
        self.phase = phase
        self.phase_started = rospy.Time.now()
        if phase == "CHASE":
            self.chase_started = self.phase_started

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

    def altitude_command(self, pose, desired_altitude, vx=0.0):
        current = pose.pose.position.z if pose is not None else 0.0
        vz = clamp(0.9 * (desired_altitude - current), -1.3, 1.3)
        return self.make_command(vx=vx, vz=vz)

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

    def publish_detection_and_image(self, projection):
        distance, horizontal_distance, visible, u_offset, v_offset, h_ratio, v_ratio = projection
        self.distance = distance
        self.horizontal_distance = horizontal_distance
        self.tracking = visible
        self.pixel_u = u_offset
        self.pixel_v = v_offset
        self.horizontal_ratio = h_ratio
        self.vertical_ratio = v_ratio

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
        if visible:
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
        image_message.header.stamp = rospy.Time.now()
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
        interceptor_command = self.make_command()
        target_command = self.make_command()

        if models is not None:
            interceptor_model_pose, _, target_model_pose, _ = models
            projection = self.project_target(interceptor_model_pose, target_model_pose)
            self.publish_detection_and_image(projection)
            self.publish_visualization(interceptor_model_pose, target_model_pose)
            self.distance_pub.publish(Float32(data=projection[0]))
            self.fov_ratio_pub.publish(Float32(data=max(projection[5], projection[6])))

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
                self.set_phase("CHASE")

        elif self.phase == "CHASE":
            self.request_offboard_and_arm()
            target_command = self.altitude_command(self.target_pose, self.target_altitude,
                                                   vx=self.target_speed)
            if (now - self.last_ibvs_command_time).to_sec() < 0.35:
                interceptor_command = self.ibvs_command
                interceptor_command.header.stamp = now
            else:
                interceptor_command = self.make_command()

            if math.isfinite(self.distance) and self.distance <= self.capture_distance:
                self.success_time = (now - self.chase_started).to_sec()
                self.set_phase("SUCCESS_HOLD")
                rospy.loginfo("[FixedCameraSim] SUCCESS: capture radius reached in %.2f s at %.2f m",
                              self.success_time, self.distance)
            elif (now - self.chase_started).to_sec() > self.chase_timeout:
                self.set_phase("TIMEOUT_HOLD")
                rospy.logwarn("[FixedCameraSim] Chase timeout; minimum result is retained for analysis")

        elif self.phase in ("SUCCESS_HOLD", "TIMEOUT_HOLD"):
            self.request_offboard_and_arm()
            interceptor_hold_altitude = self.interceptor_pose.pose.position.z
            target_hold_altitude = self.target_pose.pose.position.z
            interceptor_command = self.altitude_command(self.interceptor_pose, interceptor_hold_altitude)
            target_command = self.altitude_command(self.target_pose, target_hold_altitude)

        self.interceptor_cmd_pub.publish(interceptor_command)
        self.target_cmd_pub.publish(target_command)
        self.publish_status()
        if models is not None:
            self.write_csv(models)

    def publish_status(self):
        status = ("phase=%s | distance=%.2f m | horizontal=%.2f m | target_visible=%s | "
                  "pixel=(%.1f, %.1f) | fov_ratio=(%.2f, %.2f) | "
                  "interceptor=%s/%s | target=%s/%s" %
                  (self.phase, self.distance, self.horizontal_distance, self.tracking,
                   self.pixel_u, self.pixel_v, self.horizontal_ratio, self.vertical_ratio,
                   self.interceptor_state.mode, self.interceptor_state.armed,
                   self.target_state.mode, self.target_state.armed))
        self.status_pub.publish(String(data=status))
        rospy.loginfo_throttle(2.0, "[FixedCameraSim] %s", status)

    def write_csv(self, models):
        interceptor_pose, _, target_pose, _ = models
        command = self.ibvs_command.twist
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
