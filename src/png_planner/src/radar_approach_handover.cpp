#include <ros/ros.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <detection_msgs/Detection.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <kcf_msgs/Bbox.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

template <typename T>
T clampValue(T value, T lower, T upper) {
    return std::max(lower, std::min(value, upper));
}

double degToRad(double degrees) {
    return degrees * kPi / 180.0;
}

double wrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double pointDistance(const geometry_msgs::Point& a,
                     const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void limitHorizontal(double limit, double& x, double& y) {
    const double speed = std::hypot(x, y);
    if (speed > limit && speed > 1e-9) {
        const double scale = limit / speed;
        x *= scale;
        y *= scale;
    }
}

}  // namespace

class RadarApproachHandover {
public:
    explicit RadarApproachHandover(ros::NodeHandle& nh) {
        loadParameters(nh);
        configureRos(nh);
        control_timer_ = nh.createTimer(
            ros::Duration(1.0 / control_rate_hz_),
            &RadarApproachHandover::controlTimer, this);

        ROS_INFO("[RadarApproach] Started. Radar period=%.1f s, terminal range=%.1f m.",
                 expected_radar_period_sec_, terminal_camera_range_m_);
        ROS_INFO("[RadarApproach] Command ownership: this node alone publishes %s.",
                 final_command_topic_.c_str());
    }

private:
    enum class State { WAIT_RADAR, APPROACH, HANDOVER, RADAR_TIMEOUT };

    void loadParameters(ros::NodeHandle& nh) {
        nh.param<std::string>("radar_input_mode", radar_input_mode_, "global_gps");
        nh.param<std::string>("radar_global_topic", radar_global_topic_,
                              "/target/mavros/global_position/global");
        nh.param<std::string>("radar_local_topic", radar_local_topic_,
                              "/radar/target/position");
        nh.param<std::string>("self_global_topic", self_global_topic_,
                              "/mavros/global_position/global");
        nh.param<std::string>("self_pose_topic", self_pose_topic_,
                              "/mavros/local_position/pose");
        nh.param<std::string>("ibvs_candidate_topic", ibvs_candidate_topic_,
                              "/fixed_camera_ibvs/cmd_vel_candidate");
        nh.param<std::string>("final_command_topic", final_command_topic_,
                              "/mavros/setpoint_velocity/cmd_vel");
        nh.param<std::string>("forward_speed_topic", forward_speed_topic_,
                              "/fixed_camera_ibvs/forward_speed_setpoint");

        nh.param("detection_mode", detection_mode_, 0);
        nh.param<std::string>("detection_topic", detection_topic_, "/object_kcf");
        nh.param("minimum_detection_score", minimum_detection_score_, 0.0);
        nh.param("bbox_center_is_offset", bbox_center_is_offset_, true);
        nh.param("cx", cx_, 480.0);
        nh.param("cy", cy_, 270.0);
        nh.param("center_tolerance_u_px", center_tolerance_u_px_, 40.0);
        nh.param("center_tolerance_v_px", center_tolerance_v_px_, 40.0);
        nh.param("detection_timeout_sec", detection_timeout_sec_, 0.30);
        nh.param("required_centered_detections", required_centered_detections_, 5);

        nh.param("expected_radar_period_sec", expected_radar_period_sec_, 3.0);
        nh.param("radar_timeout_sec", radar_timeout_sec_, 7.0);
        nh.param("filter_alpha", filter_alpha_, 0.70);
        nh.param("filter_beta", filter_beta_, 0.20);
        nh.param("max_target_speed_mps", max_target_speed_mps_, 25.0);
        nh.param("max_radar_innovation_m", max_radar_innovation_m_, 80.0);

        nh.param("terminal_camera_range_m", terminal_camera_range_m_, 20.0);
        nh.param("camera_mount_pitch_deg", camera_mount_pitch_deg_, -20.0);
        nh.param("camera_mount_yaw_deg", camera_mount_yaw_deg_, 0.0);
        nh.param("camera_offset_body_x_m", camera_offset_body_x_m_, 0.0);
        nh.param("camera_offset_body_y_m", camera_offset_body_y_m_, 0.0);
        nh.param("camera_offset_body_z_m", camera_offset_body_z_m_, 0.0);
        nh.param("minimum_course_speed_mps", minimum_course_speed_mps_, 0.5);

        nh.param("position_gain_xy", position_gain_xy_, 0.55);
        nh.param("position_gain_z", position_gain_z_, 0.45);
        nh.param("yaw_gain", yaw_gain_, 1.2);
        nh.param("max_approach_speed_mps", max_approach_speed_mps_, 12.0);
        nh.param("max_vertical_speed_mps", max_vertical_speed_mps_, 2.0);
        nh.param("max_yaw_rate_radps", max_yaw_rate_radps_, 0.8);
        nh.param("closing_speed_margin_mps", closing_speed_margin_mps_, 2.0);
        nh.param("min_handover_forward_speed_mps",
                 min_handover_forward_speed_mps_, 3.0);
        nh.param("max_handover_forward_speed_mps",
                 max_handover_forward_speed_mps_, 12.0);

        nh.param("handover_position_tolerance_m",
                 handover_position_tolerance_m_, 2.5);
        double handover_yaw_tolerance_deg = 10.0;
        nh.param("handover_yaw_tolerance_deg",
                 handover_yaw_tolerance_deg, 10.0);
        handover_yaw_tolerance_rad_ = degToRad(handover_yaw_tolerance_deg);
        nh.param("ibvs_candidate_timeout_sec", ibvs_candidate_timeout_sec_, 0.20);
        nh.param("control_rate_hz", control_rate_hz_, 20.0);

        expected_radar_period_sec_ = std::max(expected_radar_period_sec_, 0.1);
        radar_timeout_sec_ = std::max(radar_timeout_sec_, expected_radar_period_sec_);
        filter_alpha_ = clampValue(filter_alpha_, 0.0, 1.0);
        filter_beta_ = clampValue(filter_beta_, 0.0, 1.0);
        max_target_speed_mps_ = std::max(max_target_speed_mps_, 0.1);
        terminal_camera_range_m_ = std::max(terminal_camera_range_m_, 1.0);
        max_approach_speed_mps_ = std::max(max_approach_speed_mps_, 0.1);
        max_vertical_speed_mps_ = std::max(max_vertical_speed_mps_, 0.1);
        max_yaw_rate_radps_ = std::max(max_yaw_rate_radps_, 0.0);
        min_handover_forward_speed_mps_ =
            std::max(min_handover_forward_speed_mps_, 0.0);
        max_handover_forward_speed_mps_ = std::max(
            max_handover_forward_speed_mps_, min_handover_forward_speed_mps_);
        required_centered_detections_ = std::max(required_centered_detections_, 1);
        control_rate_hz_ = std::max(control_rate_hz_, 1.0);
    }

    void configureRos(ros::NodeHandle& nh) {
        self_pose_sub_ = nh.subscribe(
            self_pose_topic_, 20, &RadarApproachHandover::selfPoseCallback, this);
        self_global_sub_ = nh.subscribe(
            self_global_topic_, 5, &RadarApproachHandover::selfGlobalCallback, this);

        if (radar_input_mode_ == "local_pose") {
            radar_local_sub_ = nh.subscribe(
                radar_local_topic_, 5,
                &RadarApproachHandover::radarLocalCallback, this);
        } else {
            radar_global_sub_ = nh.subscribe(
                radar_global_topic_, 5,
                &RadarApproachHandover::radarGlobalCallback, this);
        }

        if (detection_mode_ == 1) {
            detection_sub_detection_ = nh.subscribe(
                detection_topic_, 5,
                &RadarApproachHandover::detectionCallbackDetection, this);
        } else {
            detection_sub_kcf_ = nh.subscribe(
                detection_topic_, 5,
                &RadarApproachHandover::detectionCallbackKcf, this);
        }
        ibvs_candidate_sub_ = nh.subscribe(
            ibvs_candidate_topic_, 5,
            &RadarApproachHandover::ibvsCandidateCallback, this);

        final_command_pub_ =
            nh.advertise<geometry_msgs::TwistStamped>(final_command_topic_, 1);
        forward_speed_pub_ =
            nh.advertise<std_msgs::Float64>(forward_speed_topic_, 1, true);
        predicted_target_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/radar_approach/predicted_target", 1);
        handover_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "/radar_approach/handover_pose", 1);
        target_velocity_pub_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/radar_approach/estimated_target_velocity", 1);
        state_pub_ = nh.advertise<std_msgs::String>(
            "/radar_approach/state", 1, true);
        handover_pub_ = nh.advertise<std_msgs::Bool>(
            "/radar_approach/handover_complete", 1, true);
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
        self_pose_ = *message;
        self_yaw_ = tf::getYaw(message->pose.orientation);
        have_self_pose_ = true;
    }

    void selfGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr& message) {
        if (!std::isfinite(message->latitude) ||
            !std::isfinite(message->longitude) ||
            !std::isfinite(message->altitude)) {
            return;
        }
        self_global_ = *message;
        have_self_global_ = true;
    }

    void radarGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr& message) {
        if (!have_self_pose_ || !have_self_global_) {
            ROS_WARN_THROTTLE(2.0,
                "[RadarApproach] Waiting for self local pose and GPS before converting radar GPS.");
            return;
        }
        if (!std::isfinite(message->latitude) ||
            !std::isfinite(message->longitude) ||
            !std::isfinite(message->altitude)) {
            return;
        }

        GeographicLib::LocalCartesian relative_frame(
            self_global_.latitude, self_global_.longitude,
            self_global_.altitude);
        double east = 0.0;
        double north = 0.0;
        double up = 0.0;
        relative_frame.Forward(message->latitude, message->longitude,
                               message->altitude, east, north, up);

        geometry_msgs::Point measurement;
        measurement.x = self_pose_.pose.position.x + east;
        measurement.y = self_pose_.pose.position.y + north;
        measurement.z = self_pose_.pose.position.z + up;
        updateRadarFilter(measurement, ros::Time::now());
    }

    void radarLocalCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
        updateRadarFilter(message->pose.position, ros::Time::now());
    }

    void updateRadarFilter(const geometry_msgs::Point& measurement,
                           const ros::Time& received_time) {
        radar_last_received_ = received_time;

        if (!have_radar_filter_) {
            filtered_target_position_ = measurement;
            filtered_target_velocity_.x = 0.0;
            filtered_target_velocity_.y = 0.0;
            filtered_target_velocity_.z = 0.0;
            radar_filter_time_ = received_time;
            radar_sample_count_ = 1;
            have_radar_filter_ = true;
            return;
        }

        const double dt = (received_time - radar_filter_time_).toSec();
        if (dt <= 0.05) return;

        if (radar_sample_count_ == 1) {
            filtered_target_velocity_.x =
                (measurement.x - filtered_target_position_.x) / dt;
            filtered_target_velocity_.y =
                (measurement.y - filtered_target_position_.y) / dt;
            filtered_target_velocity_.z =
                (measurement.z - filtered_target_position_.z) / dt;
            limitTargetVelocity();
            filtered_target_position_ = measurement;
            radar_filter_time_ = received_time;
            radar_sample_count_ = 2;
            return;
        }

        geometry_msgs::Point predicted = filtered_target_position_;
        predicted.x += filtered_target_velocity_.x * dt;
        predicted.y += filtered_target_velocity_.y * dt;
        predicted.z += filtered_target_velocity_.z * dt;

        double rx = measurement.x - predicted.x;
        double ry = measurement.y - predicted.y;
        double rz = measurement.z - predicted.z;
        const double innovation = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (innovation > max_radar_innovation_m_ && innovation > 1e-9) {
            const double scale = max_radar_innovation_m_ / innovation;
            rx *= scale;
            ry *= scale;
            rz *= scale;
            ROS_WARN_THROTTLE(2.0,
                "[RadarApproach] Radar innovation limited from %.1f m.", innovation);
        }

        filtered_target_position_.x = predicted.x + filter_alpha_ * rx;
        filtered_target_position_.y = predicted.y + filter_alpha_ * ry;
        filtered_target_position_.z = predicted.z + filter_alpha_ * rz;
        filtered_target_velocity_.x += filter_beta_ * rx / dt;
        filtered_target_velocity_.y += filter_beta_ * ry / dt;
        filtered_target_velocity_.z += filter_beta_ * rz / dt;
        limitTargetVelocity();

        radar_filter_time_ = received_time;
        ++radar_sample_count_;
    }

    void limitTargetVelocity() {
        const double speed = std::sqrt(
            filtered_target_velocity_.x * filtered_target_velocity_.x +
            filtered_target_velocity_.y * filtered_target_velocity_.y +
            filtered_target_velocity_.z * filtered_target_velocity_.z);
        if (speed <= max_target_speed_mps_ || speed < 1e-9) return;
        const double scale = max_target_speed_mps_ / speed;
        filtered_target_velocity_.x *= scale;
        filtered_target_velocity_.y *= scale;
        filtered_target_velocity_.z *= scale;
    }

    template <typename MessageT>
    void updateDetection(const MessageT& message) {
        const ros::Time now = ros::Time::now();
        detection_time_ = now;
        detection_valid_ =
            message.is_tracking && message.score >= minimum_detection_score_;
        if (!detection_valid_) {
            centered_detection_count_ = 0;
            return;
        }

        const double raw_u = 0.5 * (message.x1 + message.x2);
        const double raw_v = 0.5 * (message.y1 + message.y2);
        const double centered_u = bbox_center_is_offset_ ? raw_u : raw_u - cx_;
        const double centered_v = bbox_center_is_offset_ ? raw_v : raw_v - cy_;
        if (std::abs(centered_u) <= center_tolerance_u_px_ &&
            std::abs(centered_v) <= center_tolerance_v_px_) {
            ++centered_detection_count_;
        } else {
            centered_detection_count_ = 0;
        }
    }

    void detectionCallbackKcf(const kcf_msgs::Bbox::ConstPtr& message) {
        updateDetection(*message);
    }

    void detectionCallbackDetection(
        const detection_msgs::Detection::ConstPtr& message) {
        updateDetection(*message);
    }

    void ibvsCandidateCallback(
        const geometry_msgs::TwistStamped::ConstPtr& message) {
        ibvs_candidate_ = *message;
        ibvs_candidate_time_ = ros::Time::now();
        have_ibvs_candidate_ = true;
    }

    geometry_msgs::Point predictedTargetAt(const ros::Time& time) const {
        geometry_msgs::Point predicted = filtered_target_position_;
        const double dt = std::max(0.0, (time - radar_filter_time_).toSec());
        predicted.x += filtered_target_velocity_.x * dt;
        predicted.y += filtered_target_velocity_.y * dt;
        predicted.z += filtered_target_velocity_.z * dt;
        return predicted;
    }

    geometry_msgs::PoseStamped computeHandoverPose(
        const geometry_msgs::Point& target, const ros::Time& stamp,
        double& desired_body_yaw) const {
        const double target_horizontal_speed = std::hypot(
            filtered_target_velocity_.x, filtered_target_velocity_.y);
        double optical_yaw = 0.0;
        if (target_horizontal_speed >= minimum_course_speed_mps_) {
            optical_yaw = std::atan2(filtered_target_velocity_.y,
                                     filtered_target_velocity_.x);
        } else {
            optical_yaw = std::atan2(
                target.y - self_pose_.pose.position.y,
                target.x - self_pose_.pose.position.x);
        }

        desired_body_yaw = wrapAngle(optical_yaw -
                                      degToRad(camera_mount_yaw_deg_));
        const double optical_elevation = -degToRad(camera_mount_pitch_deg_);
        const double horizontal = std::cos(optical_elevation);

        geometry_msgs::Point desired_camera;
        desired_camera.x = target.x - terminal_camera_range_m_ *
                                      horizontal * std::cos(optical_yaw);
        desired_camera.y = target.y - terminal_camera_range_m_ *
                                      horizontal * std::sin(optical_yaw);
        desired_camera.z = target.z - terminal_camera_range_m_ *
                                      std::sin(optical_elevation);

        const double c = std::cos(desired_body_yaw);
        const double s = std::sin(desired_body_yaw);
        const double camera_offset_world_x =
            c * camera_offset_body_x_m_ - s * camera_offset_body_y_m_;
        const double camera_offset_world_y =
            s * camera_offset_body_x_m_ + c * camera_offset_body_y_m_;

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = desired_camera.x - camera_offset_world_x;
        pose.pose.position.y = desired_camera.y - camera_offset_world_y;
        pose.pose.position.z = desired_camera.z - camera_offset_body_z_m_;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(desired_body_yaw);
        return pose;
    }

    double recommendedForwardSpeed() const {
        const double target_horizontal_speed = std::hypot(
            filtered_target_velocity_.x, filtered_target_velocity_.y);
        return clampValue(target_horizontal_speed + closing_speed_margin_mps_,
                          min_handover_forward_speed_mps_,
                          max_handover_forward_speed_mps_);
    }

    geometry_msgs::TwistStamped approachCommand(
        const geometry_msgs::PoseStamped& desired_pose,
        double desired_body_yaw, const ros::Time& now) const {
        geometry_msgs::TwistStamped command;
        command.header.stamp = now;
        command.header.frame_id = "map";
        command.twist.linear.x = filtered_target_velocity_.x +
            position_gain_xy_ *
            (desired_pose.pose.position.x - self_pose_.pose.position.x);
        command.twist.linear.y = filtered_target_velocity_.y +
            position_gain_xy_ *
            (desired_pose.pose.position.y - self_pose_.pose.position.y);
        command.twist.linear.z = filtered_target_velocity_.z +
            position_gain_z_ *
            (desired_pose.pose.position.z - self_pose_.pose.position.z);
        limitHorizontal(max_approach_speed_mps_,
                        command.twist.linear.x, command.twist.linear.y);
        command.twist.linear.z = clampValue(
            command.twist.linear.z,
            -max_vertical_speed_mps_, max_vertical_speed_mps_);
        command.twist.angular.z = clampValue(
            yaw_gain_ * wrapAngle(desired_body_yaw - self_yaw_),
            -max_yaw_rate_radps_, max_yaw_rate_radps_);
        return command;
    }

    geometry_msgs::TwistStamped zeroCommand(const ros::Time& now) const {
        geometry_msgs::TwistStamped command;
        command.header.stamp = now;
        command.header.frame_id = "map";
        return command;
    }

    void setState(State state) {
        if (state == state_ && state_published_) return;
        state_ = state;
        state_published_ = true;
        std_msgs::String message;
        switch (state_) {
            case State::WAIT_RADAR: message.data = "WAIT_RADAR"; break;
            case State::APPROACH: message.data = "APPROACH"; break;
            case State::HANDOVER: message.data = "HANDOVER_TO_IBVS"; break;
            case State::RADAR_TIMEOUT: message.data = "RADAR_TIMEOUT"; break;
        }
        state_pub_.publish(message);
        ROS_INFO("[RadarApproach] State -> %s", message.data.c_str());
    }

    void publishHandoverFlag(bool value) {
        std_msgs::Bool message;
        message.data = value;
        handover_pub_.publish(message);
    }

    void publishDiagnostics(const geometry_msgs::Point& predicted_target,
                            const geometry_msgs::PoseStamped& handover_pose,
                            const ros::Time& now) {
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = now;
        target_pose.header.frame_id = "map";
        target_pose.pose.position = predicted_target;
        target_pose.pose.orientation.w = 1.0;
        predicted_target_pub_.publish(target_pose);
        handover_pose_pub_.publish(handover_pose);

        geometry_msgs::TwistStamped velocity;
        velocity.header.stamp = now;
        velocity.header.frame_id = "map";
        velocity.twist.linear = filtered_target_velocity_;
        target_velocity_pub_.publish(velocity);
    }

    void controlTimer(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        if (!have_self_pose_ || !have_radar_filter_ || radar_sample_count_ < 2) {
            setState(State::WAIT_RADAR);
            publishHandoverFlag(false);
            final_command_pub_.publish(zeroCommand(now));
            return;
        }

        if ((now - radar_last_received_).toSec() > radar_timeout_sec_) {
            setState(State::RADAR_TIMEOUT);
            publishHandoverFlag(false);
            final_command_pub_.publish(zeroCommand(now));
            return;
        }

        const geometry_msgs::Point target_now = predictedTargetAt(now);
        double desired_body_yaw = 0.0;
        const geometry_msgs::PoseStamped handover_pose =
            computeHandoverPose(target_now, now, desired_body_yaw);
        publishDiagnostics(target_now, handover_pose, now);

        std_msgs::Float64 forward_speed;
        forward_speed.data = recommendedForwardSpeed();
        forward_speed_pub_.publish(forward_speed);

        if (!handover_complete_) {
            setState(State::APPROACH);
            const double position_error = pointDistance(
                self_pose_.pose.position, handover_pose.pose.position);
            const double yaw_error = std::abs(
                wrapAngle(desired_body_yaw - self_yaw_));
            const bool detection_fresh = detection_valid_ &&
                (now - detection_time_).toSec() <= detection_timeout_sec_;
            const bool visual_ready = detection_fresh &&
                centered_detection_count_ >= required_centered_detections_;

            if (position_error <= handover_position_tolerance_m_ &&
                yaw_error <= handover_yaw_tolerance_rad_ && visual_ready) {
                handover_complete_ = true;
                setState(State::HANDOVER);
                publishHandoverFlag(true);
                ROS_INFO("[RadarApproach] Handover: position error %.2f m, "
                         "yaw error %.1f deg, centered detections %d.",
                         position_error, yaw_error * 180.0 / kPi,
                         centered_detection_count_);
            } else {
                publishHandoverFlag(false);
                final_command_pub_.publish(
                    approachCommand(handover_pose, desired_body_yaw, now));
                return;
            }
        }

        publishHandoverFlag(true);
        if (have_ibvs_candidate_ &&
            (now - ibvs_candidate_time_).toSec() <= ibvs_candidate_timeout_sec_) {
            geometry_msgs::TwistStamped command = ibvs_candidate_;
            command.header.stamp = now;
            final_command_pub_.publish(command);
        } else {
            ROS_WARN_THROTTLE(1.0,
                "[RadarApproach] IBVS command stale after handover; publishing zero command.");
            final_command_pub_.publish(zeroCommand(now));
        }
    }

    std::string radar_input_mode_;
    std::string radar_global_topic_;
    std::string radar_local_topic_;
    std::string self_global_topic_;
    std::string self_pose_topic_;
    std::string detection_topic_;
    std::string ibvs_candidate_topic_;
    std::string final_command_topic_;
    std::string forward_speed_topic_;

    int detection_mode_ = 0;
    double minimum_detection_score_ = 0.0;
    bool bbox_center_is_offset_ = true;
    double cx_ = 480.0;
    double cy_ = 270.0;
    double center_tolerance_u_px_ = 40.0;
    double center_tolerance_v_px_ = 40.0;
    double detection_timeout_sec_ = 0.30;
    int required_centered_detections_ = 5;

    double expected_radar_period_sec_ = 3.0;
    double radar_timeout_sec_ = 7.0;
    double filter_alpha_ = 0.70;
    double filter_beta_ = 0.20;
    double max_target_speed_mps_ = 25.0;
    double max_radar_innovation_m_ = 80.0;
    double terminal_camera_range_m_ = 20.0;
    double camera_mount_pitch_deg_ = -20.0;
    double camera_mount_yaw_deg_ = 0.0;
    double camera_offset_body_x_m_ = 0.0;
    double camera_offset_body_y_m_ = 0.0;
    double camera_offset_body_z_m_ = 0.0;
    double minimum_course_speed_mps_ = 0.5;
    double position_gain_xy_ = 0.55;
    double position_gain_z_ = 0.45;
    double yaw_gain_ = 1.2;
    double max_approach_speed_mps_ = 12.0;
    double max_vertical_speed_mps_ = 2.0;
    double max_yaw_rate_radps_ = 0.8;
    double closing_speed_margin_mps_ = 2.0;
    double min_handover_forward_speed_mps_ = 3.0;
    double max_handover_forward_speed_mps_ = 12.0;
    double handover_position_tolerance_m_ = 2.5;
    double handover_yaw_tolerance_rad_ = degToRad(10.0);
    double ibvs_candidate_timeout_sec_ = 0.20;
    double control_rate_hz_ = 20.0;

    sensor_msgs::NavSatFix self_global_;
    geometry_msgs::PoseStamped self_pose_;
    double self_yaw_ = 0.0;
    bool have_self_global_ = false;
    bool have_self_pose_ = false;

    geometry_msgs::Point filtered_target_position_;
    geometry_msgs::Vector3 filtered_target_velocity_;
    ros::Time radar_filter_time_;
    ros::Time radar_last_received_;
    int radar_sample_count_ = 0;
    bool have_radar_filter_ = false;

    bool detection_valid_ = false;
    int centered_detection_count_ = 0;
    ros::Time detection_time_;
    geometry_msgs::TwistStamped ibvs_candidate_;
    ros::Time ibvs_candidate_time_;
    bool have_ibvs_candidate_ = false;
    bool handover_complete_ = false;
    State state_ = State::WAIT_RADAR;
    bool state_published_ = false;

    ros::Subscriber radar_global_sub_;
    ros::Subscriber radar_local_sub_;
    ros::Subscriber self_global_sub_;
    ros::Subscriber self_pose_sub_;
    ros::Subscriber detection_sub_kcf_;
    ros::Subscriber detection_sub_detection_;
    ros::Subscriber ibvs_candidate_sub_;
    ros::Publisher final_command_pub_;
    ros::Publisher forward_speed_pub_;
    ros::Publisher predicted_target_pub_;
    ros::Publisher handover_pose_pub_;
    ros::Publisher target_velocity_pub_;
    ros::Publisher state_pub_;
    ros::Publisher handover_pub_;
    ros::Timer control_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_approach_handover");
    ros::NodeHandle private_node("~");
    RadarApproachHandover node(private_node);
    ros::spin();
    return 0;
}
