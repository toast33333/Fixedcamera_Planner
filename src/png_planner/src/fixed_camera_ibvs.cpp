#include <ros/ros.h>

#include <detection_msgs/Detection.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <kcf_msgs/Bbox.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

#include "los_rate_kalman.h"

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

double unwrapNear(double angle, double reference) {
    return reference + wrapAngle(angle - reference);
}

tf::Vector3 normalizedOr(const tf::Vector3& value,
                         const tf::Vector3& fallback) {
    const double length = value.length();
    if (length < 1e-9) return fallback;
    return value * (1.0 / length);
}

}  // namespace

struct DetectionData {
    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
    float score = 0.0F;
    std::string class_name;
    int track_id = -1;
    bool is_tracking = false;
};

struct AttitudeSample {
    ros::Time stamp;
    tf::Quaternion attitude;
};

class FixedCameraIbvsController {
public:
    explicit FixedCameraIbvsController(ros::NodeHandle& nh) {
        loadParameters(nh);
        configureCameraMount();
        configureFilters();
        configureRos(nh);

        last_control_time_ = ros::Time::now();
        ROS_INFO("[FixedCameraIBVS] Started: fixed camera, attitude-compensated LOS, FOV barrier.");
        ROS_INFO("[FixedCameraIBVS] Visual input unchanged; detection mode=%d topic=%s.",
                 detection_mode_, detection_topic_.c_str());
        ROS_INFO("[FixedCameraIBVS] Pose=%s, image delay=%.3f s, "
                 "FOV limits: horizontal +/-%.1f deg, vertical +/-%.1f deg.",
                 pose_topic_.c_str(), image_delay_sec_,
                 fov_horizontal_limit_deg_, fov_vertical_limit_deg_);
    }

    void spin() {
        ros::Rate rate(control_rate_hz_);
        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_control_time_).toSec() : step_dt_;
            if (dt <= 0.0 || dt > 0.5) dt = step_dt_;
            last_control_time_ = now;

            if (!have_pose_) {
                rate.sleep();
                continue;
            }

            const bool target_valid =
                have_detection_ && (now - last_detection_time_).toSec() <= lost_timeout_sec_;

            if (target_valid && detection_sequence_ != processed_detection_sequence_) {
                processDetectionMeasurement();
                processed_detection_sequence_ = detection_sequence_;
            }

            if (!target_valid || !have_world_los_) {
                resetLosStateIfNeeded();
                publishLostTargetCommand(now);
                rate.sleep();
                continue;
            }

            publishTrackingCommand(now, dt);
            rate.sleep();
        }
    }

private:
    void loadParameters(ros::NodeHandle& nh) {
        nh.param("fx", fx_, 640.0);
        nh.param("fy", fy_, 640.0);
        nh.param("cx", cx_, 480.0);
        nh.param("cy", cy_, 270.0);
        nh.param("bbox_center_is_offset", bbox_center_is_offset_, true);
        nh.param("y_up_positive", y_up_positive_, true);
        nh.param("minimum_detection_score", minimum_detection_score_, 0.0);

        nh.param("camera_mount_roll_deg", camera_mount_roll_deg_, 0.0);
        nh.param("camera_mount_pitch_deg", camera_mount_pitch_deg_, -20.0);
        nh.param("camera_mount_yaw_deg", camera_mount_yaw_deg_, 0.0);
        nh.param("desired_u_offset_px", desired_u_offset_px_, 0.0);
        nh.param("desired_v_offset_px", desired_v_offset_px_, 0.0);

        nh.param("V_default", forward_speed_, 5.0);
        nh.param("lost_forward_speed", lost_forward_speed_, forward_speed_);
        nh.param("enable_forward_speed_topic", enable_forward_speed_topic_, false);
        nh.param<std::string>("forward_speed_topic", forward_speed_topic_,
                              "/fixed_camera_ibvs/forward_speed_setpoint");
        nh.param("min_forward_speed", min_forward_speed_, 1.0);
        nh.param("max_forward_speed", max_forward_speed_, 12.0);
        nh.param("forward_speed_smoothing_alpha",
                 forward_speed_smoothing_alpha_, 0.25);
        nh.param("N_nav", navigation_gain_, 2.0);
        nh.param("max_lateral_velocity", max_lateral_velocity_, 2.0);
        nh.param("k_yaw", yaw_position_gain_, 1.5);
        nh.param("k_yaw_d", yaw_rate_gain_, 0.3);
        nh.param("max_yaw_rate", max_yaw_rate_, 1.0);
        nh.param("yaw_enable_speed_ratio", yaw_enable_speed_ratio_, 0.4);

        nh.param("enable_height_control", enable_height_control_, true);
        nh.param("k_z_ff", vertical_position_gain_, 1.0);
        nh.param("N_nav_z", vertical_navigation_gain_, 2.0);
        nh.param("max_vz", max_vertical_velocity_, 1.0);
        nh.param("vertical_integrator_decay", vertical_integrator_decay_, 0.98);

        // The paper defines horizontal and vertical FOV constraints as half angles.
        // Keep the legacy full-FOV parameters as a fallback for older config files.
        double legacy_hfov_deg = 53.2;
        double legacy_vfov_deg = 41.2;
        double legacy_margin_ratio = 1.0;
        nh.param("hfov_deg", legacy_hfov_deg, 53.2);
        nh.param("vfov_deg", legacy_vfov_deg, 41.2);
        nh.param("fov_margin_ratio", legacy_margin_ratio, 1.0);
        nh.param("fov_horizontal_limit_deg", fov_horizontal_limit_deg_,
                 0.5 * legacy_hfov_deg * legacy_margin_ratio);
        nh.param("fov_vertical_limit_deg", fov_vertical_limit_deg_,
                 0.5 * legacy_vfov_deg * legacy_margin_ratio);
        nh.param("fov_barrier_gain", fov_barrier_gain_, 0.20);
        nh.param("fov_barrier_max", fov_barrier_max_, 4.0);
        nh.param("fov_slowdown_start_ratio", fov_slowdown_start_ratio_, 0.65);
        nh.param("fov_min_forward_speed_ratio", fov_min_forward_speed_ratio_, 0.25);
        nh.param("fov_emergency_yaw_ratio", fov_emergency_yaw_ratio_, 0.85);

        nh.param("pixel_thr", pixel_threshold_x_, 5.0);
        nh.param("pixel_thr_y", pixel_threshold_y_, 5.0);
        nh.param("control_rate", control_rate_hz_, 50.0);
        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("lost_timeout", lost_timeout_sec_, 0.30);
        nh.param("image_delay_sec", image_delay_sec_, 0.08);
        nh.param("max_los_prediction_sec", max_los_prediction_sec_, 0.25);
        nh.param("pose_history_sec", pose_history_sec_, 1.0);
        nh.param("enable_attitude_compensation", enable_attitude_compensation_, true);

        double lambda_alpha_legacy = 0.2;
        int lambda_window_size_legacy = 3;
        nh.param("lambda_filt_alpha", lambda_alpha_legacy, 0.2);
        nh.param("lambda_window_size", lambda_window_size_legacy, 3);
        const LegacyLosKalmanTuning lambda_tuning =
            mapLegacyLosChainToKalman(lambda_alpha_legacy, lambda_window_size_legacy);
        nh.param("lambda_kalman_q_position", lambda_q_position_, lambda_tuning.q_position);
        nh.param("lambda_kalman_q_rate", lambda_q_rate_, lambda_tuning.q_rate);
        nh.param("lambda_kalman_r", lambda_r_, lambda_tuning.r_measurement);

        double gamma_alpha_legacy = 0.2;
        int gamma_window_size_legacy = 3;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, 0.2);
        nh.param("gamma_window_size", gamma_window_size_legacy, 3);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_r_, gamma_tuning.r_measurement);

        nh.param("detection_mode", detection_mode_, 0);
        nh.param<std::string>("detection_topic", detection_topic_, "/object_kcf");
        if (detection_mode_ == 1 && detection_topic_ == "/object_kcf") {
            detection_topic_ = "/object_detections";
        }
        nh.param<std::string>("pose_topic", pose_topic_, "/mavros/local_position/pose");
        nh.param<std::string>("attitude_topic", attitude_topic_, "/mavros/imu/data");
        nh.param<std::string>("velocity_topic", velocity_topic_,
                              "/mavros/local_position/velocity_local");
        nh.param<std::string>("command_topic", command_topic_,
                              "/mavros/setpoint_velocity/cmd_vel");
        nh.param<std::string>("los_debug_topic", los_debug_topic_,
                              "/fixed_camera_ibvs/los_world");
        nh.param<std::string>("fov_debug_topic", fov_debug_topic_,
                              "/fixed_camera_ibvs/fov_state");

        fx_ = std::max(fx_, 1.0);
        fy_ = std::max(fy_, 1.0);
        forward_speed_ = std::max(forward_speed_, 0.0);
        lost_forward_speed_ = std::max(lost_forward_speed_, 0.0);
        min_forward_speed_ = std::max(min_forward_speed_, 0.0);
        max_forward_speed_ = std::max(max_forward_speed_, min_forward_speed_);
        forward_speed_smoothing_alpha_ =
            clampValue(forward_speed_smoothing_alpha_, 0.0, 1.0);
        max_yaw_rate_ = std::max(max_yaw_rate_, 0.0);
        max_vertical_velocity_ = std::max(max_vertical_velocity_, 0.0);
        max_lateral_velocity_ = std::max(max_lateral_velocity_, 0.0);
        fov_horizontal_limit_deg_ =
            clampValue(fov_horizontal_limit_deg_, 1.0, 89.0);
        fov_vertical_limit_deg_ =
            clampValue(fov_vertical_limit_deg_, 1.0, 89.0);
        fov_barrier_max_ = std::max(fov_barrier_max_, 1.0);
        fov_slowdown_start_ratio_ = clampValue(fov_slowdown_start_ratio_, 0.0, 0.99);
        fov_min_forward_speed_ratio_ = clampValue(fov_min_forward_speed_ratio_, 0.0, 1.0);
        fov_emergency_yaw_ratio_ = clampValue(fov_emergency_yaw_ratio_, 0.0, 1.0);
        control_rate_hz_ = std::max(control_rate_hz_, 1.0);
        step_dt_ = std::max(step_dt_, 1e-3);
        lost_timeout_sec_ = std::max(lost_timeout_sec_, 0.02);
        image_delay_sec_ = std::max(image_delay_sec_, 0.0);
        max_los_prediction_sec_ = std::max(max_los_prediction_sec_, 0.0);
        pose_history_sec_ = std::max(pose_history_sec_, image_delay_sec_ + 0.2);
    }

    void configureCameraMount() {
        camera_mount_rotation_.setRPY(
            degToRad(camera_mount_roll_deg_),
            degToRad(camera_mount_pitch_deg_),
            degToRad(camera_mount_yaw_deg_));

        const double desired_v_optical =
            y_up_positive_ ? -desired_v_offset_px_ : desired_v_offset_px_;
        const tf::Vector3 desired_camera(
            desired_u_offset_px_ / fx_, desired_v_optical / fy_, 1.0);
        desired_los_body_ = cameraRayToBody(desired_camera);
    }

    void configureFilters() {
        azimuth_filter_.setNoise(lambda_q_position_, lambda_q_rate_, lambda_r_);
        elevation_filter_.setNoise(gamma_q_position_, gamma_q_rate_, gamma_r_);
        azimuth_filter_.reset();
        elevation_filter_.reset();
    }

    void configureRos(ros::NodeHandle& nh) {
        if (detection_mode_ == 1) {
            detection_sub_detection_ = nh.subscribe<detection_msgs::Detection>(
                detection_topic_, 1,
                &FixedCameraIbvsController::detectionCallbackDetection, this);
        } else {
            detection_sub_kcf_ = nh.subscribe<kcf_msgs::Bbox>(
                detection_topic_, 1,
                &FixedCameraIbvsController::detectionCallbackKcf, this);
        }

        pose_sub_ = nh.subscribe(pose_topic_, 20,
                                 &FixedCameraIbvsController::poseCallback, this);
        attitude_sub_ = nh.subscribe(attitude_topic_, 50,
                                     &FixedCameraIbvsController::attitudeCallback, this);
        velocity_sub_ = nh.subscribe(velocity_topic_, 5,
                                     &FixedCameraIbvsController::velocityCallback, this);
        if (enable_forward_speed_topic_) {
            forward_speed_sub_ = nh.subscribe(
                forward_speed_topic_, 1,
                &FixedCameraIbvsController::forwardSpeedCallback, this);
        }
        command_pub_ = nh.advertise<geometry_msgs::TwistStamped>(command_topic_, 1);
        los_debug_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(los_debug_topic_, 1);
        fov_debug_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(fov_debug_topic_, 1);
    }

    tf::Vector3 cameraRayToBody(const tf::Vector3& camera_ray) const {
        // ROS optical frame: +x right, +y down, +z forward.
        // ROS body FLU frame: +x forward, +y left, +z up.
        const tf::Vector3 nominal_body(
            camera_ray.z(), -camera_ray.x(), -camera_ray.y());
        return normalizedOr(camera_mount_rotation_ * nominal_body,
                            tf::Vector3(1.0, 0.0, 0.0));
    }

    tf::Vector3 bodyRayToCamera(const tf::Vector3& body_ray) const {
        const tf::Vector3 nominal_body = camera_mount_rotation_.transpose() * body_ray;
        return normalizedOr(tf::Vector3(
                                -nominal_body.y(),
                                -nominal_body.z(),
                                nominal_body.x()),
                            tf::Vector3(0.0, 0.0, 1.0));
    }

    bool lookupAttitude(const ros::Time& stamp, tf::Quaternion& attitude) const {
        if (attitude_history_.empty()) return false;
        if (stamp <= attitude_history_.front().stamp) {
            attitude = attitude_history_.front().attitude;
            return true;
        }
        if (stamp >= attitude_history_.back().stamp) {
            attitude = attitude_history_.back().attitude;
            return true;
        }

        for (std::size_t i = 1; i < attitude_history_.size(); ++i) {
            const AttitudeSample& newer = attitude_history_[i];
            if (newer.stamp < stamp) continue;
            const AttitudeSample& older = attitude_history_[i - 1];
            const double interval = (newer.stamp - older.stamp).toSec();
            const double ratio = interval > 1e-6
                                     ? clampValue((stamp - older.stamp).toSec() / interval,
                                                  0.0, 1.0)
                                     : 0.0;
            attitude = older.attitude.slerp(newer.attitude, ratio);
            attitude.normalize();
            return true;
        }
        return false;
    }

    void processDetectionMeasurement() {
        const double raw_u = 0.5 * (last_detection_.x1 + last_detection_.x2);
        const double raw_v = 0.5 * (last_detection_.y1 + last_detection_.y2);
        const double centered_u = bbox_center_is_offset_ ? raw_u : (raw_u - cx_);
        const double centered_v = bbox_center_is_offset_ ? raw_v : (raw_v - cy_);
        const double optical_v = y_up_positive_ ? -centered_v : centered_v;

        const tf::Vector3 camera_ray(
            centered_u / fx_, optical_v / fy_, 1.0);
        const tf::Vector3 los_body_at_capture = cameraRayToBody(camera_ray);

        const ros::Time capture_time =
            last_detection_time_ - ros::Duration(image_delay_sec_);
        tf::Quaternion capture_attitude = current_attitude_;
        if (enable_attitude_compensation_) {
            lookupAttitude(capture_time, capture_attitude);
        } else {
            capture_attitude.setRPY(0.0, 0.0, 0.0);
        }

        const tf::Vector3 los_world = normalizedOr(
            tf::Matrix3x3(capture_attitude) * los_body_at_capture,
            tf::Vector3(1.0, 0.0, 0.0));
        const double azimuth_raw = std::atan2(los_world.y(), los_world.x());
        const double elevation_raw = std::atan2(
            los_world.z(), std::hypot(los_world.x(), los_world.y()));

        double azimuth = azimuth_raw;
        if (filters_initialized_) {
            azimuth = unwrapNear(azimuth_raw, azimuth_filter_.value());
        }

        double measurement_dt = step_dt_;
        if (filters_initialized_) {
            measurement_dt = (capture_time - last_measurement_time_).toSec();
            if (measurement_dt <= 0.0 || measurement_dt > 1.0) {
                measurement_dt = step_dt_;
            }
        }

        azimuth_filter_.update(azimuth, measurement_dt);
        elevation_filter_.update(elevation_raw, measurement_dt);
        filters_initialized_ = true;
        have_world_los_ = true;
        last_measurement_time_ = capture_time;
    }

    tf::Vector3 predictedWorldLos(const ros::Time& now) const {
        const double prediction_dt = clampValue(
            (now - last_measurement_time_).toSec(), 0.0, max_los_prediction_sec_);
        const double azimuth =
            azimuth_filter_.value() + azimuth_filter_.rate() * prediction_dt;
        const double elevation = clampValue(
            elevation_filter_.value() + elevation_filter_.rate() * prediction_dt,
            -0.5 * kPi + 1e-3, 0.5 * kPi - 1e-3);
        const double horizontal = std::cos(elevation);
        return tf::Vector3(horizontal * std::cos(azimuth),
                           horizontal * std::sin(azimuth),
                           std::sin(elevation));
    }

    double computeAxisFovBarrierScale(double angular_ratio) const {
        // Axis-wise rectangular adaptation of the paper's logarithmic barrier:
        // B(rho) = -0.5 log(1-rho^2), |rho| < 1.
        // Its gradient contributes the rho/(1-rho^2) term. Multiplying the
        // nominal angular error by the scale below produces the same divergence
        // toward the configured half-FOV boundary, with an actuator-safe cap.
        const double rho = clampValue(std::fabs(angular_ratio), 0.0, 0.999);
        const double barrier_scale =
            1.0 + fov_barrier_gain_ * rho * rho /
                      std::max(1.0 - rho * rho, 1e-3);
        return clampValue(barrier_scale, 1.0, fov_barrier_max_);
    }

    double computeForwardSpeedScale(double fov_ratio) const {
        if (fov_ratio <= fov_slowdown_start_ratio_) return 1.0;
        const double progress = clampValue(
            (fov_ratio - fov_slowdown_start_ratio_) /
                (1.0 - fov_slowdown_start_ratio_),
            0.0, 1.0);
        return 1.0 - progress * (1.0 - fov_min_forward_speed_ratio_);
    }

    void publishTrackingCommand(const ros::Time& now, double dt) {
        const tf::Vector3 los_world = predictedWorldLos(now);
        const tf::Matrix3x3 world_from_body(
            enable_attitude_compensation_
                ? current_attitude_
                : tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        const tf::Vector3 los_body = normalizedOr(
            world_from_body.transpose() * los_world,
            tf::Vector3(1.0, 0.0, 0.0));
        const tf::Vector3 los_camera = bodyRayToCamera(los_body);

        const double safe_h =
            std::max(degToRad(fov_horizontal_limit_deg_), 1e-3);
        const double safe_v =
            std::max(degToRad(fov_vertical_limit_deg_), 1e-3);
        const double horizontal_angle = std::atan2(los_camera.x(), los_camera.z());
        const double vertical_angle = std::atan2(los_camera.y(), los_camera.z());
        const double horizontal_ratio = std::fabs(horizontal_angle) / safe_h;
        const double vertical_ratio = std::fabs(vertical_angle) / safe_v;
        const double fov_ratio = std::max(horizontal_ratio, vertical_ratio);

        const double horizontal_barrier_scale =
            computeAxisFovBarrierScale(horizontal_ratio);
        const double vertical_barrier_scale =
            computeAxisFovBarrierScale(vertical_ratio);
        const double barrier_scale =
            std::max(horizontal_barrier_scale, vertical_barrier_scale);
        const double speed_scale = computeForwardSpeedScale(fov_ratio);

        const double current_yaw_to_los = std::atan2(los_body.y(), los_body.x());
        const double desired_yaw_to_los =
            std::atan2(desired_los_body_.y(), desired_los_body_.x());
        const double yaw_error = wrapAngle(current_yaw_to_los - desired_yaw_to_los);

        const double current_elevation = std::atan2(
            los_body.z(), std::hypot(los_body.x(), los_body.y()));
        const double desired_elevation = std::atan2(
            desired_los_body_.z(),
            std::hypot(desired_los_body_.x(), desired_los_body_.y()));
        const double elevation_error = current_elevation - desired_elevation;

        const double predicted_u = los_camera.z() > 1e-4
                                       ? fx_ * los_camera.x() / los_camera.z()
                                       : std::copysign(fx_ * 10.0, los_camera.x());
        const double predicted_optical_v = los_camera.z() > 1e-4
                                               ? fy_ * los_camera.y() / los_camera.z()
                                               : std::copysign(fy_ * 10.0, los_camera.y());
        const double predicted_input_v =
            y_up_positive_ ? -predicted_optical_v : predicted_optical_v;

        updateYawEnableState();
        const bool fov_emergency = fov_ratio >= fov_emergency_yaw_ratio_;
        double yaw_rate_command = 0.0;
        if (yaw_control_enabled_ || fov_emergency) {
            const double position_term =
                std::fabs(predicted_u - desired_u_offset_px_) > pixel_threshold_x_
                    ? yaw_position_gain_ * horizontal_barrier_scale * yaw_error
                    : 0.0;
            yaw_rate_command = clampValue(
                position_term + yaw_rate_gain_ * azimuth_filter_.rate(),
                -max_yaw_rate_, max_yaw_rate_);
        }

        const double commanded_forward_speed = forward_speed_ * speed_scale;
        const double lateral_acceleration =
            navigation_gain_ * commanded_forward_speed * azimuth_filter_.rate();
        const double lateral_velocity = clampValue(
            lateral_acceleration * dt,
            -max_lateral_velocity_, max_lateral_velocity_);

        double vertical_velocity = 0.0;
        if (enable_height_control_) {
            if (std::fabs(predicted_input_v - desired_v_offset_px_) > pixel_threshold_y_) {
                const double vertical_acceleration =
                    vertical_navigation_gain_ * commanded_forward_speed *
                    elevation_filter_.rate();
                vertical_velocity_integral_ += vertical_acceleration * dt;
            } else {
                vertical_velocity_integral_ *= vertical_integrator_decay_;
            }
            vertical_velocity_integral_ = clampValue(
                vertical_velocity_integral_,
                -max_vertical_velocity_, max_vertical_velocity_);
            vertical_velocity = clampValue(
                vertical_position_gain_ * vertical_barrier_scale * elevation_error +
                    vertical_velocity_integral_,
                -max_vertical_velocity_, max_vertical_velocity_);
        }

        publishBodyVelocity(now, commanded_forward_speed,
                            lateral_velocity, vertical_velocity,
                            yaw_rate_command);
        publishDebug(now, los_world, horizontal_ratio, vertical_ratio, barrier_scale);

        if (fov_ratio >= 1.0) {
            ROS_WARN_THROTTLE(1.0,
                              "[FixedCameraIBVS] Predicted LOS crossed safe FOV boundary "
                              "(horizontal=%.2f, vertical=%.2f).",
                              horizontal_ratio, vertical_ratio);
        }
    }

    void publishLostTargetCommand(const ros::Time& now) {
        publishBodyVelocity(now, lost_forward_speed_, 0.0, 0.0, 0.0);
    }

    void publishBodyVelocity(const ros::Time& stamp,
                             double forward,
                             double left,
                             double up,
                             double yaw_rate) {
        tf::Matrix3x3 attitude_matrix(current_attitude_);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        attitude_matrix.getRPY(roll, pitch, yaw);

        geometry_msgs::TwistStamped command;
        command.header.stamp = stamp;
        command.twist.linear.x = forward * std::cos(yaw) - left * std::sin(yaw);
        command.twist.linear.y = forward * std::sin(yaw) + left * std::cos(yaw);
        command.twist.linear.z = up;
        command.twist.angular.z = yaw_rate;
        command_pub_.publish(command);
    }

    void publishDebug(const ros::Time& stamp,
                      const tf::Vector3& los_world,
                      double horizontal_ratio,
                      double vertical_ratio,
                      double barrier_scale) {
        geometry_msgs::Vector3Stamped los_message;
        los_message.header.stamp = stamp;
        los_message.header.frame_id = "map";
        los_message.vector.x = los_world.x();
        los_message.vector.y = los_world.y();
        los_message.vector.z = los_world.z();
        los_debug_pub_.publish(los_message);

        geometry_msgs::Vector3Stamped fov_message;
        fov_message.header.stamp = stamp;
        fov_message.header.frame_id = "camera_optical";
        fov_message.vector.x = horizontal_ratio;
        fov_message.vector.y = vertical_ratio;
        fov_message.vector.z = barrier_scale;
        fov_debug_pub_.publish(fov_message);
    }

    void resetLosStateIfNeeded() {
        if (!filters_initialized_ && !have_world_los_) return;
        azimuth_filter_.reset();
        elevation_filter_.reset();
        filters_initialized_ = false;
        have_world_los_ = false;
        vertical_velocity_integral_ = 0.0;
    }

    void updateYawEnableState() {
        if (yaw_control_enabled_) return;
        if (current_horizontal_speed_ >= forward_speed_ * yaw_enable_speed_ratio_) {
            yaw_control_enabled_ = true;
            ROS_INFO("[FixedCameraIBVS] Yaw control enabled at %.2f m/s.",
                     current_horizontal_speed_);
        }
    }

    template <typename Message>
    void acceptDetection(const Message& message) {
        last_detection_.x1 = message.x1;
        last_detection_.y1 = message.y1;
        last_detection_.x2 = message.x2;
        last_detection_.y2 = message.y2;
        last_detection_.score = message.score;
        last_detection_.class_name = message.class_name;
        last_detection_.track_id = message.track_id;
        last_detection_.is_tracking = message.is_tracking;
        last_detection_time_ = ros::Time::now();
        have_detection_ =
            message.is_tracking && message.score >= minimum_detection_score_;
        ++detection_sequence_;
    }

    void detectionCallbackKcf(const kcf_msgs::Bbox::ConstPtr& message) {
        acceptDetection(*message);
    }

    void detectionCallbackDetection(
        const detection_msgs::Detection::ConstPtr& message) {
        acceptDetection(*message);
    }

    void recordAttitude(const geometry_msgs::Quaternion& orientation,
                        const ros::Time& message_stamp) {
        tf::Quaternion attitude(
            orientation.x, orientation.y, orientation.z, orientation.w);
        if (attitude.length2() < 1e-9) return;
        attitude.normalize();

        AttitudeSample sample;
        sample.stamp = message_stamp.isZero() ? ros::Time::now() : message_stamp;
        sample.attitude = attitude;
        if (attitude_history_.empty() || sample.stamp >= attitude_history_.back().stamp) {
            attitude_history_.push_back(sample);
        }

        if (current_attitude_stamp_.isZero() || sample.stamp >= current_attitude_stamp_) {
            current_attitude_ = attitude;
            current_attitude_stamp_ = sample.stamp;
        }

        const ros::Time oldest_allowed = sample.stamp - ros::Duration(pose_history_sec_);
        while (attitude_history_.size() > 2 &&
               attitude_history_[1].stamp < oldest_allowed) {
            attitude_history_.pop_front();
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
        recordAttitude(message->pose.orientation, message->header.stamp);
        have_pose_ = true;
    }

    void attitudeCallback(const sensor_msgs::Imu::ConstPtr& message) {
        recordAttitude(message->orientation, message->header.stamp);
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& message) {
        current_horizontal_speed_ = std::hypot(
            message->twist.linear.x, message->twist.linear.y);
    }

    void forwardSpeedCallback(const std_msgs::Float64::ConstPtr& message) {
        if (!std::isfinite(message->data)) return;
        const double requested = clampValue(
            message->data, min_forward_speed_, max_forward_speed_);
        forward_speed_ += forward_speed_smoothing_alpha_ *
                          (requested - forward_speed_);
    }

    double fx_ = 640.0;
    double fy_ = 640.0;
    double cx_ = 480.0;
    double cy_ = 270.0;
    bool bbox_center_is_offset_ = true;
    bool y_up_positive_ = true;
    double minimum_detection_score_ = 0.0;

    double camera_mount_roll_deg_ = 0.0;
    double camera_mount_pitch_deg_ = -20.0;
    double camera_mount_yaw_deg_ = 0.0;
    double desired_u_offset_px_ = 0.0;
    double desired_v_offset_px_ = 0.0;
    tf::Matrix3x3 camera_mount_rotation_;
    tf::Vector3 desired_los_body_{1.0, 0.0, 0.0};

    double forward_speed_ = 5.0;
    double lost_forward_speed_ = 5.0;
    bool enable_forward_speed_topic_ = false;
    std::string forward_speed_topic_ =
        "/fixed_camera_ibvs/forward_speed_setpoint";
    double min_forward_speed_ = 1.0;
    double max_forward_speed_ = 12.0;
    double forward_speed_smoothing_alpha_ = 0.25;
    double navigation_gain_ = 2.0;
    double max_lateral_velocity_ = 2.0;
    double yaw_position_gain_ = 1.5;
    double yaw_rate_gain_ = 0.3;
    double max_yaw_rate_ = 1.0;
    double yaw_enable_speed_ratio_ = 0.4;
    bool yaw_control_enabled_ = false;

    bool enable_height_control_ = true;
    double vertical_position_gain_ = 1.0;
    double vertical_navigation_gain_ = 2.0;
    double max_vertical_velocity_ = 1.0;
    double vertical_integrator_decay_ = 0.98;
    double vertical_velocity_integral_ = 0.0;

    double fov_horizontal_limit_deg_ = 26.6;
    double fov_vertical_limit_deg_ = 20.6;
    double fov_barrier_gain_ = 0.20;
    double fov_barrier_max_ = 4.0;
    double fov_slowdown_start_ratio_ = 0.65;
    double fov_min_forward_speed_ratio_ = 0.25;
    double fov_emergency_yaw_ratio_ = 0.85;

    double pixel_threshold_x_ = 5.0;
    double pixel_threshold_y_ = 5.0;
    double control_rate_hz_ = 50.0;
    bool use_real_dt_ = true;
    double step_dt_ = 0.02;
    double lost_timeout_sec_ = 0.30;
    double image_delay_sec_ = 0.08;
    double max_los_prediction_sec_ = 0.25;
    double pose_history_sec_ = 1.0;
    bool enable_attitude_compensation_ = true;

    double lambda_q_position_ = 1e-4;
    double lambda_q_rate_ = 5e-1;
    double lambda_r_ = 2e-4;
    double gamma_q_position_ = 1e-4;
    double gamma_q_rate_ = 5e-1;
    double gamma_r_ = 2e-4;
    ScalarRateKalmanFilter azimuth_filter_;
    ScalarRateKalmanFilter elevation_filter_;
    bool filters_initialized_ = false;
    bool have_world_los_ = false;
    ros::Time last_measurement_time_;

    int detection_mode_ = 0;
    std::string detection_topic_;
    std::string pose_topic_;
    std::string attitude_topic_;
    std::string velocity_topic_;
    std::string command_topic_;
    std::string los_debug_topic_;
    std::string fov_debug_topic_;

    DetectionData last_detection_;
    ros::Time last_detection_time_;
    std::uint64_t detection_sequence_ = 0;
    std::uint64_t processed_detection_sequence_ = 0;
    bool have_detection_ = false;

    tf::Quaternion current_attitude_{0.0, 0.0, 0.0, 1.0};
    ros::Time current_attitude_stamp_;
    std::deque<AttitudeSample> attitude_history_;
    bool have_pose_ = false;
    double current_horizontal_speed_ = 0.0;
    ros::Time last_control_time_;

    ros::Subscriber detection_sub_kcf_;
    ros::Subscriber detection_sub_detection_;
    ros::Subscriber pose_sub_;
    ros::Subscriber attitude_sub_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber forward_speed_sub_;
    ros::Publisher command_pub_;
    ros::Publisher los_debug_pub_;
    ros::Publisher fov_debug_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixed_camera_ibvs");
    ros::NodeHandle private_node("~");
    FixedCameraIbvsController controller(private_node);
    controller.spin();
    return 0;
}
