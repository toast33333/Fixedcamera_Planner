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
#include <vector>

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
    enum class State {
        WAIT_RADAR, APPROACH, VISUAL_ALIGN, HANDOVER, RADAR_TIMEOUT
    };

    struct TimedRadarMeasurement {
        geometry_msgs::Point position;
        ros::Time stamp;
    };

    struct AxisKalmanState {
        double position = 0.0;
        double velocity = 0.0;
        double p00 = 1.0;
        double p01 = 0.0;
        double p10 = 0.0;
        double p11 = 1.0;
    };

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
        nh.param("radar_bootstrap_samples", radar_bootstrap_samples_, 2);
        nh.param("filter_alpha", filter_alpha_, 0.70);
        nh.param("filter_beta", filter_beta_, 0.20);
        nh.param("radar_measurement_std_m", radar_measurement_std_m_, 5.773502692);
        nh.param("radar_process_accel_std_mps2",
                 radar_process_accel_std_mps2_, 0.20);
        nh.param("radar_bootstrap_huber_delta_m",
                 radar_bootstrap_huber_delta_m_, 10.0);
        nh.param("radar_nis_soft_threshold", radar_nis_soft_threshold_, 9.21);
        nh.param("radar_nis_reject_threshold", radar_nis_reject_threshold_, 25.0);
        nh.param("target_altitude_process_std_mps",
                 target_altitude_process_std_mps_, 0.10);
        nh.param("max_target_speed_mps", max_target_speed_mps_, 25.0);
        nh.param("max_target_horizontal_speed_mps",
                 max_target_horizontal_speed_mps_, -1.0);
        nh.param("max_target_vertical_speed_mps",
                 max_target_vertical_speed_mps_, -1.0);
        nh.param("max_radar_innovation_m", max_radar_innovation_m_, 80.0);

        nh.param("terminal_camera_range_m", terminal_camera_range_m_, 20.0);
        nh.param("camera_mount_pitch_deg", camera_mount_pitch_deg_, -20.0);
        nh.param("camera_mount_yaw_deg", camera_mount_yaw_deg_, 0.0);
        nh.param("camera_offset_body_x_m", camera_offset_body_x_m_, 0.0);
        nh.param("camera_offset_body_y_m", camera_offset_body_y_m_, 0.0);
        nh.param("camera_offset_body_z_m", camera_offset_body_z_m_, 0.0);
        nh.param("enforce_minimum_handover_altitude",
                 enforce_minimum_handover_altitude_, false);
        nh.param("minimum_handover_altitude_m",
                 minimum_handover_altitude_m_, 0.0);
        nh.param("minimum_course_speed_mps", minimum_course_speed_mps_, 0.5);
        nh.param("course_full_confidence_std_deg",
                 course_full_confidence_std_deg_, 12.0);
        nh.param("course_zero_confidence_std_deg",
                 course_zero_confidence_std_deg_, 35.0);

        nh.param("position_gain_xy", position_gain_xy_, 0.55);
        nh.param("position_gain_z", position_gain_z_, 0.45);
        nh.param("target_velocity_feedforward_gain_xy",
                 target_velocity_feedforward_gain_xy_, 1.0);
        nh.param("target_velocity_feedforward_gain_z",
                 target_velocity_feedforward_gain_z_, 1.0);
        nh.param("yaw_gain", yaw_gain_, 1.2);
        nh.param("max_approach_speed_mps", max_approach_speed_mps_, 12.0);
        nh.param("max_vertical_speed_mps", max_vertical_speed_mps_, 2.0);
        nh.param("max_yaw_rate_radps", max_yaw_rate_radps_, 0.8);
        nh.param("max_position_correction_speed_mps",
                 max_position_correction_speed_mps_, 6.0);
        nh.param("near_target_range_m", near_target_range_m_, 35.0);
        nh.param("near_position_correction_speed_mps",
                 near_position_correction_speed_mps_, 1.5);
        nh.param("use_braking_speed_profile",
                 use_braking_speed_profile_, false);
        nh.param("approach_braking_deceleration_mps2",
                 approach_braking_deceleration_mps2_, 2.0);
        nh.param("approach_braking_position_buffer_m",
                 approach_braking_position_buffer_m_, 2.5);
        nh.param("max_approach_acceleration_mps2",
                 max_approach_acceleration_mps2_, 1.5);
        nh.param("max_vertical_acceleration_mps2",
                 max_vertical_acceleration_mps2_, 0.8);
        nh.param("max_yaw_acceleration_radps2",
                 max_yaw_acceleration_radps2_, 0.6);
        nh.param("radar_coast_slowdown_start_sec",
                 radar_coast_slowdown_start_sec_, 5.0);
        nh.param("radar_coast_max_speed_mps", radar_coast_max_speed_mps_, 1.0);
        nh.param("visual_alignment_blend", visual_alignment_blend_, 0.5);
        nh.param("visual_alignment_max_vz_mps",
                 visual_alignment_max_vz_mps_, 0.6);
        nh.param("visual_alignment_max_yaw_rate_radps",
                 visual_alignment_max_yaw_rate_radps_, 0.35);
        nh.param("closing_speed_margin_mps", closing_speed_margin_mps_, 2.0);
        nh.param("min_handover_forward_speed_mps",
                 min_handover_forward_speed_mps_, 3.0);
        nh.param("max_handover_forward_speed_mps",
                 max_handover_forward_speed_mps_, 12.0);

        nh.param("handover_position_tolerance_m",
                 handover_position_tolerance_m_, 2.5);
        nh.param("use_legacy_pose_handover_gate",
                 use_legacy_pose_handover_gate_, false);
        nh.param("handover_min_estimated_range_m",
                 handover_min_estimated_range_m_, 5.0);
        nh.param("handover_max_estimated_range_m",
                 handover_max_estimated_range_m_, 45.0);
        nh.param("max_handover_velocity_std_mps",
                 max_handover_velocity_std_mps_, 1.0);
        nh.param("max_handover_radar_age_sec",
                 max_handover_radar_age_sec_, 5.0);
        nh.param("max_handover_command_yaw_rate_radps",
                 max_handover_command_yaw_rate_radps_, 0.30);
        nh.param("max_handover_closing_speed_mps",
                 max_handover_closing_speed_mps_, 2.0);
        double handover_yaw_tolerance_deg = 10.0;
        nh.param("handover_yaw_tolerance_deg",
                 handover_yaw_tolerance_deg, 10.0);
        handover_yaw_tolerance_rad_ = degToRad(handover_yaw_tolerance_deg);
        nh.param("ibvs_candidate_timeout_sec", ibvs_candidate_timeout_sec_, 0.20);
        nh.param("handover_blend_duration_sec",
                 handover_blend_duration_sec_, 0.0);
        nh.param("control_rate_hz", control_rate_hz_, 20.0);

        expected_radar_period_sec_ = std::max(expected_radar_period_sec_, 0.1);
        radar_timeout_sec_ = std::max(radar_timeout_sec_, expected_radar_period_sec_);
        radar_bootstrap_samples_ = std::max(radar_bootstrap_samples_, 2);
        filter_alpha_ = clampValue(filter_alpha_, 0.0, 1.0);
        filter_beta_ = clampValue(filter_beta_, 0.0, 1.0);
        radar_measurement_std_m_ = std::max(radar_measurement_std_m_, 0.01);
        radar_process_accel_std_mps2_ =
            std::max(radar_process_accel_std_mps2_, 0.0);
        radar_bootstrap_huber_delta_m_ =
            std::max(radar_bootstrap_huber_delta_m_, 0.01);
        radar_nis_soft_threshold_ = std::max(radar_nis_soft_threshold_, 0.01);
        radar_nis_reject_threshold_ = std::max(
            radar_nis_reject_threshold_, radar_nis_soft_threshold_);
        target_altitude_process_std_mps_ =
            std::max(target_altitude_process_std_mps_, 0.0);
        max_target_speed_mps_ = std::max(max_target_speed_mps_, 0.1);
        target_velocity_feedforward_gain_xy_ =
            std::max(target_velocity_feedforward_gain_xy_, 0.0);
        target_velocity_feedforward_gain_z_ =
            std::max(target_velocity_feedforward_gain_z_, 0.0);
        terminal_camera_range_m_ = std::max(terminal_camera_range_m_, 1.0);
        max_approach_speed_mps_ = std::max(max_approach_speed_mps_, 0.1);
        max_vertical_speed_mps_ = std::max(max_vertical_speed_mps_, 0.1);
        max_yaw_rate_radps_ = std::max(max_yaw_rate_radps_, 0.0);
        max_position_correction_speed_mps_ =
            std::max(max_position_correction_speed_mps_, 0.0);
        near_target_range_m_ = std::max(near_target_range_m_, 1.0);
        near_position_correction_speed_mps_ = clampValue(
            near_position_correction_speed_mps_, 0.0,
            max_position_correction_speed_mps_);
        approach_braking_deceleration_mps2_ =
            std::max(approach_braking_deceleration_mps2_, 0.01);
        approach_braking_position_buffer_m_ =
            std::max(approach_braking_position_buffer_m_, 0.0);
        max_approach_acceleration_mps2_ =
            std::max(max_approach_acceleration_mps2_, 0.0);
        max_vertical_acceleration_mps2_ =
            std::max(max_vertical_acceleration_mps2_, 0.0);
        max_yaw_acceleration_radps2_ =
            std::max(max_yaw_acceleration_radps2_, 0.0);
        radar_coast_slowdown_start_sec_ = clampValue(
            radar_coast_slowdown_start_sec_, 0.0, radar_timeout_sec_);
        radar_coast_max_speed_mps_ = std::max(radar_coast_max_speed_mps_, 0.0);
        visual_alignment_blend_ = clampValue(visual_alignment_blend_, 0.0, 1.0);
        visual_alignment_max_vz_mps_ =
            std::max(visual_alignment_max_vz_mps_, 0.0);
        visual_alignment_max_yaw_rate_radps_ =
            std::max(visual_alignment_max_yaw_rate_radps_, 0.0);
        min_handover_forward_speed_mps_ =
            std::max(min_handover_forward_speed_mps_, 0.0);
        max_handover_forward_speed_mps_ = std::max(
            max_handover_forward_speed_mps_, min_handover_forward_speed_mps_);
        handover_min_estimated_range_m_ =
            std::max(handover_min_estimated_range_m_, 0.0);
        handover_max_estimated_range_m_ = std::max(
            handover_max_estimated_range_m_, handover_min_estimated_range_m_);
        max_handover_velocity_std_mps_ =
            std::max(max_handover_velocity_std_mps_, 0.0);
        max_handover_radar_age_sec_ =
            std::max(max_handover_radar_age_sec_, 0.0);
        max_handover_command_yaw_rate_radps_ =
            std::max(max_handover_command_yaw_rate_radps_, 0.0);
        max_handover_closing_speed_mps_ =
            std::max(max_handover_closing_speed_mps_, 0.0);
        handover_blend_duration_sec_ =
            std::max(handover_blend_duration_sec_, 0.0);
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
        const ros::Time received_time = ros::Time::now();
        const ros::Time measurement_time =
            validMeasurementTime(message->header.stamp, received_time);
        updateRadarFilter(measurement, measurement_time, received_time);
    }

    void radarLocalCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
        const ros::Time received_time = ros::Time::now();
        const ros::Time measurement_time =
            validMeasurementTime(message->header.stamp, received_time);
        updateRadarFilter(message->pose.position, measurement_time, received_time);
    }

    ros::Time validMeasurementTime(const ros::Time& message_time,
                                   const ros::Time& received_time) const {
        if (message_time.isZero()) return received_time;
        if (message_time > received_time + ros::Duration(0.1)) {
            ROS_WARN_THROTTLE(
                2.0,
                "[RadarApproach] Radar timestamp is in the future; using receipt time.");
            return received_time;
        }
        return message_time;
    }

    void updateRadarFilter(const geometry_msgs::Point& measurement,
                           const ros::Time& measurement_time,
                           const ros::Time& received_time) {
        if (!std::isfinite(measurement.x) || !std::isfinite(measurement.y) ||
            !std::isfinite(measurement.z)) {
            ROS_WARN_THROTTLE(2.0,
                "[RadarApproach] Ignoring non-finite radar measurement.");
            return;
        }

        if (!have_radar_filter_) {
            filtered_target_position_ = measurement;
            filtered_target_velocity_.x = 0.0;
            filtered_target_velocity_.y = 0.0;
            filtered_target_velocity_.z = 0.0;
            radar_filter_time_ = measurement_time;
            radar_sample_count_ = 1;
            have_radar_filter_ = true;
            radar_last_received_ = received_time;
            radar_bootstrap_measurements_.clear();
            radar_bootstrap_measurements_.push_back(
                TimedRadarMeasurement{measurement, measurement_time});
            return;
        }

        if (!radar_filter_ready_) {
            if (!radar_bootstrap_measurements_.empty() &&
                measurement_time <=
                    radar_bootstrap_measurements_.back().stamp + ros::Duration(0.05)) {
                ROS_WARN_THROTTLE(2.0,
                    "[RadarApproach] Ignoring duplicate or out-of-order radar sample.");
                return;
            }
            radar_bootstrap_measurements_.push_back(
                TimedRadarMeasurement{measurement, measurement_time});
            filtered_target_position_ = measurement;
            radar_filter_time_ = measurement_time;
            radar_last_received_ = received_time;
            radar_sample_count_ =
                static_cast<int>(radar_bootstrap_measurements_.size());
            if (radar_sample_count_ < radar_bootstrap_samples_) return;

            if (!initializeRadarFilterFromBootstrap()) {
                ROS_WARN_THROTTLE(
                    2.0,
                    "[RadarApproach] Radar bootstrap fit is degenerate; "
                    "waiting for another sample.");
                return;
            }
            radar_filter_ready_ = true;
            radar_bootstrap_measurements_.clear();
            ROS_INFO("[RadarApproach] Horizontal-CV Kalman filter ready after "
                     "%d robust bootstrap samples; velocity sigma %.2f m/s.",
                     radar_sample_count_, horizontalVelocityStd());
            return;
        }

        const double dt = (measurement_time - radar_filter_time_).toSec();
        if (dt <= 0.05) {
            ROS_WARN_THROTTLE(2.0,
                "[RadarApproach] Ignoring duplicate or out-of-order radar sample.");
            return;
        }

        predictAxis(x_filter_, dt);
        predictAxis(y_filter_, dt);
        altitude_variance_ +=
            std::pow(target_altitude_process_std_mps_ * dt, 2.0);
        radar_filter_time_ = measurement_time;

        const double residual_x = measurement.x - x_filter_.position;
        const double residual_y = measurement.y - y_filter_.position;
        const double measurement_variance =
            radar_measurement_std_m_ * radar_measurement_std_m_;
        const double innovation_variance_x = x_filter_.p00 + measurement_variance;
        const double innovation_variance_y = y_filter_.p00 + measurement_variance;
        const double nis = residual_x * residual_x / innovation_variance_x +
                           residual_y * residual_y / innovation_variance_y;
        const double horizontal_innovation = std::hypot(residual_x, residual_y);
        if (!std::isfinite(nis) || nis > radar_nis_reject_threshold_ ||
            (max_radar_innovation_m_ > 0.0 &&
             horizontal_innovation > max_radar_innovation_m_)) {
            ++radar_rejected_sample_count_;
            ROS_WARN_THROTTLE(2.0,
                "[RadarApproach] Radar sample rejected: NIS %.1f, "
                "horizontal innovation %.1f m.", nis, horizontal_innovation);
            syncFilteredState();
            return;
        }

        double robust_weight = 1.0;
        if (nis > radar_nis_soft_threshold_) {
            robust_weight = std::sqrt(radar_nis_soft_threshold_ / nis);
        }
        const double robust_measurement_variance =
            measurement_variance / std::max(robust_weight, 1e-3);
        updateAxis(x_filter_, measurement.x, robust_measurement_variance);
        updateAxis(y_filter_, measurement.y, robust_measurement_variance);

        const double altitude_residual =
            measurement.z - filtered_target_position_.z;
        const double altitude_innovation_variance =
            altitude_variance_ + measurement_variance;
        const double altitude_nis = altitude_residual * altitude_residual /
                                    altitude_innovation_variance;
        if (std::isfinite(altitude_nis) &&
            altitude_nis <= radar_nis_reject_threshold_) {
            double altitude_weight = 1.0;
            if (altitude_nis > radar_nis_soft_threshold_) {
                altitude_weight = std::sqrt(
                    radar_nis_soft_threshold_ / altitude_nis);
            }
            const double altitude_r =
                measurement_variance / std::max(altitude_weight, 1e-3);
            const double gain = altitude_variance_ /
                                (altitude_variance_ + altitude_r);
            filtered_target_position_.z += gain * altitude_residual;
            altitude_variance_ = std::max(
                (1.0 - gain) * altitude_variance_, 1e-6);
        }

        radar_last_received_ = received_time;
        ++radar_sample_count_;
        syncFilteredState();
        limitTargetVelocity();
    }

    bool initializeRadarFilterFromBootstrap() {
        const std::size_t count = radar_bootstrap_measurements_.size();
        if (count < 2) return false;

        const ros::Time reference = radar_bootstrap_measurements_.back().stamp;
        std::vector<double> weights(count, 1.0);
        double position_x = 0.0;
        double velocity_x = 0.0;
        double position_y = 0.0;
        double velocity_y = 0.0;
        double scratch_p00 = 0.0;
        double scratch_p01 = 0.0;
        double scratch_p11 = 0.0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            if (!weightedLineFit(0, weights, reference, position_x, velocity_x,
                                 scratch_p00, scratch_p01, scratch_p11) ||
                !weightedLineFit(1, weights, reference, position_y, velocity_y,
                                 scratch_p00, scratch_p01, scratch_p11)) {
                return false;
            }
            for (std::size_t index = 0; index < count; ++index) {
                const TimedRadarMeasurement& sample =
                    radar_bootstrap_measurements_[index];
                const double sample_dt = (sample.stamp - reference).toSec();
                const double residual = std::hypot(
                    sample.position.x - (position_x + velocity_x * sample_dt),
                    sample.position.y - (position_y + velocity_y * sample_dt));
                weights[index] = residual <= radar_bootstrap_huber_delta_m_ ?
                    1.0 : radar_bootstrap_huber_delta_m_ / residual;
            }
        }

        double x_p00 = 0.0;
        double x_p01 = 0.0;
        double x_p11 = 0.0;
        double y_p00 = 0.0;
        double y_p01 = 0.0;
        double y_p11 = 0.0;
        if (!weightedLineFit(0, weights, reference, position_x, velocity_x,
                             x_p00, x_p01, x_p11) ||
            !weightedLineFit(1, weights, reference, position_y, velocity_y,
                             y_p00, y_p01, y_p11)) {
            return false;
        }

        x_filter_.position = position_x;
        x_filter_.velocity = velocity_x;
        x_filter_.p00 = x_p00;
        x_filter_.p01 = x_p01;
        x_filter_.p10 = x_p01;
        x_filter_.p11 = x_p11;
        y_filter_.position = position_y;
        y_filter_.velocity = velocity_y;
        y_filter_.p00 = y_p00;
        y_filter_.p01 = y_p01;
        y_filter_.p10 = y_p01;
        y_filter_.p11 = y_p11;

        filtered_target_position_.z = robustBootstrapAltitude(weights);
        double weight_sum = 0.0;
        for (double weight : weights) weight_sum += weight;
        altitude_variance_ =
            radar_measurement_std_m_ * radar_measurement_std_m_ /
            std::max(weight_sum, 1.0);
        radar_filter_time_ = reference;
        syncFilteredState();
        limitTargetVelocity();

        return std::isfinite(filtered_target_position_.x) &&
            std::isfinite(filtered_target_position_.y) &&
            std::isfinite(filtered_target_position_.z) &&
            std::isfinite(filtered_target_velocity_.x) &&
            std::isfinite(filtered_target_velocity_.y) &&
            std::isfinite(filtered_target_velocity_.z);
    }

    double sampleCoordinate(const geometry_msgs::Point& point, int axis) const {
        if (axis == 0) return point.x;
        if (axis == 1) return point.y;
        return point.z;
    }

    bool weightedLineFit(int axis, const std::vector<double>& weights,
                         const ros::Time& reference, double& position,
                         double& velocity, double& p00, double& p01,
                         double& p11) const {
        if (weights.size() != radar_bootstrap_measurements_.size()) return false;
        double weight_sum = 0.0;
        double mean_time = 0.0;
        double mean_value = 0.0;
        for (std::size_t index = 0; index < weights.size(); ++index) {
            const double weight = std::max(weights[index], 1e-6);
            const TimedRadarMeasurement& sample =
                radar_bootstrap_measurements_[index];
            const double time = (sample.stamp - reference).toSec();
            weight_sum += weight;
            mean_time += weight * time;
            mean_value += weight * sampleCoordinate(sample.position, axis);
        }
        if (weight_sum <= 1e-9) return false;
        mean_time /= weight_sum;
        mean_value /= weight_sum;

        double denominator = 0.0;
        double numerator = 0.0;
        for (std::size_t index = 0; index < weights.size(); ++index) {
            const double weight = std::max(weights[index], 1e-6);
            const TimedRadarMeasurement& sample =
                radar_bootstrap_measurements_[index];
            const double centered_time =
                (sample.stamp - reference).toSec() - mean_time;
            denominator += weight * centered_time * centered_time;
            numerator += weight * centered_time *
                         sampleCoordinate(sample.position, axis);
        }
        if (!std::isfinite(denominator) || denominator <= 1e-9) return false;

        velocity = numerator / denominator;
        position = mean_value - velocity * mean_time;
        const double measurement_variance =
            radar_measurement_std_m_ * radar_measurement_std_m_;
        p11 = measurement_variance / denominator;
        p00 = measurement_variance *
              (1.0 / weight_sum + mean_time * mean_time / denominator);
        p01 = -measurement_variance * mean_time / denominator;
        return std::isfinite(position) && std::isfinite(velocity) &&
               std::isfinite(p00) && std::isfinite(p01) && std::isfinite(p11);
    }

    double robustBootstrapAltitude(const std::vector<double>& xy_weights) const {
        double altitude = 0.0;
        double weight_sum = 0.0;
        for (std::size_t index = 0; index < xy_weights.size(); ++index) {
            const double weight = std::max(xy_weights[index], 1e-6);
            altitude += weight * radar_bootstrap_measurements_[index].position.z;
            weight_sum += weight;
        }
        altitude /= std::max(weight_sum, 1e-9);
        for (int iteration = 0; iteration < 4; ++iteration) {
            double numerator = 0.0;
            double denominator = 0.0;
            for (std::size_t index = 0; index < xy_weights.size(); ++index) {
                const double residual = std::abs(
                    radar_bootstrap_measurements_[index].position.z - altitude);
                const double huber_weight =
                    residual <= radar_bootstrap_huber_delta_m_ ?
                    1.0 : radar_bootstrap_huber_delta_m_ / residual;
                const double weight = std::max(xy_weights[index], 1e-6) *
                                      huber_weight;
                numerator += weight *
                    radar_bootstrap_measurements_[index].position.z;
                denominator += weight;
            }
            altitude = numerator / std::max(denominator, 1e-9);
        }
        return altitude;
    }

    void predictAxis(AxisKalmanState& state, double dt) const {
        state.position += state.velocity * dt;
        const double old_p00 = state.p00;
        const double old_p01 = state.p01;
        const double old_p10 = state.p10;
        const double old_p11 = state.p11;
        const double accel_variance =
            radar_process_accel_std_mps2_ * radar_process_accel_std_mps2_;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        state.p00 = old_p00 + dt * (old_p01 + old_p10) +
                    dt2 * old_p11 + 0.25 * accel_variance * dt4;
        state.p01 = old_p01 + dt * old_p11 +
                    0.5 * accel_variance * dt3;
        state.p10 = old_p10 + dt * old_p11 +
                    0.5 * accel_variance * dt3;
        state.p11 = old_p11 + accel_variance * dt2;
    }

    void updateAxis(AxisKalmanState& state, double measurement,
                    double measurement_variance) const {
        const double innovation_variance = state.p00 + measurement_variance;
        const double gain_position = state.p00 / innovation_variance;
        const double gain_velocity = state.p10 / innovation_variance;
        const double residual = measurement - state.position;
        state.position += gain_position * residual;
        state.velocity += gain_velocity * residual;

        const double old_p00 = state.p00;
        const double old_p01 = state.p01;
        const double old_p10 = state.p10;
        const double old_p11 = state.p11;
        state.p00 = (1.0 - gain_position) * old_p00;
        state.p01 = (1.0 - gain_position) * old_p01;
        state.p10 = old_p10 - gain_velocity * old_p00;
        state.p11 = old_p11 - gain_velocity * old_p01;
        const double symmetric_cross = 0.5 * (state.p01 + state.p10);
        state.p01 = symmetric_cross;
        state.p10 = symmetric_cross;
        state.p00 = std::max(state.p00, 1e-6);
        state.p11 = std::max(state.p11, 1e-6);
    }

    void syncFilteredState() {
        filtered_target_position_.x = x_filter_.position;
        filtered_target_position_.y = y_filter_.position;
        filtered_target_velocity_.x = x_filter_.velocity;
        filtered_target_velocity_.y = y_filter_.velocity;
        filtered_target_velocity_.z = 0.0;
    }

    double horizontalVelocityStd() const {
        return std::sqrt(std::max(x_filter_.p11, y_filter_.p11));
    }

    double horizontalVelocityStdAt(const ros::Time& time) const {
        const double dt = std::max(0.0, (time - radar_filter_time_).toSec());
        const double process_variance =
            radar_process_accel_std_mps2_ * radar_process_accel_std_mps2_ *
            dt * dt;
        return std::sqrt(std::max(x_filter_.p11, y_filter_.p11) +
                         process_variance);
    }

    void limitTargetVelocity() {
        if (max_target_horizontal_speed_mps_ > 0.0) {
            limitHorizontal(max_target_horizontal_speed_mps_,
                            filtered_target_velocity_.x,
                            filtered_target_velocity_.y);
        }
        if (max_target_vertical_speed_mps_ > 0.0) {
            filtered_target_velocity_.z = clampValue(
                filtered_target_velocity_.z,
                -max_target_vertical_speed_mps_,
                max_target_vertical_speed_mps_);
        }
        const double speed = std::sqrt(
            filtered_target_velocity_.x * filtered_target_velocity_.x +
            filtered_target_velocity_.y * filtered_target_velocity_.y +
            filtered_target_velocity_.z * filtered_target_velocity_.z);
        if (speed > max_target_speed_mps_ && speed >= 1e-9) {
            const double scale = max_target_speed_mps_ / speed;
            filtered_target_velocity_.x *= scale;
            filtered_target_velocity_.y *= scale;
            filtered_target_velocity_.z *= scale;
        }
        x_filter_.velocity = filtered_target_velocity_.x;
        y_filter_.velocity = filtered_target_velocity_.y;
    }

    template <typename MessageT>
    void updateDetection(const MessageT& message) {
        const ros::Time now = ros::Time::now();
        const bool detection_stream_continuous = detection_time_.isZero() ||
            (now - detection_time_).toSec() <= detection_timeout_sec_;
        detection_time_ = now;
        detection_valid_ =
            message.is_tracking && message.score >= minimum_detection_score_;
        if (!detection_valid_) {
            centered_detection_count_ = 0;
            return;
        }

        if (!detection_stream_continuous) {
            centered_detection_count_ = 0;
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
        const double line_of_sight_yaw = std::atan2(
            target.y - self_pose_.pose.position.y,
            target.x - self_pose_.pose.position.x);
        double optical_yaw = line_of_sight_yaw;
        if (target_horizontal_speed >= minimum_course_speed_mps_) {
            const double motion_yaw = std::atan2(filtered_target_velocity_.y,
                                                 filtered_target_velocity_.x);
            const double course_std_deg = 180.0 / kPi * std::atan2(
                horizontalVelocityStdAt(stamp), target_horizontal_speed);
            const double confidence = clampValue(
                (course_zero_confidence_std_deg_ - course_std_deg) /
                std::max(course_zero_confidence_std_deg_ -
                         course_full_confidence_std_deg_, 1e-6),
                0.0, 1.0);
            optical_yaw = wrapAngle(
                line_of_sight_yaw + confidence *
                wrapAngle(motion_yaw - line_of_sight_yaw));
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
        if (enforce_minimum_handover_altitude_) {
            pose.pose.position.z = std::max(
                pose.pose.position.z, minimum_handover_altitude_m_);
        }
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
        double desired_body_yaw, double target_range,
        const ros::Time& now) {
        geometry_msgs::TwistStamped command;
        command.header.stamp = now;
        command.header.frame_id = "map";
        double correction_x = position_gain_xy_ *
            (desired_pose.pose.position.x - self_pose_.pose.position.x);
        double correction_y = position_gain_xy_ *
            (desired_pose.pose.position.y - self_pose_.pose.position.y);
        const double far_blend = clampValue(
            (target_range - near_target_range_m_) /
            std::max(near_target_range_m_, 1.0), 0.0, 1.0);
        double correction_limit = near_position_correction_speed_mps_ +
            far_blend * (max_position_correction_speed_mps_ -
                         near_position_correction_speed_mps_);
        if (use_braking_speed_profile_) {
            const double handover_position_error = std::hypot(
                desired_pose.pose.position.x - self_pose_.pose.position.x,
                desired_pose.pose.position.y - self_pose_.pose.position.y);
            const double braking_distance = std::max(
                handover_position_error -
                    approach_braking_position_buffer_m_,
                0.0);
            correction_limit = std::min(
                max_position_correction_speed_mps_,
                std::sqrt(2.0 * approach_braking_deceleration_mps2_ *
                          braking_distance));
        }
        limitHorizontal(correction_limit, correction_x, correction_y);
        command.twist.linear.x =
            target_velocity_feedforward_gain_xy_ * filtered_target_velocity_.x +
            correction_x;
        command.twist.linear.y =
            target_velocity_feedforward_gain_xy_ * filtered_target_velocity_.y +
            correction_y;
        command.twist.linear.z =
            target_velocity_feedforward_gain_z_ * filtered_target_velocity_.z +
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

        const bool detection_fresh = detection_valid_ &&
            (now - detection_time_).toSec() <= detection_timeout_sec_;
        const bool candidate_fresh = have_ibvs_candidate_ &&
            (now - ibvs_candidate_time_).toSec() <= ibvs_candidate_timeout_sec_;
        if (detection_fresh && candidate_fresh) {
            const double visual_vz = clampValue(
                ibvs_candidate_.twist.linear.z,
                -visual_alignment_max_vz_mps_,
                visual_alignment_max_vz_mps_);
            const double visual_yaw_rate = clampValue(
                ibvs_candidate_.twist.angular.z,
                -visual_alignment_max_yaw_rate_radps_,
                visual_alignment_max_yaw_rate_radps_);
            command.twist.linear.z =
                (1.0 - visual_alignment_blend_) * command.twist.linear.z +
                visual_alignment_blend_ * visual_vz;
            command.twist.angular.z =
                (1.0 - visual_alignment_blend_) * command.twist.angular.z +
                visual_alignment_blend_ * visual_yaw_rate;
        }

        const double radar_age = std::max(
            0.0, (now - radar_last_received_).toSec());
        if (radar_age > radar_coast_slowdown_start_sec_) {
            limitHorizontal(radar_coast_max_speed_mps_,
                            command.twist.linear.x,
                            command.twist.linear.y);
        }
        return rateLimitApproachCommand(command, now);
    }

    geometry_msgs::TwistStamped rateLimitApproachCommand(
        const geometry_msgs::TwistStamped& requested, const ros::Time& now) {
        if (!have_last_approach_command_) {
            last_approach_command_ = zeroCommand(now);
            last_approach_command_time_ = now;
            have_last_approach_command_ = true;
        }
        const double dt = clampValue(
            (now - last_approach_command_time_).toSec(), 0.0, 0.20);
        geometry_msgs::TwistStamped limited = requested;
        double delta_x = requested.twist.linear.x -
                         last_approach_command_.twist.linear.x;
        double delta_y = requested.twist.linear.y -
                         last_approach_command_.twist.linear.y;
        limitHorizontal(max_approach_acceleration_mps2_ * dt,
                        delta_x, delta_y);
        limited.twist.linear.x =
            last_approach_command_.twist.linear.x + delta_x;
        limited.twist.linear.y =
            last_approach_command_.twist.linear.y + delta_y;
        limited.twist.linear.z = last_approach_command_.twist.linear.z +
            clampValue(requested.twist.linear.z -
                       last_approach_command_.twist.linear.z,
                       -max_vertical_acceleration_mps2_ * dt,
                       max_vertical_acceleration_mps2_ * dt);
        limited.twist.angular.z = last_approach_command_.twist.angular.z +
            clampValue(requested.twist.angular.z -
                       last_approach_command_.twist.angular.z,
                       -max_yaw_acceleration_radps2_ * dt,
                       max_yaw_acceleration_radps2_ * dt);
        limited.header.stamp = now;
        last_approach_command_ = limited;
        last_approach_command_time_ = now;
        return limited;
    }

    void resetApproachCommandLimiter(const ros::Time& now) {
        last_approach_command_ = zeroCommand(now);
        last_approach_command_time_ = now;
        have_last_approach_command_ = false;
    }

    geometry_msgs::TwistStamped zeroCommand(const ros::Time& now) const {
        geometry_msgs::TwistStamped command;
        command.header.stamp = now;
        command.header.frame_id = "map";
        return command;
    }

    geometry_msgs::TwistStamped blendedIbvsCommand(
        const geometry_msgs::TwistStamped& ibvs_command,
        const ros::Time& now) const {
        geometry_msgs::TwistStamped command = ibvs_command;
        command.header.stamp = now;
        if (handover_blend_duration_sec_ <= 0.0 ||
            !have_handover_blend_start_command_ || handover_time_.isZero()) {
            return command;
        }

        const double elapsed_sec = std::max(0.0, (now - handover_time_).toSec());
        const double blend = clampValue(
            elapsed_sec / handover_blend_duration_sec_, 0.0, 1.0);
        const double keep = 1.0 - blend;
        command.twist.linear.x =
            keep * handover_blend_start_command_.twist.linear.x +
            blend * ibvs_command.twist.linear.x;
        command.twist.linear.y =
            keep * handover_blend_start_command_.twist.linear.y +
            blend * ibvs_command.twist.linear.y;
        command.twist.linear.z =
            keep * handover_blend_start_command_.twist.linear.z +
            blend * ibvs_command.twist.linear.z;
        command.twist.angular.z =
            keep * handover_blend_start_command_.twist.angular.z +
            blend * ibvs_command.twist.angular.z;
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
            case State::VISUAL_ALIGN: message.data = "VISUAL_ALIGN"; break;
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
        if (!have_self_pose_) {
            setState(State::WAIT_RADAR);
            publishHandoverFlag(false);
            resetApproachCommandLimiter(now);
            final_command_pub_.publish(zeroCommand(now));
            return;
        }

        if (!handover_complete_) {
            if (!have_radar_filter_ || !radar_filter_ready_) {
                setState(State::WAIT_RADAR);
                publishHandoverFlag(false);
                resetApproachCommandLimiter(now);
                final_command_pub_.publish(zeroCommand(now));
                return;
            }

            if ((now - radar_last_received_).toSec() > radar_timeout_sec_) {
                setState(State::RADAR_TIMEOUT);
                publishHandoverFlag(false);
                resetApproachCommandLimiter(now);
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

            const double position_error = pointDistance(
                self_pose_.pose.position, handover_pose.pose.position);
            const double yaw_error = std::abs(
                wrapAngle(desired_body_yaw - self_yaw_));
            const double target_range = pointDistance(
                self_pose_.pose.position, target_now);
            const bool detection_fresh = detection_valid_ &&
                (now - detection_time_).toSec() <= detection_timeout_sec_;
            setState(detection_fresh ? State::VISUAL_ALIGN : State::APPROACH);
            const bool visual_ready = detection_fresh &&
                centered_detection_count_ >= required_centered_detections_;
            const bool ibvs_candidate_fresh = have_ibvs_candidate_ &&
                (now - ibvs_candidate_time_).toSec() <=
                    ibvs_candidate_timeout_sec_;
            const geometry_msgs::TwistStamped approach_command =
                approachCommand(handover_pose, desired_body_yaw,
                                target_range, now);

            const double radar_age = std::max(
                0.0, (now - radar_last_received_).toSec());
            const double velocity_std = horizontalVelocityStdAt(now);
            const bool radar_healthy =
                radar_sample_count_ >= radar_bootstrap_samples_ &&
                velocity_std <= max_handover_velocity_std_mps_ &&
                radar_age <= max_handover_radar_age_sec_;
            const bool range_safe =
                target_range >= handover_min_estimated_range_m_ &&
                target_range <= handover_max_estimated_range_m_;
            const bool pose_gate_satisfied = !use_legacy_pose_handover_gate_ ||
                (position_error <= handover_position_tolerance_m_ &&
                 yaw_error <= handover_yaw_tolerance_rad_);
            const double horizontal_range = std::hypot(
                target_now.x - self_pose_.pose.position.x,
                target_now.y - self_pose_.pose.position.y);
            double closing_speed = 0.0;
            if (horizontal_range > 1e-6) {
                closing_speed =
                    ((approach_command.twist.linear.x -
                      filtered_target_velocity_.x) *
                         (target_now.x - self_pose_.pose.position.x) +
                     (approach_command.twist.linear.y -
                      filtered_target_velocity_.y) *
                         (target_now.y - self_pose_.pose.position.y)) /
                    horizontal_range;
            }
            const bool command_stable =
                std::abs(approach_command.twist.angular.z) <=
                    max_handover_command_yaw_rate_radps_ &&
                closing_speed <= max_handover_closing_speed_mps_;

            if (visual_ready && ibvs_candidate_fresh && radar_healthy &&
                range_safe && pose_gate_satisfied && command_stable) {
                handover_time_ = now;
                handover_blend_start_command_ = approach_command;
                handover_blend_start_command_.header.stamp = now;
                have_handover_blend_start_command_ = true;
                handover_complete_ = true;
                setState(State::HANDOVER);
                publishHandoverFlag(true);
                ROS_INFO("[RadarApproach] Handover: estimated range %.2f m, "
                         "pose error %.2f m, yaw error %.1f deg, "
                         "velocity sigma %.2f m/s, radar age %.2f s, "
                         "closing speed %.2f m/s, centered detections %d.",
                         target_range,
                         position_error, yaw_error * 180.0 / kPi,
                         velocity_std, radar_age, closing_speed,
                         centered_detection_count_);
            } else {
                publishHandoverFlag(false);
                final_command_pub_.publish(approach_command);
                return;
            }
        }

        publishHandoverFlag(true);
        const bool ibvs_candidate_fresh = have_ibvs_candidate_ &&
            (now - ibvs_candidate_time_).toSec() <= ibvs_candidate_timeout_sec_;
        if (ibvs_candidate_fresh) {
            final_command_pub_.publish(
                blendedIbvsCommand(ibvs_candidate_, now));
        } else {
            // Do not resume an approach-to-IBVS blend after a safety stop.  If
            // visual guidance later recovers, ownership is already with IBVS
            // and its current command is used directly.
            have_handover_blend_start_command_ = false;
            ROS_WARN_THROTTLE(1.0,
                "[RadarApproach] IBVS command stale after handover; "
                "publishing zero command.");
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
    int radar_bootstrap_samples_ = 2;
    double filter_alpha_ = 0.70;
    double filter_beta_ = 0.20;
    double radar_measurement_std_m_ = 5.773502692;
    double radar_process_accel_std_mps2_ = 0.20;
    double radar_bootstrap_huber_delta_m_ = 10.0;
    double radar_nis_soft_threshold_ = 9.21;
    double radar_nis_reject_threshold_ = 25.0;
    double target_altitude_process_std_mps_ = 0.10;
    double max_target_speed_mps_ = 25.0;
    double max_target_horizontal_speed_mps_ = -1.0;
    double max_target_vertical_speed_mps_ = -1.0;
    double max_radar_innovation_m_ = 80.0;
    double terminal_camera_range_m_ = 20.0;
    double camera_mount_pitch_deg_ = -20.0;
    double camera_mount_yaw_deg_ = 0.0;
    double camera_offset_body_x_m_ = 0.0;
    double camera_offset_body_y_m_ = 0.0;
    double camera_offset_body_z_m_ = 0.0;
    bool enforce_minimum_handover_altitude_ = false;
    double minimum_handover_altitude_m_ = 0.0;
    double minimum_course_speed_mps_ = 0.5;
    double course_full_confidence_std_deg_ = 12.0;
    double course_zero_confidence_std_deg_ = 35.0;
    double position_gain_xy_ = 0.55;
    double position_gain_z_ = 0.45;
    double target_velocity_feedforward_gain_xy_ = 1.0;
    double target_velocity_feedforward_gain_z_ = 1.0;
    double yaw_gain_ = 1.2;
    double max_approach_speed_mps_ = 12.0;
    double max_vertical_speed_mps_ = 2.0;
    double max_yaw_rate_radps_ = 0.8;
    double max_position_correction_speed_mps_ = 6.0;
    double near_target_range_m_ = 35.0;
    double near_position_correction_speed_mps_ = 1.5;
    bool use_braking_speed_profile_ = false;
    double approach_braking_deceleration_mps2_ = 2.0;
    double approach_braking_position_buffer_m_ = 2.5;
    double max_approach_acceleration_mps2_ = 1.5;
    double max_vertical_acceleration_mps2_ = 0.8;
    double max_yaw_acceleration_radps2_ = 0.6;
    double radar_coast_slowdown_start_sec_ = 5.0;
    double radar_coast_max_speed_mps_ = 1.0;
    double visual_alignment_blend_ = 0.5;
    double visual_alignment_max_vz_mps_ = 0.6;
    double visual_alignment_max_yaw_rate_radps_ = 0.35;
    double closing_speed_margin_mps_ = 2.0;
    double min_handover_forward_speed_mps_ = 3.0;
    double max_handover_forward_speed_mps_ = 12.0;
    double handover_position_tolerance_m_ = 2.5;
    double handover_yaw_tolerance_rad_ = degToRad(10.0);
    bool use_legacy_pose_handover_gate_ = false;
    double handover_min_estimated_range_m_ = 5.0;
    double handover_max_estimated_range_m_ = 45.0;
    double max_handover_velocity_std_mps_ = 1.0;
    double max_handover_radar_age_sec_ = 5.0;
    double max_handover_command_yaw_rate_radps_ = 0.30;
    double max_handover_closing_speed_mps_ = 2.0;
    double ibvs_candidate_timeout_sec_ = 0.20;
    double handover_blend_duration_sec_ = 0.0;
    double control_rate_hz_ = 20.0;

    sensor_msgs::NavSatFix self_global_;
    geometry_msgs::PoseStamped self_pose_;
    double self_yaw_ = 0.0;
    bool have_self_global_ = false;
    bool have_self_pose_ = false;

    geometry_msgs::Point filtered_target_position_;
    geometry_msgs::Vector3 filtered_target_velocity_;
    AxisKalmanState x_filter_;
    AxisKalmanState y_filter_;
    double altitude_variance_ = 1.0;
    ros::Time radar_filter_time_;
    ros::Time radar_last_received_;
    int radar_sample_count_ = 0;
    int radar_rejected_sample_count_ = 0;
    bool have_radar_filter_ = false;
    bool radar_filter_ready_ = false;
    std::vector<TimedRadarMeasurement> radar_bootstrap_measurements_;

    bool detection_valid_ = false;
    int centered_detection_count_ = 0;
    ros::Time detection_time_;
    geometry_msgs::TwistStamped ibvs_candidate_;
    ros::Time ibvs_candidate_time_;
    bool have_ibvs_candidate_ = false;
    geometry_msgs::TwistStamped handover_blend_start_command_;
    geometry_msgs::TwistStamped last_approach_command_;
    ros::Time last_approach_command_time_;
    bool have_last_approach_command_ = false;
    ros::Time handover_time_;
    bool have_handover_blend_start_command_ = false;
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
