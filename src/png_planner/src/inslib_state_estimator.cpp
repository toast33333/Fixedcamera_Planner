#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include "ahrs.h"
#include "ins.h"
#include "nav_suite.h"
}

namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double degrees) {
    return degrees * kPi / 180.0;
}

bool finite3(double x, double y, double z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

const char* modeName(nav_suite_mode_t mode) {
    switch (mode) {
        case NAV_SUITE_MODE_FULL:
            return "FULL";
        case NAV_SUITE_MODE_COASTING:
            return "COASTING";
        case NAV_SUITE_MODE_ATTITUDE_ONLY:
            return "ATTITUDE_ONLY";
        case NAV_SUITE_MODE_NONE:
        default:
            return "NONE";
    }
}

}  // namespace

class InslibStateEstimator {
public:
    InslibStateEstimator()
        : nh_(), pnh_("~"), suite_(new nav_suite_t{}) {
        loadParameters();
        initializeFilter();
        configureRos();

        ROS_INFO("[INSLIB] State estimator started. IMU=%s, GNSS=%s, "
                 "GNSS velocity=%s, mag=%s, baro=%s.",
                 imu_topic_.c_str(), use_gnss_position_ ? "on" : "off",
                 use_gnss_velocity_ ? "on" : "off",
                 use_magnetometer_ ? "on" : "off",
                 use_barometer_ ? "on" : "off");
        ROS_WARN("[INSLIB] This is an alternate state source. Keep the original "
                 "PX4-state launch available until replay and ground tests pass.");
    }

private:
    template <std::size_t N>
    void loadFloatArray(const std::string& name, float (&output)[N]) {
        std::vector<double> values;
        if (!pnh_.getParam(name, values)) return;
        if (values.size() != N) {
            throw std::runtime_error(name + " must contain exactly " +
                                     std::to_string(N) + " values");
        }
        for (std::size_t i = 0; i < N; ++i) {
            if (!std::isfinite(values[i])) {
                throw std::runtime_error(name + " contains a non-finite value");
            }
            output[i] = static_cast<float>(values[i]);
        }
    }

    void loadParameters() {
        pnh_.param<std::string>("topics/imu", imu_topic_, "/mavros/imu/data_raw");
        pnh_.param<std::string>("topics/gnss_position", gnss_position_topic_,
                                "/mavros/global_position/raw/fix");
        pnh_.param<std::string>("topics/gnss_velocity", gnss_velocity_topic_,
                                "/mavros/global_position/raw/gps_vel");
        pnh_.param<std::string>("topics/magnetometer", magnetometer_topic_,
                                "/mavros/imu/mag");
        pnh_.param<std::string>("topics/barometer", barometer_topic_,
                                "/mavros/imu/static_pressure");

        pnh_.param<std::string>("outputs/imu", output_imu_topic_,
                                "/inslib/imu/data");
        pnh_.param<std::string>("outputs/pose", output_pose_topic_,
                                "/inslib/local_position/pose");
        pnh_.param<std::string>("outputs/velocity", output_velocity_topic_,
                                "/inslib/local_position/velocity_local");
        pnh_.param<std::string>("outputs/odometry", output_odometry_topic_,
                                "/inslib/odometry");
        pnh_.param<std::string>("outputs/ready", output_ready_topic_,
                                "/inslib/state_valid");
        pnh_.param<std::string>("outputs/mode", output_mode_topic_,
                                "/inslib/mode");
        pnh_.param<std::string>("outputs/attitude_stddev", output_attitude_stddev_topic_,
                                "/inslib/attitude_stddev_deg");
        pnh_.param<std::string>("outputs/world_frame", world_frame_id_, "map");
        pnh_.param<std::string>("outputs/body_frame", body_frame_id_, "base_link");
        pnh_.param("outputs/world_yaw_alignment_deg", world_yaw_alignment_deg_, 0.0);
        pnh_.param("outputs/max_attitude_stddev_deg", max_attitude_stddev_deg_, 5.0);
        pnh_.param("outputs/status_rate_hz", status_rate_hz_, 5.0);

        pnh_.param("sensors/use_gnss_position", use_gnss_position_, true);
        pnh_.param("sensors/use_gnss_velocity", use_gnss_velocity_, true);
        pnh_.param("sensors/use_magnetometer", use_magnetometer_, false);
        pnh_.param("sensors/use_barometer", use_barometer_, true);

        pnh_.param<std::string>("frames/imu_body", imu_body_frame_, "flu");
        pnh_.param<std::string>("frames/magnetometer_body", magnetometer_body_frame_, "flu");
        pnh_.param<std::string>("frames/gnss_velocity_world", gnss_velocity_world_frame_,
                                "enu");
        if (imu_body_frame_ != "flu" && imu_body_frame_ != "frd") {
            throw std::runtime_error("frames/imu_body must be 'flu' or 'frd'");
        }
        if (magnetometer_body_frame_ != "flu" && magnetometer_body_frame_ != "frd") {
            throw std::runtime_error("frames/magnetometer_body must be 'flu' or 'frd'");
        }
        if (gnss_velocity_world_frame_ != "enu" && gnss_velocity_world_frame_ != "ned") {
            throw std::runtime_error("frames/gnss_velocity_world must be 'enu' or 'ned'");
        }

        pnh_.param("imu/nominal_rate_hz", nominal_imu_rate_hz_, 100.0);
        pnh_.param("imu/minimum_rate_hz", minimum_imu_rate_hz_, 80.0);
        pnh_.param("imu/accelerometer_noise_density_mps2_sqrthz",
                   accelerometer_noise_density_, 0.0);
        pnh_.param("imu/gyroscope_noise_density_rps_sqrthz",
                   gyroscope_noise_density_, 0.0);

        pnh_.param("gnss/additional_delay_ms", gnss_additional_delay_ms_, 0);
        pnh_.param("gnss/max_pair_time_difference_ms", gnss_pair_tolerance_ms_, 30);
        pnh_.param("gnss/fallback_position_stddev_horizontal_m",
                   fallback_gnss_position_stddev_h_, 1.5);
        pnh_.param("gnss/fallback_position_stddev_vertical_m",
                   fallback_gnss_position_stddev_v_, 3.0);
        pnh_.param("gnss/velocity_stddev_horizontal_mps",
                   gnss_velocity_stddev_h_, 0.20);
        pnh_.param("gnss/velocity_stddev_vertical_mps",
                   gnss_velocity_stddev_v_, 0.30);

        pnh_.param("magnetometer/wmm_year", wmm_year_, 2026.75);
        pnh_.param("barometer/altitude_stddev_m", barometer_altitude_stddev_m_, 2.0);
        pnh_.param("auxiliary/max_sample_age_sec", max_auxiliary_age_sec_, 0.25);

        pnh_.param("filter/auto_init", auto_init_, true);
        pnh_.param("filter/kalman_update_period_sec", kalman_update_period_sec_, 0.02);
        pnh_.param("filter/max_prediction_time_sec", max_prediction_time_sec_, 0.10);
        pnh_.param("filter/max_deadreckoning_sec", max_deadreckoning_sec_, 10.0);
        pnh_.param("filter/allow_unlimited_deadreckoning",
                   allow_unlimited_deadreckoning_, false);
        pnh_.param("filter/disable_outlier_rejection", disable_outlier_rejection_, false);
        pnh_.param("filter/disable_auto_zupt", disable_auto_zupt_, false);
        pnh_.param("filter/disable_baro_height", disable_baro_height_, false);

        pnh_.param("initial/position_stddev_m", initial_position_stddev_m_, 5.0);
        pnh_.param("initial/velocity_stddev_mps", initial_velocity_stddev_mps_, 1.0);
        pnh_.param("initial/roll_pitch_stddev_deg", initial_roll_pitch_stddev_deg_, 5.0);
        pnh_.param("initial/yaw_stddev_deg", initial_yaw_stddev_deg_, 180.0);
        pnh_.param("initial/accelerometer_bias_stddev_mps2",
                   initial_accelerometer_bias_stddev_, 0.20);
        pnh_.param("initial/gyroscope_bias_stddev_rps",
                   initial_gyroscope_bias_stddev_, 0.02);
        pnh_.param("process/accelerometer_bias_random_walk_mps2_sqrthz",
                   accelerometer_bias_random_walk_, 0.0);
        pnh_.param("process/gyroscope_bias_random_walk_rps_sqrthz",
                   gyroscope_bias_random_walk_, 0.0);

        pnh_.param("gnss/position_covariance_scale", gnss_position_covariance_scale_, 1.0);
        pnh_.param("gnss/velocity_covariance_scale", gnss_velocity_covariance_scale_, 1.0);
        pnh_.param("gnss/position_stddev_floor_horizontal_m",
                   gnss_position_floor_h_, 0.5);
        pnh_.param("gnss/position_stddev_floor_vertical_m",
                   gnss_position_floor_v_, 1.0);
        pnh_.param("gnss/velocity_stddev_floor_horizontal_mps",
                   gnss_velocity_floor_h_, 0.10);
        pnh_.param("gnss/velocity_stddev_floor_vertical_mps",
                   gnss_velocity_floor_v_, 0.15);
        pnh_.param("gnss/position_decimation", gnss_position_decimation_, 5);
        pnh_.param("gnss/init_dwell_sec", gnss_init_dwell_sec_, 2.0);
        pnh_.param("gnss/start_max_position_stddev_horizontal_m",
                   gnss_start_position_h_, 2.0);
        pnh_.param("gnss/start_max_position_stddev_vertical_m",
                   gnss_start_position_v_, 3.0);
        pnh_.param("gnss/start_max_velocity_stddev_horizontal_mps",
                   gnss_start_velocity_h_, 0.30);
        pnh_.param("gnss/start_max_velocity_stddev_vertical_mps",
                   gnss_start_velocity_v_, 0.50);
        pnh_.param("gnss/stop_max_position_stddev_horizontal_m",
                   gnss_stop_position_h_, 5.0);
        pnh_.param("gnss/stop_max_position_stddev_vertical_m",
                   gnss_stop_position_v_, 7.0);
        pnh_.param("gnss/stop_max_velocity_stddev_horizontal_mps",
                   gnss_stop_velocity_h_, 0.50);
        pnh_.param("gnss/stop_max_velocity_stddev_vertical_mps",
                   gnss_stop_velocity_v_, 0.70);
    }

    void initializeFilter() {
        ins_init_t initial{};
        initial.pos_init_stddev_m = static_cast<float>(initial_position_stddev_m_);
        initial.vel_init_stddev_mps = static_cast<float>(initial_velocity_stddev_mps_);
        initial.rpy_init_stddev_rad[0] =
            static_cast<float>(degToRad(initial_roll_pitch_stddev_deg_));
        initial.rpy_init_stddev_rad[1] =
            static_cast<float>(degToRad(initial_roll_pitch_stddev_deg_));
        initial.rpy_init_stddev_rad[2] =
            static_cast<float>(degToRad(initial_yaw_stddev_deg_));
        initial.acc_bias_init_stddev_mps2 =
            static_cast<float>(initial_accelerometer_bias_stddev_);
        initial.gyr_bias_init_stddev_rps =
            static_cast<float>(initial_gyroscope_bias_stddev_);
        initial.acc_bias_pred_stddev_mps2_sqrts =
            static_cast<float>(accelerometer_bias_random_walk_);
        initial.gyr_bias_pred_stddev_rps_sqrts =
            static_cast<float>(gyroscope_bias_random_walk_);

        ins_options_t options{};
        options.auto_init = auto_init_;
        options.kalman_update_dt_sec = static_cast<float>(kalman_update_period_sec_);
        options.max_prediction_time_sec = static_cast<float>(max_prediction_time_sec_);
        options.max_deadreckoning_sec = static_cast<float>(max_deadreckoning_sec_);
        options.allow_unlimited_deadreckoning = allow_unlimited_deadreckoning_;
        options.chi2_disable = disable_outlier_rejection_;
        options.auto_zupt_disable = disable_auto_zupt_;
        options.baro_height_disable = disable_baro_height_;
        options.gnss_pos_cov_scale = static_cast<float>(gnss_position_covariance_scale_);
        options.gnss_vel_cov_scale = static_cast<float>(gnss_velocity_covariance_scale_);
        options.gnss_pos_stddev_floor_hor_m = static_cast<float>(gnss_position_floor_h_);
        options.gnss_pos_stddev_floor_ver_m = static_cast<float>(gnss_position_floor_v_);
        options.gnss_vel_stddev_floor_hor_mps = static_cast<float>(gnss_velocity_floor_h_);
        options.gnss_vel_stddev_floor_ver_mps = static_cast<float>(gnss_velocity_floor_v_);
        options.gnss_pos_decimation = gnss_position_decimation_;
        options.gnss_init_dwell_sec = static_cast<float>(gnss_init_dwell_sec_);
        options.gnss_start_max_horizontal_pos_stddev_m =
            static_cast<float>(gnss_start_position_h_);
        options.gnss_start_max_vertical_pos_stddev_m =
            static_cast<float>(gnss_start_position_v_);
        options.gnss_start_max_horizontal_vel_stddev_mps =
            static_cast<float>(gnss_start_velocity_h_);
        options.gnss_start_max_vertical_vel_stddev_mps =
            static_cast<float>(gnss_start_velocity_v_);
        options.gnss_stop_max_horizontal_pos_stddev_m =
            static_cast<float>(gnss_stop_position_h_);
        options.gnss_stop_max_vertical_pos_stddev_m =
            static_cast<float>(gnss_stop_position_v_);
        options.gnss_stop_max_horizontal_vel_stddev_mps =
            static_cast<float>(gnss_stop_velocity_h_);
        options.gnss_stop_max_vertical_vel_stddev_mps =
            static_cast<float>(gnss_stop_velocity_v_);

        loadFloatArray("calibration/accelerometer_fixed_bias_mps2",
                       options.imu_acc_fixed_bias);
        loadFloatArray("calibration/gyroscope_fixed_bias_rps",
                       options.imu_gyr_fixed_bias);
        loadFloatArray("calibration/accelerometer_matrix_column_major",
                       options.imu_acc_misalignment);
        loadFloatArray("calibration/gyroscope_matrix_column_major",
                       options.imu_gyr_misalignment);
        loadFloatArray("calibration/magnetometer_fixed_bias_ut",
                       options.mag_fixed_bias);
        loadFloatArray("calibration/magnetometer_matrix_column_major",
                       options.mag_misalignment);

        std::vector<double> lever_arm;
        if (pnh_.getParam("gnss/antenna_lever_arm_frd_m", lever_arm)) {
            if (lever_arm.size() != 3) {
                throw std::runtime_error(
                    "gnss/antenna_lever_arm_frd_m must contain exactly 3 values");
            }
            for (std::size_t i = 0; i < 3; ++i) {
                gnss_lever_arm_frd_[i] = static_cast<float>(lever_arm[i]);
            }
        }

        if (nav_suite_init(suite_.get(), &initial, &options) != 0) {
            throw std::runtime_error("nav_suite_init failed");
        }
    }

    void configureRos() {
        const ros::TransportHints hints = ros::TransportHints().tcpNoDelay();
        imu_sub_ = nh_.subscribe(imu_topic_, 200,
                                 &InslibStateEstimator::imuCallback, this, hints);
        if (use_gnss_position_) {
            gnss_position_sub_ = nh_.subscribe(
                gnss_position_topic_, 10,
                &InslibStateEstimator::gnssPositionCallback, this, hints);
        }
        if (use_gnss_velocity_) {
            gnss_velocity_sub_ = nh_.subscribe(
                gnss_velocity_topic_, 10,
                &InslibStateEstimator::gnssVelocityCallback, this, hints);
        }
        if (use_magnetometer_) {
            magnetometer_sub_ = nh_.subscribe(
                magnetometer_topic_, 30,
                &InslibStateEstimator::magnetometerCallback, this, hints);
        }
        if (use_barometer_) {
            barometer_sub_ = nh_.subscribe(
                barometer_topic_, 30,
                &InslibStateEstimator::barometerCallback, this, hints);
        }

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_imu_topic_, 20);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_pose_topic_, 10);
        velocity_pub_ =
            nh_.advertise<geometry_msgs::TwistStamped>(output_velocity_topic_, 10);
        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odometry_topic_, 10);
        ready_pub_ = nh_.advertise<std_msgs::Bool>(output_ready_topic_, 10);
        mode_pub_ = nh_.advertise<std_msgs::String>(output_mode_topic_, 2, true);
        attitude_stddev_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
            output_attitude_stddev_topic_, 5);
    }

    void gnssPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& message) {
        if (message->status.status < sensor_msgs::NavSatStatus::STATUS_FIX ||
            !finite3(message->latitude, message->longitude, message->altitude)) {
            return;
        }
        pending_gnss_position_ = *message;
        have_pending_gnss_position_ = true;
    }

    void gnssVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& message) {
        const geometry_msgs::Vector3& velocity = message->twist.linear;
        if (!finite3(velocity.x, velocity.y, velocity.z)) return;
        pending_gnss_velocity_ = *message;
        have_pending_gnss_velocity_ = true;
    }

    void magnetometerCallback(const sensor_msgs::MagneticField::ConstPtr& message) {
        const geometry_msgs::Vector3& field = message->magnetic_field;
        if (!finite3(field.x, field.y, field.z)) return;
        pending_magnetometer_ = *message;
        have_pending_magnetometer_ = true;
    }

    void barometerCallback(const sensor_msgs::FluidPressure::ConstPtr& message) {
        if (!std::isfinite(message->fluid_pressure) || message->fluid_pressure <= 0.0) {
            return;
        }
        pending_barometer_ = *message;
        have_pending_barometer_ = true;
    }

    static ros::Time effectiveStamp(const ros::Time& stamp) {
        return stamp.isZero() ? ros::Time::now() : stamp;
    }

    bool measurementDelayMs(const ros::Time& sample_stamp,
                            const ros::Time& imu_stamp,
                            int additional_delay_ms,
                            int& delay_ms) const {
        const ros::Time stamp = sample_stamp.isZero() ? imu_stamp : sample_stamp;
        double age_sec = (imu_stamp - stamp).toSec();
        if (age_sec < -0.02) {
            ROS_WARN_THROTTLE(2.0,
                              "[INSLIB] Sensor timestamp is ahead of the IMU clock by %.1f ms.",
                              -1000.0 * age_sec);
            age_sec = 0.0;
        }
        delay_ms = additional_delay_ms +
                   static_cast<int>(std::lround(1000.0 * std::max(0.0, age_sec)));
        if (delay_ms > INS_MAX_DELAY_MS) {
            ROS_WARN_THROTTLE(2.0,
                              "[INSLIB] Dropping a %d ms old aiding sample; INSLIB limit is %d ms.",
                              delay_ms, INS_MAX_DELAY_MS);
            return false;
        }
        delay_ms = std::max(0, delay_ms);
        return true;
    }

    tf::Vector3 bodyInputToFrd(const geometry_msgs::Vector3& value,
                               const std::string& input_frame) const {
        if (input_frame == "frd") return tf::Vector3(value.x, value.y, value.z);
        return tf::Vector3(value.x, -value.y, -value.z);
    }

    tf::Vector3 velocityInputToNed(const geometry_msgs::Vector3& value) const {
        if (gnss_velocity_world_frame_ == "ned") {
            return tf::Vector3(value.x, value.y, value.z);
        }
        return tf::Vector3(value.y, value.x, -value.z);
    }

    geometry_msgs::Vector3 bodyInputToOutputFlu(
        const geometry_msgs::Vector3& value) const {
        geometry_msgs::Vector3 output = value;
        if (imu_body_frame_ == "frd") {
            output.y = -value.y;
            output.z = -value.z;
        }
        return output;
    }

    void fillGnssPosition(ins_measurements_t& measurement,
                          const sensor_msgs::NavSatFix& fix) const {
        measurement.gnss_pos.is_valid = true;
        measurement.gnss_pos.llh[0] = degToRad(fix.latitude);
        measurement.gnss_pos.llh[1] = degToRad(fix.longitude);
        measurement.gnss_pos.llh[2] = fix.altitude;

        const bool covariance_known =
            fix.position_covariance_type !=
                sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
            fix.position_covariance[0] > 0.0 &&
            fix.position_covariance[4] > 0.0 &&
            fix.position_covariance[8] > 0.0;
        if (!covariance_known) {
            const float horizontal_variance = static_cast<float>(
                fallback_gnss_position_stddev_h_ *
                fallback_gnss_position_stddev_h_);
            const float vertical_variance = static_cast<float>(
                fallback_gnss_position_stddev_v_ *
                fallback_gnss_position_stddev_v_);
            measurement.gnss_pos.Qll_ned[0] = horizontal_variance;
            measurement.gnss_pos.Qll_ned[4] = horizontal_variance;
            measurement.gnss_pos.Qll_ned[8] = vertical_variance;
            return;
        }

        const double c[3][3] = {
            {0.0, 1.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.0, -1.0},
        };
        double ned_covariance[3][3]{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        ned_covariance[row][col] +=
                            c[row][i] * fix.position_covariance[3 * i + j] *
                            c[col][j];
                    }
                }
                measurement.gnss_pos.Qll_ned[col * 3 + row] =
                    static_cast<float>(ned_covariance[row][col]);
            }
        }
    }

    void fillGnssVelocity(ins_measurements_t& measurement,
                          const geometry_msgs::TwistStamped& velocity) const {
        const tf::Vector3 ned = velocityInputToNed(velocity.twist.linear);
        measurement.gnss_vel.is_valid = true;
        measurement.gnss_vel.vel_ned[0] = static_cast<float>(ned.x());
        measurement.gnss_vel.vel_ned[1] = static_cast<float>(ned.y());
        measurement.gnss_vel.vel_ned[2] = static_cast<float>(ned.z());
        const float horizontal_variance = static_cast<float>(
            gnss_velocity_stddev_h_ * gnss_velocity_stddev_h_);
        const float vertical_variance = static_cast<float>(
            gnss_velocity_stddev_v_ * gnss_velocity_stddev_v_);
        measurement.gnss_vel.Qll_ned[0] = horizontal_variance;
        measurement.gnss_vel.Qll_ned[4] = horizontal_variance;
        measurement.gnss_vel.Qll_ned[8] = vertical_variance;
    }

    void addPendingAiding(ins_measurements_t& measurement,
                          const ros::Time& imu_stamp) {
        int position_delay_ms = 0;
        int velocity_delay_ms = 0;
        const bool use_position =
            have_pending_gnss_position_ &&
            measurementDelayMs(pending_gnss_position_.header.stamp, imu_stamp,
                               gnss_additional_delay_ms_, position_delay_ms);
        const bool use_velocity =
            have_pending_gnss_velocity_ &&
            measurementDelayMs(pending_gnss_velocity_.header.stamp, imu_stamp,
                               gnss_additional_delay_ms_, velocity_delay_ms);

        bool consume_position = use_position;
        bool consume_velocity = use_velocity;
        if (use_position && use_velocity &&
            std::abs(position_delay_ms - velocity_delay_ms) >
                gnss_pair_tolerance_ms_) {
            // INSLIB has one GNSS delay for the position/velocity bundle. Fuse
            // the position now and leave velocity for the next IMU epoch rather
            // than assigning one measurement the wrong time of validity.
            consume_velocity = false;
        }

        if (consume_position) {
            fillGnssPosition(measurement, pending_gnss_position_);
            measurement.gnss_delay_ms = position_delay_ms;
            have_pending_gnss_position_ = false;
            if (!wmm_initialized_) {
                const double lat_rad = degToRad(pending_gnss_position_.latitude);
                const double lon_rad = degToRad(pending_gnss_position_.longitude);
                ins_set_magnetic_model_from_position(
                    &suite_->ins, lat_rad, lon_rad, static_cast<float>(wmm_year_));
                ahrs_set_position(&suite_->ahrs, static_cast<float>(lat_rad),
                                  static_cast<float>(lon_rad),
                                  static_cast<float>(wmm_year_));
                wmm_initialized_ = true;
            }
        } else if (have_pending_gnss_position_ && !use_position) {
            have_pending_gnss_position_ = false;
        }

        if (consume_velocity) {
            fillGnssVelocity(measurement, pending_gnss_velocity_);
            measurement.gnss_delay_ms =
                std::max(measurement.gnss_delay_ms, velocity_delay_ms);
            have_pending_gnss_velocity_ = false;
        } else if (have_pending_gnss_velocity_ && !use_velocity) {
            have_pending_gnss_velocity_ = false;
        }

        if (measurement.gnss_pos.is_valid || measurement.gnss_vel.is_valid) {
            for (int i = 0; i < 3; ++i) {
                measurement.gnss_leverarm_b[i] = gnss_lever_arm_frd_[i];
            }
        }

        if (have_pending_magnetometer_) {
            const ros::Time stamp = effectiveStamp(pending_magnetometer_.header.stamp);
            if (std::abs((imu_stamp - stamp).toSec()) <= max_auxiliary_age_sec_) {
                const tf::Vector3 frd = bodyInputToFrd(
                    pending_magnetometer_.magnetic_field,
                    magnetometer_body_frame_);
                measurement.mag.is_valid = true;
                measurement.mag.data[0] = static_cast<float>(1e6 * frd.x());
                measurement.mag.data[1] = static_cast<float>(1e6 * frd.y());
                measurement.mag.data[2] = static_cast<float>(1e6 * frd.z());
            }
            have_pending_magnetometer_ = false;
        }

        if (have_pending_barometer_) {
            const ros::Time stamp = effectiveStamp(pending_barometer_.header.stamp);
            if (std::abs((imu_stamp - stamp).toSec()) <= max_auxiliary_age_sec_) {
                measurement.baro.is_valid = true;
                measurement.baro.pressure_pa =
                    static_cast<float>(pending_barometer_.fluid_pressure);
                measurement.baro.stddev_m =
                    static_cast<float>(barometer_altitude_stddev_m_);
            }
            have_pending_barometer_ = false;
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
        if (!finite3(message->angular_velocity.x, message->angular_velocity.y,
                     message->angular_velocity.z) ||
            !finite3(message->linear_acceleration.x, message->linear_acceleration.y,
                     message->linear_acceleration.z)) {
            ROS_WARN_THROTTLE(1.0, "[INSLIB] Dropping non-finite IMU sample.");
            return;
        }

        const ros::Time stamp = effectiveStamp(message->header.stamp);
        double dt = 1.0 / std::max(1.0, nominal_imu_rate_hz_);
        if (!last_imu_stamp_.isZero()) {
            dt = (stamp - last_imu_stamp_).toSec();
            if (dt <= 0.0) {
                ROS_WARN_THROTTLE(1.0, "[INSLIB] Non-monotonic IMU timestamp; sample dropped.");
                return;
            }
        }
        last_imu_stamp_ = stamp;
        last_raw_imu_ = *message;

        const double observed_rate = 1.0 / std::max(dt, 1e-6);
        if (observed_rate < minimum_imu_rate_hz_) {
            ROS_WARN_THROTTLE(2.0,
                              "[INSLIB] IMU rate %.1f Hz is below configured minimum %.1f Hz.",
                              observed_rate, minimum_imu_rate_hz_);
        }

        ins_measurements_t measurement{};
        measurement.timestamp = static_cast<ins_time_us_t>(stamp.toNSec() / 1000ULL);
        measurement.strapdown_dt_sec = static_cast<float>(dt);

        const tf::Vector3 acceleration = bodyInputToFrd(
            message->linear_acceleration, imu_body_frame_);
        const tf::Vector3 angular_velocity = bodyInputToFrd(
            message->angular_velocity, imu_body_frame_);
        measurement.acc.is_valid = true;
        measurement.gyr.is_valid = true;
        for (int i = 0; i < 3; ++i) {
            measurement.acc.data[i] = static_cast<float>(acceleration[i]);
            measurement.gyr.data[i] = static_cast<float>(angular_velocity[i]);
            if (accelerometer_noise_density_ > 0.0) {
                measurement.acc.Qll_diag[i] = static_cast<float>(
                    accelerometer_noise_density_ * accelerometer_noise_density_);
            }
            if (gyroscope_noise_density_ > 0.0) {
                measurement.gyr.Qll_diag[i] = static_cast<float>(
                    gyroscope_noise_density_ * gyroscope_noise_density_);
            }
        }

        addPendingAiding(measurement, stamp);
        nav_suite_update(suite_.get(), &measurement);
        publishState(stamp);
    }

    tf::Vector3 rotateOutputWorld(const tf::Vector3& enu) const {
        const double yaw = degToRad(world_yaw_alignment_deg_);
        return tf::Vector3(std::cos(yaw) * enu.x() - std::sin(yaw) * enu.y(),
                           std::sin(yaw) * enu.x() + std::cos(yaw) * enu.y(),
                           enu.z());
    }

    tf::Vector3 nedToOutputEnu(const float ned[3]) const {
        return rotateOutputWorld(tf::Vector3(ned[1], ned[0], -ned[2]));
    }

    tf::Quaternion bodyToOutputQuaternion(const tf::Quaternion& q_ned_frd) const {
        const tf::Matrix3x3 ned_from_frd(q_ned_frd);
        const tf::Matrix3x3 enu_from_ned(
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0);
        const tf::Matrix3x3 frd_from_flu(
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0);
        const double yaw = degToRad(world_yaw_alignment_deg_);
        const tf::Matrix3x3 aligned_enu_from_enu(
            std::cos(yaw), -std::sin(yaw), 0.0,
            std::sin(yaw), std::cos(yaw), 0.0,
            0.0, 0.0, 1.0);

        const tf::Matrix3x3 output_from_flu =
            aligned_enu_from_enu * enu_from_ned * ned_from_frd * frd_from_flu;
        tf::Quaternion quaternion;
        output_from_flu.getRotation(quaternion);
        quaternion.normalize();
        return quaternion;
    }

    bool getBestAttitude(tf::Quaternion& output) const {
        float quaternion[4]{};
        if (ins_get_quaternion(&suite_->ins, quaternion)) {
            const tf::Quaternion q_ned_frd(
                quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
            output = bodyToOutputQuaternion(q_ned_frd);
            return true;
        }

        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        if (!nav_suite_get_rpy(suite_.get(), &roll, &pitch, &yaw)) return false;
        tf::Quaternion q_ned_frd;
        q_ned_frd.setRPY(roll, pitch, yaw);
        output = bodyToOutputQuaternion(q_ned_frd);
        return true;
    }

    void publishState(const ros::Time& stamp) {
        const nav_suite_mode_t mode = nav_suite_get_mode(suite_.get());
        tf::Quaternion attitude;
        const bool attitude_valid = getBestAttitude(attitude);

        float position_ned[3]{};
        float velocity_ned[3]{};
        const bool position_valid =
            ins_get_position_local(&suite_->ins, position_ned);
        const bool velocity_valid =
            ins_get_velocity_ned(&suite_->ins, velocity_ned);

        float roll_stddev = 0.0f;
        float pitch_stddev = 0.0f;
        float yaw_stddev = 0.0f;
        const bool stddev_valid = ins_get_rpy_stddev(
            &suite_->ins, &roll_stddev, &pitch_stddev, &yaw_stddev);
        const double maximum_attitude_stddev_deg = stddev_valid
            ? std::max({roll_stddev, pitch_stddev, yaw_stddev}) * 180.0 / kPi
            : std::numeric_limits<double>::infinity();
        const bool attitude_quality_ok =
            max_attitude_stddev_deg_ <= 0.0 ||
            maximum_attitude_stddev_deg <= max_attitude_stddev_deg_;
        const bool navigation_mode_ok =
            mode == NAV_SUITE_MODE_FULL || mode == NAV_SUITE_MODE_COASTING;
        const bool state_valid = navigation_mode_ok && attitude_valid &&
                                 position_valid && velocity_valid &&
                                 attitude_quality_ok;

        std_msgs::Bool ready_message;
        ready_message.data = state_valid;
        ready_pub_.publish(ready_message);

        if (attitude_valid) {
            sensor_msgs::Imu imu_message = last_raw_imu_;
            imu_message.header.stamp = stamp;
            imu_message.header.frame_id = body_frame_id_;
            imu_message.angular_velocity =
                bodyInputToOutputFlu(last_raw_imu_.angular_velocity);
            imu_message.linear_acceleration =
                bodyInputToOutputFlu(last_raw_imu_.linear_acceleration);
            tf::quaternionTFToMsg(attitude, imu_message.orientation);
            std::fill(imu_message.orientation_covariance.begin(),
                      imu_message.orientation_covariance.end(), 0.0);
            if (stddev_valid) {
                imu_message.orientation_covariance[0] = roll_stddev * roll_stddev;
                imu_message.orientation_covariance[4] = pitch_stddev * pitch_stddev;
                imu_message.orientation_covariance[8] = yaw_stddev * yaw_stddev;
            } else {
                imu_message.orientation_covariance[0] = -1.0;
            }
            imu_pub_.publish(imu_message);
        }

        if (attitude_valid && position_valid && velocity_valid) {
            const tf::Vector3 position_enu = nedToOutputEnu(position_ned);
            const tf::Vector3 velocity_enu = nedToOutputEnu(velocity_ned);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = world_frame_id_;
            pose.pose.position.x = position_enu.x();
            pose.pose.position.y = position_enu.y();
            pose.pose.position.z = position_enu.z();
            tf::quaternionTFToMsg(attitude, pose.pose.orientation);
            pose_pub_.publish(pose);

            geometry_msgs::TwistStamped velocity;
            velocity.header = pose.header;
            velocity.twist.linear.x = velocity_enu.x();
            velocity.twist.linear.y = velocity_enu.y();
            velocity.twist.linear.z = velocity_enu.z();
            velocity.twist.angular =
                bodyInputToOutputFlu(last_raw_imu_.angular_velocity);
            velocity_pub_.publish(velocity);

            nav_msgs::Odometry odometry;
            odometry.header = pose.header;
            odometry.child_frame_id = body_frame_id_;
            odometry.pose.pose = pose.pose;
            odometry.twist.twist = velocity.twist;
            odometry_pub_.publish(odometry);
        }

        if (stddev_valid) {
            geometry_msgs::Vector3Stamped stddev;
            stddev.header.stamp = stamp;
            stddev.header.frame_id = body_frame_id_;
            stddev.vector.x = roll_stddev * 180.0 / kPi;
            stddev.vector.y = pitch_stddev * 180.0 / kPi;
            stddev.vector.z = yaw_stddev * 180.0 / kPi;
            attitude_stddev_pub_.publish(stddev);
        }

        const double status_period = 1.0 / std::max(0.1, status_rate_hz_);
        if (last_status_publish_.isZero() ||
            (stamp - last_status_publish_).toSec() >= status_period) {
            std_msgs::String mode_message;
            mode_message.data = modeName(mode);
            mode_pub_.publish(mode_message);
            last_status_publish_ = stamp;
            ROS_INFO_THROTTLE(5.0,
                              "[INSLIB] mode=%s valid=%s attitude_sigma_max=%.2f deg",
                              modeName(mode), state_valid ? "true" : "false",
                              maximum_attitude_stddev_deg);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<nav_suite_t> suite_;

    std::string imu_topic_;
    std::string gnss_position_topic_;
    std::string gnss_velocity_topic_;
    std::string magnetometer_topic_;
    std::string barometer_topic_;
    std::string output_imu_topic_;
    std::string output_pose_topic_;
    std::string output_velocity_topic_;
    std::string output_odometry_topic_;
    std::string output_ready_topic_;
    std::string output_mode_topic_;
    std::string output_attitude_stddev_topic_;
    std::string world_frame_id_;
    std::string body_frame_id_;
    std::string imu_body_frame_;
    std::string magnetometer_body_frame_;
    std::string gnss_velocity_world_frame_;

    bool use_gnss_position_ = true;
    bool use_gnss_velocity_ = true;
    bool use_magnetometer_ = false;
    bool use_barometer_ = true;
    bool auto_init_ = true;
    bool allow_unlimited_deadreckoning_ = false;
    bool disable_outlier_rejection_ = false;
    bool disable_auto_zupt_ = false;
    bool disable_baro_height_ = false;

    double world_yaw_alignment_deg_ = 0.0;
    double max_attitude_stddev_deg_ = 5.0;
    double status_rate_hz_ = 5.0;
    double nominal_imu_rate_hz_ = 100.0;
    double minimum_imu_rate_hz_ = 80.0;
    double accelerometer_noise_density_ = 0.0;
    double gyroscope_noise_density_ = 0.0;
    int gnss_additional_delay_ms_ = 0;
    int gnss_pair_tolerance_ms_ = 30;
    double fallback_gnss_position_stddev_h_ = 1.5;
    double fallback_gnss_position_stddev_v_ = 3.0;
    double gnss_velocity_stddev_h_ = 0.20;
    double gnss_velocity_stddev_v_ = 0.30;
    double wmm_year_ = 2026.75;
    double barometer_altitude_stddev_m_ = 2.0;
    double max_auxiliary_age_sec_ = 0.25;
    double kalman_update_period_sec_ = 0.02;
    double max_prediction_time_sec_ = 0.10;
    double max_deadreckoning_sec_ = 10.0;
    double initial_position_stddev_m_ = 5.0;
    double initial_velocity_stddev_mps_ = 1.0;
    double initial_roll_pitch_stddev_deg_ = 5.0;
    double initial_yaw_stddev_deg_ = 180.0;
    double initial_accelerometer_bias_stddev_ = 0.20;
    double initial_gyroscope_bias_stddev_ = 0.02;
    double accelerometer_bias_random_walk_ = 0.0;
    double gyroscope_bias_random_walk_ = 0.0;
    double gnss_position_covariance_scale_ = 1.0;
    double gnss_velocity_covariance_scale_ = 1.0;
    double gnss_position_floor_h_ = 0.5;
    double gnss_position_floor_v_ = 1.0;
    double gnss_velocity_floor_h_ = 0.10;
    double gnss_velocity_floor_v_ = 0.15;
    int gnss_position_decimation_ = 5;
    double gnss_init_dwell_sec_ = 2.0;
    double gnss_start_position_h_ = 2.0;
    double gnss_start_position_v_ = 3.0;
    double gnss_start_velocity_h_ = 0.30;
    double gnss_start_velocity_v_ = 0.50;
    double gnss_stop_position_h_ = 5.0;
    double gnss_stop_position_v_ = 7.0;
    double gnss_stop_velocity_h_ = 0.50;
    double gnss_stop_velocity_v_ = 0.70;
    float gnss_lever_arm_frd_[3]{0.0f, 0.0f, 0.0f};

    bool have_pending_gnss_position_ = false;
    bool have_pending_gnss_velocity_ = false;
    bool have_pending_magnetometer_ = false;
    bool have_pending_barometer_ = false;
    bool wmm_initialized_ = false;
    sensor_msgs::NavSatFix pending_gnss_position_;
    geometry_msgs::TwistStamped pending_gnss_velocity_;
    sensor_msgs::MagneticField pending_magnetometer_;
    sensor_msgs::FluidPressure pending_barometer_;
    sensor_msgs::Imu last_raw_imu_;
    ros::Time last_imu_stamp_;
    ros::Time last_status_publish_;

    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_position_sub_;
    ros::Subscriber gnss_velocity_sub_;
    ros::Subscriber magnetometer_sub_;
    ros::Subscriber barometer_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher odometry_pub_;
    ros::Publisher ready_pub_;
    ros::Publisher mode_pub_;
    ros::Publisher attitude_stddev_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "inslib_state_estimator");
    try {
        InslibStateEstimator estimator;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[INSLIB] Initialization failed: %s", error.what());
        return 1;
    }
    return 0;
}
