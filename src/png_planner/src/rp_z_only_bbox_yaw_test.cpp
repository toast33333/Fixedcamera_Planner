#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <kcf_msgs/Bbox.h>
#include <detection_msgs/Detection.h>

#include <cmath>
#include <string>

#include "los_rate_kalman.h"

template<typename T>
T clampValue(T v, T min_val, T max_val) {
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

struct DetectionData {
    float x1, y1, x2, y2;
    float score;
    std::string class_name;
    int track_id;
    bool is_tracking;
};

class ZOnlyBboxYawInterceptor {
public:
    explicit ZOnlyBboxYawInterceptor(ros::NodeHandle& nh) {
        nh.param("fx", fx_, 640.0);
        nh.param("fy", fy_, 640.0);
        nh.param("cx", cx_, 480.0);
        nh.param("cy", cy_, 270.0);

        nh.param("bbox_center_is_offset", bbox_center_is_offset_, true);
        nh.param("y_up_positive", y_up_positive_, true);

        nh.param("V_default", V_default_, 5.0);
        nh.param("N_nav_z", N_nav_z_, 2.0);

        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("pixel_thr", pixel_thr_, 5.0);
        nh.param("pixel_thr_y", pixel_thr_y_, 5.0);

        nh.param("k_yaw", k_yaw_, 0.8);
        nh.param("k_yaw_d", k_yaw_d_, 0.3);
        nh.param("max_yaw_rate", max_yaw_rate_, 1.0);

        double lambda_alpha_legacy = 0.2;
        int lambda_window_size_legacy = 3;
        nh.param("lambda_filt_alpha", lambda_alpha_legacy, 0.2);
        nh.param("lambda_window_size", lambda_window_size_legacy, 3);
        const LegacyLosKalmanTuning lambda_tuning =
            mapLegacyLosChainToKalman(lambda_alpha_legacy, lambda_window_size_legacy);
        nh.param("lambda_kalman_q_position", lambda_kalman_q_position_, lambda_tuning.q_position);
        nh.param("lambda_kalman_q_rate", lambda_kalman_q_rate_, lambda_tuning.q_rate);
        nh.param("lambda_kalman_r", lambda_kalman_r_, lambda_tuning.r_measurement);

        nh.param("v_center_alpha", v_center_alpha_, 0.2);

        double gamma_alpha_legacy = 0.2;
        int gamma_window_size_legacy = 3;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, 0.2);
        nh.param("gamma_window_size", gamma_window_size_legacy, 3);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_kalman_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_kalman_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_kalman_r_, gamma_tuning.r_measurement);

        nh.param("k_z_ff", k_z_ff_, 1.0);
        nh.param("max_vz", max_vz_, 1.0);
        nh.param("enable_height_control", enable_height_control_, true);

        int msg_mode = 0;
        std::string topic_name;
        nh.param("detection_mode", msg_mode, 0);
        nh.param<std::string>("detection_topic", topic_name, "/object_kcf");
        if (msg_mode == 1 && topic_name == "/object_kcf") {
            topic_name = "/object_detections";
        }

        if (msg_mode == 1) {
            sub_det_detection_ = nh.subscribe<detection_msgs::Detection>(
                topic_name, 1, &ZOnlyBboxYawInterceptor::detectionCbDetection, this);
            ROS_INFO("[ZOnlyBboxYawTest] Subscribed to %s (detection_msgs/Detection)",
                     topic_name.c_str());
        } else {
            sub_det_kcf_ = nh.subscribe<kcf_msgs::Bbox>(
                topic_name, 1, &ZOnlyBboxYawInterceptor::detectionCbKcf, this);
            ROS_INFO("[ZOnlyBboxYawTest] Subscribed to %s (kcf_msgs/Bbox)",
                     topic_name.c_str());
        }

        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        last_time_ = ros::Time::now();
        have_det_ = false;

        lambda_filter_.setNoise(
            lambda_kalman_q_position_,
            lambda_kalman_q_rate_,
            lambda_kalman_r_);
        lambda_filter_.reset();

        gamma_filter_.setNoise(
            gamma_kalman_q_position_,
            gamma_kalman_q_rate_,
            gamma_kalman_r_);
        gamma_filter_.reset();
        vz_png_int_ = 0.0;

        ROS_INFO("[ZOnlyBboxYawTest] Started: yaw control + z control from bbox y, no x/y velocity.");
    }

    void spin() {
        ros::Rate rate(50.0);

        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_time_).toSec() : step_dt_;
            if (dt <= 0.0) {
                dt = step_dt_;
            }
            last_time_ = now;

            double yaw_rate_cmd = 0.0;
            double vz_cmd = 0.0;

            if (have_det_) {
                const double raw_u = 0.5 * (last_det_.x1 + last_det_.x2);
                const double raw_v = 0.5 * (last_det_.y1 + last_det_.y2);

                const double centered_u = bbox_center_is_offset_ ? raw_u : (raw_u - cx_);
                const double centered_v = bbox_center_is_offset_ ? raw_v : (raw_v - cy_);

                const double lambda_meas = std::atan2(-centered_u, fx_);
                lambda_filter_.update(lambda_meas, dt);

                const double lambda = lambda_filter_.value();
                const double lambda_dot = lambda_filter_.rate();
                const double ex = fx_ * std::tan(lambda);

                if (std::fabs(ex) > pixel_thr_) {
                    yaw_rate_cmd = k_yaw_ * lambda + k_yaw_d_ * lambda_dot;
                    yaw_rate_cmd = clampValue(
                        yaw_rate_cmd,
                        -max_yaw_rate_,
                        max_yaw_rate_);
                }

                if (!v_center_inited_) {
                    v_center_filt_ = centered_v;
                    v_center_inited_ = true;
                } else {
                    v_center_filt_ =
                        v_center_alpha_ * centered_v +
                        (1.0 - v_center_alpha_) * v_center_filt_;
                }

                if (enable_height_control_) {
                    const double ey = y_up_positive_ ? v_center_filt_ : -v_center_filt_;
                    const double gamma_meas = std::atan2(ey, fy_);
                    gamma_filter_.update(gamma_meas, dt);

                    const double gamma = gamma_filter_.value();
                    const double gamma_dot = gamma_filter_.rate();

                    if (std::fabs(ey) > pixel_thr_y_) {
                        const double a_z = N_nav_z_ * V_default_ * gamma_dot;
                        vz_png_int_ += a_z * dt;
                    } else {
                        vz_png_int_ *= 0.98;
                    }

                    const double vz_ff = k_z_ff_ * gamma;
                    vz_cmd = clampValue(vz_ff + vz_png_int_, -max_vz_, max_vz_);
                } else {
                    gamma_filter_.reset();
                    vz_png_int_ = 0.0;
                }
            } else {
                lambda_filter_.reset();
                gamma_filter_.reset();
                v_center_inited_ = false;
                vz_png_int_ = 0.0;
            }

            geometry_msgs::TwistStamped cmd;
            cmd.header.stamp = now;
            cmd.twist.linear.x = 0.0;
            cmd.twist.linear.y = 0.0;
            cmd.twist.linear.z = vz_cmd;
            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = yaw_rate_cmd;

            pub_cmd_.publish(cmd);
            rate.sleep();
        }
    }

private:
    void detectionCbKcf(const kcf_msgs::Bbox::ConstPtr& msg) {
        if (!msg->is_tracking) {
            have_det_ = false;
            lambda_filter_.reset();
            gamma_filter_.reset();
            v_center_inited_ = false;
            return;
        }

        last_det_.x1 = msg->x1;
        last_det_.y1 = msg->y1;
        last_det_.x2 = msg->x2;
        last_det_.y2 = msg->y2;
        last_det_.score = msg->score;
        last_det_.class_name = msg->class_name;
        last_det_.track_id = msg->track_id;
        last_det_.is_tracking = msg->is_tracking;
        have_det_ = true;
    }

    void detectionCbDetection(const detection_msgs::Detection::ConstPtr& msg) {
        if (!msg->is_tracking) {
            have_det_ = false;
            lambda_filter_.reset();
            gamma_filter_.reset();
            v_center_inited_ = false;
            return;
        }

        last_det_.x1 = msg->x1;
        last_det_.y1 = msg->y1;
        last_det_.x2 = msg->x2;
        last_det_.y2 = msg->y2;
        last_det_.score = msg->score;
        last_det_.class_name = msg->class_name;
        last_det_.track_id = msg->track_id;
        last_det_.is_tracking = msg->is_tracking;
        have_det_ = true;
    }

    double fx_ = 640.0;
    double fy_ = 640.0;
    double cx_ = 480.0;
    double cy_ = 270.0;

    bool bbox_center_is_offset_ = true;
    bool y_up_positive_ = true;

    double V_default_ = 5.0;
    double N_nav_z_ = 2.0;

    bool use_real_dt_ = true;
    double step_dt_ = 0.02;
    double pixel_thr_ = 5.0;
    double pixel_thr_y_ = 5.0;

    double k_yaw_ = 0.8;
    double k_yaw_d_ = 0.3;
    double max_yaw_rate_ = 1.0;
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;

    double v_center_alpha_ = 0.2;
    double gamma_kalman_q_position_ = 1e-4;
    double gamma_kalman_q_rate_ = 5e-1;
    double gamma_kalman_r_ = 2e-4;
    double k_z_ff_ = 1.0;
    double max_vz_ = 1.0;
    bool enable_height_control_ = true;

    bool have_det_ = false;
    bool v_center_inited_ = false;
    ros::Time last_time_;

    double v_center_filt_ = 0.0;
    double vz_png_int_ = 0.0;

    ScalarRateKalmanFilter lambda_filter_;
    ScalarRateKalmanFilter gamma_filter_;

    DetectionData last_det_{};

    ros::Subscriber sub_det_kcf_;
    ros::Subscriber sub_det_detection_;
    ros::Publisher pub_cmd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rp_z_only_bbox_yaw_test_node");
    ros::NodeHandle nh("~");

    ZOnlyBboxYawInterceptor node(nh);
    node.spin();
    return 0;
}
