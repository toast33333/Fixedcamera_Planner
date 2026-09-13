#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <kcf_msgs/Bbox.h>
#include <detection_msgs/Detection.h>
#include <tf/tf.h>

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

class PNGInterceptor {
public:
    explicit PNGInterceptor(ros::NodeHandle& nh) {
        nh.param("fx", fx_, 640.0);
        nh.param("fy", fy_, 640.0);
        nh.param("cx", cx_, 480.0);
        nh.param("cy", cy_, 270.0);

        nh.param("bbox_center_is_offset", bbox_center_is_offset_, true);
        nh.param("y_up_positive", y_up_positive_, true);

        nh.param("V_default", V_default_, 5.0);
        nh.param("N_nav", N_nav_, 2.0);
        nh.param("N_nav_z", N_nav_z_, 2.0);

        nh.param("max_yaw_rate", max_yaw_rate_, 1.0);
        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("pixel_thr", pixel_thr_, 5.0);
        nh.param("pixel_thr_y", pixel_thr_y_, 5.0);

        nh.param("k_yaw", k_yaw_, 0.8);
        nh.param("k_yaw_d", k_yaw_d_, 0.3);

        double lambda_alpha_legacy = 0.2;
        int lambda_window_size_legacy = 3;
        nh.param("lambda_filt_alpha", lambda_alpha_legacy, 0.2);
        nh.param("lambda_window_size", lambda_window_size_legacy, 3);
        const LegacyLosKalmanTuning lambda_tuning =
            mapLegacyLosChainToKalman(lambda_alpha_legacy, lambda_window_size_legacy);
        nh.param("lambda_kalman_q_position", lambda_kalman_q_position_, lambda_tuning.q_position);
        nh.param("lambda_kalman_q_rate", lambda_kalman_q_rate_, lambda_tuning.q_rate);
        nh.param("lambda_kalman_r", lambda_kalman_r_, lambda_tuning.r_measurement);

        double gamma_alpha_legacy = 0.2;
        int gamma_window_size_legacy = 3;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, 0.2);
        nh.param("gamma_window_size", gamma_window_size_legacy, 3);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_kalman_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_kalman_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_kalman_r_, gamma_tuning.r_measurement);

        nh.param("v_center_alpha", v_center_alpha_, 0.2);
        nh.param("k_z_ff", k_z_ff_, 1.0);
        nh.param("max_vz", max_vz_, 1.0);
        nh.param("enable_height_control", enable_height_control_, true);

        nh.param("yaw_enable_speed_ratio", yaw_enable_speed_ratio_, 0.8);
        nh.param<std::string>(
            "velocity_topic", velocity_topic_, "/mavros/local_position/velocity_local");

        int msg_mode = 0;
        std::string topic_name;
        nh.param("detection_mode", msg_mode, 0);
        nh.param<std::string>("detection_topic", topic_name, "/object_kcf");
        if (msg_mode == 1 && topic_name == "/object_kcf") {
            topic_name = "/object_detections";
        }

        if (msg_mode == 1) {
            sub_det_detection_ = nh.subscribe<detection_msgs::Detection>(
                topic_name, 1, &PNGInterceptor::detectionCbDetection, this);
            ROS_INFO("[PNGBboxVz] Subscribed to %s (detection_msgs/Detection)", topic_name.c_str());
        } else {
            sub_det_kcf_ = nh.subscribe<kcf_msgs::Bbox>(
                topic_name, 1, &PNGInterceptor::detectionCbKcf, this);
            ROS_INFO("[PNGBboxVz] Subscribed to %s (kcf_msgs/Bbox)", topic_name.c_str());
        }

        sub_pose_ = nh.subscribe("/mavros/local_position/pose", 1,
                                 &PNGInterceptor::poseCb, this);
        sub_vel_ = nh.subscribe(
            velocity_topic_, 1, &PNGInterceptor::velocityCb, this);

        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        last_time_ = ros::Time::now();
        current_speed_ = 0.0;
        yaw_control_enabled_ = false;
        have_det_ = false;
        have_pose_ = false;

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

        const double yaw_enable_speed = V_default_ * yaw_enable_speed_ratio_;
        ROS_INFO("[PNGBboxVz] Started (planar PNG + yaw PD + z from bbox y).");
        ROS_INFO("[PNGBboxVz] Yaw is enabled after horizontal speed reaches %.3f m/s.",
                 yaw_enable_speed);
    }

    void spin() {
        ros::Rate rate(50.0);

        while (ros::ok()) {
            ros::spinOnce();

            if (!have_pose_) {
                rate.sleep();
                continue;
            }

            const ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_time_).toSec() : step_dt_;
            if (dt <= 0.0) {
                dt = step_dt_;
            }
            last_time_ = now;

            double vx_body = V_default_;
            double vy_body = 0.0;
            double yaw_rate_cmd = 0.0;
            double vz_cmd = 0.0;

            updateYawEnableState();

            if (have_det_) {
                const double raw_u = 0.5 * (last_det_.x1 + last_det_.x2);
                const double raw_v = 0.5 * (last_det_.y1 + last_det_.y2);

                const double centered_u = bbox_center_is_offset_ ? raw_u : (raw_u - cx_);
                const double centered_v = bbox_center_is_offset_ ? raw_v : (raw_v - cy_);

                if (!v_center_inited_) {
                    v_center_filt_ = centered_v;
                    v_center_inited_ = true;
                } else {
                    v_center_filt_ =
                        v_center_alpha_ * centered_v +
                        (1.0 - v_center_alpha_) * v_center_filt_;
                }

                const double lambda_meas = std::atan2(-centered_u, fx_);
                lambda_filter_.update(lambda_meas, dt);

                const double lambda = lambda_filter_.value();
                const double lambda_dot = lambda_filter_.rate();
                const double ex = fx_ * std::tan(lambda);

                const double a_n = N_nav_ * V_default_ * lambda_dot;
                const double yaw_des = a_n / V_default_ * dt;
                vx_body = V_default_ * std::cos(yaw_des);
                vy_body = V_default_ * std::sin(yaw_des);

                if (yaw_control_enabled_ && std::fabs(ex) > pixel_thr_) {
                    yaw_rate_cmd = k_yaw_ * lambda + k_yaw_d_ * lambda_dot;
                    yaw_rate_cmd = clampValue(
                        yaw_rate_cmd,
                        -max_yaw_rate_,
                        max_yaw_rate_);
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

            tf::Quaternion q(
                curr_pose_.pose.orientation.x,
                curr_pose_.pose.orientation.y,
                curr_pose_.pose.orientation.z,
                curr_pose_.pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            m.getRPY(roll, pitch, yaw);

            const double cmd_vx =
                vx_body * std::cos(yaw) - vy_body * std::sin(yaw);
            const double cmd_vy =
                vx_body * std::sin(yaw) + vy_body * std::cos(yaw);

            geometry_msgs::TwistStamped cmd;
            cmd.header.stamp = now;
            cmd.twist.linear.x = cmd_vx;
            cmd.twist.linear.y = cmd_vy;
            cmd.twist.linear.z = vz_cmd;
            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = yaw_rate_cmd;

            pub_cmd_.publish(cmd);
            rate.sleep();
        }
    }

private:
    void updateYawEnableState() {
        if (yaw_control_enabled_ || !have_vel_) {
            if (!have_vel_) {
                ROS_WARN_THROTTLE(1.0,
                                  "[PNGBboxVz] Waiting for velocity on %s, yaw control remains disabled.",
                                  velocity_topic_.c_str());
            }
            return;
        }

        const double yaw_enable_speed = V_default_ * yaw_enable_speed_ratio_;
        if (current_speed_ >= yaw_enable_speed) {
            yaw_control_enabled_ = true;
            ROS_INFO("[PNGBboxVz] Horizontal speed %.3f m/s reached threshold %.3f m/s, yaw control enabled.",
                     current_speed_, yaw_enable_speed);
        } else {
            ROS_INFO_THROTTLE(1.0,
                              "[PNGBboxVz] Accelerating: horizontal speed %.3f m/s, yaw enable threshold %.3f m/s.",
                              current_speed_, yaw_enable_speed);
        }
    }

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

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_pose_ = *msg;
        have_pose_ = true;
    }

    void velocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        const double vx = msg->twist.linear.x;
        const double vy = msg->twist.linear.y;
        current_speed_ = std::sqrt(vx * vx + vy * vy);
        have_vel_ = true;
    }

    double fx_ = 640.0;
    double fy_ = 640.0;
    double cx_ = 480.0;
    double cy_ = 270.0;

    bool bbox_center_is_offset_ = true;
    bool y_up_positive_ = true;

    double V_default_ = 5.0;
    double N_nav_ = 2.0;
    double N_nav_z_ = 2.0;
    double max_yaw_rate_ = 1.0;
    bool use_real_dt_ = true;
    double step_dt_ = 0.02;
    double pixel_thr_ = 5.0;
    double pixel_thr_y_ = 5.0;

    double k_yaw_ = 0.8;
    double k_yaw_d_ = 0.3;
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;
    double gamma_kalman_q_position_ = 1e-4;
    double gamma_kalman_q_rate_ = 5e-1;
    double gamma_kalman_r_ = 2e-4;

    double v_center_alpha_ = 0.2;
    double k_z_ff_ = 1.0;
    double max_vz_ = 1.0;
    bool enable_height_control_ = true;

    double yaw_enable_speed_ratio_ = 0.8;
    std::string velocity_topic_ = "/mavros/local_position/velocity_local";

    bool have_det_ = false;
    bool have_pose_ = false;
    bool have_vel_ = false;
    bool yaw_control_enabled_ = false;
    bool v_center_inited_ = false;

    ros::Time last_time_;

    double current_speed_ = 0.0;
    double v_center_filt_ = 0.0;
    double vz_png_int_ = 0.0;

    ScalarRateKalmanFilter lambda_filter_;
    ScalarRateKalmanFilter gamma_filter_;

    DetectionData last_det_{};
    geometry_msgs::PoseStamped curr_pose_;

    ros::Subscriber sub_det_kcf_;
    ros::Subscriber sub_det_detection_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_vel_;
    ros::Publisher pub_cmd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rp_png_bbox_vz_node");
    ros::NodeHandle nh("~");

    PNGInterceptor node(nh);
    node.spin();
    return 0;
}
