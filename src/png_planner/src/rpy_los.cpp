#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <deque>
#include <string>
#include <tf/tf.h>

#include "los_rate_kalman.h"

// 简单 clamp
inline double clamp(double v, double min_val, double max_val) {
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

class PNGInterceptor {
public:
    PNGInterceptor(ros::NodeHandle& nh) {
        // ================= 参数 =================
        nh.param("V_default",     V_default_,     5.0);   // 前向速度
        nh.param("N_nav",         N_nav_,         2.0);   // (保留) 你可不用
        nh.param("max_yaw_rate",  max_yaw_rate_,  1.0);
        nh.param("use_real_dt",   use_real_dt_,   true);
        nh.param("step_dt",       step_dt_,       0.02);
        int legacy_window_size = 10;
        nh.param("window_size", legacy_window_size, 10);
        double legacy_alpha = 0.2;
        nh.param("filt_alpha", legacy_alpha, 0.2);
        double lambda_alpha_legacy = legacy_alpha;
        int lambda_window_size_legacy = legacy_window_size;
        nh.param("lambda_filt_alpha", lambda_alpha_legacy, legacy_alpha);
        nh.param("lambda_window_size", lambda_window_size_legacy, legacy_window_size);
        const LegacyLosKalmanTuning lambda_tuning =
            mapLegacyLosChainToKalman(lambda_alpha_legacy, lambda_window_size_legacy);
        nh.param("lambda_kalman_q_position", lambda_kalman_q_position_, lambda_tuning.q_position);
        nh.param("lambda_kalman_q_rate", lambda_kalman_q_rate_, lambda_tuning.q_rate);
        nh.param("lambda_kalman_r", lambda_kalman_r_, lambda_tuning.r_measurement);
        double gamma_alpha_legacy = legacy_alpha;
        int gamma_window_size_legacy = legacy_window_size;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, legacy_alpha);
        nh.param("gamma_window_size", gamma_window_size_legacy, legacy_window_size);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_kalman_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_kalman_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_kalman_r_, gamma_tuning.r_measurement);

        // yaw PD（你现有结构）
        nh.param("k_yaw",   k_yaw_,   0.08);
        nh.param("k_yaw_d", k_yaw_d_, 0.008);

        // 垂直控制（LOS pitch 相对水平面）
        nh.param("k_z_ff",  k_z_ff_,  1.0);
        nh.param("N_nav_z", N_nav_z_, 2.0);

        // 目标丢失判定（串口云台角度反馈超时）
        nh.param("lost_timeout", lost_timeout_, 0.2); // 秒

        // 丢失时垂直速度策略：默认 0（更安全）
        nh.param("lost_vz_hold", lost_vz_hold_, 0.0);
        nh.param<std::string>("attack_enable_topic", attack_enable_topic_,
                              std::string("/png_start"));
        nh.param<std::string>("gimbal_angle_topic", gimbal_angle_topic_,
                              std::string("/gimbal_angles"));
        nh.param("start_enabled", attack_enabled_, false);

        // ================= 订阅 =================
        sub_pose_ = nh.subscribe("/mavros/local_position/pose", 1,
                                 &PNGInterceptor::poseCb, this);
        sub_gimbal_angles_ = nh.subscribe(gimbal_angle_topic_, 1,
                                          &PNGInterceptor::gimbalAnglesCb, this);
        sub_enable_ = nh.subscribe(attack_enable_topic_, 1,
                                   &PNGInterceptor::enableCb, this);

        // ================= 发布 =================
        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        // ================= 初始化 =================
        have_pose_ = false;
        have_gimbal_angles_ = false;

        last_time_ = ros::Time::now();
        last_gimbal_time_ = ros::Time(0);

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

        target_lost_ = true; // 没收到云台角度前视为丢失

        ROS_INFO("[PNGInterceptor] Started. attack_enabled=%s gimbal_angle_topic=%s",
                 attack_enabled_ ? "true" : "false", gimbal_angle_topic_.c_str());
    }

    void spin() {
        ros::Rate rate(50.0);

        while (ros::ok()) {
            ros::spinOnce();

            if (!have_pose_) {
                rate.sleep();
                continue;
            }

            if (!attack_enabled_) {
                rate.sleep();
                continue;
            }

            ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_time_).toSec() : step_dt_;
            if (dt <= 0.0) dt = step_dt_;
            last_time_ = now;

            // 机体姿态
            tf::Quaternion q(
                curr_pose_.pose.orientation.x,
                curr_pose_.pose.orientation.y,
                curr_pose_.pose.orientation.z,
                curr_pose_.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll_b, pitch_b, yaw_b;
            m.getRPY(roll_b, pitch_b, yaw_b);

            // ============================
            // 目标丢失判定（云台角度超时）
            // ============================
            bool lost_now = true;
            if (have_gimbal_angles_) {
                double age = (now - last_gimbal_time_).toSec();
                lost_now = (age > lost_timeout_);
            }

            // 丢失 -> 前飞保持 + 航向冻结
            if (lost_now) {
                if (!target_lost_) {
                    ROS_WARN("[PNGInterceptor] Target LOST -> hold forward & freeze yaw.");
                }
                target_lost_ = true;

                publishHoldCommand(now, yaw_b);
                rate.sleep();
                continue;
            }

            // 重新获得目标：清空历史，避免窗口差分“跳变”
            if (target_lost_) {
                ROS_INFO("[PNGInterceptor] Target RECOVERED -> resume guidance.");
                lambda_filter_.reset();
                gamma_filter_.reset();
                target_lost_ = false;
            }

            // ============================
            // 1) LOS 角定义
            // ============================
            const double lambda_meas = -gimbal_yaw_;
            const double gamma_meas = gimbal_pitch_;
            lambda_filter_.update(lambda_meas, dt);
            gamma_filter_.update(gamma_meas, dt);
            const double lambda = lambda_filter_.value();
            const double lambda_dot = lambda_filter_.rate();
            const double gamma = gamma_filter_.value();
            const double gamma_dot = gamma_filter_.rate();

            // ============================
            // 3) yaw 控制（PD）
            // ============================
            double yaw_rate_cmd = k_yaw_ * lambda + k_yaw_d_ * lambda_dot;
            yaw_rate_cmd = clamp(yaw_rate_cmd, -max_yaw_rate_, max_yaw_rate_);

            // ============================
            // 4) 垂直速度（LOS pitch）
            // ============================
            double vz_cmd = k_z_ff_ * gamma + N_nav_z_ * gamma_dot;

            // ============================
            // 5) 前向飞行（无机体侧向速度）
            // ============================
            double V_rel = V_default_;

            // 速度始终沿“当前机头方向”投影到世界系
            double vx_world = V_rel * std::cos(yaw_b);
            double vy_world = V_rel * std::sin(yaw_b);

            // ============================
            // 6) 发布命令
            // ============================
            geometry_msgs::TwistStamped cmd;
            cmd.header.stamp = now;

            cmd.twist.linear.x  = vx_world;
            cmd.twist.linear.y  = vy_world;
            cmd.twist.linear.z  = vz_cmd;

            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = yaw_rate_cmd;

            pub_cmd_.publish(cmd);

            rate.sleep();
        }
    }

private:
    // ================= 参数 =================
    double V_default_, N_nav_;
    double max_yaw_rate_;
    bool   use_real_dt_;
    double step_dt_;

    double k_yaw_, k_yaw_d_;
    double k_z_ff_, N_nav_z_;

    double lost_timeout_;
    double lost_vz_hold_;
    std::string attack_enable_topic_;
    std::string gimbal_angle_topic_;

    // ================= 状态 =================
    bool have_pose_, have_gimbal_angles_;
    bool target_lost_;
    bool attack_enabled_;

    ros::Time last_time_;
    ros::Time last_gimbal_time_;

    // 云台角度（相对机体）
    double gimbal_yaw_   = 0.0;
    double gimbal_pitch_ = 0.0;

    geometry_msgs::PoseStamped curr_pose_;

    // ================= ROS =================
    ros::Subscriber sub_pose_, sub_gimbal_angles_, sub_enable_;
    ros::Publisher  pub_cmd_;

    // ================= 历史数据 =================
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;
    double gamma_kalman_q_position_ = 1e-4;
    double gamma_kalman_q_rate_ = 5e-1;
    double gamma_kalman_r_ = 2e-4;
    ScalarRateKalmanFilter lambda_filter_;
    ScalarRateKalmanFilter gamma_filter_;

    // ================= 回调 =================
    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_pose_ = *msg;
        have_pose_ = true;
    }

    void enableCb(const std_msgs::Bool::ConstPtr& msg) {
        attack_enabled_ = msg->data;
        last_time_ = ros::Time::now();
        last_gimbal_time_ = ros::Time(0);
        have_gimbal_angles_ = false;
        target_lost_ = true;
        lambda_filter_.reset();
        gamma_filter_.reset();
        ROS_INFO("[PNGInterceptor] attack_enabled=%s", attack_enabled_ ? "true" : "false");
    }

    void gimbalAnglesCb(const geometry_msgs::Twist::ConstPtr& msg) {
        if (!attack_enabled_) {
            return;
        }
        gimbal_yaw_   = msg->angular.z;
        gimbal_pitch_ = msg->angular.y;
        have_gimbal_angles_ = true;
        last_gimbal_time_ = ros::Time::now();
    }

    // ================= 丢失时：前飞保持 + 航向冻结 =================
    void publishHoldCommand(const ros::Time& now, double yaw_b) {
        double V_rel = V_default_;

        // 前飞保持：沿当前航向继续前进
        double vx_world = V_rel * std::cos(yaw_b);
        double vy_world = V_rel * std::sin(yaw_b);

        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = now;

        cmd.twist.linear.x  = vx_world;
        cmd.twist.linear.y  = vy_world;
        cmd.twist.linear.z  = lost_vz_hold_; // 默认 0

        // 航向冻结：yaw_rate = 0
        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0;

        pub_cmd_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_los");
    ros::NodeHandle nh("~");

    PNGInterceptor node(nh);
    node.spin();
    return 0;
}
