#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <deque>
#include <string>
#include <tf/tf.h>

#include "los_rate_kalman.h"

inline double clamp(double v, double min_val, double max_val) {
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

// 仅进行航向控制与前向速度，不进行高度（z 轴）控制
class PNGNoAlt {
public:
    explicit PNGNoAlt(ros::NodeHandle& nh) {
        // 参数
        nh.param("V_default",     V_default_,     5.0);   // 前向速度（恒定）
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

        nh.param("k_yaw",   k_yaw_,   0.08);   // yaw 比例
        nh.param("k_yaw_d", k_yaw_d_, 0.008);  // yaw 微分

        nh.param("lost_timeout", lost_timeout_, 0.2); // 秒，云台角度超时判定
        nh.param<std::string>("gimbal_angle_topic", gimbal_angle_topic_,
                              std::string("/gimbal_angles"));

        // 订阅：姿态、云台角度
        sub_pose_ = nh.subscribe("/mavros/local_position/pose", 1,
                                 &PNGNoAlt::poseCb, this);
        sub_gimbal_angles_ = nh.subscribe(gimbal_angle_topic_, 1,
                                          &PNGNoAlt::gimbalAnglesCb, this);

        // 发布：速度控制
        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        have_pose_ = false;
        have_gimbal_angles_ = false;
        target_lost_ = true;

        last_time_ = ros::Time::now();
        last_gimbal_time_ = ros::Time(0);

        lambda_filter_.setNoise(
            lambda_kalman_q_position_,
            lambda_kalman_q_rate_,
            lambda_kalman_r_);
        lambda_filter_.reset();

        ROS_INFO("[PNGNoAlt] Started (forward-only, yaw PD, no z control). gimbal_angle_topic=%s",
                 gimbal_angle_topic_.c_str());
    }

    void spin() {
        ros::Rate rate(50.0);
        while (ros::ok()) {
            ros::spinOnce();

            if (!have_pose_) { rate.sleep(); continue; }

            ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_time_).toSec() : step_dt_;
            if (dt <= 0.0) dt = step_dt_;
            last_time_ = now;

            // 机体航向（yaw_b）
            tf::Quaternion q(
                curr_pose_.pose.orientation.x,
                curr_pose_.pose.orientation.y,
                curr_pose_.pose.orientation.z,
                curr_pose_.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll_b, pitch_b, yaw_b;
            m.getRPY(roll_b, pitch_b, yaw_b);

            // 目标丢失判定：云台角度超时
            bool lost_now = true;
            if (have_gimbal_angles_) {
                double age = (now - last_gimbal_time_).toSec();
                lost_now = (age > lost_timeout_);
            }

            if (lost_now) {
                if (!target_lost_) {
                    ROS_WARN("[PNGNoAlt] Target LOST -> hold forward, freeze yaw.");
                }
                target_lost_ = true;

                publishHoldCommand(now, yaw_b);
                rate.sleep();
                continue;
            }

            if (target_lost_) {
                ROS_INFO("[PNGNoAlt] Target RECOVERED -> resume yaw guidance.");
                lambda_filter_.reset();
                target_lost_ = false;
            }

            // LOS yaw（不控制高度，忽略 pitch）
            const double lambda_meas = -gimbal_yaw_;
            lambda_filter_.update(lambda_meas, dt);
            const double lambda = lambda_filter_.value();
            const double lambda_dot = lambda_filter_.rate();

            // yaw PD
            double yaw_rate_cmd = k_yaw_ * lambda + k_yaw_d_ * lambda_dot;
            yaw_rate_cmd = clamp(yaw_rate_cmd, -max_yaw_rate_, max_yaw_rate_);

            // 前向飞行（沿机头方向），不控制高度（z=0）
            double V_rel = V_default_;
            double vx_world = V_rel * std::cos(yaw_b);
            double vy_world = V_rel * std::sin(yaw_b);

            geometry_msgs::TwistStamped cmd;
            cmd.header.stamp = now;
            cmd.twist.linear.x = vx_world;
            cmd.twist.linear.y = vy_world;
            cmd.twist.linear.z = 0.0; // 不做高度控制

            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = yaw_rate_cmd;
            pub_cmd_.publish(cmd);

            rate.sleep();
        }
    }

private:
    // 参数
    double V_default_;
    double max_yaw_rate_;
    bool   use_real_dt_;
    double step_dt_;
    double k_yaw_, k_yaw_d_;
    double lost_timeout_;
    std::string gimbal_angle_topic_;

    // 状态
    bool have_pose_, have_gimbal_angles_;
    bool target_lost_;
    ros::Time last_time_;
    ros::Time last_gimbal_time_;

    // 云台角度（仅使用 yaw）
    double gimbal_yaw_ = 0.0;

    geometry_msgs::PoseStamped curr_pose_;

    // ROS
    ros::Subscriber sub_pose_, sub_gimbal_angles_;
    ros::Publisher  pub_cmd_;

    // 历史窗口
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;
    ScalarRateKalmanFilter lambda_filter_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        curr_pose_ = *msg;
        have_pose_ = true;
    }

    void gimbalAnglesCb(const geometry_msgs::Twist::ConstPtr& msg) {
        gimbal_yaw_ = msg->angular.z;
        have_gimbal_angles_ = true;
        last_gimbal_time_ = ros::Time::now();
    }

    // 丢失目标：保持前进、航向冻结、z=0
    void publishHoldCommand(const ros::Time& now, double yaw_b) {
        double V_rel = V_default_;
        double vx_world = V_rel * std::cos(yaw_b);
        double vy_world = V_rel * std::sin(yaw_b);

        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = now;

        cmd.twist.linear.x = vx_world;
        cmd.twist.linear.y = vy_world;
        cmd.twist.linear.z = 0.0; // 不做高度控制

        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0; // 冻结航向

        pub_cmd_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_los_no_alt");
    ros::NodeHandle nh("~");
    PNGNoAlt node(nh);
    node.spin();
    return 0;
}
