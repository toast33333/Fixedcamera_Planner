#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <deque>
#include <cmath>
#include <string>

#include "los_rate_kalman.h"

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// PNG 横向控制（不做高度控制，z=0）
class PNGLateralNoAlt {
public:
    explicit PNGLateralNoAlt(ros::NodeHandle& nh) {
        // 参数（参考 gimbal_png.cpp，但去掉垂向相关）
        nh.param("V_forward", V_forward_, 5.0);        // 前向速度 (m/s)
        nh.param("N_nav", N_nav_, 3.0);                // 导引比例 N (yaw)
        nh.param("v_lat_max", v_lat_max_, 3.0);        // 横向速度限幅 (m/s)
        nh.param("tau_lat", tau_lat_, 0.5);            // 横向速度时间常数 (s)
        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("lost_timeout", lost_timeout_, 0.2);
        int legacy_window_size = 5;
        nh.param("window_size", legacy_window_size, 5);
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
        nh.param<std::string>("gimbal_angle_topic", gimbal_angle_topic_,
                              std::string("/gimbal_angles"));

        sub_pose_ = nh.subscribe("/mavros/local_position/pose", 1,
                                 &PNGLateralNoAlt::poseCb, this);
        sub_gimbal_angles_ = nh.subscribe(gimbal_angle_topic_, 1,
                                          &PNGLateralNoAlt::gimbalAnglesCb, this);

        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        have_pose_ = false;
        have_gimbal_angles_ = false;
        last_time_ = ros::Time::now();
        last_gimbal_time_ = ros::Time(0);
        v_lat_ = 0.0;
        lambda_filter_.setNoise(
            lambda_kalman_q_position_,
            lambda_kalman_q_rate_,
            lambda_kalman_r_);
        lambda_filter_.reset();

        ROS_INFO("[PNGLateralNoAlt] Started: forward + lateral PNG, z=0. gimbal_angle_topic=%s",
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

            bool lost = true;
            if (have_gimbal_angles_) {
                double age = (now - last_gimbal_time_).toSec();
                lost = age > lost_timeout_;
            }

            // 若丢失目标：保持前飞，侧向速度归零，z=0
            if (lost) {
                v_lat_ = 0.0;
                lambda_filter_.reset();
                publishCmd(now, V_forward_, v_lat_, 0.0);
                rate.sleep();
                continue;
            }

            const double lambda_meas = -gimbal_yaw_;
            lambda_filter_.update(lambda_meas, dt);
            const double lambda = lambda_filter_.value();
            const double lambda_dot = lambda_filter_.rate();

            // PNG 横向加速度 a = N * V * lambda_dot
            double a_lat = N_nav_ * V_forward_ * lambda_dot;

            // 一阶惯性: v_lat += a_lat * dt，再滤波
            double v_lat_target = v_lat_ + a_lat * dt;
            v_lat_target = clamp(v_lat_target, -v_lat_max_, v_lat_max_);
            double alpha = dt / std::max(tau_lat_, dt);
            v_lat_ = (1.0 - alpha) * v_lat_ + alpha * v_lat_target;

            // 不做高度控制：v_z=0
            publishCmd(now, V_forward_, v_lat_, 0.0);
            rate.sleep();
        }
    }

private:
    // 参数
    double V_forward_, N_nav_, v_lat_max_, tau_lat_;
    bool   use_real_dt_;
    double step_dt_;
    double lost_timeout_;
    std::string gimbal_angle_topic_;

    // 状态
    bool have_pose_, have_gimbal_angles_;
    ros::Time last_time_, last_gimbal_time_;
    double gimbal_yaw_ = 0.0;
    double v_lat_ = 0.0;
    geometry_msgs::PoseStamped pose_;
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;
    ScalarRateKalmanFilter lambda_filter_;

    // ROS
    ros::Subscriber sub_pose_, sub_gimbal_angles_;
    ros::Publisher  pub_cmd_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pose_ = *msg;
        have_pose_ = true;
    }

    void gimbalAnglesCb(const geometry_msgs::Twist::ConstPtr& msg) {
        gimbal_yaw_ = msg->angular.z;
        have_gimbal_angles_ = true;
        last_gimbal_time_ = ros::Time::now();
    }

    void publishCmd(const ros::Time& stamp, double V_fwd, double v_lat,
                    double v_z) {
        // 机体航向
        tf::Quaternion q(
            pose_.pose.orientation.x,
            pose_.pose.orientation.y,
            pose_.pose.orientation.z,
            pose_.pose.orientation.w);
        double roll_b, pitch_b, yaw_b;
        tf::Matrix3x3(q).getRPY(roll_b, pitch_b, yaw_b);

        double vx_world = V_fwd * std::cos(yaw_b) - v_lat * std::sin(yaw_b);
        double vy_world = V_fwd * std::sin(yaw_b) + v_lat * std::cos(yaw_b);

        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = stamp;
        cmd.twist.linear.x = vx_world;
        cmd.twist.linear.y = vy_world;
        cmd.twist.linear.z = 0.0; // 不做高度控制
        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0;
        pub_cmd_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_png_no_alt");
    ros::NodeHandle nh("~");
    PNGLateralNoAlt node(nh);
    node.spin();
    return 0;
}
