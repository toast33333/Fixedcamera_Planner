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

class PNGLateral {
public:
    explicit PNGLateral(ros::NodeHandle& nh) {
        nh.param("V_forward", V_forward_, 5.0);        // 预设前向速度 (m/s)
        nh.param("N_nav", N_nav_, 3.0);                // 导引比例 N (yaw)
        nh.param("v_lat_max", v_lat_max_, 3.0);        // 横向速度限幅 (m/s)
        nh.param("tau_lat", tau_lat_, 0.5);            // 横向速度时间常数 (s) 用于 a->v
        nh.param("k_z_ff", k_z_ff_, 1.0);              // pitch 前馈
        nh.param("N_nav_z", N_nav_z_, 2.0);            // pitch 导引比例
        nh.param("v_z_max", v_z_max_, 2.0);            // 垂向速度限幅 (m/s)
        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("lost_timeout", lost_timeout_, 0.2);
        nh.param("lost_vz_hold", lost_vz_hold_, 0.0);
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
        double gamma_alpha_legacy = legacy_alpha;
        int gamma_window_size_legacy = legacy_window_size;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, legacy_alpha);
        nh.param("gamma_window_size", gamma_window_size_legacy, legacy_window_size);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_kalman_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_kalman_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_kalman_r_, gamma_tuning.r_measurement);
        nh.param<std::string>("gimbal_angle_topic", gimbal_angle_topic_,
                              std::string("/gimbal_angles"));

        sub_pose_ = nh.subscribe("/mavros/local_position/pose", 1,
                                 &PNGLateral::poseCb, this);
        sub_gimbal_angles_ = nh.subscribe(gimbal_angle_topic_, 1,
                                          &PNGLateral::gimbalAnglesCb, this);

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
        gamma_filter_.setNoise(
            gamma_kalman_q_position_,
            gamma_kalman_q_rate_,
            gamma_kalman_r_);
        gamma_filter_.reset();

        ROS_INFO("[PNGLateral] Forward+side PNG velocity node started. gimbal_angle_topic=%s",
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

            // 若丢失目标，保持前飞，侧向速度归零，垂向保持
            if (lost) {
                v_lat_ = 0.0;
                lambda_filter_.reset();
                gamma_filter_.reset();
                publishCmd(now, V_forward_, v_lat_, lost_vz_hold_, 0.0);
                rate.sleep();
                continue;
            }

            const double lambda_meas = -gimbal_yaw_;
            lambda_filter_.update(lambda_meas, dt);
            const double lambda = lambda_filter_.value();
            const double lambda_dot = lambda_filter_.rate();

            const double gamma_meas = gimbal_pitch_;
            gamma_filter_.update(gamma_meas, dt);
            const double gamma = gamma_filter_.value();
            const double gamma_dot = gamma_filter_.rate();

            // PNG 横向加速度 a = N * V * lambda_dot
            double a_lat = N_nav_ * V_forward_ * lambda_dot;

            // 简单一阶惯性: v_lat += a_lat * dt, 再经时间常数滤波
            double v_lat_target = v_lat_ + a_lat * dt;
            v_lat_target = clamp(v_lat_target, -v_lat_max_, v_lat_max_);
            double alpha = dt / std::max(tau_lat_, dt);
            v_lat_ = (1.0 - alpha) * v_lat_ + alpha * v_lat_target;

            // 垂向速度（按 pitch 导引）
            double v_z_cmd = k_z_ff_ * gamma + N_nav_z_ * gamma_dot;
            v_z_cmd = clamp(v_z_cmd, -v_z_max_, v_z_max_);

            publishCmd(now, V_forward_, v_lat_, v_z_cmd, 0.0);
            rate.sleep();
        }
    }

private:
    // 参数
    double V_forward_, N_nav_, v_lat_max_, tau_lat_;
    double k_z_ff_, N_nav_z_, v_z_max_;
    bool   use_real_dt_;
    double step_dt_;
    double lost_timeout_, lost_vz_hold_;
    std::string gimbal_angle_topic_;

    // 状态
    bool have_pose_, have_gimbal_angles_;
    ros::Time last_time_, last_gimbal_time_;
    double gimbal_yaw_ = 0.0;
    double gimbal_pitch_ = 0.0;
    double v_lat_ = 0.0;
    geometry_msgs::PoseStamped pose_;
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;
    double gamma_kalman_q_position_ = 1e-4;
    double gamma_kalman_q_rate_ = 5e-1;
    double gamma_kalman_r_ = 2e-4;
    ScalarRateKalmanFilter lambda_filter_;
    ScalarRateKalmanFilter gamma_filter_;

    // ROS
    ros::Subscriber sub_pose_, sub_gimbal_angles_;
    ros::Publisher  pub_cmd_;

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pose_ = *msg;
        have_pose_ = true;
    }

    void gimbalAnglesCb(const geometry_msgs::Twist::ConstPtr& msg) {
        gimbal_yaw_ = msg->angular.z;
        gimbal_pitch_ = msg->angular.y;
        have_gimbal_angles_ = true;
        last_gimbal_time_ = ros::Time::now();
    }

    void publishCmd(const ros::Time& stamp, double V_fwd, double v_lat,
                    double v_z, double yaw_rate_cmd) {
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
        cmd.twist.linear.z = have_gimbal_angles_ ? v_z : lost_vz_hold_;
        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = yaw_rate_cmd;
        pub_cmd_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_png");
    ros::NodeHandle nh("~");
    PNGLateral node(nh);
    node.spin();
    return 0;
}
