#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <string>

class TargetSITLSim {
public:
    TargetSITLSim(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : home_set_(false),
          target_initialized_(false),
          path_seq_(0)
    {
        pnh.param<std::string>("self_gps_topic", self_gps_topic_,
                               std::string("/mavros/global_position/global"));
        pnh.param<std::string>("target_gps_topic", target_gps_topic_,
                               std::string("/target/mavros/global_position/global"));
        pnh.param<std::string>("target_vel_topic", target_vel_topic_,
                               std::string("/target/mavros/global_position/raw/gps_vel"));
        pnh.param<std::string>("target_pose_topic", target_pose_topic_,
                               std::string("/target_sim/local_pose"));
        pnh.param<std::string>("target_path_topic", target_path_topic_,
                               std::string("/target_sim/path"));
        pnh.param<std::string>("target_marker_topic", target_marker_topic_,
                               std::string("/target_sim/markers"));
        pnh.param<std::string>("fixed_frame", fixed_frame_, std::string("world"));

        pnh.param("publish_rate", publish_rate_, 10.0);
        pnh.param("relative_distance_m", relative_distance_m_, 100.0);
        pnh.param("relative_bearing_deg", relative_bearing_deg_, 0.0);
        pnh.param("relative_altitude_m", relative_altitude_m_, 0.0);
        pnh.param("speed_mps", speed_mps_, 60.0);
        pnh.param("course_deg", course_deg_, 90.0);
        pnh.param("vertical_speed_mps", vertical_speed_mps_, 0.0);
        pnh.param("marker_scale", marker_scale_, 60.0);
        pnh.param("path_max_points", path_max_points_, 200);

        if (publish_rate_ <= 0.0) {
            publish_rate_ = 10.0;
        }
        if (relative_distance_m_ < 0.0) {
            relative_distance_m_ = 10000.0;
        }
        if (path_max_points_ < 2) {
            path_max_points_ = 2;
        }
        if (marker_scale_ <= 0.0) {
            marker_scale_ = 60.0;
        }

        self_gps_sub_ = nh.subscribe(self_gps_topic_, 1, &TargetSITLSim::selfGpsCb, this);

        target_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>(target_gps_topic_, 1);
        target_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>(target_vel_topic_, 1);
        target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 1);
        target_path_pub_ = nh.advertise<nav_msgs::Path>(target_path_topic_, 1, true);
        target_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(target_marker_topic_, 1);

        target_path_.header.frame_id = fixed_frame_;

        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
                                &TargetSITLSim::timerCb, this);

        ROS_INFO("[TargetSITLSim] Waiting for self GPS on %s", self_gps_topic_.c_str());
    }

private:
    ros::Subscriber self_gps_sub_;
    ros::Publisher target_gps_pub_;
    ros::Publisher target_vel_pub_;
    ros::Publisher target_pose_pub_;
    ros::Publisher target_path_pub_;
    ros::Publisher target_marker_pub_;
    ros::Timer timer_;

    std::string self_gps_topic_;
    std::string target_gps_topic_;
    std::string target_vel_topic_;
    std::string target_pose_topic_;
    std::string target_path_topic_;
    std::string target_marker_topic_;
    std::string fixed_frame_;

    double publish_rate_;
    double relative_distance_m_;
    double relative_bearing_deg_;
    double relative_altitude_m_;
    double speed_mps_;
    double course_deg_;
    double vertical_speed_mps_;
    double marker_scale_;
    int path_max_points_;

    bool home_set_;
    bool target_initialized_;
    ros::Time last_update_time_;
    int path_seq_;

    sensor_msgs::NavSatFix self_gps_;
    sensor_msgs::NavSatFix target_gps_;
    geometry_msgs::TwistStamped target_vel_;
    geometry_msgs::PoseStamped target_pose_;
    nav_msgs::Path target_path_;

    GeographicLib::LocalCartesian geo_converter_;

    double self_local_x_{0.0};
    double self_local_y_{0.0};
    double self_local_z_{0.0};

    double target_local_x_{0.0};
    double target_local_y_{0.0};
    double target_local_z_{0.0};

    static double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void selfGpsCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        self_gps_ = *msg;

        if (!home_set_) {
            geo_converter_.Reset(self_gps_.latitude, self_gps_.longitude, self_gps_.altitude);
            home_set_ = true;
            ROS_INFO("[TargetSITLSim] Home set from self GPS: lat=%.7f lon=%.7f alt=%.2f",
                     self_gps_.latitude, self_gps_.longitude, self_gps_.altitude);
        }

        geo_converter_.Forward(self_gps_.latitude, self_gps_.longitude, self_gps_.altitude,
                               self_local_x_, self_local_y_, self_local_z_);

        if (!target_initialized_) {
            initializeTarget();
        }
    }

    void initializeTarget()
    {
        const double bearing_rad = deg2rad(relative_bearing_deg_);

        // ENU: x-east, y-north；bearing 采用航向习惯：0=北，90=东
        target_local_x_ = self_local_x_ + relative_distance_m_ * std::sin(bearing_rad);
        target_local_y_ = self_local_y_ + relative_distance_m_ * std::cos(bearing_rad);
        target_local_z_ = self_local_z_ + relative_altitude_m_;

        last_update_time_ = ros::Time::now();
        target_initialized_ = true;

        updateKinematics(0.0);
        ROS_INFO("[TargetSITLSim] Target initialized. distance=%.1f m bearing=%.1f deg rel_alt=%.1f m",
                 relative_distance_m_, relative_bearing_deg_, relative_altitude_m_);
    }

    void timerCb(const ros::TimerEvent& event)
    {
        if (!home_set_ || !target_initialized_) {
            return;
        }

        double dt = (event.current_real - last_update_time_).toSec();
        if (dt < 0.0) {
            dt = 0.0;
        }
        last_update_time_ = event.current_real;

        updateKinematics(dt);
        publishTarget(event.current_real);
    }

    void updateKinematics(double dt)
    {
        const double course_rad = deg2rad(course_deg_);

        const double vx = speed_mps_ * std::sin(course_rad);
        const double vy = speed_mps_ * std::cos(course_rad);
        const double vz = vertical_speed_mps_;

        target_local_x_ += vx * dt;
        target_local_y_ += vy * dt;
        target_local_z_ += vz * dt;

        target_vel_.header.stamp = ros::Time::now();
        target_vel_.header.frame_id = fixed_frame_;
        target_vel_.twist.linear.x = vx;
        target_vel_.twist.linear.y = vy;
        target_vel_.twist.linear.z = vz;
        target_vel_.twist.angular.x = 0.0;
        target_vel_.twist.angular.y = 0.0;
        target_vel_.twist.angular.z = 0.0;
    }

    void publishTarget(const ros::Time& stamp)
    {
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        geo_converter_.Reverse(target_local_x_, target_local_y_, target_local_z_, lat, lon, alt);

        target_gps_.header.stamp = stamp;
        target_gps_.header.frame_id = "gps";
        target_gps_.latitude = lat;
        target_gps_.longitude = lon;
        target_gps_.altitude = alt;
        target_gps_.status = self_gps_.status;
        target_gps_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        target_pose_.header.stamp = stamp;
        target_pose_.header.frame_id = fixed_frame_;
        target_pose_.pose.position.x = target_local_x_;
        target_pose_.pose.position.y = target_local_y_;
        target_pose_.pose.position.z = target_local_z_;

        const double yaw = normalizeAngle(std::atan2(target_vel_.twist.linear.y,
                                                     target_vel_.twist.linear.x));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        target_pose_.pose.orientation.x = q.x();
        target_pose_.pose.orientation.y = q.y();
        target_pose_.pose.orientation.z = q.z();
        target_pose_.pose.orientation.w = q.w();

        geometry_msgs::PoseStamped path_pose = target_pose_;
        target_path_.header.stamp = stamp;
        target_path_.poses.push_back(path_pose);
        if (static_cast<int>(target_path_.poses.size()) > path_max_points_) {
            target_path_.poses.erase(target_path_.poses.begin());
        }

        target_gps_pub_.publish(target_gps_);
        target_vel_pub_.publish(target_vel_);
        target_pose_pub_.publish(target_pose_);
        target_path_pub_.publish(target_path_);
        publishMarkers(stamp);
    }

    void publishMarkers(const ros::Time& stamp)
    {
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker target_marker;
        target_marker.header.stamp = stamp;
        target_marker.header.frame_id = fixed_frame_;
        target_marker.ns = "target_sim";
        target_marker.id = 0;
        target_marker.type = visualization_msgs::Marker::SPHERE;
        target_marker.action = visualization_msgs::Marker::ADD;
        target_marker.pose = target_pose_.pose;
        target_marker.scale.x = marker_scale_;
        target_marker.scale.y = marker_scale_;
        target_marker.scale.z = marker_scale_ * 0.5;
        target_marker.color.r = 1.0f;
        target_marker.color.g = 0.2f;
        target_marker.color.b = 0.1f;
        target_marker.color.a = 0.9f;

        visualization_msgs::Marker velocity_marker;
        velocity_marker.header.stamp = stamp;
        velocity_marker.header.frame_id = fixed_frame_;
        velocity_marker.ns = "target_sim";
        velocity_marker.id = 1;
        velocity_marker.type = visualization_msgs::Marker::ARROW;
        velocity_marker.action = visualization_msgs::Marker::ADD;
        velocity_marker.pose = target_pose_.pose;
        velocity_marker.scale.x = std::max(5.0, speed_mps_ * 2.0);
        velocity_marker.scale.y = std::max(2.0, marker_scale_ * 0.12);
        velocity_marker.scale.z = std::max(2.0, marker_scale_ * 0.12);
        velocity_marker.color.r = 0.1f;
        velocity_marker.color.g = 0.8f;
        velocity_marker.color.b = 1.0f;
        velocity_marker.color.a = 0.9f;

        marker_array.markers.push_back(target_marker);
        marker_array.markers.push_back(velocity_marker);
        target_marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_sitl_sim");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    TargetSITLSim node(nh, pnh);
    ros::spin();
    return 0;
}
