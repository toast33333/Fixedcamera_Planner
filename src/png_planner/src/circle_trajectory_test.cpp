#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
double clampValue(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(maximum, value));
}

double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class CircleTrajectoryTest
{
public:
    explicit CircleTrajectoryTest(ros::NodeHandle& nh) : nh_(nh)
    {
        nh_.param<std::string>("pose_topic", pose_topic_, "/mavros/local_position/pose");
        nh_.param<std::string>("velocity_topic", velocity_topic_,
                               "/mavros/setpoint_velocity/cmd_vel");
        nh_.param<std::string>("velocity_feedback_topic", feedback_topic_,
                               "/mavros/local_position/velocity_local");
        nh_.param<std::string>("start_topic", start_topic_, "/circle_trajectory/start");
        nh_.param<std::string>("status_topic", status_topic_, "/circle_trajectory/status");
        nh_.param<std::string>("finished_topic", finished_topic_, "/circle_trajectory/finished");
        nh_.param<std::string>("desired_path_topic", desired_path_topic_,
                               "/circle_trajectory/desired_path");
        nh_.param<std::string>("actual_path_topic", actual_path_topic_,
                               "/circle_trajectory/actual_path");
        nh_.param<std::string>("csv_path", csv_path_, std::string("circle_trajectory_test.csv"));
        nh_.param<std::string>("frame_id", frame_id_, std::string("map"));

        nh_.param("rate", rate_, 20.0);
        nh_.param("require_start_signal", require_start_signal_, false);
        nh_.param("geofence_radius", geofence_radius_, 50.0);
        nh_.param("geofence_margin", geofence_margin_, 5.0);
        nh_.param("stop_buffer", stop_buffer_, 3.0);
        nh_.param("position_timeout", position_timeout_, 0.5);
        nh_.param("start_delay", start_delay_, 3.0);
        nh_.param("minimum_speed", minimum_speed_, 5.0);
        nh_.param("maximum_speed", maximum_speed_, 15.0);
        nh_.param("speed_step", speed_step_, 2.0);
        nh_.param("start_radius", start_radius_, 20.0);
        nh_.param("radial_rate", radial_rate_, 0.5);
        nh_.param("trajectory_error_limit", trajectory_error_limit_, 3.0);
        nh_.param("return_tolerance", return_tolerance_, 1.0);
        nh_.param("return_timeout", return_timeout_, 30.0);
        nh_.param("entry_tolerance", entry_tolerance_, 0.8);
        nh_.param("entry_timeout", entry_timeout_, 30.0);
        nh_.param("positioning_speed", positioning_speed_, 3.0);
        nh_.param("position_kp", position_kp_, 1.0);
        nh_.param("correction_speed_limit", correction_speed_limit_, 2.0);
        nh_.param("altitude_kp", altitude_kp_, 1.0);
        nh_.param("max_vertical_speed", max_vertical_speed_, 1.0);
        nh_.param("yaw_kp", yaw_kp_, 1.5);
        nh_.param("max_yaw_rate", max_yaw_rate_, 1.0);
        nh_.param("center_tolerance", center_tolerance_, 0.5);
        nh_.param("clockwise", clockwise_, false);

        validateParameters();
        for (double speed = minimum_speed_; speed <= maximum_speed_ + 1.0e-6;
             speed += speed_step_) {
            speeds_.push_back(speed);
        }

        pose_sub_ = nh_.subscribe(pose_topic_, 1, &CircleTrajectoryTest::poseCallback, this);
        velocity_sub_ = nh_.subscribe(
            feedback_topic_, 1, &CircleTrajectoryTest::velocityCallback, this);
        start_sub_ = nh_.subscribe(start_topic_, 1, &CircleTrajectoryTest::startCallback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(velocity_topic_, 1);
        status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
        finished_pub_ = nh_.advertise<std_msgs::Bool>(finished_topic_, 1, true);
        desired_path_pub_ = nh_.advertise<nav_msgs::Path>(desired_path_topic_, 1, true);
        actual_path_pub_ = nh_.advertise<nav_msgs::Path>(actual_path_topic_, 1, true);
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / rate_), &CircleTrajectoryTest::timerCallback, this);

        desired_path_.header.frame_id = frame_id_;
        actual_path_.header.frame_id = frame_id_;
        openCsv();
        publishStatus(require_start_signal_ ? "WAIT_POSE_AND_START" : "WAIT_POSE");
        ROS_WARN("[CircleTrajectoryTest] Waiting for local pose on %s. "
                 "Command topic: %s; %zu speed trials.",
                 pose_topic_.c_str(), velocity_topic_.c_str(), speeds_.size());
    }

    ~CircleTrajectoryTest()
    {
        if (command_ownership_started_) publishStop();
    }

private:
    enum State {
        WAIT_POSE,
        WAIT_START,
        DELAY,
        MOVE_TO_START,
        SPIRAL,
        RETURN_HOME,
        FINISHED,
        ABORTED
    };

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, velocity_sub_, start_sub_;
    ros::Publisher velocity_pub_, status_pub_, finished_pub_;
    ros::Publisher desired_path_pub_, actual_path_pub_;
    ros::Timer timer_;

    std::string pose_topic_, velocity_topic_, feedback_topic_, start_topic_;
    std::string status_topic_, finished_topic_, desired_path_topic_, actual_path_topic_;
    std::string csv_path_, frame_id_;
    std::ofstream csv_;
    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped feedback_;
    nav_msgs::Path desired_path_, actual_path_;
    ros::Time last_pose_time_, last_feedback_time_, state_start_;
    ros::Time previous_pose_stamp_;

    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.0;
    double yaw_ = 0.0;
    double previous_yaw_ = 0.0;
    double measured_yaw_rate_ = 0.0;
    double rate_ = 20.0;
    double geofence_radius_ = 50.0;
    double geofence_margin_ = 5.0;
    double stop_buffer_ = 3.0;
    double position_timeout_ = 0.5;
    double start_delay_ = 3.0;
    double minimum_speed_ = 5.0;
    double maximum_speed_ = 15.0;
    double speed_step_ = 2.0;
    double start_radius_ = 20.0;
    double radial_rate_ = 0.5;
    double trajectory_error_limit_ = 3.0;
    double return_tolerance_ = 1.0;
    double return_timeout_ = 30.0;
    double entry_tolerance_ = 0.8;
    double entry_timeout_ = 30.0;
    double positioning_speed_ = 3.0;
    double position_kp_ = 1.0;
    double correction_speed_limit_ = 2.0;
    double altitude_kp_ = 1.0;
    double max_vertical_speed_ = 1.0;
    double yaw_kp_ = 1.5;
    double max_yaw_rate_ = 1.0;
    double center_tolerance_ = 0.5;
    bool clockwise_ = false;
    bool require_start_signal_ = false;
    bool start_requested_ = false;
    bool have_pose_ = false;
    bool have_feedback_ = false;
    bool origin_set_ = false;
    bool command_ownership_started_ = false;
    State state_ = WAIT_POSE;
    std::vector<double> speeds_;
    std::size_t speed_index_ = 0;
    std::size_t trial_index_ = 0;
    std::size_t samples_ = 0;
    std::size_t path_decimation_ = 0;
    double spiral_elapsed_ = 0.0;
    double speed_sum_ = 0.0;
    double yaw_rate_sum_ = 0.0;
    double error_sum_ = 0.0;
    double max_error_ = 0.0;

    void validateParameters()
    {
        rate_ = std::max(5.0, rate_);
        geofence_margin_ = std::max(0.5, geofence_margin_);
        stop_buffer_ = std::max(0.0, stop_buffer_);
        position_timeout_ = std::max(0.1, position_timeout_);
        start_delay_ = std::max(0.0, start_delay_);
        if (geofence_radius_ <= geofence_margin_ + stop_buffer_ || start_radius_ <= 0.0 ||
            start_radius_ >= geofence_radius_ - geofence_margin_ - stop_buffer_ ||
            minimum_speed_ <= 0.0 || maximum_speed_ < minimum_speed_ || speed_step_ <= 0.0 ||
            radial_rate_ <= 0.0 || minimum_speed_ <= radial_rate_ ||
            trajectory_error_limit_ <= 0.0 || return_tolerance_ <= 0.0 ||
            return_timeout_ <= 0.0 || entry_tolerance_ <= 0.0 || entry_timeout_ <= 0.0 ||
            positioning_speed_ <= 0.0 || position_kp_ < 0.0 || correction_speed_limit_ < 0.0 ||
            altitude_kp_ < 0.0 || max_vertical_speed_ <= 0.0 || yaw_kp_ < 0.0 ||
            max_yaw_rate_ <= 0.0 || center_tolerance_ <= 0.0 ||
            center_tolerance_ >= start_radius_) {
            throw std::runtime_error("Invalid circle-trajectory or geofence parameters");
        }
    }

    void openCsv()
    {
        csv_.open(csv_path_.c_str(), std::ios::out | std::ios::trunc);
        if (!csv_) throw std::runtime_error("Cannot open CSV output: " + csv_path_);
        csv_ << "ros_time,trial,status,speed_cmd_mps,spiral_elapsed_s,"
                "final_radius_m,trajectory_error_mean_m,trajectory_error_max_m,"
                "speed_actual_mean_mps,yaw_rate_actual_mean_radps,"
                "estimated_curvature_mean_1pm,distance_from_origin_m\n";
        csv_.flush();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose_ = *msg;
        const ros::Time now = ros::Time::now();
        last_pose_time_ = now;
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z, msg->pose.orientation.w);
        double roll = 0.0;
        double pitch = 0.0;
        double new_yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, new_yaw);
        if (have_pose_) {
            const double dt = (now - previous_pose_stamp_).toSec();
            if (dt > 1.0e-4 && dt < 1.0) {
                measured_yaw_rate_ = wrapAngle(new_yaw - previous_yaw_) / dt;
            }
        }
        previous_pose_stamp_ = now;
        previous_yaw_ = new_yaw;
        yaw_ = new_yaw;
        have_pose_ = true;

        if (!origin_set_ && (!require_start_signal_ || start_requested_)) {
            latchOrigin(now);
        } else if (!origin_set_ && require_start_signal_) {
            state_ = WAIT_START;
        }
    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        feedback_ = *msg;
        last_feedback_time_ = ros::Time::now();
        have_feedback_ = true;
    }

    void startCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (!msg->data || origin_set_) return;
        start_requested_ = true;
        if (have_pose_) latchOrigin(ros::Time::now());
    }

    void latchOrigin(const ros::Time& now)
    {
        origin_x_ = pose_.pose.position.x;
        origin_y_ = pose_.pose.position.y;
        origin_z_ = pose_.pose.position.z;
        origin_set_ = true;
        command_ownership_started_ = true;
        state_start_ = now;
        setState(DELAY, "DELAY");
        ROS_WARN("[CircleTrajectoryTest] Origin=(%.2f, %.2f, %.2f); "
                 "starting after %.1f s.",
                 origin_x_, origin_y_, origin_z_, start_delay_);
    }

    double dx() const { return pose_.pose.position.x - origin_x_; }
    double dy() const { return pose_.pose.position.y - origin_y_; }
    double distanceFromOrigin() const { return std::hypot(dx(), dy()); }

    void publishStatus(const std::string& text)
    {
        std_msgs::String message;
        message.data = text;
        status_pub_.publish(message);
    }

    void setState(State state, const std::string& text)
    {
        state_ = state;
        publishStatus(text);
    }

    void publishFinished()
    {
        std_msgs::Bool message;
        message.data = true;
        finished_pub_.publish(message);
    }

    void publishCommand(double vx, double vy, double vz, double yaw_rate)
    {
        geometry_msgs::TwistStamped command;
        command.header.stamp = ros::Time::now();
        command.header.frame_id = frame_id_;
        command.twist.linear.x = vx;
        command.twist.linear.y = vy;
        command.twist.linear.z = vz;
        command.twist.angular.z = yaw_rate;
        velocity_pub_.publish(command);
    }

    void publishStop() { publishCommand(0.0, 0.0, 0.0, 0.0); }

    void abortTest(const std::string& reason)
    {
        publishStop();
        setState(ABORTED, "ABORTED: " + reason);
        publishFinished();
        ROS_ERROR("[CircleTrajectoryTest] ABORT: %s", reason.c_str());
    }

    void beginMoveToStart(const ros::Time& now)
    {
        if (speed_index_ >= speeds_.size()) {
            setState(FINISHED, "FINISHED");
            publishStop();
            publishFinished();
            ROS_WARN("[CircleTrajectoryTest] All %zu speed trials complete. CSV: %s",
                     speeds_.size(), csv_path_.c_str());
            return;
        }
        ++trial_index_;
        state_start_ = now;
        setState(MOVE_TO_START, "MOVE_TO_START");
        ROS_WARN("[CircleTrajectoryTest] Trial %zu, nominal speed %.2f m/s: "
                 "moving to the %.1f m outer radius.",
                 trial_index_, speeds_[speed_index_], start_radius_);
    }

    void beginSpiral(const ros::Time& now)
    {
        spiral_elapsed_ = 0.0;
        samples_ = 0;
        speed_sum_ = 0.0;
        yaw_rate_sum_ = 0.0;
        error_sum_ = 0.0;
        max_error_ = 0.0;
        desired_path_.poses.clear();
        actual_path_.poses.clear();
        state_start_ = now;
        setState(SPIRAL, "SPIRAL");
        ROS_WARN("[CircleTrajectoryTest] Trial %zu started at %.2f m/s (%s).",
                 trial_index_, speeds_[speed_index_], clockwise_ ? "clockwise" : "counter-clockwise");
    }

    void writeTrial(const ros::Time& now, const std::string& status)
    {
        const double speed_average = samples_ ? speed_sum_ / samples_ : 0.0;
        const double yaw_average = samples_ ? yaw_rate_sum_ / samples_ : 0.0;
        const double error_average = samples_ ? error_sum_ / samples_ : 0.0;
        const double curvature = speed_average > 1.0e-3
                                     ? std::fabs(yaw_average) / speed_average
                                     : 0.0;
        csv_ << now.toSec() << ',' << trial_index_ << ',' << status << ','
             << speeds_[speed_index_] << ',' << spiral_elapsed_ << ','
             << distanceFromOrigin() << ',' << error_average << ',' << max_error_ << ','
             << speed_average << ',' << yaw_average << ',' << curvature << ','
             << distanceFromOrigin() << '\n';
        csv_.flush();
        ROS_INFO("[CircleTrajectoryTest] Trial %zu %s: mean/max error %.2f/%.2f m; "
                 "mean speed %.2f m/s.",
                 trial_index_, status.c_str(), error_average, max_error_, speed_average);
    }

    void startReturn(const ros::Time& now, const std::string& reason)
    {
        writeTrial(now, reason);
        state_start_ = now;
        setState(RETURN_HOME, "RETURN_HOME: " + reason);
        ROS_WARN("[CircleTrajectoryTest] Trial %zu complete; returning to the hover origin.",
                 trial_index_);
    }

    void commandToward(double target_x, double target_y, double maximum_speed)
    {
        const double error_x = target_x - pose_.pose.position.x;
        const double error_y = target_y - pose_.pose.position.y;
        const double distance = std::hypot(error_x, error_y);
        double vx = 0.0;
        double vy = 0.0;
        if (distance > 1.0e-6) {
            const double speed = std::min(maximum_speed, std::max(0.3, position_kp_ * distance));
            vx = speed * error_x / distance;
            vy = speed * error_y / distance;
        }
        const double desired_yaw = distance > 1.0e-6 ? std::atan2(vy, vx) : yaw_;
        const double yaw_rate = clampValue(yaw_kp_ * wrapAngle(desired_yaw - yaw_),
                                           -max_yaw_rate_, max_yaw_rate_);
        const double vz = clampValue(altitude_kp_ * (origin_z_ - pose_.pose.position.z),
                                     -max_vertical_speed_, max_vertical_speed_);
        publishCommand(vx, vy, vz, yaw_rate);
    }

    void publishPaths(const ros::Time& now, double desired_x, double desired_y)
    {
        if (++path_decimation_ % 4 != 0) return;
        geometry_msgs::PoseStamped desired;
        desired.header.stamp = now;
        desired.header.frame_id = frame_id_;
        desired.pose.position.x = desired_x;
        desired.pose.position.y = desired_y;
        desired.pose.position.z = origin_z_;
        desired.pose.orientation.w = 1.0;
        desired_path_.header.stamp = now;
        desired_path_.poses.push_back(desired);
        actual_path_.header.stamp = now;
        geometry_msgs::PoseStamped actual = pose_;
        actual.header.stamp = now;
        actual.header.frame_id = frame_id_;
        actual_path_.poses.push_back(actual);
        if (desired_path_.poses.size() > 3000) desired_path_.poses.erase(desired_path_.poses.begin());
        if (actual_path_.poses.size() > 3000) actual_path_.poses.erase(actual_path_.poses.begin());
        desired_path_pub_.publish(desired_path_);
        actual_path_pub_.publish(actual_path_);
    }

    void runSpiral(const ros::Time& now)
    {
        if (!have_feedback_ || (now - last_feedback_time_).toSec() > position_timeout_) {
            abortTest("velocity feedback timed out");
            return;
        }

        const double elapsed = (now - state_start_).toSec();
        const double radius = std::max(0.0, start_radius_ - radial_rate_ * elapsed);
        if (radius <= center_tolerance_) {
            spiral_elapsed_ = elapsed;
            startReturn(now, "CENTER_REACHED");
            return;
        }

        const double nominal_speed = speeds_[speed_index_];
        const double tangential_speed =
            std::sqrt(std::max(0.0, nominal_speed * nominal_speed - radial_rate_ * radial_rate_));
        const double direction = clockwise_ ? -1.0 : 1.0;
        const double theta = direction * tangential_speed / radial_rate_ *
                             std::log(start_radius_ / radius);
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        const double desired_x = origin_x_ + radius * cos_theta;
        const double desired_y = origin_y_ + radius * sin_theta;
        const double error_x = desired_x - pose_.pose.position.x;
        const double error_y = desired_y - pose_.pose.position.y;
        const double error = std::hypot(error_x, error_y);
        if (error > trajectory_error_limit_) {
            spiral_elapsed_ = elapsed;
            startReturn(now, "ERROR_LIMIT");
            return;
        }

        double correction_x = position_kp_ * error_x;
        double correction_y = position_kp_ * error_y;
        const double correction_norm = std::hypot(correction_x, correction_y);
        if (correction_norm > correction_speed_limit_ && correction_norm > 1.0e-9) {
            const double scale = correction_speed_limit_ / correction_norm;
            correction_x *= scale;
            correction_y *= scale;
        }

        const double tangent_x = direction * -sin_theta;
        const double tangent_y = direction * cos_theta;
        double vx = -radial_rate_ * cos_theta + tangential_speed * tangent_x + correction_x;
        double vy = -radial_rate_ * sin_theta + tangential_speed * tangent_y + correction_y;
        const double planar_speed = std::hypot(vx, vy);
        const double command_limit = nominal_speed + correction_speed_limit_;
        if (planar_speed > command_limit && planar_speed > 1.0e-9) {
            const double scale = command_limit / planar_speed;
            vx *= scale;
            vy *= scale;
        }

        const double desired_yaw = std::atan2(vy, vx);
        const double yaw_rate_feedforward = direction * tangential_speed / radius;
        const double yaw_rate = clampValue(
            yaw_rate_feedforward + yaw_kp_ * wrapAngle(desired_yaw - yaw_),
            -max_yaw_rate_, max_yaw_rate_);
        const double vz = clampValue(altitude_kp_ * (origin_z_ - pose_.pose.position.z),
                                     -max_vertical_speed_, max_vertical_speed_);
        publishCommand(vx, vy, vz, yaw_rate);
        publishPaths(now, desired_x, desired_y);

        spiral_elapsed_ = elapsed;
        speed_sum_ += std::hypot(feedback_.twist.linear.x, feedback_.twist.linear.y);
        yaw_rate_sum_ += measured_yaw_rate_;
        error_sum_ += error;
        max_error_ = std::max(max_error_, error);
        ++samples_;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();
        if (state_ == WAIT_POSE || state_ == WAIT_START) return;
        if (state_ == FINISHED || state_ == ABORTED) {
            publishStop();
            return;
        }
        if (!have_pose_ || (now - last_pose_time_).toSec() > position_timeout_) {
            abortTest("local position feedback timed out");
            return;
        }

        const double actual_speed = have_feedback_
                                        ? std::hypot(feedback_.twist.linear.x,
                                                     feedback_.twist.linear.y)
                                        : maximum_speed_;
        const double dynamic_buffer =
            std::max(stop_buffer_, actual_speed * position_timeout_ + 2.0);
        if (distanceFromOrigin() >= geofence_radius_ - geofence_margin_ - dynamic_buffer) {
            abortTest("geofence safety boundary reached");
            return;
        }

        if (state_ == DELAY) {
            publishStop();
            if ((now - state_start_).toSec() >= start_delay_) beginMoveToStart(now);
            return;
        }

        if (state_ == MOVE_TO_START) {
            const double target_x = origin_x_ + start_radius_;
            const double target_y = origin_y_;
            const double error = std::hypot(target_x - pose_.pose.position.x,
                                            target_y - pose_.pose.position.y);
            if (error <= entry_tolerance_) {
                beginSpiral(now);
                return;
            }
            if ((now - state_start_).toSec() > entry_timeout_) {
                abortTest("timed out while moving to the outer-radius start point");
                return;
            }
            commandToward(target_x, target_y, positioning_speed_);
            return;
        }

        if (state_ == RETURN_HOME) {
            const double home_distance = distanceFromOrigin();
            if (home_distance <= return_tolerance_) {
                publishStop();
                ++speed_index_;
                state_start_ = now;
                if (speed_index_ >= speeds_.size()) {
                    setState(FINISHED, "FINISHED");
                    publishFinished();
                    ROS_WARN("[CircleTrajectoryTest] Returned home; all trials complete. CSV: %s",
                             csv_path_.c_str());
                } else {
                    setState(DELAY, "DELAY_BEFORE_NEXT_TRIAL");
                    ROS_WARN("[CircleTrajectoryTest] Returned home; next speed %.2f m/s.",
                             speeds_[speed_index_]);
                }
                return;
            }
            if ((now - state_start_).toSec() > return_timeout_) {
                abortTest("return-to-origin timed out");
                return;
            }
            commandToward(origin_x_, origin_y_, positioning_speed_);
            return;
        }

        if (state_ == SPIRAL) runSpiral(now);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_trajectory_test");
    ros::NodeHandle nh("~");
    try {
        CircleTrajectoryTest test(nh);
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[CircleTrajectoryTest] %s", error.what());
        return 1;
    }
    return 0;
}
