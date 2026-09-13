#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <kcf_msgs/Bbox.h>
#include <detection_msgs/Detection.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

class StrategyFSM {
public:
    explicit StrategyFSM(ros::NodeHandle& nh)
        : phase_(APPROACH)
    {
        // 全局引导参数
        nh.param("threshold_dist1", dist1_, 800.0);
        nh.param("threshold_dist2", dist2_, 2000.0);
        nh.param("threshold_dist3", dist3_, 5000.0);
        nh.param("threshold_dist4", dist4_, 10000.0);
        nh.param("speed_threshold", speed_threshold_, 100.0);
        nh.param("offset_A", offset_A_, 300.0);
        nh.param("offset_B", offset_B_, 300.0);
        nh.param("offset_C", offset_C_, 200.0);
        nh.param("self_speed", self_speed_, 30.0);
        nh.param("png_trigger_dist", png_trigger_dist_, 200.0);
        nh.param("fsm_update_rate", fsm_rate_, 5.0);
        nh.param("cmd_smooth_alpha", cmd_smooth_alpha_, 0.2);
        nh.param("yaw_smooth_alpha", yaw_smooth_alpha_, 0.3);

        // 搜索衔接参数
        nh.param("search_entry_dist", search_entry_dist_, 30.0);
        nh.param("search_abort_dist", search_abort_dist_, 60.0);
        nh.param("search_timeout_sec", search_timeout_sec_, 8.0);
        nh.param("search_flight_speed", search_flight_speed_, self_speed_);
        nh.param("search_reach_tolerance", search_reach_tolerance_, 8.0);
        nh.param("search_yaw_kp", search_yaw_kp_, 1.2);
        nh.param("search_max_yaw_rate", search_max_yaw_rate_, 0.8);
        nh.param("lock_confirm_count", lock_confirm_count_, 5);
        nh.param("track_timeout_sec", track_timeout_sec_, 0.5);
        nh.param("min_track_score", min_track_score_, 0.0);

        // 锁定话题参数
        nh.param("detection_mode", detection_mode_, 0);
        nh.param<std::string>("detection_topic", detection_topic_, std::string("/object_kcf"));
        if (detection_mode_ == 1 && detection_topic_ == "/object_kcf") {
            detection_topic_ = "/object_detections";
        }

        if (fsm_rate_ <= 0.0) {
            fsm_rate_ = 5.0;
        }
        if (search_entry_dist_ < 0.0) {
            search_entry_dist_ = 0.0;
        }
        if (search_abort_dist_ < search_entry_dist_) {
            search_abort_dist_ = search_entry_dist_ + 10.0;
        }
        if (search_flight_speed_ <= 0.0) {
            search_flight_speed_ = std::max(1.0, self_speed_);
        }
        if (search_reach_tolerance_ < 0.0) {
            search_reach_tolerance_ = 0.0;
        }
        if (search_yaw_kp_ < 0.0) {
            search_yaw_kp_ = 0.0;
        }
        if (search_max_yaw_rate_ <= 0.0) {
            search_max_yaw_rate_ = 0.8;
        }
        if (lock_confirm_count_ < 1) {
            lock_confirm_count_ = 1;
        }
        if (track_timeout_sec_ <= 0.0) {
            track_timeout_sec_ = 0.5;
        }

        pos_cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
        search_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
        png_trigger_pub_ = nh.advertise<std_msgs::Bool>("/png_start", 1);
        search_enable_pub_ = nh.advertise<std_msgs::Bool>("/enable_search", 1, true);

        ROS_INFO("[FSM] Waiting for initial /target/mavros/global_position/global...");
        ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/target/mavros/global_position/global");
        ROS_INFO("[FSM] Initial target GPS received. FSM starting.");

        target_pos_sub_ = nh.subscribe("/target/mavros/global_position/global", 1,
                                       &StrategyFSM::targetPosCallback, this);
        target_vel_sub_ = nh.subscribe("/target/mavros/global_position/raw/gps_vel", 1,
                                       &StrategyFSM::targetVelCallback, this);
        self_gps_sub_ = nh.subscribe("/mavros/global_position/global", 1,
                                     &StrategyFSM::selfGpsCallback, this);
        self_pose_sub_ = nh.subscribe("/mavros/local_position/pose", 1,
                                      &StrategyFSM::selfPoseCallback, this);

        if (detection_mode_ == 1) {
            det_sub_ = nh.subscribe<detection_msgs::Detection>(
                detection_topic_, 1, &StrategyFSM::detectionCallback, this);
            ROS_INFO("[FSM] Lock source: %s (detection_msgs/Detection)", detection_topic_.c_str());
        } else {
            kcf_sub_ = nh.subscribe<kcf_msgs::Bbox>(
                detection_topic_, 1, &StrategyFSM::kcfCallback, this);
            ROS_INFO("[FSM] Lock source: %s (kcf_msgs/Bbox)", detection_topic_.c_str());
        }

        last_eval_time_ = ros::Time(0);
        last_track_time_ = ros::Time(0);
        search_start_time_ = ros::Time(0);

        home_set_ = false;
        target_received_ = false;
        have_pose_ = false;
        attack_started_ = false;
        search_enable_state_ = false;
        cmd_init_ = false;
        yaw_init_ = false;
        track_count_ = 0;
        latest_track_score_ = 0.0;
        current_strategy_ = 'D';

        publishSearchEnable(false, true);
        ROS_INFO("Strategy FSM initialized: APPROACH -> SEARCH -> ATTACK.");
    }

private:
    enum Phase {
        APPROACH = 0,
        SEARCH = 1,
        ATTACK = 2
    };

    ros::Subscriber target_pos_sub_;
    ros::Subscriber target_vel_sub_;
    ros::Subscriber self_gps_sub_;
    ros::Subscriber self_pose_sub_;
    ros::Subscriber kcf_sub_;
    ros::Subscriber det_sub_;

    ros::Publisher pos_cmd_pub_;
    ros::Publisher search_vel_pub_;
    ros::Publisher png_trigger_pub_;
    ros::Publisher search_enable_pub_;

    sensor_msgs::NavSatFix target_gps_;
    geometry_msgs::TwistStamped target_vel_;
    sensor_msgs::NavSatFix self_gps_;
    geometry_msgs::PoseStamped self_pose_;
    geometry_msgs::Point target_local_;
    geometry_msgs::Point self_local_;

    double dist1_, dist2_, dist3_, dist4_;
    double speed_threshold_;
    double offset_A_, offset_B_, offset_C_;
    double self_speed_;
    double png_trigger_dist_;
    double fsm_rate_;
    double cmd_smooth_alpha_;
    double yaw_smooth_alpha_;

    double search_entry_dist_;
    double search_abort_dist_;
    double search_timeout_sec_;
    double search_flight_speed_;
    double search_reach_tolerance_;
    double search_yaw_kp_;
    double search_max_yaw_rate_;
    int lock_confirm_count_;
    double track_timeout_sec_;
    double min_track_score_;
    int detection_mode_;
    std::string detection_topic_;

    GeographicLib::LocalCartesian geo_converter_;

    ros::Time last_eval_time_;
    ros::Time last_track_time_;
    ros::Time search_start_time_;

    bool home_set_;
    bool target_received_;
    bool have_pose_;
    bool attack_started_;
    bool search_enable_state_;
    bool cmd_init_;
    bool yaw_init_;
    int track_count_;
    double latest_track_score_;
    char current_strategy_;
    Phase phase_;

    geometry_msgs::Point cmd_pos_prev_;
    double yaw_last_{0.0};

    static double clampValue(double v, double min_v, double max_v)
    {
        return std::max(min_v, std::min(max_v, v));
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static double distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static const char* phaseName(Phase phase)
    {
        switch (phase) {
        case APPROACH:
            return "APPROACH";
        case SEARCH:
            return "SEARCH";
        case ATTACK:
            return "ATTACK";
        default:
            return "UNKNOWN";
        }
    }

    void targetPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        target_gps_ = *msg;
        target_received_ = true;

        if (home_set_) {
            geo_converter_.Forward(target_gps_.latitude, target_gps_.longitude, target_gps_.altitude,
                                   target_local_.x, target_local_.y, target_local_.z);
            evaluateStrategy();
        }
    }

    void targetVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        target_vel_ = *msg;
    }

    void selfGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        self_gps_ = *msg;

        if (!home_set_) {
            geo_converter_.Reset(self_gps_.latitude, self_gps_.longitude, self_gps_.altitude);
            home_set_ = true;
            ROS_INFO("[FSM] Home origin set from self GPS: lat=%.7f lon=%.7f alt=%.2f",
                     self_gps_.latitude, self_gps_.longitude, self_gps_.altitude);
        }

        geo_converter_.Forward(self_gps_.latitude, self_gps_.longitude, self_gps_.altitude,
                               self_local_.x, self_local_.y, self_local_.z);

        if (home_set_ && target_received_) {
            geo_converter_.Forward(target_gps_.latitude, target_gps_.longitude, target_gps_.altitude,
                                   target_local_.x, target_local_.y, target_local_.z);
            evaluateStrategy();
        }
    }

    void selfPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        self_pose_ = *msg;
        have_pose_ = true;
    }

    void kcfCallback(const kcf_msgs::Bbox::ConstPtr& msg)
    {
        updateTrackStatus(msg->is_tracking, msg->score);
    }

    void detectionCallback(const detection_msgs::Detection::ConstPtr& msg)
    {
        updateTrackStatus(msg->is_tracking, msg->score);
    }

    void updateTrackStatus(bool is_tracking, double score)
    {
        const ros::Time now = ros::Time::now();
        const bool valid = is_tracking && (score >= min_track_score_);

        if (!valid) {
            track_count_ = 0;
            latest_track_score_ = score;
            return;
        }

        if (last_track_time_.isZero() || (now - last_track_time_).toSec() > track_timeout_sec_) {
            track_count_ = 1;
        } else {
            ++track_count_;
        }

        last_track_time_ = now;
        latest_track_score_ = score;
    }

    void evaluateStrategy()
    {
        if (!home_set_ || !target_received_) {
            return;
        }

        const ros::Time now = ros::Time::now();
        if ((now - last_eval_time_).toSec() < (1.0 / fsm_rate_)) {
            return;
        }
        last_eval_time_ = now;

        if (!last_track_time_.isZero() && (now - last_track_time_).toSec() > track_timeout_sec_) {
            track_count_ = 0;
        }

        const double rel_x = target_local_.x - self_local_.x;
        const double rel_y = target_local_.y - self_local_.y;
        const double rel_z = target_local_.z - self_local_.z;
        const double distance = std::sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z);
        const double target_speed = std::sqrt(
            target_vel_.twist.linear.x * target_vel_.twist.linear.x +
            target_vel_.twist.linear.y * target_vel_.twist.linear.y +
            target_vel_.twist.linear.z * target_vel_.twist.linear.z);

        const bool pointing_to_origin = isPointingToOrigin(rel_x, rel_y, rel_z, target_vel_);
        current_strategy_ = decideStrategy(distance, target_speed, pointing_to_origin);

        geometry_msgs::PoseStamped approach_cmd;
        const bool approach_cmd_valid = computePosCmd(current_strategy_, approach_cmd);
        const double dist_to_cmd = approach_cmd_valid ?
            distance3D(self_local_, approach_cmd.pose.position) :
            std::numeric_limits<double>::infinity();

        const double time_to_reach = estimateTimeToReach(distance);
        const double pred_dist_horiz = computePredictedHorizontalDistance(time_to_reach);

        switch (phase_) {
        case APPROACH:
            handleApproachState(approach_cmd_valid, approach_cmd, distance);
            break;
        case SEARCH:
            handleSearchState(now, approach_cmd_valid, approach_cmd, distance);
            break;
        case ATTACK:
            publishSearchEnable(false);
            break;
        }

        ROS_INFO_STREAM_THROTTLE(1.0,
            "[FSM] phase=" << phaseName(phase_)
            << " strategy=" << current_strategy_
            << " target_speed=" << target_speed
            << " rel_dist=" << distance
            << " dist_to_cmd=" << dist_to_cmd
            << " pred_horiz=" << pred_dist_horiz
            << " track_count=" << track_count_
            << " track_score=" << latest_track_score_
            << " search_enable=" << (search_enable_state_ ? "true" : "false"));
    }

    void handleApproachState(bool approach_cmd_valid,
                             const geometry_msgs::PoseStamped& approach_cmd,
                             double relative_distance)
    {
        publishSearchEnable(false);

        if (!approach_cmd_valid) {
            return;
        }

        pos_cmd_pub_.publish(approach_cmd);

        if (relative_distance <= search_entry_dist_) {
            transitionTo(SEARCH, "Relative distance reached search threshold.");
        }
    }

    void handleSearchState(const ros::Time& now,
                           bool approach_cmd_valid,
                           const geometry_msgs::PoseStamped& approach_cmd,
                           double relative_distance)
    {
        publishSearchEnable(true);

        if (hasStableLock(now)) {
            transitionTo(ATTACK, "Stable target lock confirmed.");
            return;
        }

        if (!approach_cmd_valid) {
            transitionTo(APPROACH, "No valid global search command.");
            return;
        }

        publishSearchVelocityCmd(now, approach_cmd.pose.position);

        const double search_elapsed = (now - search_start_time_).toSec();
        if (search_timeout_sec_ > 0.0 && search_elapsed > search_timeout_sec_) {
            transitionTo(APPROACH, "Search timeout, back to approach.");
            return;
        }

        if (relative_distance > search_abort_dist_) {
            transitionTo(APPROACH, "Relative distance exceeded search abort threshold.");
            return;
        }
    }

    void transitionTo(Phase new_phase, const std::string& reason)
    {
        if (phase_ == new_phase) {
            return;
        }

        const Phase old_phase = phase_;
        phase_ = new_phase;

        if (new_phase == APPROACH) {
            publishSearchEnable(false, true);
        } else if (new_phase == SEARCH) {
            publishSearchEnable(true, true);
            search_start_time_ = ros::Time::now();
            track_count_ = 0;
            last_track_time_ = ros::Time(0);
        } else if (new_phase == ATTACK) {
            publishSearchEnable(false, true);
            triggerAttack();
        }

        ROS_WARN("[FSM] %s -> %s | %s",
                 phaseName(old_phase), phaseName(new_phase), reason.c_str());
    }

    void triggerAttack()
    {
        if (attack_started_) {
            return;
        }

        std_msgs::Bool png_msg;
        png_msg.data = true;
        png_trigger_pub_.publish(png_msg);
        attack_started_ = true;
        ROS_WARN("[FSM] PNG attack triggered.");
    }

    void publishSearchEnable(bool enable, bool force = false)
    {
        if (!force && search_enable_state_ == enable) {
            return;
        }

        std_msgs::Bool msg;
        msg.data = enable;
        search_enable_pub_.publish(msg);
        search_enable_state_ = enable;
    }

    void publishSearchVelocityCmd(const ros::Time& now, const geometry_msgs::Point& target_point)
    {
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = now;

        const double dx = target_point.x - self_local_.x;
        const double dy = target_point.y - self_local_.y;
        const double dz = target_point.z - self_local_.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > search_reach_tolerance_ && dist > 1e-3) {
            const double scale = search_flight_speed_ / dist;
            cmd.twist.linear.x = dx * scale;
            cmd.twist.linear.y = dy * scale;
            cmd.twist.linear.z = dz * scale;
        } else {
            const double target_vel_norm = std::sqrt(
                target_vel_.twist.linear.x * target_vel_.twist.linear.x +
                target_vel_.twist.linear.y * target_vel_.twist.linear.y +
                target_vel_.twist.linear.z * target_vel_.twist.linear.z);

            if (target_vel_norm > 1e-3) {
                const double scale = search_flight_speed_ / target_vel_norm;
                cmd.twist.linear.x = target_vel_.twist.linear.x * scale;
                cmd.twist.linear.y = target_vel_.twist.linear.y * scale;
                cmd.twist.linear.z = target_vel_.twist.linear.z * scale;
            } else {
                cmd.twist.linear.x = 0.0;
                cmd.twist.linear.y = 0.0;
                cmd.twist.linear.z = 0.0;
            }
        }

        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = computeSearchYawRate();
        search_vel_pub_.publish(cmd);
    }

    double computeSearchYawRate() const
    {
        if (!have_pose_) {
            return 0.0;
        }

        const double desired_yaw = std::atan2(target_local_.y - self_local_.y,
                                              target_local_.x - self_local_.x);

        tf2::Quaternion q(
            self_pose_.pose.orientation.x,
            self_pose_.pose.orientation.y,
            self_pose_.pose.orientation.z,
            self_pose_.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll = 0.0;
        double pitch = 0.0;
        double current_yaw = 0.0;
        m.getRPY(roll, pitch, current_yaw);

        const double yaw_error = normalizeAngle(desired_yaw - current_yaw);
        return clampValue(search_yaw_kp_ * yaw_error,
                          -search_max_yaw_rate_, search_max_yaw_rate_);
    }

    bool hasStableLock(const ros::Time& now) const
    {
        if (track_count_ < lock_confirm_count_) {
            return false;
        }
        if (last_track_time_.isZero()) {
            return false;
        }
        return (now - last_track_time_).toSec() <= track_timeout_sec_;
    }

    char decideStrategy(double distance, double target_speed, bool pointing_to_origin) const
    {
        if (target_speed > speed_threshold_) {
            return 'A';
        }
        if (pointing_to_origin) {
            if (distance < dist1_) return 'A';
            if (distance < dist2_) return 'B';
            if (distance < dist3_) return 'C';
            return 'D';
        }
        if (distance < dist2_) return 'B';
        if (distance < dist3_) return 'B';
        return 'C';
    }

    bool isPointingToOrigin(double rel_x, double rel_y, double rel_z,
                            const geometry_msgs::TwistStamped& vel) const
    {
        const double dot = rel_x * vel.twist.linear.x +
                           rel_y * vel.twist.linear.y +
                           rel_z * vel.twist.linear.z;
        return dot < 0.0;
    }

    bool computePosCmd(char strategy, geometry_msgs::PoseStamped& cmd)
    {
        if (strategy == 'D') {
            return false;
        }

        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";

        const double v_norm = std::sqrt(
            target_vel_.twist.linear.x * target_vel_.twist.linear.x +
            target_vel_.twist.linear.y * target_vel_.twist.linear.y);
        if (v_norm < 1e-3) {
            return false;
        }

        const double vx = target_vel_.twist.linear.x / v_norm;
        const double vy = target_vel_.twist.linear.y / v_norm;

        double offset_x = 0.0;
        double offset_y = 0.0;
        if (strategy == 'A') {
            offset_x = vx * offset_A_;
            offset_y = vy * offset_A_;
        } else if (strategy == 'B') {
            const double angle = M_PI / 4.0;
            offset_x = -offset_B_ * (vx * std::cos(angle) - vy * std::sin(angle));
            offset_y = -offset_B_ * (vx * std::sin(angle) + vy * std::cos(angle));
        } else if (strategy == 'C') {
            const double angle = -M_PI / 4.0;
            offset_x = -offset_C_ * (vx * std::cos(angle) - vy * std::sin(angle));
            offset_y = -offset_C_ * (vx * std::sin(angle) + vy * std::cos(angle));
        }

        cmd.pose.position.x = target_local_.x + offset_x;
        cmd.pose.position.y = target_local_.y + offset_y;
        cmd.pose.position.z = target_local_.z;

        if (cmd_init_) {
            cmd.pose.position.x = cmd_smooth_alpha_ * cmd.pose.position.x +
                                  (1.0 - cmd_smooth_alpha_) * cmd_pos_prev_.x;
            cmd.pose.position.y = cmd_smooth_alpha_ * cmd.pose.position.y +
                                  (1.0 - cmd_smooth_alpha_) * cmd_pos_prev_.y;
            cmd.pose.position.z = cmd_smooth_alpha_ * cmd.pose.position.z +
                                  (1.0 - cmd_smooth_alpha_) * cmd_pos_prev_.z;
        } else {
            cmd_init_ = true;
        }

        cmd_pos_prev_ = cmd.pose.position;
        updateOrientationToTarget(cmd);
        return true;
    }

    void updateOrientationToTarget(geometry_msgs::PoseStamped& cmd)
    {
        const double rel_x = target_local_.x - self_local_.x;
        const double rel_y = target_local_.y - self_local_.y;

        if (std::fabs(rel_x) + std::fabs(rel_y) > 1e-3) {
            const double yaw_instant = std::atan2(rel_y, rel_x);
            if (yaw_init_) {
                yaw_last_ = yaw_smooth_alpha_ * yaw_instant +
                            (1.0 - yaw_smooth_alpha_) * yaw_last_;
            } else {
                yaw_last_ = yaw_instant;
                yaw_init_ = true;
            }
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_last_);
        q.normalize();

        cmd.pose.orientation.x = q.x();
        cmd.pose.orientation.y = q.y();
        cmd.pose.orientation.z = q.z();
        cmd.pose.orientation.w = q.w();
    }

    double estimateTimeToReach(double distance) const
    {
        if (self_speed_ <= 1e-3) {
            return 0.0;
        }
        return std::min(distance / self_speed_, 8.0);
    }

    double computePredictedHorizontalDistance(double time_to_reach) const
    {
        const double pred_target_x = target_local_.x + target_vel_.twist.linear.x * time_to_reach;
        const double pred_target_y = target_local_.y + target_vel_.twist.linear.y * time_to_reach;
        const double pred_rel_x = pred_target_x - self_local_.x;
        const double pred_rel_y = pred_target_y - self_local_.y;
        return std::sqrt(pred_rel_x * pred_rel_x + pred_rel_y * pred_rel_y);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "strategy_fsm_node");
    ros::NodeHandle nh("~");
    StrategyFSM fsm(nh);
    ros::spin();
    return 0;
}
