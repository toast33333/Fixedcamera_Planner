#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/UInt8MultiArray.h>
#include <detection_msgs/Detection.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/TimesyncStatus.h>

#include <atlink.h>
#include <atlink_types.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <tf/transform_datatypes.h>
#include <vector>

class AtlinkRosNode {
 public:
  AtlinkRosNode() : nh_(), pnh_("~"), link_(nullptr), fd_(-1), takeoff_pending_(false),
                    have_state_(false), have_gps_(false) {
    pnh_.param<std::string>("serial_device", device_, "/dev/ttyS6");
    pnh_.param("baud", baud_, 57600);
    pnh_.param("local_address", local_address_, 1);
    pnh_.param("takeoff_altitude", takeoff_altitude_, 5.0);
    pnh_.param("takeoff_altitude_tolerance", takeoff_altitude_tolerance_, 0.3);
    pnh_.param("takeoff_latitude", takeoff_latitude_, 0.0);
    pnh_.param("takeoff_longitude", takeoff_longitude_, 0.0);
    pnh_.param<std::string>("serial", serial_, "");
    pnh_.param<std::string>("arming_service", arming_service_, "/mavros/cmd/arming");
    pnh_.param<std::string>("set_mode_service", set_mode_service_, "/mavros/set_mode");
    pnh_.param<std::string>("takeoff_mode", takeoff_mode_, "GUIDED");
    pnh_.param("hover_seconds", hover_seconds_, 2.0);
    pnh_.param<std::string>("target_setpoint_topic", target_setpoint_topic_, "/mavros/setpoint_raw/global");
    pnh_.param<std::string>("takeoff_service", takeoff_service_, "/mavros/cmd/takeoff");
    pnh_.param<std::string>("mavros_state_topic", state_topic_, "/mavros/state");
    pnh_.param<std::string>("gps_topic", gps_topic_, "/mavros/global_position/global");
    pnh_.param<std::string>("relative_altitude_topic", relative_altitude_topic_, "/mavros/global_position/rel_alt");
    pnh_.param<std::string>("velocity_topic", velocity_topic_, "/mavros/local_position/velocity_local");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/mavros/local_position/pose");
    pnh_.param<std::string>("battery_topic", battery_topic_, "/mavros/battery");
    pnh_.param<std::string>("detection_topic", detection_topic_, "/object_detections");
    pnh_.param<std::string>("flight_phase_ext_topic", flight_phase_ext_topic_, "/atlink/flight_phase_ext");
    pnh_.param<std::string>("timesync_status_topic", timesync_status_topic_, "/mavros/timesync_status");
    pnh_.param("mavros_state_stale_s", mavros_state_stale_s_, 5.0);
    pnh_.param("timesync_rtt_warn_ms", timesync_rtt_warn_ms_, 50.0);
    pnh_.param("timesync_warn_hold_s", timesync_warn_hold_s_, 3.0);
    pnh_.param("vision_stale_s", vision_stale_s_, 5.0);
    pnh_.param("health_startup_grace_s", health_startup_grace_s_, 20.0);
    pnh_.param("monitor_vision_health", monitor_vision_health_, true);
    if (local_address_ < 1 || local_address_ > 0xFFFE)
      throw std::runtime_error("local_address must be in range 1..65534");
    if (!std::isfinite(takeoff_altitude_) || takeoff_altitude_ <= 0.0)
      throw std::runtime_error("takeoff_altitude must be positive and finite");
    if (!std::isfinite(takeoff_altitude_tolerance_) || takeoff_altitude_tolerance_ <= 0.0)
      throw std::runtime_error("takeoff_altitude_tolerance must be positive and finite");
    if (!std::isfinite(hover_seconds_) || hover_seconds_ < 0.0)
      throw std::runtime_error("hover_seconds must be non-negative and finite");
    if (!std::isfinite(mavros_state_stale_s_) || mavros_state_stale_s_ <= 0.0 ||
        !std::isfinite(timesync_rtt_warn_ms_) || timesync_rtt_warn_ms_ <= 0.0 ||
        !std::isfinite(timesync_warn_hold_s_) || timesync_warn_hold_s_ < 0.0 ||
        !std::isfinite(vision_stale_s_) || vision_stale_s_ <= 0.0 ||
        !std::isfinite(health_startup_grace_s_) || health_startup_grace_s_ < 0.0)
      throw std::runtime_error("health-monitor thresholds must be finite and non-negative");

    start_wall_time_ = ros::WallTime::now();

    std::memset(&status_, 0, sizeof(status_));
    const size_t serial_len = std::min(serial_.size(), sizeof(status_.serial));
    if (serial_len) std::memcpy(status_.serial, serial_.data(), serial_len);
    if (serial_.size() > sizeof(status_.serial))
      ROS_WARN("AT-Link serial truncated to %zu bytes", sizeof(status_.serial));
    status_.lon_e7 = status_.lat_e7 = status_.alt_amsl_mm = status_.alt_rel_mm = ATLINK_POS_UNKNOWN;
    status_.battery_pct = ATLINK_BATT_UNKNOWN;
    status_.flight_phase = ATLINK_PHASE_UNKNOWN;
    status_.flight_direction_cdeg = ATLINK_DIRECTION_UNKNOWN;
    vision_.flight_phase_ext = ATLINK_PHASE_EXT_UNKNOWN;
    vision_.x1 = vision_.y1 = vision_.x2 = vision_.y2 = ATLINK_BBOX_NONE;
    vision_.track_id = ATLINK_TRACK_INVALID;
    vision_.class_id = ATLINK_CLASS_UNKNOWN;

    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("Cannot open " + device_ + ": " + std::strerror(errno));
    configureSerial();

    atlink_cfg_t cfg{};
    cfg.local_addr = static_cast<uint16_t>(local_address_);
    link_ = atlink_create(&cfg);
    if (!link_) throw std::runtime_error("atlink_create failed");
    atlink_set_sky_tx(link_, &AtlinkRosNode::txCallback, this);
    atlink_set_on_takeoff(link_, &AtlinkRosNode::takeoffCallback, this);
    atlink_set_on_target(link_, &AtlinkRosNode::targetCallback, this);
    atlink_set_on_abort(link_, &AtlinkRosNode::abortCallback, this);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service_);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service_);
    takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_service_);
    target_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>(target_setpoint_topic_, 10);
    state_sub_ = nh_.subscribe(state_topic_, 10, &AtlinkRosNode::stateCallback, this);
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &AtlinkRosNode::gpsCallback, this);
    relative_alt_sub_ = nh_.subscribe(relative_altitude_topic_, 10, &AtlinkRosNode::relativeAltitudeCallback, this);
    velocity_sub_ = nh_.subscribe(velocity_topic_, 10, &AtlinkRosNode::velocityCallback, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 10, &AtlinkRosNode::poseCallback, this);
    battery_sub_ = nh_.subscribe(battery_topic_, 10, &AtlinkRosNode::batteryCallback, this);
    detection_sub_ = nh_.subscribe(detection_topic_, 10, &AtlinkRosNode::detectionCallback, this);
    flight_phase_ext_sub_ = nh_.subscribe(flight_phase_ext_topic_, 10, &AtlinkRosNode::flightPhaseExtCallback, this);
    timesync_status_sub_ = nh_.subscribe(timesync_status_topic_, 10, &AtlinkRosNode::timesyncStatusCallback, this);
    command_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("atlink/command", 10);
    response_sub_ = nh_.subscribe("atlink/response", 10, &AtlinkRosNode::responseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.01), &AtlinkRosNode::poll, this);
    ROS_INFO("AT-Link v3.2 on %s @ %d, address=%d; detection=%s phase=%s",
             device_.c_str(), baud_, local_address_, detection_topic_.c_str(), flight_phase_ext_topic_.c_str());
  }

  ~AtlinkRosNode() {
    if (link_) atlink_destroy(link_);
    if (fd_ >= 0) close(fd_);
  }

 private:
  static speed_t baudConstant(int baud) {
    switch (baud) {
      case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
      case 57600: return B57600; case 115200: return B115200;
#ifdef B230400
      case 230400: return B230400;
#endif
      default: throw std::runtime_error("Unsupported serial baud rate");
    }
  }

  void configureSerial() {
    termios options{};
    if (tcgetattr(fd_, &options) != 0) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&options);
    const speed_t speed = baudConstant(baud_);
    cfsetispeed(&options, speed); cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB; options.c_cflag &= ~PARENB; options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    if (tcsetattr(fd_, TCSANOW, &options) != 0) throw std::runtime_error("tcsetattr failed");
  }

  static int txCallback(void* user, const uint8_t* frame, uint16_t len) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    size_t sent = 0;
    while (sent < len) {
      const ssize_t n = write(self->fd_, frame + sent, len - sent);
      if (n > 0) sent += static_cast<size_t>(n);
      else if (n < 0 && errno == EINTR) continue;
      else return -1;
    }
    return 0;
  }

  static void takeoffCallback(const atlink_takeoff_t* cmd, void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    if (self->takeoff_pending_) {
      ROS_WARN("Ignoring overlapping AT-Link takeoff callback; an earlier request is still pending");
      return;
    }
    self->pending_takeoff_ = *cmd;
    self->takeoff_pending_ = true;
  }

  static void abortCallback(int airborne, void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    self->abort_pending_ = airborne != 0;
  }

  static void targetCallback(const atlink_target_fix_t* fix, void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    if (!fix) return;
    self->target_lon_e7_ = fix->lon_e7;
    self->target_lat_e7_ = fix->lat_e7;
    self->target_alt_mm_ = fix->alt_amsl_mm;
    self->target_id_ = fix->target_id;
    self->have_target_ = true;
    ROS_INFO_THROTTLE(1.0, "AT-Link target update id=%u lon=%.7f lat=%.7f alt=%.2f m",
                      self->target_id_, self->target_lon_e7_ / 1e7,
                      self->target_lat_e7_ / 1e7, self->target_alt_mm_ / 1000.0);
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    have_state_ = true;
    last_state_wall_time_ = ros::WallTime::now();
    connected_ = msg->connected;
    armed_ = msg->armed;
    status_.flight_phase = msg->armed ? ATLINK_PHASE_TAKING_OFF : ATLINK_PHASE_STANDBY;
  }

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    have_gps_ = msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
                std::isfinite(msg->latitude) && std::isfinite(msg->longitude) && std::isfinite(msg->altitude);
    if (have_gps_) {
      status_.lat_e7 = static_cast<int32_t>(std::llround(msg->latitude * 1e7));
      status_.lon_e7 = static_cast<int32_t>(std::llround(msg->longitude * 1e7));
      status_.alt_amsl_mm = static_cast<int32_t>(std::llround(msg->altitude * 1000.0));
    }
  }

  void relativeAltitudeCallback(const std_msgs::Float64::ConstPtr& msg) {
    if (!std::isfinite(msg->data)) return;
    relative_altitude_m_ = msg->data;
    have_relative_altitude_ = true;
    status_.alt_rel_mm = static_cast<int32_t>(std::llround(msg->data * 1000.0));
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const double east = msg->twist.linear.x, north = msg->twist.linear.y;
    if (!std::isfinite(east) || !std::isfinite(north)) {
      status_.flight_direction_cdeg = ATLINK_DIRECTION_UNKNOWN;
      return;
    }
    const double speed = std::hypot(east, north);
    status_.ground_speed_cms = static_cast<uint16_t>(std::min(65535L, std::lround(speed * 100.0)));
    if (!have_gps_ || speed < 0.5) {
      status_.flight_direction_cdeg = ATLINK_DIRECTION_UNKNOWN;
      return;
    }
    double direction_deg = std::atan2(east, north) * 180.0 / M_PI;
    if (direction_deg < 0.0) direction_deg += 360.0;
    status_.flight_direction_cdeg =
        static_cast<uint16_t>(std::lround(direction_deg * 100.0) % 36000);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // AT-Link v3.2 requires true-north ground-track direction, not vehicle yaw.
    // Keep the subscription for launch compatibility, but derive direction only
    // from the ENU velocity callback above.
    (void)msg;
  }

  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    if (std::isfinite(msg->percentage)) status_.battery_pct = static_cast<uint8_t>(std::max(0L, std::min(100L, std::lround(msg->percentage * 100.0))));
    if (std::isfinite(msg->voltage)) status_.battery_mv = static_cast<uint16_t>(std::max(0L, std::min(65535L, std::lround(msg->voltage * 1000.0))));
  }

  void detectionCallback(const detection_msgs::Detection::ConstPtr& msg) {
    if (!msg) return;
    have_detection_message_ = true;
    last_detection_wall_time_ = ros::WallTime::now();
    const bool valid = msg->is_tracking && std::isfinite(msg->x1) && std::isfinite(msg->y1) &&
                       std::isfinite(msg->x2) && std::isfinite(msg->y2) &&
                       std::isfinite(msg->score) && msg->x1 >= 0.0 && msg->y1 >= 0.0 &&
                       msg->x2 >= msg->x1 && msg->y2 >= msg->y1 &&
                       msg->x2 < static_cast<double>(ATLINK_BBOX_NONE) &&
                       msg->y2 < static_cast<double>(ATLINK_BBOX_NONE);
    vision_.detection_valid = valid ? 1 : 0;
    vision_.tracking = valid ? 1 : 0;
    vision_.track_id = valid ? msg->track_id : ATLINK_TRACK_INVALID;
    vision_.class_id = ATLINK_CLASS_UNKNOWN;
    vision_.score_u16 = valid ? static_cast<uint16_t>(std::lround(
        std::max(0.0, std::min(1.0, static_cast<double>(msg->score))) * ATLINK_SCORE_MAX)) : 0;
    if (valid) {
      vision_.x1 = static_cast<uint32_t>(std::llround(msg->x1));
      vision_.y1 = static_cast<uint32_t>(std::llround(msg->y1));
      vision_.x2 = static_cast<uint32_t>(std::llround(msg->x2));
      vision_.y2 = static_cast<uint32_t>(std::llround(msg->y2));
    } else {
      vision_.x1 = vision_.y1 = vision_.x2 = vision_.y2 = ATLINK_BBOX_NONE;
    }
    const size_t name_len = valid ? std::min(msg->class_name.size(),
                                             static_cast<size_t>(ATLINK_CLASS_NAME_MAX)) : 0;
    vision_.name_len = static_cast<uint8_t>(name_len);
    std::memset(vision_.name, 0, sizeof(vision_.name));
    if (name_len) std::memcpy(vision_.name, msg->class_name.data(), name_len);
  }

  void timesyncStatusCallback(const mavros_msgs::TimesyncStatus::ConstPtr& msg) {
    if (!msg || !std::isfinite(msg->round_trip_time_ms)) return;
    const ros::WallTime now = ros::WallTime::now();
    have_timesync_status_ = true;
    last_timesync_wall_time_ = now;
    last_timesync_rtt_ms_ = msg->round_trip_time_ms;
    if (last_timesync_rtt_ms_ > timesync_rtt_warn_ms_) {
      if (!timesync_over_limit_) {
        timesync_over_limit_ = true;
        timesync_over_limit_since_ = now;
      }
    } else {
      timesync_over_limit_ = false;
    }
  }

  void flightPhaseExtCallback(const std_msgs::UInt8::ConstPtr& msg) {
    if (!msg) return;
    const uint8_t phase = msg->data;
    if (phase <= ATLINK_PHASE_EXT_LANDED || phase == ATLINK_PHASE_EXT_UNKNOWN) {
      vision_.flight_phase_ext = phase;
      have_external_phase_ext_ = true;
    } else {
      ROS_WARN_THROTTLE(1.0, "Ignoring invalid AT-Link v3.2 flight phase %u", phase);
    }
  }

  void responseCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 2) { ROS_WARN("atlink/response expects [result, note_code]"); return; }
    const char* note = msg->data[1] ? "ROS action failed" : "";
    if (!takeoff_pending_) return;
    completeTakeoff(msg->data[0], note);
    takeoff_pending_ = false;
  }

  void processTakeoff() {
    if (!takeoff_pending_) return;
    takeoff_pending_ = false;
    if (!have_state_ || !connected_ || !have_gps_ || armed_) {
      completeTakeoff(ATLINK_RESULT_TEMPORARILY_REJECTED, "MAVROS/GPS not ready or aircraft is armed");
      return;
    }
    mavros_msgs::SetMode mode;
    mode.request.custom_mode = takeoff_mode_;
    if (!set_mode_client_.call(mode) || !mode.response.mode_sent) {
      completeTakeoff(ATLINK_RESULT_FAILED, "MAVROS set_mode failed");
      return;
    }
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    if (!arming_client_.call(arm) || !arm.response.success) {
      completeTakeoff(ATLINK_RESULT_FAILED, "MAVROS arming service failed");
      return;
    }
    mavros_msgs::CommandTOL takeoff;
    takeoff.request.min_pitch = 0.0;
    takeoff.request.yaw = 0.0;
    takeoff.request.latitude = takeoff_latitude_ == 0.0 ? status_.lat_e7 / 1e7 : takeoff_latitude_;
    takeoff.request.longitude = takeoff_longitude_ == 0.0 ? status_.lon_e7 / 1e7 : takeoff_longitude_;
    takeoff.request.altitude = takeoff_altitude_;
    if (!takeoff_client_.call(takeoff) || !takeoff.response.success) {
      completeTakeoff(ATLINK_RESULT_FAILED, "MAVROS takeoff service failed");
      return;
    }
    status_.flight_phase = ATLINK_PHASE_TAKING_OFF;
    target_lon_e7_ = pending_takeoff_.target_lon_e7;
    target_lat_e7_ = pending_takeoff_.target_lat_e7;
    target_alt_mm_ = pending_takeoff_.target_alt_mm;
    target_id_ = pending_takeoff_.target_id;
    have_target_ = true;
    waiting_for_takeoff_altitude_ = true;
    hovering_at_takeoff_altitude_ = false;
    target_flight_active_ = false;
    ROS_INFO("MAVROS accepted takeoff request id=%u altitude=%.2f m", pending_takeoff_.req_id, takeoff_altitude_);
  }

  void processAbort() {
    if (!abort_pending_) return;
    abort_pending_ = false;
    target_flight_active_ = false;
    waiting_for_takeoff_altitude_ = false;
    hovering_at_takeoff_altitude_ = false;
    have_target_ = false;
    if (!connected_) return;
    if (armed_) {
      mavros_msgs::CommandTOL land;
      land.request.min_pitch = 0.0;
      land.request.yaw = 0.0;
      land.request.latitude = status_.lat_e7 / 1e7;
      land.request.longitude = status_.lon_e7 / 1e7;
      land.request.altitude = 0.0;
      if (!nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land").call(land) || !land.response.success)
        ROS_ERROR("MAVROS land service failed after AT-Link abort");
    } else {
      mavros_msgs::CommandBool disarm;
      disarm.request.value = false;
      if (!arming_client_.call(disarm) || !disarm.response.success)
        ROS_ERROR("MAVROS disarm service failed after AT-Link abort");
    }
  }

  void publishTargetSetpoint() {
    if (!target_flight_active_ || !have_target_) return;
    mavros_msgs::GlobalPositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    sp.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                   mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                   mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                   mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                   mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                   mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                   mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                   mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    sp.latitude = target_lat_e7_ / 1e7;
    sp.longitude = target_lon_e7_ / 1e7;
    sp.altitude = target_alt_mm_ / 1000.0;
    target_pub_.publish(sp);
    status_.flight_phase = ATLINK_PHASE_INTERCEPTING;
  }

  void completeTakeoff(uint8_t result, const char* note) {
    if (atlink_takeoff_fail(link_, result, note) != 0)
      ROS_WARN("Could not report takeoff result: %s", note);
  }

  void poll(const ros::TimerEvent&) {
    uint8_t bytes[512];
    for (;;) {
      const ssize_t n = read(fd_, bytes, sizeof(bytes));
      if (n > 0) rx_.insert(rx_.end(), bytes, bytes + n);
      else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        ROS_ERROR_STREAM_THROTTLE(1.0, "AT-Link serial read: " << std::strerror(errno));
      else break;
    }
    const uint64_t now_ms = static_cast<uint64_t>(ros::WallTime::now().toNSec() / 1000000ULL);
    if (!rx_.empty()) { atlink_sky_input(link_, rx_.data(), rx_.size(), now_ms); rx_.clear(); }
    status_.gps_valid = have_gps_ ? 1 : 0;
    const ros::WallTime wall_now = ros::WallTime::now();
    const bool state_stale = have_state_ &&
        (wall_now - last_state_wall_time_).toSec() > mavros_state_stale_s_;
    const bool timesync_stale = have_timesync_status_ &&
        (wall_now - last_timesync_wall_time_).toSec() > mavros_state_stale_s_;
    const bool timesync_high = timesync_over_limit_ && !timesync_stale &&
        (wall_now - timesync_over_limit_since_).toSec() >= timesync_warn_hold_s_;
    const bool beyond_startup_grace =
        (wall_now - start_wall_time_).toSec() >= health_startup_grace_s_;
    const bool vision_stale = monitor_vision_health_ && beyond_startup_grace &&
        (!have_detection_message_ ||
         (wall_now - last_detection_wall_time_).toSec() > vision_stale_s_);

    if (!have_state_ || !connected_)
      status_.fault_code = ATLINK_FAULT_FC_LOST;
    else if (state_stale)
      status_.fault_code = ATLINK_FAULT_MAVROS_STATE_STALE;
    else if (timesync_high)
      status_.fault_code = ATLINK_FAULT_MAVROS_TIMESYNC_HIGH;
    else if (vision_stale)
      status_.fault_code = ATLINK_FAULT_VISION_STALE;
    else if (!have_gps_)
      status_.fault_code = ATLINK_FAULT_GPS_NO_FIX;
    else
      status_.fault_code = ATLINK_FAULT_NONE;

    if (status_.fault_code != last_logged_fault_code_) {
      if (status_.fault_code == ATLINK_FAULT_NONE)
        ROS_INFO("AT-Link health alarm cleared");
      else
        ROS_WARN("AT-Link health fault=%u (state_connected=%d state_stale=%d timesync_rtt=%.1fms vision_stale=%d gps=%d)",
                 status_.fault_code, connected_ ? 1 : 0, state_stale ? 1 : 0,
                 last_timesync_rtt_ms_, vision_stale ? 1 : 0, have_gps_ ? 1 : 0);
      last_logged_fault_code_ = status_.fault_code;
    }
    if (waiting_for_takeoff_altitude_) {
      if (have_relative_altitude_ &&
          relative_altitude_m_ >= takeoff_altitude_ - takeoff_altitude_tolerance_) {
        waiting_for_takeoff_altitude_ = false;
        hovering_at_takeoff_altitude_ = true;
        hover_until_ = ros::Time::now() + ros::Duration(hover_seconds_);
        ROS_INFO("Takeoff altitude reached: %.2f m (target %.2f m); hovering for %.2f s",
                 relative_altitude_m_, takeoff_altitude_, hover_seconds_);
      }
    }
    if (hovering_at_takeoff_altitude_ && ros::Time::now() >= hover_until_) {
      hovering_at_takeoff_altitude_ = false;
      target_flight_active_ = true;
      ROS_INFO("Hover complete; starting flight to target id=%u", target_id_);
    }
    if (!target_flight_active_)
      status_.flight_phase = armed_ ? ATLINK_PHASE_TAKING_OFF : ATLINK_PHASE_STANDBY;
    vision_.target_valid = have_target_ ? 1 : 0;
    if (!have_external_phase_ext_) {
      vision_.flight_phase_ext = target_flight_active_ ? ATLINK_PHASE_EXT_INTERCEPTING :
                                  (armed_ ? ATLINK_PHASE_EXT_TAKING_OFF : ATLINK_PHASE_EXT_STANDBY);
    }
    atlink_set_status(link_, &status_);
    atlink_set_vision(link_, &vision_);
    atlink_tick(link_, now_ms);
    processTakeoff();
    publishTargetSetpoint();
    processAbort();
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, target_pub_;
  ros::Subscriber response_sub_, state_sub_, gps_sub_, relative_alt_sub_, velocity_sub_, pose_sub_, battery_sub_;
  ros::Subscriber detection_sub_, flight_phase_ext_sub_, timesync_status_sub_;
  ros::ServiceClient set_mode_client_, arming_client_, takeoff_client_;
  ros::Timer timer_;
  atlink_t* link_;
  atlink_takeoff_t pending_takeoff_{};
  atlink_status_t status_{};
  atlink_vision_t vision_{};
  bool takeoff_pending_, abort_pending_{false}, have_state_, connected_{false}, armed_{false}, have_gps_;
  int fd_, baud_{57600}, local_address_{1};
  double takeoff_altitude_{5.0}, takeoff_latitude_{0.0}, takeoff_longitude_{0.0};
  double hover_seconds_{2.0}, takeoff_altitude_tolerance_{0.3};
  double relative_altitude_m_{0.0};
  int32_t target_lon_e7_{ATLINK_POS_UNKNOWN}, target_lat_e7_{ATLINK_POS_UNKNOWN}, target_alt_mm_{ATLINK_POS_UNKNOWN};
  uint16_t target_id_{0};
  ros::Time hover_until_;
  bool target_flight_active_{false}, have_target_{false};
  bool have_relative_altitude_{false}, waiting_for_takeoff_altitude_{false};
  bool hovering_at_takeoff_altitude_{false};
  bool have_external_phase_ext_{false};
  bool monitor_vision_health_{true}, have_detection_message_{false};
  bool have_timesync_status_{false}, timesync_over_limit_{false};
  double mavros_state_stale_s_{5.0}, timesync_rtt_warn_ms_{50.0};
  double timesync_warn_hold_s_{3.0}, vision_stale_s_{5.0}, health_startup_grace_s_{20.0};
  double last_timesync_rtt_ms_{0.0};
  uint16_t last_logged_fault_code_{std::numeric_limits<uint16_t>::max()};
  ros::WallTime start_wall_time_, last_state_wall_time_, last_detection_wall_time_;
  ros::WallTime last_timesync_wall_time_, timesync_over_limit_since_;
  std::string device_, serial_, arming_service_, set_mode_service_, takeoff_mode_, takeoff_service_, target_setpoint_topic_, state_topic_, gps_topic_, relative_altitude_topic_, velocity_topic_, pose_topic_, battery_topic_, detection_topic_, flight_phase_ext_topic_, timesync_status_topic_;
  std::vector<uint8_t> rx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "atlink_ros_node");
  try {
    AtlinkRosNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("AT-Link node initialization failed: %s", e.what());
    return 1;
  }
  return 0;
}
