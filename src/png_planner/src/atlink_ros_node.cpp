#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8MultiArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>

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
#include <vector>

class AtlinkRosNode {
 public:
  AtlinkRosNode() : nh_(), pnh_("~"), link_(nullptr), fd_(-1), takeoff_pending_(false),
                    have_state_(false), have_gps_(false), have_extended_state_(false),
                    on_ground_(false), airborne_(false) {
    pnh_.param<std::string>("serial_device", device_, "/dev/ttyUSB0");
    pnh_.param("baud", baud_, 115200);
    pnh_.param("serial_retry_sec", serial_retry_sec_, 1.0);
    pnh_.param("local_address", local_address_, 1);
    pnh_.param("takeoff_altitude", takeoff_altitude_, 5.0);
    pnh_.param("takeoff_latitude", takeoff_latitude_, 0.0);
    pnh_.param("takeoff_longitude", takeoff_longitude_, 0.0);
    pnh_.param<std::string>("serial", serial_, "");
    pnh_.param<std::string>("arming_service", arming_service_, "/mavros/cmd/arming");
    pnh_.param<std::string>("takeoff_service", takeoff_service_, "/mavros/cmd/takeoff");
    pnh_.param<std::string>("mavros_state_topic", state_topic_, "/mavros/state");
    pnh_.param<std::string>("mavros_extended_state_topic", extended_state_topic_, "/mavros/extended_state");
    pnh_.param<std::string>("gps_topic", gps_topic_, "/mavros/global_position/global");
    pnh_.param<std::string>("relative_altitude_topic", relative_altitude_topic_,
                            "/mavros/global_position/rel_alt");
    pnh_.param<std::string>("battery_topic", battery_topic_, "/mavros/battery");
    pnh_.param<std::string>("velocity_topic", velocity_topic_,
                            "/mavros/local_position/velocity_local");
    if (local_address_ < 1 || local_address_ > 0xFFFE)
      throw std::runtime_error("local_address must be in range 1..65534");
    if (!std::isfinite(takeoff_altitude_) || takeoff_altitude_ <= 0.0)
      throw std::runtime_error("takeoff_altitude must be positive and finite");

    std::memset(status_.serial, 0, sizeof(status_.serial));
    std::memcpy(status_.serial, serial_.data(), std::min(serial_.size(), sizeof(status_.serial)));
    status_.lon_e7 = status_.lat_e7 = status_.alt_amsl_mm = status_.alt_rel_mm = ATLINK_POS_UNKNOWN;
    status_.battery_pct = ATLINK_BATT_UNKNOWN;
    status_.flight_phase = ATLINK_PHASE_UNKNOWN;

    atlink_cfg_t cfg{};
    cfg.local_addr = static_cast<uint16_t>(local_address_);
    link_ = atlink_create(&cfg);
    if (!link_) throw std::runtime_error("atlink_create failed");
    atlink_set_sky_tx(link_, &AtlinkRosNode::txCallback, this);
    atlink_set_on_takeoff(link_, &AtlinkRosNode::takeoffCallback, this);
    atlink_set_on_target(link_, &AtlinkRosNode::targetCallback, this);
    atlink_set_on_abort(link_, &AtlinkRosNode::abortCallback, this);
    atlink_set_on_link_lost(link_, &AtlinkRosNode::linkLostCallback, this);
    atlink_set_status(link_, &status_);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(arming_service_);
    takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_service_);
    state_sub_ = nh_.subscribe(state_topic_, 10, &AtlinkRosNode::stateCallback, this);
    extended_state_sub_ = nh_.subscribe(extended_state_topic_, 10, &AtlinkRosNode::extendedStateCallback, this);
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &AtlinkRosNode::gpsCallback, this);
    relative_altitude_sub_ = nh_.subscribe(
        relative_altitude_topic_, 10,
        &AtlinkRosNode::relativeAltitudeCallback, this);
    battery_sub_ = nh_.subscribe(
        battery_topic_, 10, &AtlinkRosNode::batteryCallback, this);
    velocity_sub_ = nh_.subscribe(
        velocity_topic_, 10, &AtlinkRosNode::velocityCallback, this);
    command_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("atlink/command", 10);
    response_sub_ = nh_.subscribe("atlink/response", 10, &AtlinkRosNode::responseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.01), &AtlinkRosNode::poll, this);
    next_serial_retry_ = ros::WallTime(0);
    ensureSerialConnected();
    ROS_INFO("AT-Link v3 configured for %s @ %d, address=%d; MAVROS arming=%s takeoff=%s",
             device_.c_str(), baud_, local_address_, arming_service_.c_str(),
             takeoff_service_.c_str());
  }

  ~AtlinkRosNode() {
    if (link_) atlink_destroy(link_);
    closeSerial();
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

  bool configureSerial() {
    termios options{};
    if (tcgetattr(fd_, &options) != 0) return false;
    cfmakeraw(&options);
    const speed_t speed = baudConstant(baud_);
    cfsetispeed(&options, speed); cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB; options.c_cflag &= ~PARENB; options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    return tcsetattr(fd_, TCSANOW, &options) == 0;
  }

  void closeSerial() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool ensureSerialConnected() {
    if (fd_ >= 0) return true;
    const ros::WallTime now = ros::WallTime::now();
    if (now < next_serial_retry_) return false;
    next_serial_retry_ = now + ros::WallDuration(std::max(0.1, serial_retry_sec_));

    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      ROS_WARN_STREAM_THROTTLE(5.0, "AT-Link waiting for " << device_ << ": "
                                       << std::strerror(errno));
      return false;
    }
    if (!configureSerial()) {
      const int saved_errno = errno;
      closeSerial();
      ROS_ERROR_STREAM_THROTTLE(5.0, "AT-Link cannot configure " << device_ << ": "
                                        << std::strerror(saved_errno));
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    rx_.clear();
    ROS_INFO("AT-Link serial connected: %s @ %d", device_.c_str(), baud_);
    return true;
  }

  static int txCallback(void* user, const uint8_t* frame, uint16_t len) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    if (self->fd_ < 0) return -1;
    size_t sent = 0;
    while (sent < len) {
      const ssize_t n = write(self->fd_, frame + sent, len - sent);
      if (n > 0) sent += static_cast<size_t>(n);
      else if (n < 0 && errno == EINTR) continue;
      else {
        self->closeSerial();
        return -1;
      }
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

  static void targetCallback(const atlink_target_fix_t* fix, void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    std_msgs::UInt8MultiArray msg;
    msg.data = {0x01, 0x02};
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(fix);
    msg.data.insert(msg.data.end(), bytes, bytes + sizeof(*fix));
    self->command_pub_.publish(msg);
  }

  static void abortCallback(int airborne, void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    std_msgs::UInt8MultiArray msg;
    msg.data = {0x01, 0x03, static_cast<uint8_t>(airborne ? 1 : 0)};
    self->command_pub_.publish(msg);
  }

  static void linkLostCallback(void* user) {
    AtlinkRosNode* self = static_cast<AtlinkRosNode*>(user);
    std_msgs::UInt8MultiArray msg;
    msg.data = {0x01, 0x03, static_cast<uint8_t>(self->airborne_ ? 1 : 0)};
    self->command_pub_.publish(msg);
    ROS_ERROR("AT-Link downlink timeout; published abort/return request");
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    have_state_ = true;
    connected_ = msg->connected;
    armed_ = msg->armed;
  }

  void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    have_extended_state_ = msg->landed_state != mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
    airborne_ = (msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR);
    on_ground_ = (msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND);
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
    status_.alt_rel_mm = static_cast<int32_t>(std::llround(msg->data * 1000.0));
  }

  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    if (std::isfinite(msg->voltage) && msg->voltage >= 0.0) {
      const double millivolts = std::min(65535.0, msg->voltage * 1000.0);
      status_.battery_mv = static_cast<uint16_t>(std::llround(millivolts));
    }
    if (std::isfinite(msg->percentage) && msg->percentage >= 0.0) {
      const double percent = std::max(0.0, std::min(100.0, msg->percentage * 100.0));
      status_.battery_pct = static_cast<uint8_t>(std::lround(percent));
    }
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const double speed_mps = std::hypot(msg->twist.linear.x, msg->twist.linear.y);
    if (!std::isfinite(speed_mps)) return;
    status_.ground_speed_cms = static_cast<uint16_t>(
        std::llround(std::min(655.35, std::max(0.0, speed_mps)) * 100.0));
  }

  void responseCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 2) { ROS_WARN("atlink/response expects [result, note_code]"); return; }
    const char* note = msg->data[1] ? "ROS action failed" : "";
    if (atlink_takeoff_fail(link_, msg->data[0], note) != 0)
      ROS_WARN("AT-Link response ignored (no active takeoff request or invalid result)");
  }

  void processTakeoff() {
    if (!takeoff_pending_) return;
    takeoff_pending_ = false;
    if (!have_state_ || !connected_ || !have_gps_ || !have_extended_state_ || !on_ground_) {
      failTakeoff(ATLINK_RESULT_TEMPORARILY_REJECTED, "MAVROS/GPS not ready or aircraft is not confirmed on ground");
      return;
    }
    mavros_msgs::CommandBool arm;
    arm.request.value = true;
    if (!arming_client_.call(arm) || !arm.response.success) {
      failTakeoff(ATLINK_RESULT_FAILED, "MAVROS arming service failed");
      return;
    }
    mavros_msgs::CommandTOL takeoff;
    takeoff.request.min_pitch = 0.0;
    takeoff.request.yaw = 0.0;
    takeoff.request.latitude = takeoff_latitude_ == 0.0 ? status_.lat_e7 / 1e7 : takeoff_latitude_;
    takeoff.request.longitude = takeoff_longitude_ == 0.0 ? status_.lon_e7 / 1e7 : takeoff_longitude_;
    takeoff.request.altitude = takeoff_altitude_;
    if (!takeoff_client_.call(takeoff) || !takeoff.response.success) {
      failTakeoff(ATLINK_RESULT_FAILED, "MAVROS takeoff service failed");
      return;
    }
    ROS_INFO("MAVROS accepted takeoff request id=%u altitude=%.2f m", pending_takeoff_.req_id, takeoff_altitude_);
  }

  void failTakeoff(uint8_t result, const char* note) {
    if (atlink_takeoff_fail(link_, result, note) != 0)
      ROS_WARN("Could not report takeoff failure to AT-Link: %s", note);
    else
      ROS_ERROR("Takeoff request failed: %s", note);
  }

  void updateStatus() {
    status_.gps_valid = have_gps_ ? 1 : 0;
    status_.fault_code = !have_state_ || !connected_ ? ATLINK_FAULT_FC_LOST :
                         (!have_gps_ ? ATLINK_FAULT_GPS_NO_FIX : ATLINK_FAULT_NONE);
    status_.flight_phase = airborne_ ? ATLINK_PHASE_TAKING_OFF :
                           (on_ground_ ? ATLINK_PHASE_STANDBY : ATLINK_PHASE_UNKNOWN);
  }

  void poll(const ros::TimerEvent&) {
    const uint64_t now_ms = static_cast<uint64_t>(
        ros::WallTime::now().toNSec() / 1000000ULL);
    if (!ensureSerialConnected()) {
      updateStatus();
      atlink_set_status(link_, &status_);
      atlink_tick(link_, now_ms);
      return;
    }
    uint8_t bytes[512];
    for (;;) {
      const ssize_t n = read(fd_, bytes, sizeof(bytes));
      if (n > 0) rx_.insert(rx_.end(), bytes, bytes + n);
      else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "AT-Link serial read: " << std::strerror(errno));
        closeSerial();
        break;
      }
      else break;
    }
    if (!rx_.empty()) { atlink_sky_input(link_, rx_.data(), rx_.size(), now_ms); rx_.clear(); }
    updateStatus();
    atlink_set_status(link_, &status_);
    atlink_tick(link_, now_ms);
    processTakeoff();
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_;
  ros::Subscriber response_sub_, state_sub_, extended_state_sub_, gps_sub_;
  ros::Subscriber relative_altitude_sub_, battery_sub_, velocity_sub_;
  ros::ServiceClient arming_client_, takeoff_client_;
  ros::Timer timer_;
  atlink_t* link_;
  atlink_status_t status_{};
  atlink_takeoff_t pending_takeoff_{};
  bool takeoff_pending_, have_state_, connected_{false}, armed_{false}, have_gps_;
  bool have_extended_state_, on_ground_, airborne_;
  int fd_, baud_{115200}, local_address_{1};
  double serial_retry_sec_{1.0};
  double takeoff_altitude_{5.0}, takeoff_latitude_{0.0}, takeoff_longitude_{0.0};
  std::string device_, serial_, arming_service_, takeoff_service_, state_topic_;
  std::string extended_state_topic_, gps_topic_, relative_altitude_topic_;
  std::string battery_topic_, velocity_topic_;
  ros::WallTime next_serial_retry_;
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

