#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/SetBool.h>

#include <dirent.h>
#include <errno.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::string trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) return "";
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

std::string readTextFile(const std::string& path) {
  std::ifstream stream(path.c_str());
  if (!stream) return "";
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

void writeTextFile(const std::string& path, const std::string& value) {
  std::ofstream stream(path.c_str());
  if (!stream) {
    throw std::runtime_error("cannot open " + path + ": " +
                             std::string(strerror(errno)));
  }
  stream << value;
  stream.flush();
  if (!stream) throw std::runtime_error("cannot write " + path);
}

bool pathExists(const std::string& path) {
  std::ifstream stream(path.c_str());
  return stream.good();
}

}  // namespace

class CameraPitchServoSysfsPwm {
 public:
  CameraPitchServoSysfsPwm() : nh_(), pnh_("~") {
    pnh_.param<std::string>("camera_pitch_servo_pwm_topic", pwm_topic_,
                            "/camera_pitch_servo/pulse_width_us");
    pnh_.param<std::string>("camera_pitch_servo_mapper_output_enabled_topic",
                            mapper_enabled_topic_,
                            "/camera_pitch_servo/output_enabled");
    pnh_.param<std::string>("camera_pitch_servo_mapper_command_valid_topic",
                            mapper_command_valid_topic_,
                            "/camera_pitch_servo/command_valid");
    pnh_.param<std::string>("camera_pitch_servo_hardware_enable_service",
                            hardware_enable_service_,
                            "/camera_pitch_servo/set_hardware_output_enabled");
    pnh_.param<std::string>("camera_pitch_servo_hardware_armed_topic",
                            hardware_armed_topic_,
                            "/camera_pitch_servo/hardware_armed");
    pnh_.param<std::string>("camera_pitch_servo_hardware_active_topic",
                            hardware_active_topic_,
                            "/camera_pitch_servo/hardware_active");
    pnh_.param<std::string>("camera_pitch_servo_pwm_sysfs_root", sysfs_root_,
                            "/sys/class/pwm");
    pnh_.param<std::string>("camera_pitch_servo_pwm_of_fullname",
                            expected_of_fullname_, "/pwm@2add0000");
    pnh_.param("camera_pitch_servo_pwm_channel", pwm_channel_, 0);
    pnh_.param("camera_pitch_servo_pwm_period_us", pwm_period_us_, 20000);
    pnh_.param("camera_pitch_servo_pwm_min_us", pwm_min_us_, 500);
    pnh_.param("camera_pitch_servo_pwm_max_us", pwm_max_us_, 2500);
    pnh_.param("camera_pitch_servo_hardware_command_timeout_sec",
               hardware_command_timeout_sec_, 0.3);

    validateParameters();
    pwm_sub_ = nh_.subscribe(pwm_topic_, 1,
                             &CameraPitchServoSysfsPwm::pwmCallback, this);
    mapper_enabled_sub_ =
        nh_.subscribe(mapper_enabled_topic_, 1,
                      &CameraPitchServoSysfsPwm::mapperEnabledCallback, this);
    mapper_command_valid_sub_ = nh_.subscribe(
        mapper_command_valid_topic_, 1,
        &CameraPitchServoSysfsPwm::mapperCommandValidCallback, this);
    hardware_armed_pub_ =
        nh_.advertise<std_msgs::Bool>(hardware_armed_topic_, 1, true);
    hardware_active_pub_ =
        nh_.advertise<std_msgs::Bool>(hardware_active_topic_, 1, true);
    hardware_enable_server_ = nh_.advertiseService(
        hardware_enable_service_, &CameraPitchServoSysfsPwm::enableCallback, this);
    const double watchdog_period_sec =
        std::min(0.1, hardware_command_timeout_sec_ * 0.5);
    watchdog_timer_ = nh_.createWallTimer(
        ros::WallDuration(watchdog_period_sec),
        &CameraPitchServoSysfsPwm::watchdogCallback, this);
    publishState();

    ROS_WARN("Camera pitch sysfs PWM adapter is SAFE/OFF. It will not access "
             "hardware until explicitly armed through %s and a fresh PWM "
             "message arrives while the mapper is enabled",
             hardware_enable_service_.c_str());
    ROS_INFO("Expected RK3576 PWM controller: OF_FULLNAME=%s channel=%d",
             expected_of_fullname_.c_str(), pwm_channel_);
  }

  ~CameraPitchServoSysfsPwm() {
    try {
      stopOutput();
    } catch (const std::exception& error) {
      ROS_ERROR("Failed to stop camera pitch PWM during shutdown: %s",
                error.what());
    }
  }

 private:
  void validateParameters() const {
    if (expected_of_fullname_.empty() || expected_of_fullname_[0] != '/') {
      throw std::runtime_error(
          "camera_pitch_servo_pwm_of_fullname must be an absolute OF path");
    }
    if (pwm_channel_ < 0 || pwm_period_us_ <= 0 || pwm_min_us_ <= 0 ||
        pwm_min_us_ >= pwm_max_us_ || pwm_max_us_ >= pwm_period_us_) {
      throw std::runtime_error("invalid camera pitch sysfs PWM parameters");
    }
    if (!std::isfinite(hardware_command_timeout_sec_) ||
        hardware_command_timeout_sec_ <= 0.0) {
      throw std::runtime_error(
          "camera_pitch_servo_hardware_command_timeout_sec must be positive");
    }
  }

  void publishState() {
    std_msgs::Bool armed;
    armed.data = hardware_armed_;
    hardware_armed_pub_.publish(armed);

    std_msgs::Bool active;
    active.data = hardware_active_;
    hardware_active_pub_.publish(active);
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response) {
    if (!request.data) {
      hardware_armed_ = false;
      fresh_pwm_after_arm_ = false;
      mapper_disarmed_since_hardware_arm_ = false;
      try {
        stopOutput();
        response.success = true;
        response.message = "hardware output disarmed and PWM disabled";
      } catch (const std::exception& error) {
        response.success = false;
        response.message = error.what();
      }
      publishState();
      return true;
    }

    // Re-arming also invalidates the previous hardware state and command.
    try {
      stopOutput();
    } catch (const std::exception& error) {
      hardware_armed_ = false;
      response.success = false;
      response.message = error.what();
      publishState();
      return true;
    }

    // Arming alone intentionally performs no sysfs write. The PWM topic is
    // non-latched, so only a pulse created after both gates are open can move
    // the mechanism.
    hardware_armed_ = true;
    fresh_pwm_after_arm_ = false;
    mapper_disarmed_since_hardware_arm_ =
        have_mapper_enabled_state_ && !mapper_enabled_;
    publishState();
    response.success = true;
    response.message =
        "hardware armed; waiting for a mapper disarm/re-arm cycle and fresh command";
    ROS_WARN("Camera pitch PWM hardware gate ARMED; no hardware write yet");
    return true;
  }

  void mapperEnabledCallback(const std_msgs::Bool::ConstPtr& message) {
    have_mapper_enabled_state_ = true;
    mapper_enabled_ = message->data;
    if (!mapper_enabled_) {
      if (hardware_armed_) mapper_disarmed_since_hardware_arm_ = true;
      fresh_pwm_after_arm_ = false;
      try {
        stopOutput();
      } catch (const std::exception& error) {
        ROS_ERROR("Failed to stop camera pitch PWM after mapper disarm: %s",
                  error.what());
        hardware_armed_ = false;
      }
      publishState();
    }
  }

  void mapperCommandValidCallback(const std_msgs::Bool::ConstPtr& message) {
    mapper_command_valid_ = message->data;
  }

  void pwmCallback(const std_msgs::UInt16::ConstPtr& message) {
    if (!hardware_armed_ || !mapper_enabled_) return;
    if (!hardware_active_ &&
        (!mapper_disarmed_since_hardware_arm_ || !mapper_command_valid_)) {
      ROS_WARN_THROTTLE(
          2.0,
          "Ignoring camera pitch PWM until mapper is disarmed, re-armed, and "
          "given a fresh angle command after hardware arming");
      return;
    }

    const int pulse_us = static_cast<int>(message->data);
    if (pulse_us < pwm_min_us_ || pulse_us > pwm_max_us_) {
      ROS_ERROR("Rejecting out-of-range camera pitch pulse: %d us", pulse_us);
      return;
    }

    fresh_pwm_after_arm_ = true;
    try {
      ensureOutputReady(pulse_us);
      writeTextFile(channel_path_ + "/duty_cycle",
                    std::to_string(pulse_us * 1000));
      last_pwm_receipt_wall_ = ros::WallTime::now();
    } catch (const std::exception& error) {
      ROS_ERROR("Camera pitch PWM hardware write failed: %s", error.what());
      hardware_armed_ = false;
      fresh_pwm_after_arm_ = false;
      try {
        stopOutput();
      } catch (const std::exception& stop_error) {
        ROS_ERROR("Camera pitch PWM cleanup also failed: %s", stop_error.what());
      }
      publishState();
    }
  }

  void watchdogCallback(const ros::WallTimerEvent&) {
    if (!hardware_active_) return;
    const double age_sec =
        (ros::WallTime::now() - last_pwm_receipt_wall_).toSec();
    if (!std::isfinite(age_sec) || age_sec <= hardware_command_timeout_sec_) {
      return;
    }

    ROS_ERROR("Camera pitch PWM watchdog timed out after %.3f s; "
              "disarming and disabling hardware output",
              age_sec);
    hardware_armed_ = false;
    fresh_pwm_after_arm_ = false;
    mapper_disarmed_since_hardware_arm_ = false;
    try {
      stopOutput();
    } catch (const std::exception& error) {
      ROS_ERROR("Camera pitch PWM watchdog cleanup failed: %s", error.what());
    }
    publishState();
  }

  std::string discoverPwmChip() const {
    DIR* directory = opendir(sysfs_root_.c_str());
    if (!directory) {
      throw std::runtime_error("cannot open " + sysfs_root_ + ": " +
                               std::string(strerror(errno)));
    }

    std::string match;
    for (dirent* entry = readdir(directory); entry != NULL;
         entry = readdir(directory)) {
      const std::string name(entry->d_name);
      if (name.compare(0, 7, "pwmchip") != 0) continue;
      const std::string candidate = sysfs_root_ + "/" + name;
      const std::string uevent = readTextFile(candidate + "/device/uevent");
      std::istringstream lines(uevent);
      std::string line;
      while (std::getline(lines, line)) {
        if (trim(line) == "OF_FULLNAME=" + expected_of_fullname_) {
          if (!match.empty()) {
            closedir(directory);
            throw std::runtime_error("multiple pwmchips match " +
                                     expected_of_fullname_);
          }
          match = candidate;
        }
      }
    }
    closedir(directory);

    if (match.empty()) {
      throw std::runtime_error(
          "PWM controller " + expected_of_fullname_ +
          " is not registered; enable pwm1_6ch_0 and disable UART4 in the "
          "RK3576 device tree before hardware testing");
    }
    return match;
  }

  void ensureOutputReady(int initial_pulse_us) {
    if (hardware_active_) return;
    if (!hardware_armed_ || !mapper_enabled_ || !fresh_pwm_after_arm_) {
      throw std::runtime_error("camera pitch PWM gates are not all open");
    }
    if (!mapper_disarmed_since_hardware_arm_ || !mapper_command_valid_) {
      throw std::runtime_error("camera pitch mapper re-arm handshake is incomplete");
    }

    const std::string pwmchip_path = discoverPwmChip();
    const int npwm = std::stoi(trim(readTextFile(pwmchip_path + "/npwm")));
    if (pwm_channel_ >= npwm) {
      throw std::runtime_error("configured PWM channel is outside npwm");
    }

    const std::string channel_path =
        pwmchip_path + "/pwm" + std::to_string(pwm_channel_);
    if (pathExists(channel_path + "/enable")) {
      throw std::runtime_error(
          "PWM channel is already exported; refusing to take ownership");
    }

    // Do not retain the path until export succeeds. If another process races
    // us and owns the channel, cleanup must never disable its PWM output.
    writeTextFile(pwmchip_path + "/export", std::to_string(pwm_channel_));
    pwmchip_path_ = pwmchip_path;
    channel_path_ = channel_path;
    owns_export_ = true;
    for (int attempt = 0; attempt < 50; ++attempt) {
      if (pathExists(channel_path_ + "/enable")) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!pathExists(channel_path_ + "/enable")) {
      throw std::runtime_error("PWM channel did not appear after export");
    }

    writeTextFile(channel_path_ + "/enable", "0");
    writeTextFile(channel_path_ + "/period",
                  std::to_string(pwm_period_us_ * 1000));
    writeTextFile(channel_path_ + "/duty_cycle",
                  std::to_string(initial_pulse_us * 1000));
    writeTextFile(channel_path_ + "/enable", "1");
    hardware_active_ = true;
    publishState();
    ROS_WARN("Camera pitch PWM hardware output ACTIVE on %s/pwm%d",
             pwmchip_path_.c_str(), pwm_channel_);
  }

  void stopOutput() {
    std::string first_error;
    if (!channel_path_.empty() && pathExists(channel_path_ + "/enable")) {
      try {
        writeTextFile(channel_path_ + "/enable", "0");
      } catch (const std::exception& error) {
        first_error = error.what();
      }
    }
    hardware_active_ = false;

    if (owns_export_ && !pwmchip_path_.empty()) {
      try {
        writeTextFile(pwmchip_path_ + "/unexport", std::to_string(pwm_channel_));
      } catch (const std::exception& error) {
        if (first_error.empty()) first_error = error.what();
      }
    }
    owns_export_ = false;
    channel_path_.clear();
    pwmchip_path_.clear();
    publishState();
    if (!first_error.empty()) throw std::runtime_error(first_error);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pwm_sub_;
  ros::Subscriber mapper_enabled_sub_;
  ros::Subscriber mapper_command_valid_sub_;
  ros::Publisher hardware_armed_pub_;
  ros::Publisher hardware_active_pub_;
  ros::ServiceServer hardware_enable_server_;
  ros::WallTimer watchdog_timer_;

  std::string pwm_topic_;
  std::string mapper_enabled_topic_;
  std::string mapper_command_valid_topic_;
  std::string hardware_enable_service_;
  std::string hardware_armed_topic_;
  std::string hardware_active_topic_;
  std::string sysfs_root_;
  std::string expected_of_fullname_;
  std::string pwmchip_path_;
  std::string channel_path_;
  int pwm_channel_{0};
  int pwm_period_us_{20000};
  int pwm_min_us_{500};
  int pwm_max_us_{2500};
  double hardware_command_timeout_sec_{0.3};
  bool mapper_enabled_{false};
  bool have_mapper_enabled_state_{false};
  bool mapper_command_valid_{false};
  bool mapper_disarmed_since_hardware_arm_{false};
  bool hardware_armed_{false};
  bool fresh_pwm_after_arm_{false};
  bool hardware_active_{false};
  bool owns_export_{false};
  ros::WallTime last_pwm_receipt_wall_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_pitch_servo_sysfs_pwm");
  try {
    CameraPitchServoSysfsPwm node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Camera pitch sysfs PWM adapter failed: %s", error.what());
    return 1;
  }
  return 0;
}
