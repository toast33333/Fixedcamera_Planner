#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace {

double clampValue(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

}  // namespace

class CameraPitchServoOpenLoop {
 public:
  CameraPitchServoOpenLoop() : nh_(), pnh_("~") {
    pnh_.param<std::string>("camera_pitch_servo_command_topic", command_topic_,
                            "/camera_pitch_servo/command_deg");
    pnh_.param<std::string>("camera_pitch_servo_pwm_topic", pwm_topic_,
                            "/camera_pitch_servo/pulse_width_us");
    pnh_.param<std::string>("camera_pitch_servo_estimated_topic", estimated_topic_,
                            "/camera_pitch_servo/estimated_deg");
    pnh_.param<std::string>("camera_pitch_servo_open_loop_state_topic", state_topic_,
                            "/camera_pitch_servo/open_loop_state");
    pnh_.param<std::string>("camera_pitch_servo_enable_service", enable_service_name_,
                            "/camera_pitch_servo/set_output_enabled");
    pnh_.param<std::string>("camera_pitch_servo_output_enabled_topic",
                            output_enabled_topic_,
                            "/camera_pitch_servo/output_enabled");
    pnh_.param<std::string>("camera_pitch_servo_command_valid_topic",
                            command_valid_topic_,
                            "/camera_pitch_servo/command_valid");
    pnh_.param("camera_pitch_servo_neutral_deg", neutral_deg_, 0.0);
    // Keep the mapper calibration range separate from the visual controller's
    // normal operating limits. This allows bench calibration over the verified
    // servo range without widening the in-flight controller range.
    pnh_.param("camera_pitch_servo_mapper_min_deg", command_min_deg_, -48.0);
    pnh_.param("camera_pitch_servo_mapper_max_deg", command_max_deg_, 48.0);
    pnh_.param<std::string>("camera_pitch_servo_calibration_mode",
                            calibration_mode_, "measured");
    // Keep the old single-gain parameter as a fallback so an older field YAML
    // retains the measured relationship after this binary is updated.
    double legacy_pwm_us_per_deg = 10.132249;
    pnh_.param("camera_pitch_servo_pwm_us_per_deg",
               legacy_pwm_us_per_deg, 10.132249);
    pnh_.param("camera_pitch_servo_measured_pwm_us_per_deg",
               measured_pwm_us_per_deg_, legacy_pwm_us_per_deg);
    pnh_.param("camera_pitch_servo_datasheet_pwm_us_per_deg",
               datasheet_pwm_us_per_deg_, 500.0 / 45.0);
    selectCalibration();
    pnh_.param("camera_pitch_servo_pwm_min_us", pwm_min_us_, 1000);
    pnh_.param("camera_pitch_servo_pwm_neutral_us", pwm_neutral_us_, 1500);
    pnh_.param("camera_pitch_servo_pwm_max_us", pwm_max_us_, 2000);
    pnh_.param("camera_pitch_servo_pwm_period_us", pwm_period_us_, 20000);
    pnh_.param("camera_pitch_servo_pwm_reverse", pwm_reverse_, false);
    pnh_.param("camera_pitch_servo_driver_rate_hz", update_rate_hz_, 50.0);
    pnh_.param("camera_pitch_servo_driver_max_rate_degps", max_rate_degps_, 60.0);
    pnh_.param("camera_pitch_servo_command_timeout_sec", command_timeout_sec_, 0.5);
    pnh_.param("camera_pitch_servo_return_on_command_timeout",
                return_on_timeout_, true);
    pnh_.param("camera_pitch_servo_output_enabled", output_enabled_, false);

    validateParameters();
    target_deg_ = neutral_deg_;
    estimated_deg_ = neutral_deg_;
    last_update_ = ros::Time::now();

    command_sub_ = nh_.subscribe(command_topic_, 5,
                                 &CameraPitchServoOpenLoop::commandCallback, this);
    // PWM commands are deliberately not latched. A hardware adapter that
    // starts later must never receive an old pulse from this mapper.
    pwm_pub_ = nh_.advertise<std_msgs::UInt16>(pwm_topic_, 1, false);
    estimated_pub_ = nh_.advertise<std_msgs::Float64>(estimated_topic_, 1, true);
    state_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(state_topic_, 1);
    output_enabled_pub_ =
        nh_.advertise<std_msgs::Bool>(output_enabled_topic_, 1, true);
    command_valid_pub_ =
        nh_.advertise<std_msgs::Bool>(command_valid_topic_, 1, true);
    enable_service_ = nh_.advertiseService(
        enable_service_name_, &CameraPitchServoOpenLoop::enableCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_hz_),
                             &CameraPitchServoOpenLoop::update, this);

    publishState(ros::Time::now());
    ROS_INFO("Camera pitch open-loop mapper: command=%s pwm=%s, limits=[%.1f, %.1f] deg, "
             "calibration=%s gain=%.6f us/deg (measured=%.6f datasheet=%.6f) "
             "pulse=[%d, %d, %d] us period=%d us reverse=%s",
             command_topic_.c_str(), pwm_topic_.c_str(), command_min_deg_,
             command_max_deg_, calibration_mode_.c_str(), pwm_us_per_deg_,
             measured_pwm_us_per_deg_, datasheet_pwm_us_per_deg_,
             pwm_min_us_, pwm_neutral_us_, pwm_max_us_,
             pwm_period_us_, pwm_reverse_ ? "true" : "false");
    if (output_enabled_) {
      ROS_WARN("Camera pitch PWM output is armed by configuration, but no pulse "
               "will be published until a fresh angle command is received");
    } else {
      ROS_INFO("Camera pitch PWM output is disabled; arm explicitly through %s",
               enable_service_name_.c_str());
    }
  }

 private:
  void selectCalibration() {
    if (calibration_mode_ == "measured") {
      pwm_us_per_deg_ = measured_pwm_us_per_deg_;
      return;
    }
    if (calibration_mode_ == "datasheet") {
      pwm_us_per_deg_ = datasheet_pwm_us_per_deg_;
      return;
    }
    throw std::runtime_error(
        "camera_pitch_servo_calibration_mode must be 'measured' or 'datasheet'");
  }

  void validateParameters() {
    if (!std::isfinite(neutral_deg_) || !std::isfinite(command_min_deg_) ||
        !std::isfinite(command_max_deg_) || command_min_deg_ >= command_max_deg_ ||
        neutral_deg_ < command_min_deg_ || neutral_deg_ > command_max_deg_) {
      throw std::runtime_error("invalid camera pitch command angle limits");
    }
    if (!std::isfinite(measured_pwm_us_per_deg_) ||
        measured_pwm_us_per_deg_ <= 0.0 ||
        !std::isfinite(datasheet_pwm_us_per_deg_) ||
        datasheet_pwm_us_per_deg_ <= 0.0 ||
        !std::isfinite(pwm_us_per_deg_) || pwm_us_per_deg_ <= 0.0) {
      throw std::runtime_error(
          "camera pitch servo calibration gains must be positive");
    }
    if (pwm_min_us_ <= 0 || pwm_min_us_ >= pwm_neutral_us_ ||
        pwm_neutral_us_ >= pwm_max_us_ || pwm_max_us_ >= pwm_period_us_) {
      throw std::runtime_error("invalid camera pitch PWM range or period");
    }
    if (!std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0 ||
        !std::isfinite(max_rate_degps_) || max_rate_degps_ <= 0.0 ||
        !std::isfinite(command_timeout_sec_) || command_timeout_sec_ < 0.0) {
      throw std::runtime_error("invalid camera pitch mapper timing parameters");
    }
  }

  void commandCallback(const std_msgs::Float64::ConstPtr& message) {
    if (!std::isfinite(message->data)) {
      ROS_WARN_THROTTLE(1.0, "Ignoring non-finite camera pitch command");
      return;
    }
    if (!output_enabled_) {
      ROS_WARN_THROTTLE(2.0,
                        "Ignoring camera pitch command while PWM output is disabled");
      return;
    }
    if (message->data < command_min_deg_ || message->data > command_max_deg_) {
      ROS_WARN_THROTTLE(1.0,
                        "Clamping camera pitch command %.2f deg to [%.2f, %.2f] deg",
                        message->data, command_min_deg_, command_max_deg_);
    }
    target_deg_ = clampValue(message->data, command_min_deg_, command_max_deg_);
    last_command_receipt_ = ros::Time::now();
    have_command_ = true;
    command_valid_ = true;
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response) {
    const ros::Time now = ros::Time::now();
    output_enabled_ = request.data;

    // Every arm/disarm transition invalidates any previously received command.
    // This prevents an old latched/queued angle from moving the mechanism.
    have_command_ = false;
    command_valid_ = false;
    target_deg_ = neutral_deg_;
    estimated_deg_ = neutral_deg_;
    last_update_ = now;
    publishState(now);

    response.success = true;
    if (output_enabled_) {
      response.message =
          "output armed; waiting for a fresh angle command before publishing PWM";
      ROS_WARN("Camera pitch PWM output ARMED; waiting for a fresh command on %s",
               command_topic_.c_str());
    } else {
      response.message = "output disabled; PWM publishing stopped";
      ROS_INFO("Camera pitch PWM output DISABLED");
    }
    return true;
  }

  uint16_t angleToPulse(double command_deg) const {
    double relative_deg = command_deg - neutral_deg_;
    if (pwm_reverse_) relative_deg = -relative_deg;
    const double pulse = clampValue(
        static_cast<double>(pwm_neutral_us_) + pwm_us_per_deg_ * relative_deg,
        static_cast<double>(pwm_min_us_), static_cast<double>(pwm_max_us_));
    return static_cast<uint16_t>(std::lround(pulse));
  }

  void update(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_update_).toSec();
    last_update_ = now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) dt = 1.0 / update_rate_hz_;

    if (have_command_ && command_timeout_sec_ > 0.0 &&
        (now - last_command_receipt_).toSec() > command_timeout_sec_) {
      if (command_valid_) {
        ROS_WARN("Camera pitch command timed out%s",
                 return_on_timeout_
                     ? "; returning to neutral at the configured rate limit"
                     : "; holding the last rate-limited angle");
      }
      command_valid_ = false;
      if (return_on_timeout_) target_deg_ = neutral_deg_;
    }

    const double maximum_step = max_rate_degps_ * dt;
    const double remaining = target_deg_ - estimated_deg_;
    estimated_deg_ += clampValue(remaining, -maximum_step, maximum_step);
    estimated_deg_ = clampValue(estimated_deg_, command_min_deg_, command_max_deg_);
    publishState(now);
  }

  void publishState(const ros::Time& stamp) {
    const uint16_t pulse_us = angleToPulse(estimated_deg_);
    const bool pwm_active = output_enabled_ && have_command_;
    if (pwm_active) {
      std_msgs::UInt16 pulse;
      pulse.data = pulse_us;
      pwm_pub_.publish(pulse);
    }

    std_msgs::Float64 estimated;
    estimated.data = estimated_deg_;
    estimated_pub_.publish(estimated);

    geometry_msgs::Vector3Stamped state;
    state.header.stamp = stamp;
    state.header.frame_id = "camera_pitch_servo_open_loop";
    state.vector.x = target_deg_;
    state.vector.y = estimated_deg_;
    // A zero z value explicitly means that no PWM pulse is being published.
    state.vector.z = pwm_active ? pulse_us : 0.0;
    state_pub_.publish(state);

    std_msgs::Bool output_enabled;
    output_enabled.data = output_enabled_;
    output_enabled_pub_.publish(output_enabled);

    std_msgs::Bool command_valid;
    command_valid.data = command_valid_;
    command_valid_pub_.publish(command_valid);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber command_sub_;
  ros::Publisher pwm_pub_;
  ros::Publisher estimated_pub_;
  ros::Publisher state_pub_;
  ros::Publisher output_enabled_pub_;
  ros::Publisher command_valid_pub_;
  ros::ServiceServer enable_service_;
  ros::Timer timer_;

  std::string command_topic_;
  std::string pwm_topic_;
  std::string estimated_topic_;
  std::string state_topic_;
  std::string enable_service_name_;
  std::string output_enabled_topic_;
  std::string command_valid_topic_;
  double neutral_deg_{0.0};
  double command_min_deg_{-48.0};
  double command_max_deg_{48.0};
  std::string calibration_mode_{"measured"};
  double measured_pwm_us_per_deg_{10.132249};
  double datasheet_pwm_us_per_deg_{500.0 / 45.0};
  double pwm_us_per_deg_{10.132249};
  int pwm_min_us_{1000};
  int pwm_neutral_us_{1500};
  int pwm_max_us_{2000};
  int pwm_period_us_{20000};
  bool pwm_reverse_{false};
  double update_rate_hz_{50.0};
  double max_rate_degps_{60.0};
  double command_timeout_sec_{0.5};
  bool return_on_timeout_{true};
  bool output_enabled_{false};
  bool have_command_{false};
  bool command_valid_{false};
  double target_deg_{0.0};
  double estimated_deg_{0.0};
  ros::Time last_command_receipt_;
  ros::Time last_update_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_pitch_servo_open_loop");
  try {
    CameraPitchServoOpenLoop node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Camera pitch open-loop mapper failed: %s", error.what());
    return 1;
  }
  return 0;
}
