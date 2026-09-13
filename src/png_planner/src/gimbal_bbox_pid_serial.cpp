#include <ros/ros.h>
#include <kcf_msgs/Bbox.h>
#include <detection_msgs/Detection.h>
#include <geometry_msgs/Twist.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kTxPacketSize = 40;
constexpr size_t kRxPacketSize = 26;

double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
}

double radToDeg(double rad)
{
    return rad * 180.0 / kPi;
}

uint16_t calculateCrc16(const uint8_t* ptr, uint8_t len)
{
    uint16_t crc = 0;
    const uint16_t crc_table[16] = {
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
    };

    while (len--) {
        uint8_t da = static_cast<uint8_t>(crc >> 12);
        crc = static_cast<uint16_t>((crc << 4) ^ crc_table[da ^ ((*ptr) >> 4)]);
        da = static_cast<uint8_t>(crc >> 12);
        crc = static_cast<uint16_t>((crc << 4) ^ crc_table[da ^ ((*ptr) & 0x0F)]);
        ++ptr;
    }
    return crc;
}

void putInt16LE(std::vector<uint8_t>& buf, size_t idx, int16_t value)
{
    buf[idx] = static_cast<uint8_t>(value & 0xFF);
    buf[idx + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

int16_t readInt16LE(const uint8_t* data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8));
}

bool configureSerial(int fd, int baud)
{
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        ROS_ERROR_STREAM("tcgetattr failed: " << strerror(errno));
        return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    if (baud != 115200) {
        ROS_WARN_STREAM("Unsupported baud " << baud << ", fallback to 115200");
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ROS_ERROR_STREAM("tcsetattr failed: " << strerror(errno));
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

int openSerial(const std::string& device, int baud)
{
    const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        ROS_ERROR_STREAM("open " << device << " failed: " << strerror(errno));
        return -1;
    }

    if (!configureSerial(fd, baud)) {
        close(fd);
        return -1;
    }

    return fd;
}

bool writeAll(int fd, const uint8_t* data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        const ssize_t n = write(fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(1000);
                continue;
            }
            ROS_ERROR_STREAM("serial write failed: " << strerror(errno));
            return false;
        }
        sent += static_cast<size_t>(n);
    }

    tcdrain(fd);
    return true;
}

}  // namespace

class GimbalBboxPidSerial {
public:
    GimbalBboxPidSerial(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("serial_device", serial_device_, std::string("/dev/ttyS7"));
        pnh.param<int>("serial_baud", serial_baud_, 115200);
        pnh.param<double>("rate_hz", rate_hz_, 50.0);
        pnh.param<int>("serial_timeout_ms", serial_timeout_ms_, 2);

        pnh.param<double>("fx", fx_, 640.0);
        pnh.param<double>("fy", fy_, 640.0);
        pnh.param<double>("cx", cx_, 320.0);
        pnh.param<double>("cy", cy_, 240.0);
        // true: bbox coordinates are already image-center-relative offsets.
        // false: bbox coordinates are absolute image pixels and cx/cy will be subtracted.
        pnh.param<bool>("bbox_center_is_offset", bbox_center_is_offset_, true);

        pnh.param<double>("yaw_kp", yaw_pid_.kp, 0.022);
        pnh.param<double>("yaw_ki", yaw_pid_.ki, 0.0);
        pnh.param<double>("yaw_kd", yaw_pid_.kd, 0.005);
        pnh.param<double>("pitch_kp", pitch_pid_.kp, 0.022);
        pnh.param<double>("pitch_ki", pitch_pid_.ki, 0.0);
        pnh.param<double>("pitch_kd", pitch_pid_.kd, 0.005);
        pnh.param<double>("integral_limit_deg", integral_limit_deg_, 20.0);

        pnh.param<double>("return_roll_kp", return_roll_pid_.kp, 0.08);
        pnh.param<double>("return_roll_ki", return_roll_pid_.ki, 0.0);
        pnh.param<double>("return_roll_kd", return_roll_pid_.kd, 0.02);
        pnh.param<double>("return_pitch_kp", return_pitch_pid_.kp, 0.08);
        pnh.param<double>("return_pitch_ki", return_pitch_pid_.ki, 0.0);
        pnh.param<double>("return_pitch_kd", return_pitch_pid_.kd, 0.02);
        pnh.param<double>("return_yaw_kp", return_yaw_pid_.kp, 0.08);
        pnh.param<double>("return_yaw_ki", return_yaw_pid_.ki, 0.0);
        pnh.param<double>("return_yaw_kd", return_yaw_pid_.kd, 0.02);

        pnh.param<bool>("return_on_target_loss", return_on_target_loss_, true);
        pnh.param<double>("return_delta_limit_deg", return_delta_limit_deg_, 2.0);
        pnh.param<double>("roll_cmd_limit_deg", roll_cmd_limit_deg_, 45.0);
        pnh.param<double>("yaw_cmd_limit_deg", yaw_cmd_limit_deg_, 145.0);
        pnh.param<double>("pitch_cmd_min_deg", pitch_cmd_min_deg_, -45.0);
        pnh.param<double>("pitch_cmd_max_deg", pitch_cmd_max_deg_, 120.0);
        pnh.param<double>("yaw_delta_limit_deg", yaw_delta_limit_deg_, 0.5);
        pnh.param<double>("pitch_delta_limit_deg", pitch_delta_limit_deg_, 0.5);
        pnh.param<double>("target_timeout_sec", target_timeout_sec_, 0.3);

        pnh.param<double>("yaw_sign", yaw_sign_, -1.0);
        pnh.param<double>("pitch_sign", pitch_sign_, 1.0);
        pnh.param<int>("angle_work_mode", angle_work_mode_, 1);
        pnh.param<std::string>("gimbal_angle_topic", gimbal_angle_topic_,
                               std::string("/gimbal_angles"));

        int detection_mode = 0;
        std::string detection_topic;
        pnh.param<int>("detection_mode", detection_mode, 0);
        pnh.param<std::string>("detection_topic", detection_topic, std::string("/object_kcf"));
        if (detection_mode == 1 && detection_topic == "/object_kcf") {
            detection_topic = "/object_detections";
        }

        if (detection_mode == 1) {
            sub_detection_ = nh.subscribe<detection_msgs::Detection>(
                detection_topic, 1, &GimbalBboxPidSerial::detectionCb, this);
            ROS_INFO("[GimbalBboxPID] Subscribed to %s (detection_msgs/Detection)",
                     detection_topic.c_str());
        } else {
            sub_bbox_ = nh.subscribe<kcf_msgs::Bbox>(
                detection_topic, 1, &GimbalBboxPidSerial::bboxCb, this);
            ROS_INFO("[GimbalBboxPID] Subscribed to %s (kcf_msgs/Bbox)",
                     detection_topic.c_str());
        }

        gimbal_angles_pub_ = nh.advertise<geometry_msgs::Twist>(gimbal_angle_topic_, 1);
        los_pub_ = nh.advertise<geometry_msgs::Twist>("gimbal_bbox_los", 1);
        mang_pub_ = nh.advertise<geometry_msgs::Twist>("gimbal_mang", 1);

        serial_fd_ = openSerial(serial_device_, serial_baud_);
        if (serial_fd_ < 0) {
            ROS_ERROR("[GimbalBboxPID] Serial open failed. Node will stay alive but cannot control gimbal.");
        }

        const double timer_rate = rate_hz_ > 1e-6 ? rate_hz_ : 50.0;
        timer_ = nh.createTimer(ros::Duration(1.0 / timer_rate),
                                &GimbalBboxPidSerial::timerCb, this);

        ROS_INFO("[GimbalBboxPID] serial=%s baud=%d rate=%.1fHz fx=%.1f fy=%.1f angle_work_mode=%d gimbal_angle_topic=%s return_on_loss=%s",
                 serial_device_.c_str(), serial_baud_, timer_rate, fx_, fy_, angle_work_mode_,
                 gimbal_angle_topic_.c_str(),
                 return_on_target_loss_ ? "true" : "false");
    }

    ~GimbalBboxPidSerial()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    struct PidState {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double integral = 0.0;
        double prev_error = 0.0;
        bool have_prev = false;
    };

    struct BoxState {
        double x1 = 0.0;
        double y1 = 0.0;
        double x2 = 0.0;
        double y2 = 0.0;
        ros::Time stamp;
        bool valid = false;
    };

    ros::Subscriber sub_bbox_;
    ros::Subscriber sub_detection_;
    ros::Publisher gimbal_angles_pub_;
    ros::Publisher los_pub_;
    ros::Publisher mang_pub_;
    ros::Timer timer_;

    std::string serial_device_;
    int serial_baud_ = 115200;
    int serial_timeout_ms_ = 2;
    double rate_hz_ = 50.0;

    double fx_ = 640.0;
    double fy_ = 640.0;
    double cx_ = 320.0;
    double cy_ = 240.0;
    bool bbox_center_is_offset_ = true;

    PidState yaw_pid_;
    PidState pitch_pid_;
    PidState return_roll_pid_;
    PidState return_pitch_pid_;
    PidState return_yaw_pid_;
    double integral_limit_deg_ = 20.0;
    bool return_on_target_loss_ = true;
    double return_delta_limit_deg_ = 2.0;
    double roll_cmd_limit_deg_ = 45.0;
    double yaw_cmd_limit_deg_ = 145.0;
    double pitch_cmd_min_deg_ = 0.0;
    double pitch_cmd_max_deg_ = 30.0;
    double yaw_delta_limit_deg_ = 3.0;
    double pitch_delta_limit_deg_ = 3.0;
    double target_timeout_sec_ = 0.3;
    double yaw_sign_ = 1.0;
    double pitch_sign_ = 1.0;
    int angle_work_mode_ = 1;
    std::string gimbal_angle_topic_;

    int serial_fd_ = -1;
    std::vector<uint8_t> rx_buffer_;
    bool manual_mode_confirmed_ = false;
    uint8_t trig_ = 0;
    bool initialized_from_mang_ = false;

    bool have_mang_ = false;
    double cam_roll_deg_ = 0.0;
    double cam_pitch_deg_ = 0.0;
    double cam_yaw_deg_ = 0.0;
    double mang_pitch_deg_ = 0.0;
    double mang_roll_deg_ = 0.0;
    double mang_yaw_deg_ = 0.0;
    double cmd_roll_deg_ = 0.0;
    double cmd_pitch_deg_ = 0.0;
    double cmd_yaw_deg_ = 0.0;

    BoxState box_;

    void resetPid(PidState& pid)
    {
        pid.integral = 0.0;
        pid.prev_error = 0.0;
        pid.have_prev = false;
    }

    double updatePid(PidState& pid, double error_deg, double dt)
    {
        if (dt <= 1e-6) {
            dt = 1.0 / std::max(rate_hz_, 1.0);
        }

        pid.integral += error_deg * dt;
        pid.integral = clampValue(pid.integral, -integral_limit_deg_, integral_limit_deg_);

        double derivative = 0.0;
        if (pid.have_prev) {
            derivative = (error_deg - pid.prev_error) / dt;
        }

        pid.prev_error = error_deg;
        pid.have_prev = true;

        return pid.kp * error_deg + pid.ki * pid.integral + pid.kd * derivative;
    }

    std::vector<uint8_t> buildTxPacket(double roll_deg, double pitch_deg, double yaw_deg, bool hold_current_rate)
    {
        std::vector<uint8_t> pkt(kTxPacketSize, 0);
        pkt[0] = 0xA9;
        pkt[1] = 0x5B;

        const uint8_t cmd_value = manual_mode_confirmed_ ? 0u : 4u;
        pkt[2] = static_cast<uint8_t>(((cmd_value & 0x1Fu) << 3) | (trig_ & 0x07u));
        pkt[3] = 0x00;

        const uint8_t wk_mode =
            static_cast<uint8_t>(std::max(0, std::min(2, angle_work_mode_)));
        const uint8_t angle_mode =
            static_cast<uint8_t>((0u << 6) | (wk_mode << 4) | (0u << 3));
        const uint8_t rate_lock_mode =
            static_cast<uint8_t>((2u << 6) | (1u << 4) | (0u << 3));
        const uint8_t ctrl_mode = hold_current_rate ? rate_lock_mode : angle_mode;

        pkt[4] = ctrl_mode;
        putInt16LE(pkt, 5, hold_current_rate ? 0 :
                   static_cast<int16_t>(std::lround(roll_deg * 100.0)));

        pkt[7] = ctrl_mode;
        putInt16LE(pkt, 8, hold_current_rate ? 0 :
                   static_cast<int16_t>(std::lround(pitch_deg * 100.0)));

        pkt[10] = ctrl_mode;
        putInt16LE(pkt, 11, hold_current_rate ? 0 :
                   static_cast<int16_t>(std::lround(yaw_deg * 100.0)));

        pkt[13] = 0x00;

        const uint16_t crc = calculateCrc16(pkt.data(), static_cast<uint8_t>(kTxPacketSize - 2));
        pkt[kTxPacketSize - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        pkt[kTxPacketSize - 1] = static_cast<uint8_t>(crc & 0xFF);
        return pkt;
    }

    bool parseRxFrame(const uint8_t* frame)
    {
        if (frame[0] != 0xB5 || frame[1] != 0x9A) {
            return false;
        }

        const uint16_t crc_calc =
            calculateCrc16(frame, static_cast<uint8_t>(kRxPacketSize - 2));
        const uint8_t crc_hi = static_cast<uint8_t>((crc_calc >> 8) & 0xFF);
        const uint8_t crc_lo = static_cast<uint8_t>(crc_calc & 0xFF);
        if (frame[24] != crc_hi || frame[25] != crc_lo) {
            return false;
        }

        const uint8_t cmd = frame[5];
        const uint8_t cmd_stat = cmd & 0x07u;
        const uint8_t cmd_value = static_cast<uint8_t>((cmd >> 3) & 0x1Fu);
        if (!manual_mode_confirmed_ && cmd_value == 4u && cmd_stat == 1u) {
            manual_mode_confirmed_ = true;
            ROS_INFO("[GimbalBboxPID] Manual control command confirmed by gimbal.");
        }

        // Protocol: cam_angle[3] is ground/NED camera attitude, order roll, pitch, yaw.
        cam_roll_deg_ = readInt16LE(frame + 12) * 0.01;
        cam_pitch_deg_ = readInt16LE(frame + 14) * 0.01;
        cam_yaw_deg_ = readInt16LE(frame + 16) * 0.01;

        // Protocol: mtr_angle[3] is camera relative angle, order pitch, roll, yaw.
        mang_pitch_deg_ = readInt16LE(frame + 18) * 0.01;
        mang_roll_deg_ = readInt16LE(frame + 20) * 0.01;
        mang_yaw_deg_ = readInt16LE(frame + 22) * 0.01;
        have_mang_ = true;

        if (!initialized_from_mang_) {
            cmd_roll_deg_ = clampValue(cam_roll_deg_, -roll_cmd_limit_deg_, roll_cmd_limit_deg_);
            cmd_pitch_deg_ = clampValue(cam_pitch_deg_, pitch_cmd_min_deg_, pitch_cmd_max_deg_);
            cmd_yaw_deg_ = clampValue(cam_yaw_deg_, -yaw_cmd_limit_deg_, yaw_cmd_limit_deg_);
            initialized_from_mang_ = true;
            ROS_INFO("[GimbalBboxPID] Initialized command from current cam_att: roll=%.2f pitch=%.2f yaw=%.2f",
                     cmd_roll_deg_, cmd_pitch_deg_, cmd_yaw_deg_);
        }

        geometry_msgs::Twist msg;
        msg.angular.x = mang_roll_deg_;
        msg.angular.y = mang_pitch_deg_;
        msg.angular.z = mang_yaw_deg_;
        mang_pub_.publish(msg);
        return true;
    }

    void pollSerial()
    {
        if (serial_fd_ < 0) {
            return;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(serial_fd_, &rfds);

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = serial_timeout_ms_ * 1000;

        const int ret = select(serial_fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret <= 0 || !FD_ISSET(serial_fd_, &rfds)) {
            return;
        }

        uint8_t buf[256];
        while (true) {
            const ssize_t n = read(serial_fd_, buf, sizeof(buf));
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                ROS_ERROR_STREAM_THROTTLE(1.0, "serial read failed: " << strerror(errno));
                break;
            }
            if (n == 0) {
                break;
            }
            rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
        }

        while (rx_buffer_.size() >= kRxPacketSize) {
            if (!(rx_buffer_[0] == 0xB5 && rx_buffer_[1] == 0x9A)) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            if (parseRxFrame(rx_buffer_.data())) {
                rx_buffer_.erase(rx_buffer_.begin(),
                                 rx_buffer_.begin() + static_cast<long>(kRxPacketSize));
            } else {
                rx_buffer_.erase(rx_buffer_.begin());
            }
        }
    }

    bool targetActive(const ros::Time& now) const
    {
        return box_.valid && (now - box_.stamp).toSec() <= target_timeout_sec_;
    }

    bool isFiniteBox(double x1, double y1, double x2, double y2) const
    {
        return std::isfinite(x1) && std::isfinite(y1) &&
               std::isfinite(x2) && std::isfinite(y2);
    }

    void publishLos(double pitch_error_deg, double yaw_error_deg)
    {
        geometry_msgs::Twist los_msg;
        los_msg.angular.x = cam_roll_deg_;
        los_msg.angular.y = cam_pitch_deg_;
        los_msg.angular.z = mang_yaw_deg_;
        los_msg.linear.y = pitch_error_deg;
        los_msg.linear.z = yaw_error_deg;
        gimbal_angles_pub_.publish(los_msg);
        los_pub_.publish(los_msg);
    }

    void updateReturnToZero(double dt)
    {
        resetPid(yaw_pid_);
        resetPid(pitch_pid_);

        if (!return_on_target_loss_) {
            return;
        }

        const double roll_error_deg = -cam_roll_deg_;
        const double pitch_error_deg = -cam_pitch_deg_;
        const double yaw_error_deg = -mang_yaw_deg_;

        const double roll_delta_deg = clampValue(
            updatePid(return_roll_pid_, roll_error_deg, dt),
            -return_delta_limit_deg_,
            return_delta_limit_deg_);
        const double pitch_delta_deg = clampValue(
            updatePid(return_pitch_pid_, pitch_error_deg, dt),
            -return_delta_limit_deg_,
            return_delta_limit_deg_);
        const double yaw_delta_deg = clampValue(
            updatePid(return_yaw_pid_, yaw_error_deg, dt),
            -return_delta_limit_deg_,
            return_delta_limit_deg_);

        cmd_roll_deg_ = clampValue(cmd_roll_deg_ + roll_delta_deg,
                                   -roll_cmd_limit_deg_,
                                   roll_cmd_limit_deg_);
        cmd_pitch_deg_ = clampValue(cmd_pitch_deg_ + pitch_delta_deg,
                                    pitch_cmd_min_deg_,
                                    pitch_cmd_max_deg_);
        cmd_yaw_deg_ = clampValue(cmd_yaw_deg_ + yaw_delta_deg,
                                  -yaw_cmd_limit_deg_,
                                  yaw_cmd_limit_deg_);

        publishLos(pitch_error_deg, yaw_error_deg);

        ROS_INFO_THROTTLE(
            0.5,
            "[GimbalBboxPID] target lost, return zero | cam[r,p]=[%.2f,%.2f] mang_y=%.2f err[r,p,y]=[%.2f,%.2f,%.2f] cmd[r,p,y]=[%.2f,%.2f,%.2f]",
            cam_roll_deg_, cam_pitch_deg_, mang_yaw_deg_,
            roll_error_deg, pitch_error_deg, yaw_error_deg,
            cmd_roll_deg_, cmd_pitch_deg_, cmd_yaw_deg_);
    }

    void updateControl(const ros::Time& now, double dt)
    {
        if (!have_mang_) {
            return;
        }

        if (!targetActive(now)) {
            updateReturnToZero(dt);
            return;
        }

        resetPid(return_roll_pid_);
        resetPid(return_pitch_pid_);
        resetPid(return_yaw_pid_);

        const double raw_u = 0.5 * (box_.x1 + box_.x2);
        const double raw_v = 0.5 * (box_.y1 + box_.y2);
        const double centered_u = bbox_center_is_offset_ ? raw_u : (raw_u - cx_);
        const double centered_v = bbox_center_is_offset_ ? raw_v : (raw_v - cy_);

        const double yaw_error_deg = yaw_sign_ * radToDeg(std::atan2(-centered_u, fx_));
        const double pitch_error_deg = pitch_sign_ * radToDeg(std::atan2(centered_v, fy_));

        const double yaw_delta_deg = clampValue(
            updatePid(yaw_pid_, yaw_error_deg, dt),
            -yaw_delta_limit_deg_,
            yaw_delta_limit_deg_);
        const double pitch_delta_deg = clampValue(
            updatePid(pitch_pid_, pitch_error_deg, dt),
            -pitch_delta_limit_deg_,
            pitch_delta_limit_deg_);

        cmd_yaw_deg_ = clampValue(cmd_yaw_deg_ + yaw_delta_deg,
                                  -yaw_cmd_limit_deg_,
                                  yaw_cmd_limit_deg_);
        const double unclamped_pitch_cmd_deg = cmd_pitch_deg_ + pitch_delta_deg;
        cmd_pitch_deg_ = clampValue(unclamped_pitch_cmd_deg,
                                    pitch_cmd_min_deg_,
                                    pitch_cmd_max_deg_);
        if (std::fabs(unclamped_pitch_cmd_deg - cmd_pitch_deg_) > 1e-6) {
            ROS_WARN_THROTTLE(
                0.5,
                "[GimbalBboxPID] pitch cmd clamped: raw=%.2f clamped=%.2f limit=[%.2f, %.2f]. If pitch does not move, check pitch_sign or widen pitch limits.",
                unclamped_pitch_cmd_deg,
                cmd_pitch_deg_,
                pitch_cmd_min_deg_,
                pitch_cmd_max_deg_);
        }

        // With roll/pitch ground-stabilized, pitch LOS comes from cam_att pitch.
        // Yaw LOS is still the relative frame/yaw angle from mang.
        // Pixel error is only used to drive the centering loop.
        publishLos(pitch_error_deg, yaw_error_deg);

        ROS_INFO_THROTTLE(
            0.2,
            "[GimbalBboxPID] px_err[u,v]=[%.1f,%.1f] los[p,y]=[%.2f,%.2f] cam[r,p,y]=[%.2f,%.2f,%.2f] mang_y=%.2f err[p,y]=[%.2f,%.2f] cmd[r,p,y]=[%.2f,%.2f,%.2f]",
            centered_u, centered_v,
            cam_pitch_deg_, mang_yaw_deg_,
            cam_roll_deg_, cam_pitch_deg_, cam_yaw_deg_, mang_yaw_deg_,
            pitch_error_deg, yaw_error_deg,
            cmd_roll_deg_, cmd_pitch_deg_, cmd_yaw_deg_);
    }

    void timerCb(const ros::TimerEvent& event)
    {
        pollSerial();

        const double dt = event.current_real.isZero() || event.last_real.isZero()
                              ? 1.0 / std::max(rate_hz_, 1.0)
                              : (event.current_real - event.last_real).toSec();
        if (initialized_from_mang_) {
            updateControl(event.current_real, dt);
        }

        if (serial_fd_ < 0) {
            return;
        }

        const bool hold_current_rate = !initialized_from_mang_;
        if (!manual_mode_confirmed_) {
            trig_ = static_cast<uint8_t>((trig_ + 1u) & 0x07u);
        }

        std::vector<uint8_t> pkt = buildTxPacket(cmd_roll_deg_, cmd_pitch_deg_, cmd_yaw_deg_, hold_current_rate);
        writeAll(serial_fd_, pkt.data(), pkt.size());
    }

    void handleBox(double x1, double y1, double x2, double y2, bool is_tracking)
    {
        if (!is_tracking) {
            const bool was_valid = box_.valid;
            box_.valid = false;
            resetPid(yaw_pid_);
            resetPid(pitch_pid_);
            if (was_valid) {
                ROS_WARN("[GimbalBboxPID] is_tracking=false, discard bbox coordinates and enter return-to-zero mode.");
            }
            return;
        }

        if (!isFiniteBox(x1, y1, x2, y2)) {
            box_.valid = false;
            resetPid(yaw_pid_);
            resetPid(pitch_pid_);
            ROS_WARN_THROTTLE(1.0, "[GimbalBboxPID] Invalid bbox numbers, enter return-to-zero mode.");
            return;
        }

        box_.x1 = x1;
        box_.y1 = y1;
        box_.x2 = x2;
        box_.y2 = y2;
        box_.stamp = ros::Time::now();
        box_.valid = true;
    }

    void bboxCb(const kcf_msgs::Bbox::ConstPtr& msg)
    {
        handleBox(msg->x1, msg->y1, msg->x2, msg->y2, msg->is_tracking);
    }

    void detectionCb(const detection_msgs::Detection::ConstPtr& msg)
    {
        handleBox(msg->x1, msg->y1, msg->x2, msg->y2, msg->is_tracking);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_bbox_pid_serial");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    GimbalBboxPidSerial node(nh, pnh);
    ros::spin();
    return 0;
}
