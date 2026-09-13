#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

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

constexpr size_t kTxPacketSize = 40;
constexpr size_t kRxPacketSize = 26;

double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
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

class GimbalSearchScan {
public:
    GimbalSearchScan(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : search_enabled_(true),
          yaw_dir_(1),
          pitch_idx_(0),
          current_yaw_deg_(0.0),
          current_pitch_deg_(0.0),
          last_time_(0.0),
          dwell_until_(0.0)
    {
        pnh.param<std::string>("serial_device", serial_device_, std::string("/dev/ttyS7"));
        pnh.param<int>("serial_baud", serial_baud_, 115200);
        pnh.param<int>("serial_timeout_ms", serial_timeout_ms_, 2);
        pnh.param<int>("angle_work_mode", angle_work_mode_, 1);

        pnh.param("control_rate", control_rate_, 20.0);
        pnh.param("yaw_speed_deg", yaw_speed_deg_, 12.0);
        pnh.param("yaw_min_deg", yaw_min_deg_, -25.0);
        pnh.param("yaw_max_deg", yaw_max_deg_, 25.0);
        pnh.param("pitch_dwell_sec", pitch_dwell_sec_, 0.5);
        pnh.param("pitch_min_deg", pitch_min_deg_, 0.0);
        pnh.param("pitch_max_deg", pitch_max_deg_, 30.0);
        pnh.param("roll_cmd_deg", roll_cmd_deg_, 0.0);
        pnh.param("start_enabled", search_enabled_, true);
        pnh.param<std::string>("tracked_angle_topic", tracked_angle_topic_,
                               std::string("/gimbal_angles"));

        std::vector<double> default_pitch_levels;
        default_pitch_levels.push_back(0.0);
        default_pitch_levels.push_back(10.0);
        default_pitch_levels.push_back(20.0);
        pnh.param("pitch_levels_deg", pitch_levels_deg_, default_pitch_levels);

        if (pitch_levels_deg_.empty()) {
            pitch_levels_deg_ = default_pitch_levels;
            ROS_WARN("~pitch_levels_deg is empty, fallback to default levels.");
        }

        if (control_rate_ <= 0.0) {
            control_rate_ = 20.0;
            ROS_WARN("~control_rate must be > 0, fallback to 20.0 Hz.");
        }

        if (yaw_speed_deg_ < 0.0) {
            yaw_speed_deg_ = std::fabs(yaw_speed_deg_);
            ROS_WARN("~yaw_speed_deg is negative, use its absolute value.");
        }

        if (yaw_min_deg_ > yaw_max_deg_) {
            std::swap(yaw_min_deg_, yaw_max_deg_);
            ROS_WARN("~yaw_min_deg is larger than ~yaw_max_deg, values swapped.");
        }

        for (double& pitch : pitch_levels_deg_) {
            pitch = clampValue(pitch, pitch_min_deg_, pitch_max_deg_);
        }

        current_yaw_deg_ = yaw_min_deg_;
        pitch_idx_ = 0;
        current_pitch_deg_ = pitch_levels_deg_[pitch_idx_];
        yaw_dir_ = 1;

        serial_fd_ = openSerial(serial_device_, serial_baud_);
        if (serial_fd_ < 0) {
            ROS_ERROR("[GimbalSearchScan] Serial open failed. Scan angles will still be published.");
        }

        angle_pub_ = nh.advertise<geometry_msgs::Twist>(tracked_angle_topic_, 1);
        enable_sub_ = nh.subscribe("enable_search", 1, &GimbalSearchScan::enableCb, this);
        timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_),
                                &GimbalSearchScan::timerCb, this);

        writeCurrentCommand();
        publishAngles();

        ROS_INFO("gimbal_search_scan serial=%s baud=%d work_mode=%d yaw=[%.2f, %.2f] pitch=%.2f enabled=%s tracked_angle_topic=%s",
                 serial_device_.c_str(), serial_baud_, angle_work_mode_,
                 yaw_min_deg_, yaw_max_deg_, current_pitch_deg_,
                 search_enabled_ ? "true" : "false",
                 tracked_angle_topic_.c_str());
    }

    ~GimbalSearchScan()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

    void publishAngles()
    {
        geometry_msgs::Twist msg;
        msg.angular.x = feedback_cam_roll_deg_;
        msg.angular.y = have_feedback_ ? feedback_cam_pitch_deg_ : current_pitch_deg_;
        msg.angular.z = have_feedback_ ? feedback_mang_yaw_deg_ : current_yaw_deg_;
        angle_pub_.publish(msg);
    }

    void writeCurrentCommand()
    {
        current_pitch_deg_ = clampValue(current_pitch_deg_, pitch_min_deg_, pitch_max_deg_);

        if (serial_fd_ < 0) {
            return;
        }

        if (!manual_mode_confirmed_) {
            trig_ = static_cast<uint8_t>((trig_ + 1u) & 0x07u);
        }

        const std::vector<uint8_t> pkt = buildTxPacket(
            roll_cmd_deg_, current_pitch_deg_, current_yaw_deg_);
        writeAll(serial_fd_, pkt.data(), pkt.size());
        pollSerial();
    }

    void advancePitchLayer(const ros::Time& now)
    {
        pitch_idx_ = (pitch_idx_ + 1) % static_cast<int>(pitch_levels_deg_.size());
        current_pitch_deg_ = pitch_levels_deg_[pitch_idx_];
        dwell_until_ = now + ros::Duration(pitch_dwell_sec_);
    }

    void enableCb(const std_msgs::Bool::ConstPtr& msg)
    {
        search_enabled_ = msg->data;
        last_time_ = ros::Time(0);
        writeCurrentCommand();
        publishAngles();

        ROS_INFO("gimbal_search_scan enable_search=%s",
                 search_enabled_ ? "true" : "false");
    }

    void timerCb(const ros::TimerEvent& event)
    {
        const ros::Time now = event.current_real;

        if (last_time_.isZero()) {
            last_time_ = now;
            writeCurrentCommand();
            publishAngles();
            return;
        }

        double dt = (now - last_time_).toSec();
        last_time_ = now;

        if (dt < 0.0) {
            dt = 0.0;
        }

        if (!search_enabled_) {
            writeCurrentCommand();
            publishAngles();
            return;
        }

        if (now < dwell_until_) {
            writeCurrentCommand();
            publishAngles();
            return;
        }

        current_yaw_deg_ += static_cast<double>(yaw_dir_) * yaw_speed_deg_ * dt;

        if (current_yaw_deg_ >= yaw_max_deg_) {
            current_yaw_deg_ = yaw_max_deg_;
            yaw_dir_ = -1;
            advancePitchLayer(now);
        } else if (current_yaw_deg_ <= yaw_min_deg_) {
            current_yaw_deg_ = yaw_min_deg_;
            yaw_dir_ = 1;
            advancePitchLayer(now);
        }

        writeCurrentCommand();
        publishAngles();
    }

private:
    std::vector<uint8_t> buildTxPacket(double roll_deg, double pitch_deg, double yaw_deg)
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

        pkt[4] = angle_mode;
        putInt16LE(pkt, 5, static_cast<int16_t>(std::lround(roll_deg * 100.0)));

        pkt[7] = angle_mode;
        putInt16LE(pkt, 8, static_cast<int16_t>(std::lround(pitch_deg * 100.0)));

        pkt[10] = angle_mode;
        putInt16LE(pkt, 11, static_cast<int16_t>(std::lround(yaw_deg * 100.0)));

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
            ROS_INFO("[GimbalSearchScan] Manual control command confirmed by gimbal.");
        }

        feedback_cam_roll_deg_ = readInt16LE(frame + 12) * 0.01;
        feedback_cam_pitch_deg_ = readInt16LE(frame + 14) * 0.01;
        feedback_mang_yaw_deg_ = readInt16LE(frame + 22) * 0.01;
        have_feedback_ = true;
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

    ros::Publisher angle_pub_;
    ros::Subscriber enable_sub_;
    ros::Timer timer_;

    bool search_enabled_;
    int yaw_dir_;
    int pitch_idx_;
    double current_yaw_deg_;
    double current_pitch_deg_;
    ros::Time last_time_;
    ros::Time dwell_until_;

    std::string serial_device_;
    int serial_baud_ = 115200;
    int serial_timeout_ms_ = 2;
    int serial_fd_ = -1;
    bool manual_mode_confirmed_ = false;
    uint8_t trig_ = 0;
    std::vector<uint8_t> rx_buffer_;

    bool have_feedback_ = false;
    double feedback_cam_roll_deg_ = 0.0;
    double feedback_cam_pitch_deg_ = 0.0;
    double feedback_mang_yaw_deg_ = 0.0;

    int angle_work_mode_ = 1;
    double control_rate_;
    double yaw_speed_deg_;
    double yaw_min_deg_;
    double yaw_max_deg_;
    double pitch_min_deg_;
    double pitch_max_deg_;
    double roll_cmd_deg_;
    double pitch_dwell_sec_;
    std::vector<double> pitch_levels_deg_;
    std::string tracked_angle_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbal_search_scan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    GimbalSearchScan node(nh, pnh);
    ros::spin();
    return 0;
}
