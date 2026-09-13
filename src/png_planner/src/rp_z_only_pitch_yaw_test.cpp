#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <kcf_msgs/Bbox.h>
#include <detection_msgs/Detection.h>

#include <cmath>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include "los_rate_kalman.h"

template<typename T>
T clampValue(T v, T min_val, T max_val) {
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

struct DetectionData {
    float x1, y1, x2, y2;
    float score;
    std::string class_name;
    int track_id;
    bool is_tracking;
};

uint16_t CalculateCrc16(const uint8_t* ptr, uint8_t len)
{
    uint16_t crc = 0;
    const uint16_t crc_ta[16] = {
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
    };

    while (len--) {
        uint8_t da = (crc >> 12);
        crc = (crc << 4) ^ crc_ta[da ^ ((*ptr) >> 4)];
        da = (crc >> 12);
        crc = (crc << 4) ^ crc_ta[da ^ ((*ptr) & 0x0F)];
        ++ptr;
    }
    return crc;
}

static inline void putInt16LE(std::vector<uint8_t>& buf, size_t idx, int16_t v) {
    buf[idx] = static_cast<uint8_t>(v & 0xFF);
    buf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

bool configureSerial(int fd, int baud = 115200)
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

int openSerial(const std::string& dev)
{
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR_STREAM("open " << dev << " failed: " << strerror(errno));
        return -1;
    }

    if (!configureSerial(fd, 115200)) {
        close(fd);
        return -1;
    }

    return fd;
}

bool writeAll(int fd, const uint8_t* data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            ROS_ERROR_STREAM("write failed: " << strerror(errno));
            return false;
        }
        sent += static_cast<size_t>(n);
    }

    tcdrain(fd);
    return true;
}

std::vector<uint8_t> buildTxPacket(bool send_manual_cmd, uint8_t trig, double pitch_deg)
{
    std::vector<uint8_t> pkt(40, 0);

    pkt[0] = 0xA9;
    pkt[1] = 0x5B;

    const uint8_t cmd_value = send_manual_cmd ? 4 : 0;
    pkt[2] = static_cast<uint8_t>(((cmd_value & 0x1F) << 3) | (trig & 0x07));
    pkt[3] = 0x00;

    pkt[4] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 5, 0);

    pkt[7] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 8, static_cast<int16_t>(std::lround(pitch_deg * 100.0)));

    pkt[10] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 11, 0);

    pkt[13] = 0x00;

    const uint16_t crc = CalculateCrc16(pkt.data(), 38);
    pkt[38] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    pkt[39] = static_cast<uint8_t>(crc & 0xFF);

    return pkt;
}

bool tryReadOneFrame(int fd, std::vector<uint8_t>& frame_out, int timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return false;
    }

    uint8_t buf[256];
    ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) {
        return false;
    }

    static std::vector<uint8_t> cache;
    cache.insert(cache.end(), buf, buf + n);

    while (cache.size() >= 2) {
        if (!(cache[0] == 0xB5 && cache[1] == 0x9A)) {
            cache.erase(cache.begin());
            continue;
        }

        if (cache.size() < 26) {
            return false;
        }

        std::vector<uint8_t> frame(cache.begin(), cache.begin() + 26);
        const uint16_t crc_calc = CalculateCrc16(frame.data(), 24);
        const uint8_t crc_hi = static_cast<uint8_t>((crc_calc >> 8) & 0xFF);
        const uint8_t crc_lo = static_cast<uint8_t>(crc_calc & 0xFF);

        if (frame[24] == crc_hi && frame[25] == crc_lo) {
            frame_out = frame;
            cache.erase(cache.begin(), cache.begin() + 26);
            return true;
        }

        cache.erase(cache.begin());
    }

    return false;
}

class GimbalPort {
public:
    explicit GimbalPort(ros::NodeHandle& nh) {
        nh.param("gimbal_enable", enabled_, true);
        nh.param<std::string>("gimbal_device", device_, "/dev/ttyS7");

        if (!enabled_) {
            ROS_WARN("[ZOnlyPitchYawTest] Gimbal output disabled by parameter.");
            return;
        }

        fd_ = openSerial(device_);
        if (fd_ < 0) {
            enabled_ = false;
            ROS_WARN_STREAM("[ZOnlyPitchYawTest] Disable gimbal output because serial open failed: "
                            << device_);
            return;
        }

        ROS_INFO_STREAM("[ZOnlyPitchYawTest] Gimbal serial ready: " << device_);
    }

    ~GimbalPort() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    void sendPitch(double pitch_deg) {
        if (!enabled_ || fd_ < 0) {
            return;
        }

        const std::vector<uint8_t> tx = buildTxPacket(!manual_cmd_sent_, trig_, pitch_deg);
        if (!writeAll(fd_, tx.data(), tx.size())) {
            ROS_ERROR_STREAM_THROTTLE(1.0,
                                      "[ZOnlyPitchYawTest] Gimbal write failed, disable serial output.");
            close(fd_);
            fd_ = -1;
            enabled_ = false;
            return;
        }

        if (!manual_cmd_sent_) {
            manual_cmd_sent_ = true;
            trig_ = (trig_ + 1) & 0x07;
        }

        std::vector<uint8_t> rx;
        while (tryReadOneFrame(fd_, rx, 0)) {
        }
    }

private:
    bool enabled_ = true;
    std::string device_;
    int fd_ = -1;
    bool manual_cmd_sent_ = false;
    uint8_t trig_ = 0;
};

class ZOnlyPitchYawInterceptor {
public:
    explicit ZOnlyPitchYawInterceptor(ros::NodeHandle& nh)
        : gimbal_(nh) {
        nh.param("fx", fx_, 640.0);
        nh.param("fy", fy_, 640.0);
        nh.param("cx", cx_, 480.0);
        nh.param("cy", cy_, 270.0);

        nh.param("bbox_center_is_offset", bbox_center_is_offset_, true);
        nh.param("y_up_positive", y_up_positive_, true);

        nh.param("V_default", V_default_, 5.0);
        nh.param("N_nav_z", N_nav_z_, 2.0);

        nh.param("use_real_dt", use_real_dt_, true);
        nh.param("step_dt", step_dt_, 0.02);
        nh.param("pixel_thr", pixel_thr_, 5.0);
        nh.param("pixel_thr_y", pixel_thr_y_, 5.0);

        nh.param("k_yaw", k_yaw_, 0.8);
        nh.param("k_yaw_d", k_yaw_d_, 0.3);
        nh.param("max_yaw_rate", max_yaw_rate_, 1.0);

        double lambda_alpha_legacy = 0.2;
        int lambda_window_size_legacy = 3;
        nh.param("lambda_filt_alpha", lambda_alpha_legacy, 0.2);
        nh.param("lambda_window_size", lambda_window_size_legacy, 3);
        const LegacyLosKalmanTuning lambda_tuning =
            mapLegacyLosChainToKalman(lambda_alpha_legacy, lambda_window_size_legacy);
        nh.param("lambda_kalman_q_position", lambda_kalman_q_position_, lambda_tuning.q_position);
        nh.param("lambda_kalman_q_rate", lambda_kalman_q_rate_, lambda_tuning.q_rate);
        nh.param("lambda_kalman_r", lambda_kalman_r_, lambda_tuning.r_measurement);

        nh.param("v_center_alpha", v_center_alpha_, 0.2);

        double gamma_alpha_legacy = 0.2;
        int gamma_window_size_legacy = 3;
        nh.param("gamma_filt_alpha", gamma_alpha_legacy, 0.2);
        nh.param("gamma_window_size", gamma_window_size_legacy, 3);
        const LegacyLosKalmanTuning gamma_tuning =
            mapLegacyLosChainToKalman(gamma_alpha_legacy, gamma_window_size_legacy);
        nh.param("gamma_kalman_q_position", gamma_kalman_q_position_, gamma_tuning.q_position);
        nh.param("gamma_kalman_q_rate", gamma_kalman_q_rate_, gamma_tuning.q_rate);
        nh.param("gamma_kalman_r", gamma_kalman_r_, gamma_tuning.r_measurement);

        nh.param("k_z_ff", k_z_ff_, 1.0);
        nh.param("max_vz", max_vz_, 1.0);
        nh.param("enable_height_control", enable_height_control_, true);

        nh.param("gimbal_pitch_bias_deg", gimbal_pitch_bias_deg_, 0.0);
        nh.param("gimbal_pitch_gain", gimbal_pitch_gain_, 1.0);
        nh.param("gimbal_pitch_sign", gimbal_pitch_sign_, 1.0);
        nh.param("gimbal_pitch_kp", gimbal_pitch_kp_, 1.0);
        nh.param("gimbal_pitch_ki", gimbal_pitch_ki_, 0.0);
        nh.param("gimbal_pitch_kd", gimbal_pitch_kd_, 0.0);
        nh.param("gimbal_pitch_i_limit_deg", gimbal_pitch_i_limit_deg_, 10.0);
        nh.param("gimbal_pitch_d_alpha", gimbal_pitch_d_alpha_, 0.2);
        nh.param("gimbal_pitch_integral_decay", gimbal_pitch_integral_decay_, 0.98);
        nh.param("gimbal_pitch_delta_limit_deg", gimbal_pitch_delta_limit_deg_, 2.0);
        nh.param("gimbal_pitch_limit_deg", gimbal_pitch_limit_deg_, 30.0);
        nh.param("reset_gimbal_on_target_loss", reset_gimbal_on_target_loss_, true);
        nh.param("enable_gimbal_pitch_cmd_generation", enable_gimbal_pitch_cmd_generation_, true);
        nh.param("gimbal_pitch_los_sign", gimbal_pitch_los_sign_, 1.0);
        nh.param("gimbal_pitch_los_thr_deg", gimbal_pitch_los_thr_deg_, 1.0);

        int msg_mode = 0;
        std::string topic_name;
        nh.param("detection_mode", msg_mode, 0);
        nh.param<std::string>("detection_topic", topic_name, "/object_kcf");
        if (msg_mode == 1 && topic_name == "/object_kcf") {
            topic_name = "/object_detections";
        }

        if (msg_mode == 1) {
            sub_det_detection_ = nh.subscribe<detection_msgs::Detection>(
                topic_name, 1, &ZOnlyPitchYawInterceptor::detectionCbDetection, this);
            ROS_INFO("[ZOnlyPitchYawTest] Subscribed to %s (detection_msgs/Detection)",
                     topic_name.c_str());
        } else {
            sub_det_kcf_ = nh.subscribe<kcf_msgs::Bbox>(
                topic_name, 1, &ZOnlyPitchYawInterceptor::detectionCbKcf, this);
            ROS_INFO("[ZOnlyPitchYawTest] Subscribed to %s (kcf_msgs/Bbox)",
                     topic_name.c_str());
        }

        pub_cmd_ = nh.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 1);

        last_time_ = ros::Time::now();
        have_det_ = false;
        last_pitch_cmd_deg_ = gimbal_pitch_bias_deg_;

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
        vz_png_int_ = 0.0;

        ROS_INFO("[ZOnlyPitchYawTest] Started: yaw control + z control via gimbal pitch LOS, no x/y velocity.");
    }

    void spin() {
        ros::Rate rate(50.0);

        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            double dt = use_real_dt_ ? (now - last_time_).toSec() : step_dt_;
            if (dt <= 0.0) {
                dt = step_dt_;
            }
            last_time_ = now;

            double yaw_rate_cmd = 0.0;
            double pitch_cmd_deg = last_pitch_cmd_deg_;
            double vz_cmd = 0.0;

            if (have_det_) {
                const double raw_u = 0.5 * (last_det_.x1 + last_det_.x2);
                const double raw_v = 0.5 * (last_det_.y1 + last_det_.y2);

                const double centered_u = bbox_center_is_offset_ ? raw_u : (raw_u - cx_);
                const double centered_v = bbox_center_is_offset_ ? raw_v : (raw_v - cy_);

                const double lambda_meas = std::atan2(-centered_u, fx_);
                lambda_filter_.update(lambda_meas, dt);

                const double lambda = lambda_filter_.value();
                const double lambda_dot = lambda_filter_.rate();
                const double ex = fx_ * std::tan(lambda);

                if (std::fabs(ex) > pixel_thr_) {
                    yaw_rate_cmd = k_yaw_ * lambda + k_yaw_d_ * lambda_dot;
                    yaw_rate_cmd = clampValue(
                        yaw_rate_cmd,
                        -max_yaw_rate_,
                        max_yaw_rate_);
                }

                if (!v_center_inited_) {
                    v_center_filt_ = centered_v;
                    v_center_inited_ = true;
                } else {
                    v_center_filt_ =
                        v_center_alpha_ * centered_v +
                        (1.0 - v_center_alpha_) * v_center_filt_;
                }

                const double ey = y_up_positive_ ? v_center_filt_ : -v_center_filt_;
                if (enable_gimbal_pitch_cmd_generation_) {
                    double pitch_error_deg = 0.0;
                    if (std::fabs(ey) > pixel_thr_y_) {
                        const double pitch_los_deg = std::atan2(ey, fy_) * 180.0 / M_PI;
                        pitch_error_deg =
                            gimbal_pitch_sign_ * gimbal_pitch_gain_ * pitch_los_deg;
                    } else {
                        pitch_integral_deg_ *= gimbal_pitch_integral_decay_;
                    }

                    if (dt > 0.0) {
                        pitch_integral_deg_ += pitch_error_deg * dt;
                        pitch_integral_deg_ = clampValue(
                            pitch_integral_deg_,
                            -gimbal_pitch_i_limit_deg_,
                            gimbal_pitch_i_limit_deg_);

                        const double pitch_error_dot =
                            (pitch_error_deg - pitch_error_prev_deg_) / dt;
                        pitch_error_dot_filt_deg_ =
                            gimbal_pitch_d_alpha_ * pitch_error_dot +
                            (1.0 - gimbal_pitch_d_alpha_) * pitch_error_dot_filt_deg_;
                    }
                    pitch_error_prev_deg_ = pitch_error_deg;

                    double pitch_delta_deg =
                        gimbal_pitch_kp_ * pitch_error_deg +
                        gimbal_pitch_ki_ * pitch_integral_deg_ +
                        gimbal_pitch_kd_ * pitch_error_dot_filt_deg_;

                    pitch_delta_deg = clampValue(
                        pitch_delta_deg,
                        -gimbal_pitch_delta_limit_deg_,
                        gimbal_pitch_delta_limit_deg_);

                    pitch_cmd_deg = last_pitch_cmd_deg_ + pitch_delta_deg;
                } else {
                    pitch_cmd_deg = gimbal_pitch_bias_deg_;
                    pitch_integral_deg_ = 0.0;
                    pitch_error_prev_deg_ = 0.0;
                    pitch_error_dot_filt_deg_ = 0.0;
                }
            } else {
                lambda_filter_.reset();
                gamma_filter_.reset();
                v_center_inited_ = false;

                if (reset_gimbal_on_target_loss_) {
                    pitch_cmd_deg = gimbal_pitch_bias_deg_;
                    pitch_integral_deg_ = 0.0;
                    pitch_error_prev_deg_ = 0.0;
                    pitch_error_dot_filt_deg_ = 0.0;
                }
            }

            pitch_cmd_deg = clampValue(
                pitch_cmd_deg,
                -gimbal_pitch_limit_deg_,
                gimbal_pitch_limit_deg_);
            last_pitch_cmd_deg_ = pitch_cmd_deg;

            if (enable_height_control_ && have_det_) {
                const double gamma_meas_deg =
                    gimbal_pitch_los_sign_ *
                    (pitch_cmd_deg - gimbal_pitch_bias_deg_);
                const double gamma_meas = gamma_meas_deg * M_PI / 180.0;
                gamma_filter_.update(gamma_meas, dt);

                const double gamma = gamma_filter_.value();
                const double gamma_dot = gamma_filter_.rate();
                const double gamma_deg = gamma * 180.0 / M_PI;

                if (std::fabs(gamma_deg) > gimbal_pitch_los_thr_deg_) {
                    const double a_z = N_nav_z_ * V_default_ * gamma_dot;
                    vz_png_int_ += a_z * dt;
                } else {
                    vz_png_int_ *= 0.98;
                }

                const double vz_ff = k_z_ff_ * gamma;
                vz_cmd = clampValue(vz_ff + vz_png_int_, -max_vz_, max_vz_);
            } else {
                gamma_filter_.reset();
                vz_png_int_ = 0.0;
            }

            geometry_msgs::TwistStamped cmd;
            cmd.header.stamp = now;
            cmd.twist.linear.x = 0.0;
            cmd.twist.linear.y = 0.0;
            cmd.twist.linear.z = vz_cmd;
            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = yaw_rate_cmd;

            pub_cmd_.publish(cmd);
            if (enable_gimbal_pitch_cmd_generation_) {
                gimbal_.sendPitch(pitch_cmd_deg);
            }
            rate.sleep();
        }
    }

private:
    void detectionCbKcf(const kcf_msgs::Bbox::ConstPtr& msg) {
        if (!msg->is_tracking) {
            have_det_ = false;
            lambda_filter_.reset();
            gamma_filter_.reset();
            v_center_inited_ = false;
            return;
        }

        last_det_.x1 = msg->x1;
        last_det_.y1 = msg->y1;
        last_det_.x2 = msg->x2;
        last_det_.y2 = msg->y2;
        last_det_.score = msg->score;
        last_det_.class_name = msg->class_name;
        last_det_.track_id = msg->track_id;
        last_det_.is_tracking = msg->is_tracking;
        have_det_ = true;
    }

    void detectionCbDetection(const detection_msgs::Detection::ConstPtr& msg) {
        if (!msg->is_tracking) {
            have_det_ = false;
            lambda_filter_.reset();
            gamma_filter_.reset();
            v_center_inited_ = false;
            return;
        }

        last_det_.x1 = msg->x1;
        last_det_.y1 = msg->y1;
        last_det_.x2 = msg->x2;
        last_det_.y2 = msg->y2;
        last_det_.score = msg->score;
        last_det_.class_name = msg->class_name;
        last_det_.track_id = msg->track_id;
        last_det_.is_tracking = msg->is_tracking;
        have_det_ = true;
    }

    double fx_ = 640.0;
    double fy_ = 640.0;
    double cx_ = 480.0;
    double cy_ = 270.0;

    bool bbox_center_is_offset_ = true;
    bool y_up_positive_ = true;

    double V_default_ = 5.0;
    double N_nav_z_ = 2.0;

    bool use_real_dt_ = true;
    double step_dt_ = 0.02;
    double pixel_thr_ = 5.0;
    double pixel_thr_y_ = 5.0;

    double k_yaw_ = 0.8;
    double k_yaw_d_ = 0.3;
    double max_yaw_rate_ = 1.0;
    double lambda_kalman_q_position_ = 1e-4;
    double lambda_kalman_q_rate_ = 5e-1;
    double lambda_kalman_r_ = 2e-4;

    double v_center_alpha_ = 0.2;
    double gamma_kalman_q_position_ = 1e-4;
    double gamma_kalman_q_rate_ = 5e-1;
    double gamma_kalman_r_ = 2e-4;
    double k_z_ff_ = 1.0;
    double max_vz_ = 1.0;
    bool enable_height_control_ = true;

    double gimbal_pitch_bias_deg_ = 0.0;
    double gimbal_pitch_gain_ = 1.0;
    double gimbal_pitch_sign_ = 1.0;
    double gimbal_pitch_kp_ = 1.0;
    double gimbal_pitch_ki_ = 0.0;
    double gimbal_pitch_kd_ = 0.0;
    double gimbal_pitch_i_limit_deg_ = 10.0;
    double gimbal_pitch_d_alpha_ = 0.2;
    double gimbal_pitch_integral_decay_ = 0.98;
    double gimbal_pitch_delta_limit_deg_ = 2.0;
    double gimbal_pitch_limit_deg_ = 30.0;
    bool reset_gimbal_on_target_loss_ = true;
    bool enable_gimbal_pitch_cmd_generation_ = true;
    double gimbal_pitch_los_sign_ = 1.0;
    double gimbal_pitch_los_thr_deg_ = 1.0;

    bool have_det_ = false;
    bool v_center_inited_ = false;
    ros::Time last_time_;

    double v_center_filt_ = 0.0;
    double last_pitch_cmd_deg_ = 0.0;
    double pitch_integral_deg_ = 0.0;
    double pitch_error_prev_deg_ = 0.0;
    double pitch_error_dot_filt_deg_ = 0.0;
    double vz_png_int_ = 0.0;

    ScalarRateKalmanFilter lambda_filter_;
    ScalarRateKalmanFilter gamma_filter_;

    DetectionData last_det_{};

    ros::Subscriber sub_det_kcf_;
    ros::Subscriber sub_det_detection_;
    ros::Publisher pub_cmd_;

    GimbalPort gimbal_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rp_z_only_pitch_yaw_test_node");
    ros::NodeHandle nh("~");

    ZOnlyPitchYawInterceptor node(nh);
    node.spin();
    return 0;
}
