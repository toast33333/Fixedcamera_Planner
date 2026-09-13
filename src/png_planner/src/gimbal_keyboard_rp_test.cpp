#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <csignal>
#include <cmath>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

static volatile bool g_running = true;

void signalHandler(int) {
    g_running = false;
}

uint16_t CalculateCrc16(const uint8_t *ptr, uint8_t len)
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
        ptr++;
    }
    return crc;
}

static inline void putInt16LE(std::vector<uint8_t>& buf, size_t idx, int16_t v) {
    buf[idx]     = static_cast<uint8_t>(v & 0xFF);
    buf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline int16_t getInt16LE(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

template<typename T>
T clampValue(T v, T mn, T mx) {
    return (v < mn) ? mn : ((v > mx) ? mx : v);
}

bool configureSerial(int fd, int baud = 115200)
{
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
        return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    switch (baud) {
        case 115200: speed = B115200; break;
        default:
            std::cerr << "Unsupported baud, fallback to 115200" << std::endl;
            speed = B115200;
            break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

int openSerial(const std::string& dev)
{
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "open " << dev << " failed: " << strerror(errno) << std::endl;
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
            if (errno == EINTR) continue;
            std::cerr << "write failed: " << strerror(errno) << std::endl;
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    tcdrain(fd);
    return true;
}

class KeyboardRawMode {
public:
    KeyboardRawMode() : valid_(false) {
        if (tcgetattr(STDIN_FILENO, &old_) == 0) {
            termios raw = old_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                valid_ = true;
            }
        }
    }

    ~KeyboardRawMode() {
        if (valid_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_);
        }
    }

private:
    termios old_{};
    bool valid_;
};

bool readKeyNonBlocking(char &c, int timeout_ms = 0)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
        if (errno == EINTR) return false;
        return false;
    }
    if (ret == 0) return false;

    char ch;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    if (n == 1) {
        c = ch;
        return true;
    }
    return false;
}

// 固定 40 字节发送帧
std::vector<uint8_t> buildTxPacket(
    bool send_manual_cmd,
    uint8_t trig,
    double roll_deg,
    double pitch_deg)
{
    std::vector<uint8_t> pkt(40, 0);

    pkt[0] = 0xA9;
    pkt[1] = 0x5B;

    // 命令码 4 = 手动控制
    uint8_t cmd_value = send_manual_cmd ? 4 : 0;
    pkt[2] = static_cast<uint8_t>(((cmd_value & 0x1F) << 3) | (trig & 0x07));

    pkt[3] = 0x00; // aux

    // gbc[0] = 滚转：锁定 + 角度控制
    pkt[4] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 5, static_cast<int16_t>(std::lround(roll_deg * 100.0)));

    // gbc[1] = 俯仰：锁定 + 角度控制
    pkt[7] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 8, static_cast<int16_t>(std::lround(pitch_deg * 100.0)));

    // gbc[2] = 偏航：固定为 0
    pkt[10] = static_cast<uint8_t>((0u << 6) | (1u << 4) | (0u << 3));
    putInt16LE(pkt, 11, 0);

    // 无准确载机惯导时，valid 必须置 0
    pkt[13] = 0x00;

    uint16_t crc = CalculateCrc16(pkt.data(), 38);
    pkt[38] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    pkt[39] = static_cast<uint8_t>(crc & 0xFF);

    return pkt;
}

// 固定 26 字节返回帧
bool tryReadOneFrame(int fd, std::vector<uint8_t>& frame_out, int timeout_ms = 5)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
        if (errno == EINTR) return false;
        std::cerr << "select(serial) failed: " << strerror(errno) << std::endl;
        return false;
    }
    if (ret == 0) return false;

    uint8_t buf[256];
    ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) return false;

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

        uint16_t crc_calc = CalculateCrc16(frame.data(), 24);
        uint8_t crc_hi = static_cast<uint8_t>((crc_calc >> 8) & 0xFF);
        uint8_t crc_lo = static_cast<uint8_t>(crc_calc & 0xFF);

        if (frame[24] == crc_hi && frame[25] == crc_lo) {
            frame_out = frame;
            cache.erase(cache.begin(), cache.begin() + 26);
            return true;
        } else {
            cache.erase(cache.begin());
        }
    }

    return false;
}

void printRxBrief(const std::vector<uint8_t>& f)
{
    if (f.size() != 26) return;

    uint8_t fw_ver = f[2];
    uint8_t hw_err = f[3];

    uint8_t flag = f[4];
    uint8_t inv_flag = flag & 0x01;
    uint8_t gbc_stat = (flag >> 1) & 0x07;
    uint8_t tca_flag = (flag >> 4) & 0x01;

    uint8_t cmd = f[5];
    uint8_t cmd_stat = cmd & 0x07;
    uint8_t cmd_value = (cmd >> 3) & 0x1F;

    int16_t cam_angle_roll  = getInt16LE(&f[12]);
    int16_t cam_angle_pitch = getInt16LE(&f[14]);
    int16_t cam_angle_yaw   = getInt16LE(&f[16]);

    int16_t mtr_angle_pitch = getInt16LE(&f[18]);
    int16_t mtr_angle_roll  = getInt16LE(&f[20]);
    int16_t mtr_angle_yaw   = getInt16LE(&f[22]);

    std::cout << "\r"
              << "fw=" << static_cast<int>(fw_ver)
              << " hw_err=" << static_cast<int>(hw_err)
              << " stat=" << static_cast<int>(gbc_stat)
              << " tca=" << static_cast<int>(tca_flag)
              << " inv=" << static_cast<int>(inv_flag)
              << " cmd=" << static_cast<int>(cmd_value)
              << " cmd_stat=" << static_cast<int>(cmd_stat)
              << " | cam[r,p,y]=["
              << std::fixed << std::setprecision(2)
              << cam_angle_roll * 0.01 << ","
              << cam_angle_pitch * 0.01 << ","
              << cam_angle_yaw * 0.01 << "]"
              << " | mtr[p,r,y]=["
              << mtr_angle_pitch * 0.01 << ","
              << mtr_angle_roll * 0.01 << ","
              << mtr_angle_yaw * 0.01 << "]     "
              << std::flush;
}

void printHelp(const std::string& dev, double step_deg, double roll_limit_deg, double pitch_limit_deg)
{
    std::cout << "========================================\n";
    std::cout << "Gimbal Roll/Pitch Keyboard Test\n";
    std::cout << "Serial: " << dev << "\n";
    std::cout << "Step:   " << step_deg << " deg\n";
    std::cout << "Roll limit:  +/-" << roll_limit_deg << " deg\n";
    std::cout << "Pitch limit: +/-" << pitch_limit_deg << " deg\n";
    std::cout << "----------------------------------------\n";
    std::cout << "a : roll  +\n";
    std::cout << "d : roll  -\n";
    std::cout << "w : pitch +\n";
    std::cout << "s : pitch -\n";
    std::cout << "r : reset center\n";
    std::cout << "q : quit\n";
    std::cout << "========================================\n";
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string dev = "/dev/ttyS7";
    if (argc >= 2) {
        dev = argv[1];
    }

    const double step_deg = 2.0;
    const double roll_limit_deg  = 30.0;
    const double pitch_limit_deg = 30.0;

    double roll_deg  = 0.0;
    double pitch_deg = 0.0;

    int fd = openSerial(dev);
    if (fd < 0) {
        return 1;
    }

    KeyboardRawMode keyboard_guard;
    printHelp(dev, step_deg, roll_limit_deg, pitch_limit_deg);

    bool manual_cmd_sent = false;
    uint8_t trig = 0;
    int loop_count = 0;

    while (g_running) {
        char key = 0;
        if (readKeyNonBlocking(key, 0)) {
            if (key == 'q' || key == 'Q') {
                g_running = false;
            } else if (key == 'a' || key == 'A') {
                roll_deg += step_deg;
                roll_deg = clampValue(roll_deg, -roll_limit_deg, roll_limit_deg);
                std::cout << "\nSet roll  = " << roll_deg << " deg\n";
            } else if (key == 'd' || key == 'D') {
                roll_deg -= step_deg;
                roll_deg = clampValue(roll_deg, -roll_limit_deg, roll_limit_deg);
                std::cout << "\nSet roll  = " << roll_deg << " deg\n";
            } else if (key == 'w' || key == 'W') {
                pitch_deg += step_deg;
                pitch_deg = clampValue(pitch_deg, -pitch_limit_deg, pitch_limit_deg);
                std::cout << "\nSet pitch = " << pitch_deg << " deg\n";
            } else if (key == 's' || key == 'S') {
                pitch_deg -= step_deg;
                pitch_deg = clampValue(pitch_deg, -pitch_limit_deg, pitch_limit_deg);
                std::cout << "\nSet pitch = " << pitch_deg << " deg\n";
            } else if (key == 'r' || key == 'R') {
                roll_deg = 0.0;
                pitch_deg = 0.0;
                std::cout << "\nReset to center.\n";
            }
        }

        auto tx = buildTxPacket(!manual_cmd_sent, trig, roll_deg, pitch_deg);

        if (!writeAll(fd, tx.data(), tx.size())) {
            close(fd);
            return 1;
        }

        if (!manual_cmd_sent) {
            manual_cmd_sent = true;
            trig = (trig + 1) & 0x07;
        }

        std::vector<uint8_t> rx;
        while (tryReadOneFrame(fd, rx, 1)) {
            if (loop_count % 5 == 0) {
                printRxBrief(rx);
            }
        }

        ++loop_count;
        usleep(20000); // 50Hz
    }

    close(fd);
    std::cout << "\nExit.\n";
    return 0;
}
