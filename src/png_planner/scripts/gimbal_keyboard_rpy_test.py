#!/usr/bin/env python3
import argparse
import errno
import os
import select
import signal
import sys
import termios
import time
import tty


running = True


def signal_handler(_signum, _frame):
    global running
    running = False


def calculate_crc16(data):
    crc = 0
    crc_table = (
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    )

    for value in data:
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (value >> 4)]
        da = crc >> 12
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (value & 0x0F)]
    return crc & 0xFFFF


def put_int16_le(buf, idx, value):
    value = int(value) & 0xFFFF
    buf[idx] = value & 0xFF
    buf[idx + 1] = (value >> 8) & 0xFF


def get_int16_le(buf, idx):
    value = buf[idx] | (buf[idx + 1] << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def set_serial_raw(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    baud_const = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }.get(baud)
    if baud_const is None:
        raise ValueError("unsupported baud: {}".format(baud))

    attrs[4] = baud_const
    attrs[5] = baud_const
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def open_serial(dev, baud):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    set_serial_raw(fd, baud)
    return fd


def write_all(fd, data):
    sent = 0
    while sent < len(data):
        try:
            sent += os.write(fd, data[sent:])
        except OSError as exc:
            if exc.errno == errno.EINTR:
                continue
            raise
    termios.tcdrain(fd)


def build_tx_packet(send_manual_cmd, trig, roll_deg, pitch_deg, yaw_deg):
    pkt = bytearray(40)
    pkt[0] = 0xA9
    pkt[1] = 0x5B

    cmd_value = 4 if send_manual_cmd else 0
    pkt[2] = ((cmd_value & 0x1F) << 3) | (trig & 0x07)
    pkt[3] = 0x00

    axis_ctrl = (0 << 6) | (1 << 4) | (0 << 3)
    pkt[4] = axis_ctrl
    put_int16_le(pkt, 5, int(round(roll_deg * 100.0)))

    pkt[7] = axis_ctrl
    put_int16_le(pkt, 8, int(round(pitch_deg * 100.0)))

    pkt[10] = axis_ctrl
    put_int16_le(pkt, 11, int(round(yaw_deg * 100.0)))

    pkt[13] = 0x00

    crc = calculate_crc16(pkt[:38])
    pkt[38] = (crc >> 8) & 0xFF
    pkt[39] = crc & 0xFF
    return bytes(pkt)


class KeyboardRawMode:
    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.old_attrs = None
        if self.enabled:
            self.old_attrs = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

    def close(self):
        if self.enabled and self.old_attrs is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self.old_attrs)
            self.old_attrs = None

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.close()


def read_key_nonblocking():
    if not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return None
    return os.read(sys.stdin.fileno(), 1).decode(errors="ignore")


def try_read_frames(fd, cache):
    frames = []
    readable, _, _ = select.select([fd], [], [], 0)
    if readable:
        data = os.read(fd, 256)
        cache.extend(data)

    while len(cache) >= 2:
        if not (cache[0] == 0xB5 and cache[1] == 0x9A):
            del cache[0]
            continue
        if len(cache) < 26:
            break
        frame = bytes(cache[:26])
        crc = calculate_crc16(frame[:24])
        if frame[24] == ((crc >> 8) & 0xFF) and frame[25] == (crc & 0xFF):
            frames.append(frame)
            del cache[:26]
        else:
            del cache[0]
    return frames


def parse_rx_frame(frame):
    fw_ver = frame[2]
    hw_err = frame[3]
    flag = frame[4]
    inv_flag = flag & 0x01
    gbc_stat = (flag >> 1) & 0x07
    tca_flag = (flag >> 4) & 0x01
    cmd = frame[5]
    cmd_stat = cmd & 0x07
    cmd_value = (cmd >> 3) & 0x1F

    cam_rate_pitch = get_int16_le(frame, 6) * 0.1
    cam_rate_roll = get_int16_le(frame, 8) * 0.1
    cam_rate_yaw = get_int16_le(frame, 10) * 0.1
    cam_roll = get_int16_le(frame, 12) * 0.01
    cam_pitch = get_int16_le(frame, 14) * 0.01
    cam_yaw = get_int16_le(frame, 16) * 0.01
    mang_pitch = get_int16_le(frame, 18) * 0.01
    mang_roll = get_int16_le(frame, 20) * 0.01
    mang_yaw = get_int16_le(frame, 22) * 0.01

    return {
        "fw_ver": fw_ver,
        "hw_err": hw_err,
        "gbc_stat": gbc_stat,
        "tca_flag": tca_flag,
        "inv_flag": inv_flag,
        "cmd_value": cmd_value,
        "cmd_stat": cmd_stat,
        "cam_rate_pitch": cam_rate_pitch,
        "cam_rate_roll": cam_rate_roll,
        "cam_rate_yaw": cam_rate_yaw,
        "cam_roll": cam_roll,
        "cam_pitch": cam_pitch,
        "cam_yaw": cam_yaw,
        "mang_pitch": mang_pitch,
        "mang_roll": mang_roll,
        "mang_yaw": mang_yaw,
    }


def print_rx_brief(state, roll_cmd, pitch_cmd, yaw_cmd):
    text = (
        "\rfw={} hw_err={} stat={} tca={} inv={} cmd={} cmd_stat={}"
        " | cmd[r,p,y]=[{:.2f},{:.2f},{:.2f}] deg"
        " | cam_rate[p,r,y]=[{:.1f},{:.1f},{:.1f}] deg/s"
        " | cam_att[r,p,y]=[{:.2f},{:.2f},{:.2f}] deg"
        " | mang[p,r,y]=[{:.2f},{:.2f},{:.2f}] deg     "
    ).format(
        state["fw_ver"],
        state["hw_err"],
        state["gbc_stat"],
        state["tca_flag"],
        state["inv_flag"],
        state["cmd_value"],
        state["cmd_stat"],
        roll_cmd,
        pitch_cmd,
        yaw_cmd,
        state["cam_rate_pitch"],
        state["cam_rate_roll"],
        state["cam_rate_yaw"],
        state["cam_roll"],
        state["cam_pitch"],
        state["cam_yaw"],
        state["mang_pitch"],
        state["mang_roll"],
        state["mang_yaw"],
    )
    sys.stdout.write(text)
    sys.stdout.flush()


def print_help(args):
    print("========================================")
    print("Gimbal Roll/Pitch/Yaw Keyboard Test")
    print("Serial: {}".format(args.dev))
    print("Baud:   {}".format(args.baud))
    print("Step:   {} deg".format(args.step_deg))
    print("Roll limit:  +/-{} deg".format(args.roll_limit_deg))
    print("Pitch limit: +/-{} deg".format(args.pitch_limit_deg))
    print("Yaw limit:   +/-{} deg".format(args.yaw_limit_deg))
    print("----------------------------------------")
    print("a/d : roll  +/-")
    print("w/s : pitch +/-")
    print("j/l : yaw   -/+")
    print("r   : reset center")
    print("q   : quit")
    print("========================================")


def parse_args():
    parser = argparse.ArgumentParser(description="Keyboard RPY control for gimbal serial protocol.")
    parser.add_argument("dev", nargs="?", default="/dev/ttyS7", help="serial device")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--step-deg", type=float, default=2.0, help="keyboard step in degrees")
    parser.add_argument("--roll-limit-deg", type=float, default=30.0)
    parser.add_argument("--pitch-limit-deg", type=float, default=30.0)
    parser.add_argument("--yaw-limit-deg", type=float, default=180.0)
    parser.add_argument("--rate-hz", type=float, default=50.0, help="send rate")
    return parser.parse_args()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    args = parse_args()
    period = 1.0 / max(args.rate_hz, 1.0)

    roll_deg = 0.0
    pitch_deg = 0.0
    yaw_deg = 0.0

    fd = open_serial(args.dev, args.baud)
    rx_cache = bytearray()

    print_help(args)
    manual_cmd_sent = False
    trig = 0
    latest_rx_state = None
    last_print_time = 0.0

    try:
        with KeyboardRawMode():
            while running:
                key = read_key_nonblocking()
                if key in ("q", "Q"):
                    break
                if key in ("a", "A"):
                    roll_deg = clamp(roll_deg + args.step_deg, -args.roll_limit_deg, args.roll_limit_deg)
                    print("\nSet roll  = {:.2f} deg".format(roll_deg))
                elif key in ("d", "D"):
                    roll_deg = clamp(roll_deg - args.step_deg, -args.roll_limit_deg, args.roll_limit_deg)
                    print("\nSet roll  = {:.2f} deg".format(roll_deg))
                elif key in ("w", "W"):
                    pitch_deg = clamp(pitch_deg + args.step_deg, -args.pitch_limit_deg, args.pitch_limit_deg)
                    print("\nSet pitch = {:.2f} deg".format(pitch_deg))
                elif key in ("s", "S"):
                    pitch_deg = clamp(pitch_deg - args.step_deg, -args.pitch_limit_deg, args.pitch_limit_deg)
                    print("\nSet pitch = {:.2f} deg".format(pitch_deg))
                elif key in ("l", "L"):
                    yaw_deg = clamp(yaw_deg + args.step_deg, -args.yaw_limit_deg, args.yaw_limit_deg)
                    print("\nSet yaw   = {:.2f} deg".format(yaw_deg))
                elif key in ("j", "J"):
                    yaw_deg = clamp(yaw_deg - args.step_deg, -args.yaw_limit_deg, args.yaw_limit_deg)
                    print("\nSet yaw   = {:.2f} deg".format(yaw_deg))
                elif key in ("r", "R"):
                    roll_deg = 0.0
                    pitch_deg = 0.0
                    yaw_deg = 0.0
                    print("\nReset to center.")

                tx = build_tx_packet(not manual_cmd_sent, trig, roll_deg, pitch_deg, yaw_deg)
                write_all(fd, tx)

                if not manual_cmd_sent:
                    manual_cmd_sent = True
                    trig = (trig + 1) & 0x07

                for frame in try_read_frames(fd, rx_cache):
                    latest_rx_state = parse_rx_frame(frame)

                now = time.time()
                if latest_rx_state is not None and now - last_print_time >= 0.1:
                    print_rx_brief(latest_rx_state, roll_deg, pitch_deg, yaw_deg)
                    last_print_time = now
                time.sleep(period)
    finally:
        os.close(fd)
        print("\nExit.")


if __name__ == "__main__":
    main()
