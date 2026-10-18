#!/usr/bin/env python3
"""RK3588 ROS1 camera + RKNN YOLO + annotated image publisher.

Place this file next to ros1Func.py and rknnpool.py on the RK3588 board.
It keeps the detector as the only USB camera owner, then publishes the
annotated frame for web_video_server.
"""

from __future__ import annotations

import os
import logging
import socket
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import rospy
from detection_msgs.msg import Detection
from rknnpool import rknnPoolExecutor

try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
except ImportError:
    CvBridge = None
    Image = None

from ros1Func import myFunc


DEFAULT_CAMERA_ID = 11
DEFAULT_FRAME_WIDTH = 640
DEFAULT_FRAME_HEIGHT = 480
DEFAULT_REQUEST_FPS = 60
DEFAULT_STREAM_FPS = 10.0
DEFAULT_STREAM_QUALITY = 45
DEFAULT_STREAM_WIDTH = 512
DEFAULT_STREAM_HEIGHT = 384
DEFAULT_DIRECT_STREAM_PORT = 8080
DEFAULT_MODEL_PATH = "/home/firefly/catkin_ws/src/detection_msgs/rknnModel/yolov8-NewDrone.rknn"
DEFAULT_TPES = 3


def normalize_python_logging_levels() -> None:
    """Accept quoted ROS logging levels sometimes produced by mixed conda/ROS envs."""
    levels = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL,
    }
    for name, level in levels.items():
        logging.addLevelName(level, name)
        if hasattr(logging, "_nameToLevel"):
            logging._nameToLevel[f"'{name}'"] = level
            logging._nameToLevel[f'"{name}"'] = level


def env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return default


def env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return default


class LatestJpegBuffer:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._seq = 0
        self._jpeg = b""
        self._timestamp = 0.0

    def update(self, frame, quality: int, width: int = 0, height: int = 0,
               capture_stamp=None) -> bool:
        quality = max(30, min(int(quality), 90))
        if width > 0 and height > 0 and (frame.shape[1] != width or frame.shape[0] != height):
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return False
        with self._condition:
            self._seq += 1
            self._jpeg = encoded.tobytes()
            self._timestamp = (capture_stamp.to_sec()
                               if capture_stamp is not None else time.time())
            self._condition.notify_all()
        return True

    def wait_next(self, last_seq: int, timeout: float = 1.0):
        with self._condition:
            if self._seq == last_seq:
                self._condition.wait(timeout=timeout)
            return self._seq, self._timestamp, self._jpeg


class AegisMjpegHandler(BaseHTTPRequestHandler):
    server_version = "AegisEdgeMJPEG/1.0"

    def log_message(self, fmt, *args):
        rospy.logdebug("mjpeg: " + fmt, *args)

    def do_GET(self):
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        path = urlparse(self.path).path
        if path not in {"/", "/stream"}:
            self.send_error(404)
            return
        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Aegis edge MJPEG server. Use /stream\n")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_seq = 0
        while not rospy.is_shutdown():
            seq, ts, jpeg = self.server.frame_buffer.wait_next(last_seq, timeout=1.0)
            if seq == last_seq or not jpeg:
                continue
            last_seq = seq
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"X-Timestamp: {ts:.6f}\r\n".encode("ascii"))
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break


class AegisMjpegServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, frame_buffer: LatestJpegBuffer, stream_fps: float):
        super().__init__(server_address, AegisMjpegHandler)
        self.frame_buffer = frame_buffer
        self.stream_fps = stream_fps


def start_direct_mjpeg_server(port: int, frame_buffer: LatestJpegBuffer, stream_fps: float):
    server = AegisMjpegServer(("0.0.0.0", port), frame_buffer, stream_fps)
    thread = threading.Thread(target=server.serve_forever, name="aegis-mjpeg-server", daemon=True)
    thread.start()
    rospy.loginfo("direct MJPEG stream: http://0.0.0.0:%s/stream", port)
    return server


def open_camera(camera_id: int, width: int, height: int, request_fps: int):
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, request_fps)

    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera id={camera_id}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    rospy.loginfo(
        "camera opened: id=%s resolution=%sx%s fps=%.2f",
        camera_id,
        actual_w,
        actual_h,
        actual_fps,
    )
    return cap


def publish_image(
    image_pub,
    bridge: CvBridge,
    frame,
    frame_seq: int,
    frame_id: str,
    capture_stamp,
) -> None:
    msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    msg.header.stamp = capture_stamp
    msg.header.seq = frame_seq
    msg.header.frame_id = frame_id
    image_pub.publish(msg)


def main() -> None:
    normalize_python_logging_levels()
    rospy.init_node("aegis_edge_yolo_stream", anonymous=False)

    camera_id = int(rospy.get_param("~camera_id", env_int("AEGIS_EDGE_CAMERA_ID", DEFAULT_CAMERA_ID)))
    frame_width = int(rospy.get_param("~frame_width", env_int("AEGIS_EDGE_FRAME_WIDTH", DEFAULT_FRAME_WIDTH)))
    frame_height = int(rospy.get_param("~frame_height", env_int("AEGIS_EDGE_FRAME_HEIGHT", DEFAULT_FRAME_HEIGHT)))
    request_fps = int(rospy.get_param("~request_fps", env_int("AEGIS_EDGE_REQUEST_FPS", DEFAULT_REQUEST_FPS)))
    stream_fps = float(rospy.get_param("~stream_fps", env_float("AEGIS_EDGE_STREAM_FPS", DEFAULT_STREAM_FPS)))
    stream_quality = int(rospy.get_param("~stream_quality", env_int("AEGIS_EDGE_STREAM_QUALITY", DEFAULT_STREAM_QUALITY)))
    stream_width = int(rospy.get_param("~stream_width", env_int("AEGIS_EDGE_STREAM_WIDTH", DEFAULT_STREAM_WIDTH)))
    stream_height = int(rospy.get_param("~stream_height", env_int("AEGIS_EDGE_STREAM_HEIGHT", DEFAULT_STREAM_HEIGHT)))
    direct_stream = bool(int(rospy.get_param("~direct_stream", env_int("AEGIS_EDGE_DIRECT_STREAM", 1))))
    direct_stream_port = int(
        rospy.get_param("~direct_stream_port", env_int("AEGIS_EDGE_DIRECT_STREAM_PORT", DEFAULT_DIRECT_STREAM_PORT))
    )
    publish_ros_image = bool(int(rospy.get_param("~publish_ros_image", env_int("AEGIS_EDGE_PUBLISH_ROS_IMAGE", 0))))
    model_path = str(rospy.get_param("~model_path", os.getenv("AEGIS_EDGE_MODEL_PATH", DEFAULT_MODEL_PATH)))
    tpes = int(rospy.get_param("~tpes", env_int("AEGIS_EDGE_TPES", DEFAULT_TPES)))
    display = bool(int(os.getenv("AEGIS_EDGE_DISPLAY", "0")))

    detection_topic = str(rospy.get_param("~detection_topic", "/object_detections"))
    image_topic = str(rospy.get_param("~image_topic", "/aegis/camera/annotated"))
    frame_id = str(rospy.get_param("~frame_id", "aegis_camera"))

    detection_pub = rospy.Publisher(detection_topic, Detection, queue_size=10)
    if publish_ros_image and (CvBridge is None or Image is None):
        raise RuntimeError("publish_ros_image requires cv_bridge and sensor_msgs")
    image_pub = rospy.Publisher(image_topic, Image, queue_size=1) if publish_ros_image else None
    bridge = CvBridge() if publish_ros_image else None
    frame_buffer = LatestJpegBuffer()
    mjpeg_server = start_direct_mjpeg_server(direct_stream_port, frame_buffer, stream_fps) if direct_stream else None

    cap = open_camera(camera_id, frame_width, frame_height, request_fps)

    def wrapped_func(rknn_lite, frame_packet):
        frame, capture_stamp, frame_seq = frame_packet
        processed_frame = myFunc(
            rknn_lite, frame, detection_pub, capture_stamp, frame_id)
        return processed_frame, capture_stamp, frame_seq

    pool = rknnPoolExecutor(rknnModel=model_path, TPEs=tpes, func=wrapped_func)

    frames = 0
    capture_seq = 0
    loop_time = time.time()
    last_stream_publish = 0.0
    stream_period = 1.0 / max(stream_fps, 1.0)

    try:
        for _ in range(tpes + 1):
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("camera pre-read failed")
            capture_seq += 1
            pool.put((frame, rospy.Time.now(), capture_seq))

        rospy.loginfo("publishing detections on %s", detection_topic)
        if image_pub is not None:
            rospy.loginfo("publishing ROS annotated images on %s at %.1f FPS", image_topic, stream_fps)
        if direct_stream:
            rospy.loginfo(
                "publishing direct MJPEG at %.1f FPS, quality=%s, stream=%sx%s",
                stream_fps,
                stream_quality,
                stream_width,
                stream_height,
            )

        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret or frame is None:
                rospy.logwarn("camera read failed")
                time.sleep(0.03)
                continue

            frames += 1
            capture_seq += 1
            pool.put((frame, rospy.Time.now(), capture_seq))
            processed_packet, flag = pool.get()
            if flag is False:
                raise RuntimeError("RKNN inference worker failed")
            processed_frame, capture_stamp, processed_seq = processed_packet

            now = time.time()
            if now - last_stream_publish >= stream_period:
                if direct_stream:
                    frame_buffer.update(
                        processed_frame, stream_quality, stream_width,
                        stream_height, capture_stamp)
                if image_pub is not None:
                    publish_image(
                        image_pub, bridge, processed_frame, processed_seq,
                        frame_id, capture_stamp)
                last_stream_publish = now

            if display:
                cv2.imshow("Aegis Edge YOLO Stream", processed_frame)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break

            if frames % 30 == 0:
                dt = time.time() - loop_time
                if dt > 0:
                    rospy.loginfo("30-frame average FPS: %.2f", 30.0 / dt)
                loop_time = time.time()
    finally:
        cap.release()
        if mjpeg_server is not None:
            mjpeg_server.shutdown()
            mjpeg_server.server_close()
        pool.release()
        if display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
