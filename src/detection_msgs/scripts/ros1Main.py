#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RK3588 + ROS1 + RKNN YOLO camera preview / manual recording

Keyboard:
    r : start recording
    s : stop recording and save
    q : quit (if recording, save before exit)

By default the saved video contains the YOLO detection boxes, because the frame
returned by myFunc() is the annotated frame.

Adjust CAMERA_ID / MODEL_PATH if your device paths are different.
"""

import os
import cv2
import time
import logging
from datetime import datetime

import rospy
from detection_msgs.msg import Detection
from rknnpool import rknnPoolExecutor

# Uploaded file name is ros1Func.py.
# If your board still uses rosFunc.py, change this line to:
# from rosFunc import myFunc
from ros1Func import myFunc


# ------------------------- Configuration -------------------------
CAMERA_ID = 40
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
REQUEST_FPS = 60

MODEL_PATH = "/home/firefly/catkin_ws/src/detection_msgs/rknnModel/yolov8-NewDrone.rknn"
TPES = 3

OUTPUT_DIR = "./recordings"
# True: 保存带检测框的视频；False: 保存摄像头原始画面
SAVE_ANNOTATED = False
WINDOW_NAME = "RK3588 - R:start  S:stop  Q:quit"
# ----------------------------------------------------------------


def open_camera(camera_id):
    """Open USB camera and apply commonly used settings."""
    cap = cv2.VideoCapture(camera_id)

    # For many USB cameras on RK3588, MJPG helps achieve higher FPS.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, REQUEST_FPS)

    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 CAMERA_ID={camera_id}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"[Camera] opened: id={camera_id}, "
          f"resolution={actual_w}x{actual_h}, fps={actual_fps:.2f}")
    return cap


def get_record_fps(cap):
    """Use camera FPS when valid; otherwise use REQUEST_FPS."""
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 1 or fps > 240:
        fps = REQUEST_FPS
    return float(fps)


def create_video_writer(frame, fps):
    """
    Create writer with MP4 first; fall back to MJPG/AVI if MP4 codec
    is unavailable in the OpenCV build on the board.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    h, w = frame.shape[:2]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    mp4_path = os.path.join(OUTPUT_DIR, f"detect_{timestamp}.mp4")
    writer = cv2.VideoWriter(
        mp4_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (w, h)
    )
    if writer.isOpened():
        return writer, mp4_path

    writer.release()

    avi_path = os.path.join(OUTPUT_DIR, f"detect_{timestamp}.avi")
    writer = cv2.VideoWriter(
        avi_path,
        cv2.VideoWriter_fourcc(*"MJPG"),
        fps,
        (w, h)
    )
    if writer.isOpened():
        print("[Record] mp4v 不可用，已自动切换为 MJPG/AVI")
        return writer, avi_path

    writer.release()
    raise RuntimeError("VideoWriter 创建失败，请检查 OpenCV/FFmpeg 编码支持。")


def main():

    logging.addLevelName(10,'DEBUG')
    logging.addLevelName(20, 'INFO')
    logging.addLevelName(30, 'WARNING')
    logging.addLevelName(40, 'ERROR') 
    logging.basicConfig(level=logging.INFO)
    # 1. ROS1
    rospy.init_node("yolo_tracker_publisher", anonymous=True)
    publisher = rospy.Publisher(
        "/object_detections",
        Detection,
        queue_size=10
    )
    rospy.loginfo("ROS1发布节点已启动，话题：/object_detections")

    # 2. Camera
    cap = open_camera(CAMERA_ID)

    # 3. RKNN pool
    def wrapped_func(rknn_lite, frame_packet):
        frame, capture_stamp = frame_packet
        return myFunc(
            rknn_lite, frame, publisher, capture_stamp, "camera_optical")

    pool = rknnPoolExecutor(
        rknnModel=MODEL_PATH,
        TPEs=TPES,
        func=wrapped_func
    )

    # Recording state
    writer = None
    recording = False
    record_path = None
    record_start_time = None

    frames = 0
    loop_time = time.time()
    init_time = time.time()

    try:
        # 4. Pre-fill the inference pool
        if cap.isOpened():
            for _ in range(TPES + 1):
                ret, frame = cap.read()
                if not ret:
                    print("[Error] 摄像头预读取失败")
                    return
                pool.put((frame, rospy.Time.now()))

        print("\n========== 控制说明 ==========")
        print("按 r : 开始录制")
        print("按 s : 停止录制并保存")
        print("按 q : 退出程序（录制中会自动保存）")
        print("==============================\n")

        # 5. Main loop
        while cap.isOpened() and not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                print("[Error] 摄像头读取失败")
                break

            frames += 1

            # Push newest camera frame to RKNN worker pool
            pool.put((frame, rospy.Time.now()))

            # Get processed / annotated frame
            processed_frame, flag = pool.get()
            if flag is False:
                print("[Error] RKNN 推理线程返回失败")
                break

            # Choose what to save:
            #   SAVE_ANNOTATED=True  -> processed frame with YOLO boxes
            #   SAVE_ANNOTATED=False -> raw camera frame
            record_frame = processed_frame if SAVE_ANNOTATED else frame

            if recording and writer is not None:
                writer.write(record_frame)

            # Preview always shows detection result.
            # REC/control text is added only to preview, not to the saved video.
            display_frame = processed_frame.copy()

            if recording:
                elapsed = time.time() - record_start_time
                cv2.circle(display_frame, (25, 28), 9, (0, 0, 255), -1)
                cv2.putText(
                    display_frame,
                    f"REC {elapsed:06.1f}s",
                    (45, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )
            else:
                cv2.putText(
                    display_frame,
                    "R: START RECORD",
                    (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

            cv2.putText(
                display_frame,
                "S: STOP/SAVE   Q: QUIT",
                (15, display_frame.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1
            )

            cv2.imshow(WINDOW_NAME, display_frame)

            key = cv2.waitKey(1) & 0xFF

            # r: start
            if key == ord("r"):
                if not recording:
                    fps = get_record_fps(cap)
                    first_record_frame = processed_frame if SAVE_ANNOTATED else frame
                    writer, record_path = create_video_writer(
                        first_record_frame, fps
                    )
                    recording = True
                    record_start_time = time.time()
                    print(f"[Record] 开始录制: {record_path}")
                    print(f"[Record] 保存 FPS: {fps:.2f}")
                    print("[Record] 视频类型:", "带检测框" if SAVE_ANNOTATED else "原始画面")
                else:
                    print("[Record] 当前已经在录制中")

            # s: stop and save
            elif key == ord("s"):
                if recording:
                    writer.release()
                    writer = None
                    recording = False
                    duration = time.time() - record_start_time
                    print(f"[Record] 已停止并保存: {record_path}")
                    print(f"[Record] 录制时长: {duration:.1f}s")
                    record_path = None
                    record_start_time = None
                else:
                    print("[Record] 当前没有正在录制的视频")

            # q: quit
            elif key == ord("q"):
                print("[System] 收到退出指令")
                break

            # FPS statistics
            if frames % 30 == 0:
                dt = time.time() - loop_time
                if dt > 0:
                    print(f"30帧平均帧率:\t {30 / dt:.2f} 帧")
                loop_time = time.time()

    finally:
        # If q / ROS shutdown happens while recording, save current video cleanly.
        if writer is not None:
            writer.release()
            print(f"[Record] 程序退出，视频已自动保存: {record_path}")

        total_time = time.time() - init_time
        if total_time > 0:
            print(f"总平均帧率\t {frames / total_time:.2f} 帧")

        cap.release()
        cv2.destroyAllWindows()
        pool.release()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
