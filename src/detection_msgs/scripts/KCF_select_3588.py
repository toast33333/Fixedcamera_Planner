import cv2
import numpy as np
import time

class KCFRK3588Tracker:
    def __init__(self):
        # 1. 初始化USB摄像头（RK3588上USB摄像头默认设备为/dev/video0）
        self.cap = cv2.VideoCapture(11, cv2.CAP_V4L2)  # 强制使用V4L2驱动（RK3588推荐）
        
        # 检查摄像头是否打开成功
        if not self.cap.isOpened():
            raise IOError("无法打开USB摄像头，请检查：\n1. 摄像头是否插好\n2. 权限是否正确（sudo usermod -aG video $USER）\n3. 设备索引是否正确（尝试修改为1）")
        
        # 2. 设置摄像头分辨率（RK3588优化：640x480兼顾性能和清晰度）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 设置帧率为30（RK3588 USB摄像头常见支持帧率）
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 3. 跟踪相关变量
        self.tracker = None
        self.bbox = None  # (x, y, w, h)
        self.tracking = False
        self.selecting = False
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        
        # 4. 性能统计（可选，用于RK3588上调试）
        self.fps_counter = 0
        self.fps_start = time.time()

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标框选逻辑（与原代码一致，适配RK3588的显示交互）"""
        if not self.tracking:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.selecting = True
                self.roi_start = (x, y)
                self.roi_end = (x, y)
            elif event == cv2.EVENT_MOUSEMOVE and self.selecting:
                self.roi_end = (x, y)
            elif event == cv2.EVENT_LBUTTONUP:
                self.selecting = False
                x1 = min(self.roi_start[0], self.roi_end[0])
                y1 = min(self.roi_start[1], self.roi_end[1])
                x2 = max(self.roi_start[0], self.roi_end[0])
                y2 = max(self.roi_start[1], self.roi_end[1])
                w, h = x2 - x1, y2 - y1
                if w > 20 and h > 20:  # 过滤小框
                    self.bbox = (x1, y1, w, h)
                    print(f"框选完成，初始框：{self.bbox}")
                    # 初始化KCF跟踪器（使用当前帧）
                    ret, frame = self.cap.read()
                    if ret:
                        self.tracker = cv2.legacy.TrackerKCF_create()
                        self.tracker.init(frame, self.bbox)
                        self.tracking = True

    def run(self):
        # 创建窗口（RK3588支持HDMI显示时有效）
        cv2.namedWindow("RK3588 KCF Tracking", cv2.WINDOW_NORMAL)  # 允许窗口缩放
        cv2.resizeWindow("RK3588 KCF Tracking", 640, 640)  # 匹配摄像头分辨率
        cv2.setMouseCallback("RK3588 KCF Tracking", self.mouse_callback)
        
        print("操作说明：")
        print("1. 拖动鼠标框选目标，松开开始跟踪")
        print("2. 按 'r' 重新框选，按 'q' 退出")
        print("3. 按 'f' 显示当前帧率")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("摄像头读取失败，退出")
                break
            
            frame_copy = frame.copy()
            self.fps_counter += 1  # 帧率统计
            
            # 框选阶段
            if self.selecting:
                cv2.rectangle(frame_copy, self.roi_start, self.roi_end, (0, 255, 0), 2)
                cv2.putText(frame_copy, "框选目标...", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 跟踪阶段
            elif self.tracking:
                success, bbox = self.tracker.update(frame)
                if success:
                    x, y, w, h = [int(v) for v in bbox]
                    cv2.rectangle(frame_copy, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(frame_copy, f"ID:1 (x:{x}, y:{y})", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.putText(frame_copy, "跟踪丢失！按'r'重选", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 未开始跟踪
            else:
                cv2.putText(frame_copy, "请框选目标开始跟踪", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # 显示帧率（按'f'触发）
            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):
                fps = self.fps_counter / (time.time() - self.fps_start)
                print(f"当前帧率：{fps:.1f} FPS")
            
            # 其他按键控制
            if key == ord('q'):
                print("退出程序")
                break
            elif key == ord('r'):
                print("重新框选目标")
                self.tracking = False
                self.tracker = None
            
            # 显示画面（RK3588 HDMI输出）
            cv2.imshow("RK3588 KCF Tracking", frame_copy)
        
        # 释放资源（RK3588需确保资源释放，避免占用摄像头）
        self.cap.release()
        cv2.destroyAllWindows()
        print("资源已释放")

if __name__ == "__main__":
    try:
        tracker = KCFRK3588Tracker()
        tracker.run()
    except Exception as e:
        print(f"运行错误：{e}")