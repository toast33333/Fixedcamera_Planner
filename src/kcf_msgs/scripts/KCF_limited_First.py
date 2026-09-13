import cv2
import numpy as np
import time
import rospy
from kcf_msgs.msg import Bbox

class KCFRK3588Tracker:
    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node('kcf_tracker_node', anonymous=True)
        self.bbox_pub = rospy.Publisher('/object_kcf', Bbox, queue_size=10)
        rospy.loginfo("ROS节点初始化完成，话题：/object_kcf")

        # 2. 初始化USB摄像头
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开USB摄像头，请检查连接和权限！")
            raise IOError("摄像头初始化失败")
        
        # 设置分辨率
        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 120)    

        self.center_x = self.width / 2.0
        self.center_y = self.height / 2.0
        rospy.loginfo(f"图像中心坐标：({self.center_x}, {self.center_y})")

        # 3. 跟踪相关变量
        self.tracker = None
        self.bbox = None  # (x1, y1, w, h)
        self.tracking = False
        self.selecting = False
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.track_id = 1

        # 【新增】跟踪窗口最大尺寸限制（核心参数，可根据需求调整）
        self.max_window_width = 240   # 最大宽度（像素）
        self.max_window_height = 240  # 最大高度（像素）
        rospy.loginfo(f"跟踪窗口最大限制：{self.max_window_width}x{self.max_window_height}像素")

        # 4. 帧率统计
        self.fps_counter = 0
        self.fps_start = time.time()

    def mouse_callback(self, event, x, y, flags, param):
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
                if w > 20 and h > 20:
                    # 【修改】初始框选时限制窗口大小
                    # 计算原始框中心
                    center_x_initial = x1 + w / 2.0
                    center_y_initial = y1 + h / 2.0

                    # 限制宽高不超过最大值，保持中心不变
                    w_limited = min(w, self.max_window_width)
                    h_limited = min(h, self.max_window_height)

                    # 重新计算左上角坐标（保证中心不变）
                    x1_limited = int(center_x_initial - w_limited / 2.0)
                    y1_limited = int(center_y_initial - h_limited / 2.0)

                    self.bbox = (x1_limited, y1_limited, w_limited, h_limited)
                    rospy.loginfo(f"框选目标完成（已限制大小），初始框：{self.bbox}")

                    # 初始化跟踪器
                    ret, frame = self.cap.read()
                    if ret:
                        self.tracker = cv2.legacy.TrackerKCF_create()
                        self.tracker.init(frame, self.bbox)
                        self.tracking = True

    def run(self):
        cv2.namedWindow("RK3588 KCF Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RK3588 KCF Tracking", 640, 480)
        cv2.setMouseCallback("RK3588 KCF Tracking", self.mouse_callback)
        
        rospy.loginfo("操作说明：\n1. 框选目标开始跟踪\n2. 按'r'重新框选，'q'退出\n3. 按'f'查看帧率")

        rate = rospy.Rate(120)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("摄像头读取失败，退出")
                break
            
            frame_copy = frame.copy()
            self.fps_counter += 1

            if self.selecting:
                cv2.rectangle(frame_copy, self.roi_start, self.roi_end, (0, 255, 0), 2)
                cv2.putText(frame_copy, "框选目标...", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            elif self.tracking:
                success, bbox = self.tracker.update(frame)
                if success:
                    x_old, y_old, w_old, h_old = [int(v) for v in bbox]

                    # 【修改】跟踪过程中限制窗口大小
                    # 计算当前框中心
                    center_x_track = x_old + w_old / 2.0
                    center_y_track = y_old + h_old / 2.0

                    # 限制宽高不超过最大值
                    w_limited = min(w_old, self.max_window_width)
                    h_limited = min(h_old, self.max_window_height)

                    # 重新计算左上角坐标（保持中心不变）
                    x_old = int(center_x_track - w_limited / 2.0)
                    y_old = int(center_y_track - h_limited / 2.0)
                    w_old, h_old = w_limited, h_limited

                    # 后续坐标转换和绘制逻辑不变
                    x1_old, y1_old = x_old, y_old
                    x2_old, y2_old = x_old + w_old, y_old + h_old

                    x1_new = x1_old - self.center_x
                    y1_new = self.center_y - y1_old
                    x2_new = x2_old - self.center_x
                    y2_new = self.center_y - y2_old

                    cv2.rectangle(frame_copy, (x1_old, y1_old), (x2_old, y2_old), (0, 0, 255), 2)
                    cv2.putText(frame_copy, 
                               f"新坐标: ({x1_new:.1f}, {y1_new:.1f}) ~ ({x2_new:.1f}, {y2_new:.1f})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    bbox_msg = Bbox()
                    bbox_msg.x1 = x1_new
                    bbox_msg.y1 = y1_new
                    bbox_msg.x2 = x2_new
                    bbox_msg.y2 = y2_new
                    bbox_msg.score = 1
                    bbox_msg.class_name = "UAV"
                    bbox_msg.track_id = self.track_id
                    bbox_msg.is_tracking = True
                    self.bbox_pub.publish(bbox_msg)
                else:
                    bbox_msg = Bbox()
                    bbox_msg.score = 1
                    bbox_msg.class_name = "UAV"
                    bbox_msg.is_tracking = False
                    self.bbox_pub.publish(bbox_msg)
                    cv2.putText(frame_copy, "跟踪丢失！按'r'重选", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            else:
                bbox_msg = Bbox()
                bbox_msg.score = 1
                bbox_msg.class_name = "UAV"
                bbox_msg.is_tracking = False
                self.bbox_pub.publish(bbox_msg)
                cv2.putText(frame_copy, "请框选目标开始跟踪", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):
                fps = self.fps_counter / (time.time() - self.fps_start)
                rospy.loginfo(f"当前帧率：{fps:.1f} FPS")
            if key == ord('q'):
                rospy.loginfo("手动退出程序")
                break
            elif key == ord('r'):
                rospy.loginfo("重新框选目标")
                self.tracking = False
                self.tracker = None
            
            cv2.imshow("RK3588 KCF Tracking", frame_copy)
            rate.sleep()
        
        self.cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("资源已释放，节点退出")

if __name__ == "__main__":
    try:
        tracker = KCFRK3588Tracker()
        tracker.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"运行错误：{e}")
