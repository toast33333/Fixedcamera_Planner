import cv2
import numpy as np
import time
import rospy  # 导入ROS1 Python API
from kcf_msgs.msg import Bbox  # 导入自定义消息

class KCFRK3588Tracker:
    def __init__(self):
        # 1. 初始化ROS节点（必须在摄像头初始化前，避免延迟）
        rospy.init_node('kcf_tracker_node', anonymous=True)  # 节点名：kcf_tracker_node
        self.bbox_pub = rospy.Publisher(
            '/object_kcf',  # 发布的话题名
            Bbox,                # 消息类型
            queue_size=10        # 消息队列大小
        )
        rospy.loginfo("ROS节点初始化完成，话题：/object_kcf")

        # 2. 初始化USB摄像头（RK3588适配）
        self.cap = cv2.VideoCapture(40, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开USB摄像头，请检查连接和权限！")
            raise IOError("摄像头初始化失败")
        
        # 设置分辨率（平衡性能和精度）
        self.width = 640  # 图像宽度
        self.height = 480  # 图像高度
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 120)    

        # 计算图像中心坐标（关键：新坐标系统的原点）
        self.center_x = self.width / 2.0
        self.center_y = self.height / 2.0
        rospy.loginfo(f"图像中心坐标：({self.center_x}, {self.center_y})")

        # 3. 跟踪相关变量
        self.tracker = None
        self.bbox = None  # (x1, y1, w, h) -> 转换为(x1, y1, x2, y2)发布
        self.tracking = False
        self.selecting = False
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.track_id = 1  # KCF单目标跟踪，固定ID为1

        # 4. 帧率统计
        self.fps_counter = 0
        self.fps_start = time.time()

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标框选逻辑（不变）"""
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
                    self.bbox = (x1, y1, w, h)  # (x, y, w, h)
                    rospy.loginfo(f"框选目标完成，初始框：{self.bbox}")
                    # 初始化KCF跟踪器（适配OpenCV 4.x的legacy模块）
                    ret, frame = self.cap.read()
                    if ret:
                
                        self.tracker = cv2.legacy.TrackerKCF_create()
                        self.tracker.init(frame, self.bbox)
                        self.tracking = True

    def run(self):
        # 创建显示窗口
        cv2.namedWindow("RK3588 KCF Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RK3588 KCF Tracking", 640, 480)
        cv2.setMouseCallback("RK3588 KCF Tracking", self.mouse_callback)
        
        rospy.loginfo("操作说明：\n1. 框选目标开始跟踪\n2. 按'r'重新框选，'q'退出\n3. 按'f'查看帧率")

        # ROS循环频率控制（与摄像头帧率匹配）
        rate = rospy.Rate(120) #30

        while not rospy.is_shutdown():  # 用ROS中断信号替代原循环条件
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("摄像头读取失败，退出")
                break
            
            frame_copy = frame.copy()
            self.fps_counter += 1

            # 框选阶段
            if self.selecting:
                cv2.rectangle(frame_copy, self.roi_start, self.roi_end, (0, 255, 0), 2)
                cv2.putText(frame_copy, "框选目标...", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 跟踪阶段（核心：发布ROS消息）
            elif self.tracking:
                success, bbox = self.tracker.update(frame)
                if success:
                    x_old, y_old, w_old, h_old = [int(v) for v in bbox]
                    x1_old, y1_old = x_old, y_old  # 原始左上角
                    x2_old, y2_old = x_old + w_old, y_old + h_old  # 原始右下角

                    # 关键：将原始坐标转换为新坐标系统（中心为原点）
                    x1_new = x1_old - self.center_x  # 左上角x新坐标
                    y1_new = self.center_y - y1_old  # 左上角y新坐标（向上为正）
                    x2_new = x2_old - self.center_x  # 右下角x新坐标
                    y2_new = self.center_y - y2_old  # 右下角y新坐标（向上为正）

                    # 绘制跟踪框（仍用原始坐标，因为显示基于图像像素）
                    cv2.rectangle(frame_copy, (x1_old, y1_old), (x2_old, y2_old), (0, 0, 255), 2)
                    # 显示新坐标（方便调试）
                    cv2.putText(frame_copy, 
                               f"新坐标: ({x1_new:.1f}, {y1_new:.1f}) ~ ({x2_new:.1f}, {y2_new:.1f})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # 封装ROS消息并发布
                    bbox_msg = Bbox()
                    bbox_msg.x1 = x1_new
                    bbox_msg.y1 = y1_new
                    bbox_msg.x2 = x2_new
                    bbox_msg.y2 = y2_new
                    bbox_msg.track_id = self.track_id
                    bbox_msg.is_tracking = True
                    self.bbox_pub.publish(bbox_msg)
                else:
                    # 跟踪丢失时发布空消息
                    bbox_msg = Bbox()
                    bbox_msg.is_tracking = False
                    self.bbox_pub.publish(bbox_msg)
                    cv2.putText(frame_copy, "跟踪丢失！按'r'重选", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 未开始跟踪
            else:
                # 未跟踪时发布空消息
                bbox_msg = Bbox()
                bbox_msg.is_tracking = False
                self.bbox_pub.publish(bbox_msg)
                cv2.putText(frame_copy, "请框选目标开始跟踪", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            # 键盘事件
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
            
            # 显示画面
            cv2.imshow("RK3588 KCF Tracking", frame_copy)
            
            # 控制ROS循环频率（与摄像头同步）
            rate.sleep()
        
        # 释放资源
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
