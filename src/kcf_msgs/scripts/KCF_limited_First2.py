#!/usr/bin/env python3
import cv2
import time
import rospy
import numpy as np
import logging
from collections import deque
from kcf_msgs.msg import Bbox  # 匹配你原始的消息结构
from rknnlite.api import RKNNLite
# 复用你原有ros1Func中的核心函数和常量（仅保留必要的）
from ros1Func import letterbox, yolov8_post_process, CLASSES

class KCFTrackerWithReDetect:
    def __init__(self):
        logging.addLevelName(10,'DEBUG')
        logging.addLevelName(20, 'INFO')
        logging.addLevelName(30, 'WARNING')
        logging.addLevelName(40, 'ERROR') 
        logging.basicConfig(level=logging.INFO)
        # ========== 1. 初始化ROS ==========
        rospy.init_node('kcf_tracker_with_redetect', anonymous=True)
        self.bbox_pub = rospy.Publisher('/object_kcf', Bbox, queue_size=10)
        rospy.loginfo("KCF跟踪器(鼠标框选+尺度优化+YOLO重检)初始化")

        # ========== 2. 摄像头初始化（和你原始代码一致） ==========
        self.cap = cv2.VideoCapture(40, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开USB摄像头！")
            raise IOError("摄像头初始化失败")
        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 120)    
        self.center_x = self.width / 2.0
        self.center_y = self.height / 2.0

        # ========== 3. 鼠标框选相关（完全保留你原始逻辑） ==========
        self.tracker = None
        self.bbox = None  # (x1, y1, w, h) KCF格式
        self.tracking = False
        self.selecting = False
        self.roi_start = (0, 0)
        self.roi_end = (0, 0)
        self.track_id = 1  # 仅保留track_id（int32）

        # ========== 4. 尺度突变检测参数（你要求的核心优化） ==========
        self.scale_history = deque(maxlen=10)  # 最近10帧尺度记录
        self.scale_threshold = 0.3  # 尺度变化>30%判定为突变
        self.scale_reinit_threshold = 0.4  # >40%触发重检
        self.consecutive_scale_abnormal = 3  # 连续3帧异常触发重检
        self.scale_abnormal_count = 0  # 连续异常计数
        self.min_scale = 20  # 最小跟踪框尺寸
        self.max_window_width = 240   # 最大跟踪框宽度
        self.max_window_height = 240  # 最大跟踪框高度

        # ========== 5. YOLO重检相关（单线程，无线程池） ==========
        self.rknn_model_path = "/home/firefly/catkin_ws/src/kcf_msgs/rknnModel/yolov8-NewDrone.rknn"
        self.rknn_lite = self.init_rknn()  # 单线程初始化RKNN
        self.redetect_triggered = False  # 重检触发标记
        self.obj_thresh = 0.5  # YOLO置信度阈值（仅用于筛选目标，不发布）

        # ========== 6. 辅助参数 ==========
        self.fps_counter = 0
        self.fps_start = time.time()

    def init_rknn(self):
        """单线程初始化RKNN（无线程池）"""
        rknn_lite = RKNNLite()
        if rknn_lite.load_rknn(self.rknn_model_path) != 0:
            rospy.logerr("加载YOLO RKNN模型失败")
            raise RuntimeError("RKNN模型加载失败")
        if rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
            rospy.logerr("初始化RKNN运行时失败")
            raise RuntimeError("RKNN运行时失败")
        rospy.loginfo("YOLO RKNN模型初始化成功")
        return rknn_lite

    def mouse_callback(self, event, x, y, flags, param):
        """完全保留你原始的鼠标框选逻辑（无score）"""
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
                
                # 确保初始框不小于最小尺寸
                if w > self.min_scale and h > self.min_scale:
                    # 限制跟踪框最大尺寸（和你原始逻辑一致）
                    center_x_initial = x1 + w / 2.0
                    center_y_initial = y1 + h / 2.0
                    w_limited = min(w, self.max_window_width)
                    h_limited = min(h, self.max_window_height)
                    x1_limited = int(center_x_initial - w_limited / 2.0)
                    y1_limited = int(center_y_initial - h_limited / 2.0)

                    self.bbox = (x1_limited, y1_limited, w_limited, h_limited)
                    self.scale_history.clear()
                    self.scale_history.append((w_limited, h_limited))
                    self.scale_abnormal_count = 0
                    rospy.loginfo(f"框选目标完成，初始框：{self.bbox}")

                    # 初始化KCF跟踪器（和你原始逻辑一致，无score）
                    ret, frame = self.cap.read()
                    if ret:
                        self.tracker = cv2.legacy.TrackerKCF_create()
                        self.tracker.init(frame, self.bbox)
                        self.tracking = True
                        self.redetect_triggered = False

    def detect_scale_change(self, current_w, current_h):
        """检测尺度突变（无score相关）"""
        if len(self.scale_history) < 5:
            self.scale_history.append((current_w, current_h))
            return False, False

        # 计算历史平均尺度
        avg_w = sum(w for w, h in self.scale_history) / len(self.scale_history)
        avg_h = sum(h for w, h in self.scale_history) / len(self.scale_history)

        # 计算尺度变化率
        w_change = abs(current_w - avg_w) / avg_w
        h_change = abs(current_h - avg_h) / avg_h

        # 判断是否尺度突变/是否需要重检
        is_scale_abnormal = w_change > self.scale_threshold or h_change > self.scale_threshold
        need_redetect = w_change > self.scale_reinit_threshold or h_change > self.scale_reinit_threshold

        # 连续异常计数
        if is_scale_abnormal:
            self.scale_abnormal_count += 1
            if self.scale_abnormal_count >= self.consecutive_scale_abnormal:
                need_redetect = True  # 连续3帧异常触发重检
        else:
            self.scale_abnormal_count = 0

        self.scale_history.append((current_w, current_h))
        return is_scale_abnormal, need_redetect

    def yolo_redetect(self, frame):
        """单线程YOLO重检（仅筛选目标，不处理score）"""
        rospy.loginfo("触发YOLO重检（尺度突变/跟踪丢失）")
        # YOLO前处理（复用你ros1Func中的逻辑）
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_processed, ratio, padding = letterbox(img_rgb, new_shape=(640, 640))
        img_input = np.expand_dims(img_processed, 0)

        # 单线程推理
        outputs = self.rknn_lite.inference(inputs=[img_input], data_format=['nhwc'])
        boxes, classes, scores = yolov8_post_process(outputs)

        # 筛选最高置信度目标（仅用于内部筛选，不发布score）
        if boxes is not None and len(boxes) > 0:
            best_idx = np.argmax(scores)
            if scores[best_idx] > self.obj_thresh:
                best_box = boxes[best_idx]

                # 转换为原始图像坐标
                x1 = (best_box[0] - padding[0]) / ratio[0]
                y1 = (best_box[1] - padding[1]) / ratio[1]
                x2 = (best_box[2] - padding[0]) / ratio[0]
                y2 = (best_box[3] - padding[1]) / ratio[1]
                w = x2 - x1
                h = y2 - y1

                # 限制跟踪框尺寸（和你原始逻辑一致）
                center_x = x1 + w/2
                center_y = y1 + h/2
                w_limited = min(w, self.max_window_width)
                h_limited = min(h, self.max_window_height)
                x1_limited = int(center_x - w_limited/2)
                y1_limited = int(center_y - h_limited/2)

                # 重新初始化KCF跟踪器（无score）
                self.bbox = (x1_limited, y1_limited, w_limited, h_limited)
                self.tracker = cv2.legacy.TrackerKCF_create()
                self.tracker.init(frame, self.bbox)
                self.tracking = True
                self.scale_history.clear()
                self.scale_history.append((w_limited, h_limited))
                self.scale_abnormal_count = 0

                # 绘制重检框（无score显示）
                cv2.rectangle(frame, (int(x1_limited), int(y1_limited)), 
                              (int(x1_limited+w_limited), int(y1_limited+h_limited)), 
                              (255, 0, 255), 2)
                cv2.putText(frame, f"ReDetect: {CLASSES[int(classes[best_idx])]}", 
                            (int(x1_limited), int(y1_limited)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                return frame, True
        return frame, False

    def publish_bbox(self, x1, y1, x2, y2, is_tracking=True):
        """仅发布你原始KCF的字段：x1/x2/y1/y2/track_id/is_tracking"""
        msg = Bbox()
        # 坐标转换（和你原始逻辑一致）
        msg.x1 = x1 - self.center_x
        msg.y1 = self.center_y - y1
        msg.x2 = x2 - self.center_x
        msg.y2 = self.center_y - y2
        msg.track_id = self.track_id  # int32类型
        msg.is_tracking = is_tracking  # bool类型
        self.bbox_pub.publish(msg)

    def run(self):
        """主循环（完全保留你原始的交互逻辑，无score）"""
        cv2.namedWindow("RK3588 KCF Tracking (Mouse Select + YOLO ReDetect)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RK3588 KCF Tracking (Mouse Select + YOLO ReDetect)", 640, 480)
        cv2.setMouseCallback("RK3588 KCF Tracking (Mouse Select + YOLO ReDetect)", self.mouse_callback)
        
        rospy.loginfo("操作说明：\n1. 鼠标框选目标开始跟踪\n2. 按'r'重新框选，'q'退出\n3. 按'f'查看帧率")

        rate = rospy.Rate(120)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("摄像头读取失败")
                break
            
            frame_copy = frame.copy()
            self.fps_counter += 1

            # ========== 鼠标框选绘制（完全保留你原始逻辑） ==========
            if self.selecting:
                cv2.rectangle(frame_copy, self.roi_start, self.roi_end, (0, 255, 0), 2)
                cv2.putText(frame_copy, "框选目标...", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # ========== 跟踪逻辑（整合尺度检测+YOLO重检，无score） ==========
            elif self.tracking:
                success, bbox = self.tracker.update(frame_copy)
                if success:
                    x_old, y_old, w_old, h_old = [int(v) for v in bbox]
                    x1_old, y1_old = x_old, y_old
                    x2_old, y2_old = x_old + w_old, y_old + h_old

                    # 检测尺度突变
                    is_scale_abnormal, need_redetect = self.detect_scale_change(w_old, h_old)
                    
                    # 绘制跟踪框（尺度异常标黄色，正常标绿色）
                    color = (0, 255, 255) if is_scale_abnormal else (0, 255, 0)
                    cv2.rectangle(frame_copy, (x1_old, y1_old), (x2_old, y2_old), color, 2)
                    
                    # 尺度异常提示（无score）
                    if is_scale_abnormal:
                        cv2.putText(frame_copy, f"Scale Change: {self.scale_abnormal_count}/{self.consecutive_scale_abnormal}", 
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # 发布ROS消息（仅原始字段）
                    self.publish_bbox(x1_old, y1_old, x2_old, y2_old, is_tracking=True)

                    # 尺度突变触发YOLO重检
                    if need_redetect and not self.redetect_triggered:
                        frame_copy, redetect_success = self.yolo_redetect(frame_copy)
                        self.redetect_triggered = True  # 防止重复触发
                    elif not is_scale_abnormal:
                        self.redetect_triggered = False

                else:
                    # 跟踪丢失触发YOLO重检
                    cv2.putText(frame_copy, "跟踪丢失！触发YOLO重检...", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    # 发布丢失状态
                    self.publish_bbox(0, 0, 0, 0, is_tracking=False)
                    frame_copy, _ = self.yolo_redetect(frame_copy)
            
            # ========== 未跟踪状态（保留你原始逻辑） ==========
            else:
                cv2.putText(frame_copy, "请鼠标框选目标开始跟踪", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                # 发布未跟踪状态
                self.publish_bbox(0, 0, 0, 0, is_tracking=False)

            # ========== 键盘控制（完全保留你原始逻辑） ==========
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
                self.scale_history.clear()
                self.scale_abnormal_count = 0
                self.redetect_triggered = False
            
            cv2.imshow("RK3588 KCF Tracking (Mouse Select + YOLO ReDetect)", frame_copy)
            rate.sleep()
        
        # ========== 资源释放（保留你原始逻辑） ==========
        self.cap.release()
        self.rknn_lite.release()
        cv2.destroyAllWindows()
        rospy.loginfo("资源已释放，节点退出")

if __name__ == "__main__":
    try:
        tracker = KCFTrackerWithReDetect()
        tracker.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"运行错误：{e}")
