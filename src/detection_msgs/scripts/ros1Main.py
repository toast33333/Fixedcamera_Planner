#!/usr/bin/env python3
import cv2
import time
import logging
import rospy  # 导入ROS1 Python库
from detection_msgs.msg import Detection  # 消息类型保持不变
from rknnpool import rknnPoolExecutor
from ros1Func import myFunc  # 跟踪处理函数

def main():

    logging.addLevelName(10,'DEBUG')
    logging.addLevelName(20, 'INFO')
    logging.addLevelName(30, 'WARNING')
    logging.addLevelName(40, 'ERROR') 
    logging.basicConfig(level=logging.INFO)
     
    # 1. 初始化ROS1节点
    rospy.init_node('yolo_tracker_publisher', anonymous=True)
    # 创建发布者，话题名称为“/object_detections”，消息类型为Object
    publisher = rospy.Publisher('/object_detections', Detection, queue_size=10)
    rospy.loginfo("ROS1发布节点已启动，话题：/object_detections")

    # 2. 初始化跟踪算法
    #cap = cv2.VideoCapture('./720p60hz.mp4')  # 视频源
    cap = cv2.VideoCapture(40)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
    cap.set(cv2.CAP_PROP_FPS, 120)

    def wrapped_func(rknn_lite, frame):
        return myFunc(rknn_lite, frame, publisher)  # 传递ROS1发布者

    modelPath = "/home/firefly/catkin_ws/src/detection_msgs/rknnModel/yolov8-NewDrone.rknn"
    TPEs = 3
    pool = rknnPoolExecutor(
        rknnModel=modelPath,
        TPEs=TPEs,
        func=wrapped_func 
    )

    # 3. 初始化帧
    if cap.isOpened():
        for i in range(TPEs + 1):
            ret, frame = cap.read()
            if not ret:
                cap.release()
                del pool
                return
            pool.put(frame)

    # 4. 主循环：处理帧并发布消息
    frames, loopTime, initTime = 0, time.time(), time.time()
    try:
        while cap.isOpened() and not rospy.is_shutdown():  # ROS1判断节点是否关闭
            frames += 1
            ret, frame = cap.read()
            if not ret:
                break
            pool.put(frame)
            frame, flag = pool.get()
            if flag is False:
                break

            # 显示图像
            cv2.imshow('yolov8', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # 打印帧率
            if frames % 30 == 0:
                print(f"30帧平均帧率:\t {30 / (time.time() - loopTime)} 帧")
                loopTime = time.time()
    finally:
        # 5. 资源清理
        print(f"总平均帧率\t {frames / (time.time() - initTime)} 帧")
        cap.release()
        cv2.destroyAllWindows()
        pool.release()
        # ROS1不需要显式销毁节点

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
