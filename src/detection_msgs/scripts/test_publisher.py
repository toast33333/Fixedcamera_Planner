#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # 使用ROS标准字符串消息类型
import time

def test_publisher():
    # 初始化节点，节点名：test_publisher（确保唯一）
    rospy.init_node('test_publisher', anonymous=True)
    
    # 创建发布者，话题名：/test_topic，消息类型：String，队列大小：10
    pub = rospy.Publisher('/test_topic', String, queue_size=10)
    
    # 设置发布频率（1Hz，即每秒1次）
    rate = rospy.Rate(1)
    
    # 计数变量，用于区分不同消息
    count = 0
    
    rospy.loginfo("测试发布者已启动，开始发送消息...")
    
    # 当节点未被关闭时循环发布
    while not rospy.is_shutdown():
        # 构建消息内容（包含计数和当前时间）
        current_time = time.strftime("%H:%M:%S", time.localtime())
        message = f"测试消息 {count}，发送时间：{current_time}"
        
        # 发布消息
        pub.publish(message)
        
        # 打印日志（终端可见）
        rospy.loginfo(f"已发送：{message}")
        
        # 计数+1
        count += 1
        
        # 按设定频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        # 启动发布者
        test_publisher()
    except rospy.ROSInterruptException:
        # 捕获ROS中断信号（如Ctrl+C）
        rospy.loginfo("发布者已停止")