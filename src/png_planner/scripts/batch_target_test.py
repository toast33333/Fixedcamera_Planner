#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单次 PNG 测试脚本（手动修改 launch 参数）
- 使用 message_filters 同步目标和无人机数据
- 生成轨迹图和收敛图（相对距离 + 相对角度）
"""

import rospy
import roslaunch
import time
import os
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from message_filters import ApproximateTimeSynchronizer, Subscriber

# ===================== 用户配置 =====================
LAUNCH_FILE = "/home/negroni/png_planner_ws/src/png_planner/launch/demo_4_png.launch"
WAIT_TIME = 30.0   # 测试持续时间（秒）
SAVE_DIR = "/home/negroni/png_planner_ws/src/png_planner/scripts/test_results_v2"
# ====================================================

class DataCollector:
    """收集目标和己方数据"""
    def __init__(self):
        self.odom_x, self.odom_y, self.odom_z = [], [], []
        self.yaw, self.vel = [], []
        self.target_x, self.target_y, self.target_z = [], [], []

    def callback(self, odom_msg, target_msg):
        # 自己方
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y

        # yaw 角度
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y**2 + ori.z**2)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.odom_x.append(pos.x)
        self.odom_y.append(pos.y)
        self.odom_z.append(pos.z)
        self.yaw.append(yaw)
        self.vel.append(math.sqrt(vx**2 + vy**2))

        # 目标
        tpos = target_msg.pose.position
        self.target_x.append(tpos.x)
        self.target_y.append(tpos.y)
        self.target_z.append(tpos.z)

def plot_trajectory(collector):
    """绘制二维轨迹图"""
    os.makedirs(SAVE_DIR, exist_ok=True)
    plt.figure(figsize=(6,6))
    plt.plot(collector.odom_x, collector.odom_y, 'r-', label='Ownship (odom)')
    if collector.target_x:
        plt.plot(collector.target_x, collector.target_y, 'b--', label='Target')
    plt.scatter(0, 0, c='k', marker='x', s=80, label='Target Start')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Trajectory")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.savefig(f"{SAVE_DIR}/trajectory_12.png", dpi=200)
    plt.close()

def plot_convergence(collector):
    """绘制收敛图：相对距离 & 相对角度"""
    t_len = len(collector.odom_x)
    if t_len == 0:
        print("[警告] 没有有效数据")
        return

    ox, oy, oz, oyaw = collector.odom_x, collector.odom_y, collector.odom_z, collector.yaw
    tx, ty, tz = collector.target_x, collector.target_y, collector.target_z

    # 三维相对距离
    d_rel = [math.sqrt((ox[j]-tx[j])**2 + (oy[j]-ty[j])**2 + (oz[j]-tz[j])**2) for j in range(t_len)]

    # 相对角度：目标朝向指向无人机
    target_yaw = [math.degrees(math.atan2(oy[j]-ty[j], ox[j]-tx[j])) for j in range(t_len)]
    delta_yaw = [oyaw[j]-target_yaw[j] for j in range(t_len)]

    t = list(range(t_len))
    fig, ax1 = plt.subplots(figsize=(7,4))

    ax1.plot(t, d_rel, 'b-', label='Relative Distance [m]')
    ax1.set_xlabel("Time (samples)")
    ax1.set_ylabel("Distance [m]", color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    ax2 = ax1.twinx()
    ax2.plot(t, delta_yaw, 'r-', label='Relative Angle [deg]')
    ax2.set_ylabel("Relative Angle [deg]", color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    plt.title("Convergence")
    fig.tight_layout()
    plt.savefig(f"{SAVE_DIR}/convergence_12.png", dpi=200)
    plt.close()

def run_test():
    rospy.init_node("single_png_test", anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    print("=== 启动测试 ===")
    launch_file_with_args = roslaunch.rlutil.resolve_launch_arguments([LAUNCH_FILE])[0]
    parent = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_with_args])

    collector = DataCollector()

    # 启动 launch
    parent.start()
    print("运行中...")

    # 使用 message_filters 同步 /sim/odom 和 /target/pose
    odom_sub = Subscriber("/sim/odom", Odometry)
    target_sub = Subscriber("/target/pose", PoseStamped)
    ats = ApproximateTimeSynchronizer([odom_sub, target_sub], queue_size=100, slop=0.02)
    ats.registerCallback(collector.callback)

    # 等待测试时间
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < WAIT_TIME and not rospy.is_shutdown():
        rospy.sleep(0.01)

    parent.shutdown()
    print("测试完成，生成图像...")
    plot_trajectory(collector)
    plot_convergence(collector)
    print("✅ 测试完成，结果保存在：", SAVE_DIR)

if __name__ == "__main__":
    run_test()
