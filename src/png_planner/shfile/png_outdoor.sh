#!/bin/bash
# ============================================
# 启动 PX4 + MAVROS + PNG Planner (Outdoor 模式)
# ============================================

# 打开串口权限
sudo chmod 777 /dev/ttyACM0

# 启动 MAVROS
gnome-terminal -- bash -c "roslaunch mavros px4.launch; exec bash"

# 等待 MAVROS 启动稳定
sleep 5

# 启动 PNG Planner 主节点
gnome-terminal -- bash -c "cd ~/png_planner_ws && source devel/setup.bash && roslaunch png_planner png_outdoor.launch; exec bash"

# 等待规划器加载完成
sleep 5

# 启动目标节点
gnome-terminal -- bash -c "cd ~/png_planner_ws && source devel/setup.bash && roslaunch png_planner target_outdoor.launch; exec bash"