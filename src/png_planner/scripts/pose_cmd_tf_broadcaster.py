#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

def callback(msg):
    # 创建 TransformStamped
    t = geometry_msgs.msg.TransformStamped()

    # 时间戳和坐标系
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"   # 父坐标系（RViz 里 Fixed Frame 一般也设为 world）
    t.child_frame_id = "uav"      # 子坐标系，RViz 里可以让视角跟随这个 frame

    # 平移来自 pose_cmd 里的 position
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    # 旋转也直接用 pose_cmd 里的 orientation
    t.transform.rotation = msg.pose.orientation

    # 发送 TF
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("pose_cmd_tf_broadcaster")

    # 创建静态的 TransformBroadcaster 对象（全局）
    br = tf2_ros.TransformBroadcaster()

    # 订阅 pose_cmd（launch 里已经 remap 到 /pose_cmd 了）
    rospy.Subscriber("pose_cmd", PoseStamped, callback)

    rospy.loginfo("pose_cmd_tf_broadcaster started, listening to [pose_cmd].")

    rospy.spin()
