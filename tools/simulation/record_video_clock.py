#!/usr/bin/env python3
"""Record wall-to-Gazebo time pairs for an honest screen-video overlay."""

import csv
import sys
import time
from pathlib import Path

import rospy
from rosgraph_msgs.msg import Clock


def main():
    directory = Path(sys.argv[1])
    rospy.init_node("fixed_camera_video_clock", anonymous=True,
                    disable_signals=True)
    clock = rospy.wait_for_message("/clock", Clock, timeout=10.0)
    wall_start = time.time()
    (directory / "video_start_epoch_s.txt").write_text(
        "%.9f\n" % wall_start, encoding="utf-8")
    (directory / "video_start_ros_time_s.txt").write_text(
        "%.9f\n" % clock.clock.to_sec(), encoding="utf-8")
    with (directory / "video_clock.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["wall_time_s", "ros_time_s"])
        writer.writerow(["%.9f" % wall_start,
                         "%.9f" % clock.clock.to_sec()])
        stream.flush()
        last_wall = [wall_start]

        def callback(message):
            wall_now = time.time()
            if wall_now - last_wall[0] < 0.04:
                return
            writer.writerow(["%.9f" % wall_now,
                             "%.9f" % message.clock.to_sec()])
            stream.flush()
            last_wall[0] = wall_now

        subscriber = rospy.Subscriber("/clock", Clock, callback, queue_size=20)
        rospy.spin()
        subscriber.unregister()


if __name__ == "__main__":
    main()
