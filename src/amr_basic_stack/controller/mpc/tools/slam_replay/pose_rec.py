#!/usr/bin/python3
"""bag 재생 중 /tracked_pose 를 stamp 와 함께 CSV 로 남긴다. 사용: pose_rec.py 파일.csv"""
import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

out = open(sys.argv[1], "w", buffering=1)
out.write("stamp,x,y,qx,qy,qz,qw\n")


class Rec(Node):
    def __init__(self):
        super().__init__("pose_rec_replay")
        self.create_subscription(PoseStamped, "/tracked_pose", self.cb, 50)

    def cb(self, m):
        q = m.pose.orientation
        out.write(f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.4f},{m.pose.position.x:.5f},{m.pose.position.y:.5f},"
                  f"{q.x:.5f},{q.y:.5f},{q.z:.5f},{q.w:.5f}\n")


if __name__ == "__main__":
    rclpy.init(args=sys.argv[1:])
    try:
        rclpy.spin(Rec())
    except Exception:
        pass
