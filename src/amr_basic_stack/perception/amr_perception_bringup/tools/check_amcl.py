#!/usr/bin/env python3
"""AMCL 이 수렴해 /initialpose 를 냈는지, 그 값이 ground truth 와 맞는지 본다."""
import math
import os
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Watch(Node):
    def __init__(self):
        super().__init__('check_amcl')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.initial = []
        self.amcl = []
        self.truth = []
        self.truth_at_publish = None
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initial.append, qos)
        rel = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.amcl.append, rel)
        # 로봇은 계속 움직인다. /initialpose 가 나온 그 순간의 진짜 위치와
        # 비교해야 한다. 출발 위치와 비교하면 주행 거리만큼 오차로 보인다.
        self.create_subscription(PoseStamped, '/ground_truth',
                                 self._on_truth, 10)

    def _on_truth(self, msg):
        self.truth.append(msg)
        if self.initial and self.truth_at_publish is None:
            self.truth_at_publish = msg


def main():
    rclpy.init()
    node = Watch()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    deadline = time.time() + 60
    while time.time() < deadline and not node.initial:
        time.sleep(0.5)

    ok = True

    def check(label, cond, detail=''):
        nonlocal ok
        print(f'  [{"OK " if cond else "FAIL"}] {label}{detail}')
        ok = ok and cond

    check('/amcl_pose 가 나왔다', len(node.amcl) > 0, f' — {len(node.amcl)} 개')
    check('/initialpose 가 발행됐다', len(node.initial) > 0,
          f' — {len(node.initial)} 개')
    if not node.initial:
        sys.stdout.flush()
        os._exit(1)

    check('ground truth 를 받았다', len(node.truth) > 0)
    if not node.truth:
        sys.stdout.flush()
        os._exit(1)
    truth = node.truth_at_publish or node.truth[-1]
    gt_p = truth.pose.position
    gt_yaw = yaw_of(truth.pose.orientation)

    m = node.initial[-1]
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = yaw_of(q)
    err = math.hypot(p.x - gt_p.x, p.y - gt_p.y)
    yaw_err = abs(math.atan2(math.sin(yaw - gt_yaw), math.cos(yaw - gt_yaw)))
    print(f'\n  ground truth : ({gt_p.x:.2f}, {gt_p.y:.2f}) '
          f'{math.degrees(gt_yaw):.0f} deg  (발행 시점의 실제 위치)')
    print(f'  AMCL 추정    : ({p.x:.2f}, {p.y:.2f}) {math.degrees(yaw):.0f} deg')
    print(f'  오차         : {err:.3f} m, {math.degrees(yaw_err):.1f} deg')

    check('frame_id 가 map 이다', m.header.frame_id == 'map',
          f' — {m.header.frame_id!r}')
    check('한 번만 발행됐다 (oneshot)', len(node.initial) == 1,
          f' — {len(node.initial)} 개')
    check('위치 오차가 0.5 m 이내', err < 0.5, f' — {err:.3f} m')
    check('방향 오차가 20 deg 이내', math.degrees(yaw_err) < 20.0,
          f' — {math.degrees(yaw_err):.1f} deg')

    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
