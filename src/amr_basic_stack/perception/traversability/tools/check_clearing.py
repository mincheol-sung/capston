#!/usr/bin/env python3
"""관측 기반 해제가 실제로 도는지 확인한다 (설계문서 §6).

기준 맵을 **전부 점유(100)** 로 주고 시작한다. 초기 매핑 때 있던 물건이
지금은 없는 최악의 상황이다. 광선이 관통하면 그 셀이 통과 가능으로 바뀌어야
하고, 장애물이 있는 방향은 끝까지 막혀 있어야 한다.
"""
import os
import sys
import threading
import time

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

W, H, RES = 160, 160, 0.05
OX, OY = -4.0, -4.0

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Harness(Node):
    def __init__(self):
        super().__init__('check_clearing')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.grids = []
        self.create_subscription(OccupancyGrid, '/traversable_map_grid',
                                 self.grids.append, LATCHED)

    def base_map(self):
        """전부 점유. 해제만이 이걸 뚫을 수 있다."""
        grid = np.full((H, W), 100, dtype=np.int8)
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.width, msg.info.height, msg.info.resolution = W, H, RES
        msg.info.origin.position.x = OX
        msg.info.origin.position.y = OY
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        return msg


def cell(x, y):
    return int(np.floor((x - OX) / RES)), int(np.floor((y - OY) / RES))


def latest(node, timeout=25.0):
    node.grids.clear()
    deadline = time.time() + timeout
    while time.time() < deadline:
        node.map_pub.publish(node.base_map())
        time.sleep(0.4)
        if len(node.grids) >= 4:
            return np.asarray(node.grids[-1].data, dtype=np.int8).reshape((H, W))
    return None


def main():
    rclpy.init()
    node = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    ok = True

    def check(label, cond, detail=''):
        nonlocal ok
        print(f'  [{"OK " if cond else "FAIL"}] {label}{detail}')
        ok = ok and cond

    data = latest(node)
    if data is None:
        check('/traversable_map_grid 수신', False)
        sys.stdout.flush()
        os._exit(1)

    free = int(np.sum(data == 0))
    blocked = int(np.sum(data == 100))
    print(f'\n기준 맵은 전부 점유({W * H} 셀). 관측 후:')
    print(f'  통과 가능 {free}  /  통과 불가 {blocked}  /  미확인 '
          f'{int(np.sum(data == -1))}')

    check('해제가 일어났다', free > 0, f' — {free} 셀')

    # 정면 2 m 에 장애물이 있다 (fake_scan --obstacle 2.0)
    cx, cy = cell(1.0, 0.0)
    check('장애물 앞(1.0 m)은 해제됐다', data[cy, cx] == 0,
          f' — 값 {data[cy, cx]}')
    cx, cy = cell(2.0, 0.0)
    check('장애물 자리(2.0 m)는 막혀 있다', data[cy, cx] == 100,
          f' — 값 {data[cy, cx]}')
    cx, cy = cell(3.0, 0.0)
    check('장애물 뒤(3.0 m)는 해제되지 않았다', data[cy, cx] != 0,
          f' — 값 {data[cy, cx]}')
    # 측면은 장애물이 없어 max_range 까지 관통한다
    cx, cy = cell(0.0, 2.0)
    check('측면(장애물 없음)은 해제됐다', data[cy, cx] == 0,
          f' — 값 {data[cy, cx]}')

    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
