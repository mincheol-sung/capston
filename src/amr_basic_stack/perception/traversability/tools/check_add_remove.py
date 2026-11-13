#!/usr/bin/env python3
"""장애물 추가 → 제거 가 격자에 반영되는지 시간축으로 확인한다.

fake_scan.py 를 --remove-after 로 띄워 두고, 같은 셀의 값이
100 → (관측 기억 만료) → 0 으로 바뀌는지 지켜본다.

기준 맵은 전부 free 로 준다. 즉 여기서 100 이 되는 것은 오직 관측 때문이고,
다시 0 이 되는 것도 오직 관측(관통) 때문이다.
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
OBSTACLE_X = 2.0

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Harness(Node):
    def __init__(self):
        super().__init__('check_add_remove')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.grids = []
        self.create_subscription(OccupancyGrid, '/traversable_map_grid',
                                 self.grids.append, LATCHED)

    def base_map(self):
        grid = np.zeros((H, W), dtype=np.int8)      # 전부 free
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


def grid_now(node):
    if not node.grids:
        return None
    return np.asarray(node.grids[-1].data, dtype=np.int8).reshape((H, W))


def watch(node, seconds, label, cells=None):
    """주어진 시간 동안 격자를 지켜본다.

    cells 가 None 이면 마지막 격자에서 100 인 셀 집합을 돌려준다.
    cells 가 주어지면 그 셀들의 값 분포 변화를 찍는다.

    실제 명중 셀을 계산으로 맞히지 않고 관측에서 찾는 이유는, laser_front 가
    base_link 에서 (0.086291, -0.020494) 만큼 떨어져 있어 map 좌표가 한 셀씩
    밀리기 때문이다.
    """
    history = []
    end = time.time() + seconds
    while time.time() < end:
        node.map_pub.publish(node.base_map())
        time.sleep(0.5)
        d = grid_now(node)
        if d is None:
            continue
        if cells is None:
            snapshot = int(np.sum(d == 100))
        else:
            rows, cols = cells
            snapshot = tuple(sorted(np.unique(d[rows, cols]).tolist()))
        if not history or history[-1] != snapshot:
            history.append(snapshot)
    print(f'  {label:28s} {history}')
    return grid_now(node), history


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

    print(f'\n기준 맵은 전부 free. 정면 {OBSTACLE_X} m 에 장애물.')
    # fake_scan 이 22 s 에 장애물을 치운다. 이 스크립트는 4 s 뒤 시작하므로
    # ① 구간은 t=4..14 로 제거 전이고, ② 구간은 t=14..32 로 제거를 포함한다.
    grid, hist = watch(node, 10.0, '① 장애물 있는 동안 (100 셀 수)')
    if grid is None:
        check('/traversable_map_grid 수신', False)
        sys.stdout.flush()
        os._exit(1)

    blocked = np.argwhere(grid == 100)
    check('장애물이 100 으로 찍혔다', blocked.shape[0] > 0,
          f' — {blocked.shape[0]} 셀')
    if blocked.shape[0] == 0:
        sys.stdout.flush()
        os._exit(1)

    xs = OX + (blocked[:, 1] + 0.5) * RES
    print(f'      막힌 셀의 x 범위 {xs.min():.2f} ~ {xs.max():.2f} m '
          f'(장애물 {OBSTACLE_X} m 근처여야 한다)')
    check('막힌 셀이 장애물 거리에 있다',
          abs(np.median(xs) - OBSTACLE_X) < 0.25,
          f' — 중앙값 {np.median(xs):.2f} m')

    target = (blocked[:, 0], blocked[:, 1])
    print('  ... 장애물 제거를 기다린다 (fake_scan --remove-after 22) ...')
    grid2, hist2 = watch(node, 20.0, '② 그 셀들의 값 집합', cells=target)

    final = np.unique(grid2[target]).tolist()
    check('장애물이 사라지면 그 셀들이 통과 가능이 된다', final == [0],
          f' — 최종 값 집합 {final}')
    check('100 -> 0 전이가 실제로 관측됐다',
          any(100 in h for h in hist2) and final == [0])

    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
