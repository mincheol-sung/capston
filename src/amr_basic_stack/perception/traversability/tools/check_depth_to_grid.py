#!/usr/bin/env python3
"""D455 z-band 가 traversability_node 의 2D 격자에 실제로 반영되는지 확인한다.

핵심 장면: low_object 는 지면 위 0.04~0.09 m 다. LiDAR 마운트가 0.2178 m 이므로
2D LiDAR 평면으로는 절대 안 보인다. 그게 /traversable_map 에 통과 불가로
찍히면 D455 경로가 살아 있다는 뜻이다 (설계문서 §3.3 "추가 검출").
"""
import os
import sys
import threading
import time

from amr_interfaces.msg import TraversableMap

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

W, H, RES = 160, 120, 0.05
OX, OY = -2.0, -3.0

# fake_depth_cloud.py 의 장면 (base_footprint = map 원점)
SCENES = {
    'low_object (z 0.04~0.09, LiDAR 사각)': ((0.95, 1.10), (0.40, 0.60)),
    'box (z 0.05~0.25)':                     ((1.45, 1.60), (-0.25, 0.25)),
}
OVERHANG = ((2.45, 2.60), (-0.40, 0.40))    # z 0.9~1.1 -> 2D 에 없어야 한다


class Harness(Node):
    def __init__(self):
        super().__init__('check_depth_grid')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.got = []
        self.create_subscription(TraversableMap, '/traversable_map',
                                 self.got.append, LATCHED)

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


def cells_for(x_range, y_range):
    xs = np.arange(x_range[0], x_range[1] + 1e-9, RES / 2)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, RES / 2)
    gx = np.floor((xs - OX) / RES).astype(int)
    gy = np.floor((ys - OY) / RES).astype(int)
    return {(int(a), int(b)) for a in gx for b in gy}


def main():
    rclpy.init()
    node = Harness()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    m = node.base_map()
    deadline = time.time() + 40
    while time.time() < deadline:
        node.map_pub.publish(m)
        time.sleep(0.4)
        if len(node.got) > 12:      # depth hit threshold 를 넘길 만큼 쌓이면
            break

    ok = True

    def check(label, cond, detail=''):
        nonlocal ok
        print(f'  [{"OK " if cond else "FAIL"}] {label}{detail}')
        ok = ok and cond

    if not node.got:
        check('/traversable_map 수신', False)
        sys.stdout.flush()
        os._exit(1)
    msg = node.got[-1]
    data = np.asarray(msg.grid.data, dtype=np.int8).reshape((H, W))
    blocked = int(np.sum(data == 100))
    print(f'\n격자 {msg.grid.info.width}x{msg.grid.info.height}, '
          f'통과 불가 {blocked} 셀, 미확인 {int(np.sum(data == -1))} 셀')

    check('D455 로 통과 불가 셀이 생겼다', blocked > 0, f' — {blocked} 셀')
    for name, (xr, yr) in SCENES.items():
        want = cells_for(xr, yr)
        hit = sum(1 for (cx, cy) in want
                  if 0 <= cx < W and 0 <= cy < H and data[cy, cx] == 100)
        check(f'{name} 가 격자에 찍힌다', hit > 0, f' — {hit}/{len(want)} 셀')

    want = cells_for(*OVERHANG)
    hit = sum(1 for (cx, cy) in want
              if 0 <= cx < W and 0 <= cy < H and data[cy, cx] == 100)
    check('overhang(z 0.9~1.1)은 2D 에 안 찍힌다', hit == 0, f' — {hit} 셀')

    # 바닥이 통째로 장애물이 되지 않았는지
    check('바닥이 통째로 막히지 않았다', blocked < 0.15 * data.size,
          f' — 전체 {data.size} 중 {blocked}')

    check('unknown_fill 에 PREVIOUS 비트(관측 기억 동작)',
          bool(msg.unknown_fill & TraversableMap.UNKNOWN_FILL_PREVIOUS),
          f' — {msg.unknown_fill}')

    # daemon 스레드가 spin 중인 상태에서 rclpy.shutdown() 을 부르면
    # "terminate called without an active exception" 으로 abort 하고 종료
    # 코드가 134 가 된다. 판정은 이미 끝났으니 파괴자를 건너뛴다.
    print('\n' + ('DEPTH->GRID PASSED' if ok else 'DEPTH->GRID FAILED'))
    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    sys.exit(main())
