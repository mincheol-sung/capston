#!/usr/bin/env python3
"""STVL 이 같은 점군으로 3D voxel 과 2D costmap 을 내는지 확인."""
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

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class C(Node):
    def __init__(self):
        super().__init__('check_stvl')
        self.voxels = []
        self.costmap = []
        self.create_subscription(PointCloud2, '/voxel_grid',
                                 self.voxels.append, 5)
        self.create_subscription(OccupancyGrid, '/costmap',
                                 self.costmap.append, LATCHED)


def main():
    rclpy.init()
    node = C()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    deadline = time.time() + 25
    while time.time() < deadline and not (node.voxels and node.costmap):
        time.sleep(0.3)

    ok = True

    def check(label, cond, detail=''):
        nonlocal ok
        print(f'  [{"OK " if cond else "FAIL"}] {label}{detail}')
        ok = ok and cond

    print('=== STVL 3D voxel (/voxel_grid) ===')
    if not node.voxels:
        check('수신', False)
    else:
        pts = np.asarray(point_cloud2.read_points_numpy(
            node.voxels[-1], field_names=('x', 'y', 'z'), skip_nans=True),
            dtype=np.float64).reshape(-1, 3)
        check('voxel 이 있다', pts.shape[0] > 0, f' — {pts.shape[0]} 개')
        if pts.shape[0]:
            print(f'        z 범위 {pts[:, 2].min():.3f} ~ '
                  f'{pts[:, 2].max():.3f} m')
            layers = len(np.unique(np.round(pts[:, 2] / 0.05)))
            check('z 축으로 층이 여러 개다', layers >= 3, f' — {layers} 층')
            check('상부 돌출(0.9 m 이상)이 잡힌다', pts[:, 2].max() > 0.85,
                  f' — 최고 {pts[:, 2].max():.3f} m')

    print('=== STVL 2D costmap (/costmap) ===')
    if not node.costmap:
        check('수신', False)
    else:
        g = node.costmap[-1]
        data = np.asarray(g.data, dtype=np.int8)
        lethal = int(np.sum(data == 100))
        unknown = int(np.sum(data == -1))
        check('격자가 있다', g.info.width > 0,
              f' — {g.info.width}x{g.info.height} @ {g.info.resolution:.3f}, '
              f'frame {g.header.frame_id}')
        check('치명 셀이 있다', lethal > 0, f' — {lethal} 셀')
        check('미확인(track_unknown_space)이 유지된다', unknown > 0,
              f' — {unknown} 셀')

    # daemon 스레드가 spin 중인 상태에서 rclpy.shutdown() 을 부르면
    # "terminate called without an active exception" 으로 abort 하고 종료
    # 코드가 134 가 된다. 판정은 이미 끝났으니 파괴자를 건너뛴다.
    print('\n' + ('STVL PASSED' if ok else 'STVL FAILED'))
    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    sys.exit(main())
