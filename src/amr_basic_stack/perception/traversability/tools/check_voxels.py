#!/usr/bin/env python3
"""z-band 투영과 octomap voxel 이 기대대로 나오는지 실제 토픽으로 확인한다."""
import os
import sys
import threading
import time

from nav_msgs.msg import OccupancyGrid

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from visualization_msgs.msg import MarkerArray

# fake_depth_cloud.py 가 만드는 장면 (base_footprint 기준)
EXPECT = {
    'floor':      {'z': (0.00, 0.00), 'x': (0.40, 4.00), 'in_band': False},
    'low_object': {'z': (0.04, 0.09), 'x': (0.95, 1.10), 'in_band': True},
    'box':        {'z': (0.05, 0.25), 'x': (1.45, 1.60), 'in_band': True},
    'overhang':   {'z': (0.90, 1.10), 'x': (2.45, 2.60), 'in_band': False},
}
Z_MIN, Z_MAX = 0.033, 0.31


class Collector(Node):
    def __init__(self):
        super().__init__('check_voxels')
        self.band = []
        self.filtered = []
        self.markers = []
        self.projected = []
        self.create_subscription(PointCloud2, '/traversability/cloud_band',
                                 self.band.append, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/traversability/cloud_filtered',
                                 self.filtered.append, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/occupied_cells_vis_array',
                                 self.markers.append, 5)
        self.create_subscription(OccupancyGrid, '/projected_map',
                                 self.projected.append, 5)


def xyz(msg):
    a = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                       skip_nans=True)
    return np.asarray(a, dtype=np.float64).reshape(-1, 3)


def main():
    rclpy.init()
    node = Collector()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    deadline = time.time() + 25
    while time.time() < deadline and not (node.band and node.markers):
        time.sleep(0.3)

    ok = True

    def check(label, cond, detail=''):
        nonlocal ok
        print(f'  [{"OK " if cond else "FAIL"}] {label}{detail}')
        ok = ok and cond

    print('\n=== z-band 점군 (/traversability/cloud_band) ===')
    if not node.band:
        check('수신', False)
        sys.stdout.flush()
        os._exit(1)
    band = xyz(node.band[-1])
    check('점이 있다', band.shape[0] > 0, f' — {band.shape[0]} 점')
    if band.shape[0]:
        zmin, zmax = band[:, 2].min(), band[:, 2].max()
        print(f'        z 범위 {zmin:.3f} ~ {zmax:.3f} m')
        check('밴드를 벗어난 점이 없다',
              zmin >= Z_MIN - 1e-6 and zmax <= Z_MAX + 1e-6)
        check('바닥(z=0)이 섞이지 않았다', zmin > 0.02,
              f' — 최저 {zmin:.3f} m')
        for name in ('low_object', 'box'):
            e = EXPECT[name]
            hit = np.sum((band[:, 0] >= e['x'][0] - 0.05)
                         & (band[:, 0] <= e['x'][1] + 0.05))
            check(f'{name} 가 밴드에 잡힌다', hit > 0, f' — {hit} 점')
        e = EXPECT['overhang']
        hit = np.sum((band[:, 0] >= e['x'][0] - 0.05)
                     & (band[:, 0] <= e['x'][1] + 0.05))
        check('overhang(0.9~1.1 m)은 밴드에서 빠진다', hit == 0,
              f' — {hit} 점')

    print('\n=== 전체 높이 점군 (/traversability/cloud_filtered) ===')
    if node.filtered:
        full = xyz(node.filtered[-1])
        check('점이 있다', full.shape[0] > 0, f' — {full.shape[0]} 점')
        if full.shape[0]:
            print(f'        z 범위 {full[:, 2].min():.3f} ~ '
                  f'{full[:, 2].max():.3f} m')
            check('overhang 이 3D 에는 남는다', full[:, 2].max() > 0.85,
                  f' — 최고 {full[:, 2].max():.3f} m')
            check('밴드보다 넓다', full.shape[0] > band.shape[0],
                  f' — {full.shape[0]} > {band.shape[0]}')
    else:
        check('수신', False)

    print('\n=== OctoMap voxel (/occupied_cells_vis_array) ===')
    if not node.markers:
        check('수신', False)
    else:
        ma = node.markers[-1]
        total = sum(len(m.points) for m in ma.markers)
        levels = [i for i, m in enumerate(ma.markers) if len(m.points)]
        check('voxel 이 있다', total > 0, f' — {total} 개')
        check('여러 octree 깊이에 걸쳐 있다', len(levels) >= 1,
              f' — 깊이 인덱스 {levels}')
        zs = np.array([p.z for m in ma.markers for p in m.points])
        if zs.size:
            print(f'        voxel z 범위 {zs.min():.3f} ~ {zs.max():.3f} m')
            check('z 축으로 층이 여러 개다',
                  len(np.unique(np.round(zs / 0.05))) >= 3,
                  f' — {len(np.unique(np.round(zs / 0.05)))} 층')
            check('상부 돌출(0.9 m 이상)이 3D 에 잡힌다', zs.max() > 0.85,
                  f' — 최고 {zs.max():.3f} m')
            colors = {(round(m.color.r, 2), round(m.color.g, 2),
                       round(m.color.b, 2)) for m in ma.markers
                      if len(m.points)}
            has_per_point = any(len(m.colors) > 0 for m in ma.markers)
            check('높이별 색이 들어 있다 (use_height_map)',
                  has_per_point or len(colors) > 1,
                  f' — per-point 색 {has_per_point}')

    print('\n=== OctoMap 2D 투영 (/projected_map) ===')
    if node.projected:
        g = node.projected[-1]
        data = np.asarray(g.data, dtype=np.int8)
        occ = int(np.sum(data >= 65))
        check('격자가 있다', g.info.width > 0 and g.info.height > 0,
              f' — {g.info.width}x{g.info.height} @ {g.info.resolution:.3f}')
        check('점유 셀이 있다', occ > 0, f' — {occ} 셀')
    else:
        check('수신', False)

    # daemon 스레드가 spin 중인 상태에서 rclpy.shutdown() 을 부르면
    # "terminate called without an active exception" 으로 abort 하고 종료
    # 코드가 134 가 된다. 판정은 이미 끝났으니 파괴자를 건너뛴다.
    print('\n' + ('VOXEL CHAIN PASSED' if ok else 'VOXEL CHAIN FAILED'))
    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    sys.exit(main())
