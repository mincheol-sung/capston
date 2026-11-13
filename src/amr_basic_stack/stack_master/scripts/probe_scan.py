#!/usr/bin/env python3
"""라이다가 타깃을 왜 놓치는지 한 단계씩 좁힌다.

measure_c 에서 라이다 포착률이 낮거나 σ 가 크게 나올 때 원인은 둘 중 하나다:

  (a) /scan 자체에 점이 없다
        -> 스캔 평면(base_link z=0.178m)이 물체를 안 지난다.
           물체 자세·높이, 라이다 수평 여부를 본다.
  (b) /scan 은 멀쩡한데 /lidar_clusters 에 안 나온다
        -> 클러스터링 단계에서 탈락. 가장 흔한 원인은 배경과 병합되어
           폭이 max_width(1.5m)를 넘어 통째로 필터에 걸리는 것.

이 스크립트는 같은 방위각을 두 토픽에서 동시에 보고 어느 쪽인지 알려준다.

사용:
    python3 probe_scan.py                      # 정면
    python3 probe_scan.py --bearing -20
    python3 probe_scan.py --expect 1.9         # 타깃 거리를 알면 더 정확히 판정
"""
import argparse
import math
import statistics as st
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from amr_interfaces.msg import DetectedObjectArray


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bearing', type=float, default=0.0, help='타깃 방위 (deg, 좌가 +)')
    p.add_argument('--window', type=float, default=12.0, help='좌우로 볼 범위 (deg)')
    p.add_argument('--duration', type=float, default=5.0)
    p.add_argument('--expect', type=float, default=None,
                   help='타깃 공칭 거리(m). 주면 배경과의 간격까지 판정한다')
    a = p.parse_args()

    lo = math.radians(a.bearing - a.window)
    hi = math.radians(a.bearing + a.window)

    rclpy.init()
    n = Node('probe_scan')
    bins = defaultdict(list)          # 각도(정수 deg) -> [거리...]
    scan_frames = [0]
    clus_frames = [0]
    clus_hit = [0]
    widths = []

    def on_scan(m):
        scan_frames[0] += 1
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if not (lo <= ang <= hi):
                continue
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                bins[round(math.degrees(ang))].append(r)

    def on_clusters(m):
        clus_frames[0] += 1
        near = [o for o in m.objects
                if lo <= math.atan2(o.position.y, o.position.x) <= hi]
        if near:
            clus_hit[0] += 1
            widths.extend(o.bbox.size_x for o in near)

    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    n.create_subscription(DetectedObjectArray, '/lidar_clusters', on_clusters, 50)

    print(f'{a.duration:.0f}초 관측: 방위 {a.bearing:+.0f}° ± {a.window:.0f}°')
    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(n, timeout_sec=0.1)

    print(f'\n/scan {scan_frames[0]}프레임   /lidar_clusters {clus_frames[0]}프레임')
    if scan_frames[0] == 0:
        print('\n  ✗ /scan 이 아예 안 옵니다. 스택이 떠 있는지, 라이다가 연결됐는지 확인하세요.')
        n.destroy_node(); rclpy.shutdown()
        return
    sf = scan_frames[0]
    print(f'\n  {"각도°":>6} {"거리 중앙값":>12} {"산포":>8} {"유효율":>8}')
    for deg in sorted(bins):
        v = bins[deg]
        sd = st.stdev(v) if len(v) > 1 else 0.0
        print(f'  {deg:6d} {st.median(v):12.3f} {sd:8.3f} {100*len(v)/sf:7.0f}%')

    print(f'\n  /lidar_clusters 포착률 {100*clus_hit[0]/max(1, clus_frames[0]):.1f}%', end='')
    if widths:
        print(f'   폭 평균 {st.mean(widths):.3f} m'
              + (f'  σ={st.stdev(widths):.3f}' if len(widths) > 1 else ''))
    else:
        print('   (클러스터 없음)')

    # ── 판정 ─────────────────────────────────────────────
    print()
    tgt = [d for d in bins if a.expect is None
           or abs(st.median(bins[d]) - a.expect) < 0.35]
    scan_ok = bool(tgt) and max(len(bins[d]) for d in tgt) / sf > 0.9
    clus_ok = clus_hit[0] / max(1, clus_frames[0]) > 0.9

    if not scan_ok:
        print('  → (a) /scan 단계에서 이미 안 잡힙니다.')
        print('     스캔 평면(base_link z=0.178m)이 물체를 제대로 안 지나고 있습니다.')
        print('     물체를 세우거나 위치를 바꿔서 17.8cm 높이에 두꺼운 단면이 오게 하세요.')
    elif not clus_ok:
        print('  → (b) /scan 은 멀쩡한데 클러스터가 안 나옵니다.')
        print('     배경과 병합되어 max_width(1.5m)에 걸렸을 가능성이 큽니다.')
        if a.expect is not None:
            bg = [st.median(bins[d]) for d in bins
                  if abs(st.median(bins[d]) - a.expect) >= 0.35]
            if bg:
                print(f'     타깃 {a.expect:.2f} m, 가장 가까운 배경 {min(bg):.2f} m '
                      f'(간격 {min(bg)-a.expect:+.2f} m)')
                print('     간격이 0.3m 미만이면 물체를 배경에서 더 떼어놓으세요.')
    else:
        print('  ✓ /scan 과 클러스터 모두 정상. 이 방위에서는 라이다가 잘 보고 있습니다.')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
