#!/usr/bin/env python3
"""정지 상태에서 격자가 스스로 채워지는지(잡음 적립) 본다.

왜 이게 필요한가
----------------
로봇과 장면이 모두 정적이면 점유 셀 수는 **변하지 않아야** 한다. 그런데
늘어나면 관측이 아니라 **잡음이 적립**되고 있다는 뜻이다. octomap 의
log-odds 는 hit +0.847 / miss -0.405 로 비대칭이고(2.09 : 1), 포화된 voxel 을
임계 아래로 내리려면 miss 9 회가 필요하다 — 적립은 쉽고 해제는 어렵다.

이 스크립트로 갈리는 것
  · 정지 중 증가      -> log-odds 적립이 주범. voxel_leaf / sensor_model.max 로 간다
  · 정지 중 평평       -> 적립은 아니다. 주행 중을 봐야 한다
  · /projected_map 만 증가 -> 3D octomap 경로 문제
  · /traversable_map 만 증가 -> 2D 밴드 경로 문제 (depth_hit_threshold)

2026-09-11 실측: 정지 40 s 에서 5405 -> 5412 (+0.1 %), Δ 가 ±8 로 진동하고
감소도 나온다. **정지 적립은 없다.** 해제도 정상 동작한다.

그래서 주행 중 시험이 본론이다. octomap 은 frame_id: odom 으로 누적하므로
map->odom 튐에는 면역이지만 **odom 자체의 결함에는 그대로 노출된다** —
드라이버 stall 로 /odometry/filtered 가 predict-only 로 달리면 43.5 cm
(8.7 셀) phantom 변위가 생기고, 그 구간의 점군이 어긋난 자리에 꽂힌다.
그래서 이 스크립트는 /odom 공백도 같이 세어 표본 구간별로 나란히 찍는다.
공백이 난 구간에서 BLOCKED 가 튀면 그게 원인이다.

두 토픽 모두 nav_msgs/OccupancyGrid 라 octomap_msgs 를 import 하지 않는다.

값 규약은 traversability/encoding.py 와 같다:
    FREE 0,  BLOCKED 100,  UNKNOWN -1,  OCCUPIED_THRESHOLD 65

사용:
    python3 voxel_accum.py [총초=40] [표본간격초=5]

정지 시험: 로봇을 세워 두고 사람도 카메라 앞에서 움직이지 말 것.
주행 시험: 천천히 직진시키면서 돌린다. gap 열과 BLOCKED Δ 를 나란히 본다.

어느 쪽이든 스크립트가 실제 변위와 /odom 공백을 같이 찍으므로 사후에
"정지였나 주행이었나" 를 따로 확인할 필요가 없다.
"""
import math
import sys
import threading
import time

from nav_msgs.msg import OccupancyGrid, Odometry

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

OCCUPIED_THRESHOLD = 65      # encoding.py 와 같은 값
SENSOR_TIMEOUT = 0.1         # ekf.yaml 의 sensor_timeout. 넘으면 predict-only
GRIDS = ('/projected_map', '/traversable_map')


class Accum(Node):
    """두 격자와 odom 을 받아 표본 시점마다 점유 셀 수를 센다."""

    def __init__(self):
        super().__init__('voxel_accum')
        # 지도 토픽은 latched(TRANSIENT_LOCAL)로 내는 경우가 있다. 구독자를
        # VOLATILE 로 두면 두 경우 모두와 호환된다 — durability 는 구독자가
        # 요구하는 수준이 발행자가 제공하는 수준 이하일 때만 맞는다.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.latest = {t: None for t in GRIDS}
        for t in GRIDS:
            self.create_subscription(
                OccupancyGrid, t,
                lambda m, topic=t: self.latest.__setitem__(topic, m), qos)
        self.odom = []
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda m: self.odom.append(
                (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        # /odom 공백. ekf.yaml 의 sensor_timeout 0.1 s 를 넘으면 EKF 가
        # predict-only 로 마지막 속도를 계속 적분해 phantom 변위를 만든다
        # (2026-09-11 bag 실측 최대 1046 ms -> 43.5 cm = 8.7 셀).
        #
        # ★ 수신시각(time.monotonic())으로 재면 안 된다. 구독자가 못 따라가
        #   메시지를 흘리면 그게 발행자 공백처럼 보인다. header.stamp 는
        #   발행자가 찍은 값이라 구독자 부하와 무관하다.
        self.raw_stamps = []
        self.create_subscription(
            Odometry, '/odom',
            lambda m: self.raw_stamps.append(
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 200)

    def counts(self, topic):
        """최신 격자의 BLOCKED / FREE / UNKNOWN 셀 수. 아직 없으면 None."""
        m = self.latest[topic]
        if m is None:
            return None
        a = np.asarray(m.data, dtype=np.int16)
        return {
            'blocked': int(np.sum(a >= OCCUPIED_THRESHOLD)),
            'free': int(np.sum((a >= 0) & (a < OCCUPIED_THRESHOLD))),
            'unknown': int(np.sum(a < 0)),
            'cells': a.size,
            'res': m.info.resolution,
        }


def main():
    total = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    rclpy.init()
    node = Accum()

    # 콜백은 별도 스레드에서 쉬지 않고 처리한다. 본 루프에서 spin_once 를
    # 돌리면 호출당 콜백 하나만 처리해 초당 20 개가 상한이 되고, 토픽 3 개가
    # 각 20~50 Hz 로 오면 대부분을 흘린다 — 그러면 공백 측정이 무의미해진다.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 첫 메시지를 기다린다. 지도 발행이 1~2 Hz 라 넉넉히 준다.
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        time.sleep(0.1)
        if all(node.latest[t] is not None for t in GRIDS):
            break

    missing = [t for t in GRIDS if node.latest[t] is None]
    for t in missing:
        print(f'  X {t} 이 안 온다 — 발행자가 없거나 이름이 다르다')
    live = [t for t in GRIDS if node.latest[t] is not None]
    if not live:
        print('두 격자 모두 못 받았다. 스택이 떠 있는지, 도메인이 맞는지 확인할 것.')
        rclpy.shutdown()
        return 1

    for t in live:
        c = node.counts(t)
        print(f'  O {t}  {c["cells"]} 셀, {c["res"]:.3f} m/cell')
    print(f'\n{total:.0f} 초 동안 {step:.0f} 초 간격으로 표본. 로봇을 세워 둘 것.\n')

    hdr = f'{"t [s]":>7}'
    for t in live:
        hdr += f' | {t.split("/")[-1]:>16} {"Δ":>7}'
    hdr += f' | {"이동 [mm]":>10} {"/odom 공백":>11}'
    print(hdr)
    print('-' * (len(hdr) + 2))

    first, prev = {}, {}
    # 표본 시작 전(첫 메시지 대기 10 s)에 들어온 stamp 는 어느 구간에도
    # 속하지 않는다. 여기서 잘라내지 않으면 그 구간의 공백이 t=0 행에
    # 통째로 몰려 잘못된 상관으로 읽힌다.
    seen_stamps = len(node.raw_stamps)
    last_xy = node.odom[-1] if node.odom else None
    gap_events = []
    t0 = time.monotonic()
    while True:
        elapsed = time.monotonic() - t0
        row = f'{elapsed:7.1f}'
        for t in live:
            c = node.counts(t)
            b = c['blocked']
            first.setdefault(t, b)
            row += f' | {b:16d} {b - prev.get(t, b):+7d}'
            prev[t] = b

        # 이 구간에 움직인 거리
        xy = node.odom[-1] if node.odom else None
        moved = (math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1]) * 1000.0
                 if (xy and last_xy) else 0.0)
        last_xy = xy

        # 이 구간의 /odom 공백. 간격이 SENSOR_TIMEOUT 을 넘은 횟수와 최대값.
        #
        # ★ 슬라이스를 딱 잘라 나누면 안 된다. 공백은 stamp **쌍**으로
        #   나타나는데, 그 쌍이 표본 경계에 걸치면 앞 조각과 뒤 조각에
        #   하나씩 들어가 어느 구간에서도 안 보인다. 앞 표본의 마지막
        #   stamp 를 하나 겹쳐 시작해 경계 쌍을 살린다.
        lo = max(0, seen_stamps - 1)
        st = node.raw_stamps[lo:]
        seen_stamps = len(node.raw_stamps)
        gaps = [b - a for a, b in zip(st[:-1], st[1:]) if b - a > SENSOR_TIMEOUT]
        gap_txt = f'{len(gaps)}회 {max(gaps)*1000:.0f}ms' if gaps else '-'
        if not node.raw_stamps:
            gap_txt = 'odom 없음'
        elif len(st) < 2:
            gap_txt = '-'
        if gaps:
            gap_events.append((elapsed, len(gaps), max(gaps)))
        row += f' | {moved:10.0f} {gap_txt:>11}'
        print(row, flush=True)

        if elapsed >= total:
            break
        time.sleep(max(0.0, step - (time.monotonic() - t0 - elapsed)))

    # 정지였는지 검증 — 안 그랬으면 이 시험은 의미가 없다
    print()
    if len(node.odom) >= 2:
        a = np.array(node.odom)
        span = float(np.max(np.hypot(a[:, 0] - a[0, 0], a[:, 1] - a[0, 1])))
        print(f'주행 검사: /odometry/filtered 최대 변위 {span * 1000:.0f} mm '
              f'({len(node.odom)} 샘플)')
        if span > 0.05:
            print('  ★ 5 cm 넘게 움직였다 — 정지 시험이 아니다. 다시 할 것.')
    else:
        print('주행 검사: /odometry/filtered 를 못 받아 정지였는지 확인 못 했다')

    print()
    for t in live:
        grew = prev[t] - first[t]
        pct = 100.0 * grew / first[t] if first[t] else float('nan')
        verdict = ('적립 있음 — 잡음이 쌓인다' if grew > 0.02 * max(first[t], 1)
                   else '평평 — 적립 아님')
        print(f'{t}: {first[t]} -> {prev[t]}  ({grew:+d}, {pct:+.1f} %)  {verdict}')

    if node.raw_stamps and len(node.raw_stamps) > 2:
        st = node.raw_stamps
        allgaps = [b - a for a, b in zip(st[:-1], st[1:])]
        big = [g for g in allgaps if g > SENSOR_TIMEOUT]
        print(f'/odom 수신 {len(st)} 개, 평균 {len(st)/(st[-1]-st[0]):.1f} Hz, '
              f'최대 간격 {max(allgaps)*1000:.0f} ms')
        if big:
            print(f'  ★ sensor_timeout({SENSOR_TIMEOUT*1000:.0f} ms) 초과 공백 '
                  f'{len(big)} 회, 최대 {max(big)*1000:.0f} ms')
            print('    그 구간 EKF 는 predict-only 로 달린다 — 마지막 속도 v 에')
            print('    대해 phantom 변위 ~ v x 공백. 0.8 m/s x 0.5 s = 40 cm = 8 셀.')
            print('    공백이 난 구간에서 BLOCKED 가 튀었으면 그게 원인이다.')
        else:
            print('  공백 없음 — 이 구간에서는 odom phantom 이 원인이 아니다')
    else:
        print('/odom 을 못 받아 공백을 확인하지 못했다 (드라이버 미기동?)')

    print('\n판정 안내')
    print('  정지인데 증가        -> log-odds 적립. voxel_leaf 0.03->0.06,')
    print('                          sensor_model.max 0.97->0.85 순서')
    print('  주행 중 증가 + 공백  -> odom phantom. 드라이버 stall 을 먼저 고친다')
    print('  주행 중 증가, 공백 X -> 횡방향 잡음. 거리 컷 / depth_hit_threshold')
    print('  /traversable_map 만  -> 2D 밴드. depth_hit_threshold 2->4')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
