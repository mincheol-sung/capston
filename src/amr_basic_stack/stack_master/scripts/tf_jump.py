#!/usr/bin/env python3
"""bag 의 /tf 에서 map->odom 불연속을 잰다 — traversable map 이 map 프레임에
누적해도 되는지를 판정하는 도구.

왜 이 값이 중요한가:
  traversability_node 는 관측을 map 프레임 격자(cell)에 누적한다. map->odom 이
  움직이면 과거에 찍은 셀은 그대로 있고 새 관측만 새 좌표로 찍히므로 격자가
  어긋난다. 5 s 창 안의 배회폭이 1 셀(resolution 0.05 m) 을 넘으면 map 프레임
  누적은 성립하지 않는다.
  memory_seconds 와 free_memory_seconds 가 둘 다 0.0(만료 없음)이라 점프로
  잘못 찍힌 BLOCKED 와 잘못 뚫린 FREE 는 그 자리를 다시 관측하기 전까지 남는다.

무엇을 재는가 (2026-10-15 추가):
  odom 원점 xy  — map->odom 의 translation. 처음부터 있던 지표.
  로봇 위치 xy  — 창 시작 시각의 로봇 위치를 odom 좌표로 고정하고, 그 점이 map
                  에서 움직인 폭. 격자에 실제로 들어가는 오차는 이쪽이다. yaw 가
                  바뀌면 odom 원점과 로봇 위치가 다르게 움직이므로 원점 지표는
                  로봇이 원점에서 멀수록 틀린다.
  yaw          — map->odom yaw 의 폭. ENDPOINT_RANGE 거리의 스캔 끝점에서
                  range * dyaw 만큼 셀이 밀린다.
  09-23 bag 에서 원점 지표는 p90 7-9 셀, 로봇 위치는 3 셀, yaw 는 4 m 끝점 7-9 셀
  이었다. 주원인은 yaw 다. 로봇 위치에는 창 안 odom 자체의 drift 도 섞인다.

사용:
    python3 tf_jump.py <bag> [parent] [child] [base]
    python3 tf_jump.py ~/2026amr_ws/bags/tmap_20260909_...   # 기본 map->odom, base_footprint

/initialpose 근처(±3 s)는 사람이 찍은 리셋이라 상시 거동에서 제외한다.
"""
import bisect
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage

RESOLUTION = 0.05      # traversability.yaml 의 기준 맵 해상도
MEMORY_WINDOW = 5.0    # 배회폭 창. 2026-09-10 판정과 같은 값을 유지한다
ENDPOINT_RANGE = 4.0   # yaw 폭을 셀로 환산할 스캔 끝점 거리 [m]
RESET_GUARD = 3.0      # /initialpose 전후 이만큼을 상시 거동에서 뺀다


def read(bag, parent, child, base):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/initialpose']))
    pairs, samples, robot, resets = {}, [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/initialpose':
            resets.append(stamp * 1e-9)
            continue
        for tr in deserialize_message(data, TFMessage).transforms:
            key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            pairs[key] = pairs.get(key, 0) + 1
            if key == (parent, child):
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                # bag 수신시각을 쓴다. header.stamp 는 발행자가 미래로 찍기도 한다.
                samples.append((stamp * 1e-9, tr.transform.translation.x,
                                tr.transform.translation.y, yaw))
            elif key == (child, base):
                robot.append((stamp * 1e-9, tr.transform.translation.x,
                              tr.transform.translation.y))
    return pairs, sorted(samples), sorted(robot), resets


def report(title, values, scale, unit):
    w = np.sort(np.array(values))
    print(f'\n{title}, 창 {len(w)} 개:')
    for p in (50, 90, 99):
        v = w[int(len(w) * p / 100) - 1]
        print(f'  {p:2d} 백분위 {v*scale:8.1f} {unit} = {v/RESOLUTION:6.2f} 셀')
    print(f'  최대      {w[-1]*scale:8.1f} {unit} = {w[-1]/RESOLUTION:6.2f} 셀')
    return w[int(len(w) * 90 / 100) - 1]


def main():
    bag = sys.argv[1]
    parent = sys.argv[2] if len(sys.argv) > 2 else 'map'
    child = sys.argv[3] if len(sys.argv) > 3 else 'odom'
    base = sys.argv[4] if len(sys.argv) > 4 else 'base_footprint'
    pairs, samples, robot, resets = read(bag, parent, child, base)

    print('bag 안의 TF 쌍:')
    for key, count in sorted(pairs.items(), key=lambda kv: -kv[1])[:12]:
        print(f'  {count:7d}  {key[0]} -> {key[1]}')
    if not samples:
        print(f'\n{parent} -> {child} 가 없다.')
        return

    t = np.array([s[0] for s in samples])
    span = t[-1] - t[0]
    print(f'\n{parent} -> {child}: {len(samples)} 개, {span:.0f} s, '
          f'{len(samples)/span:.1f} Hz')
    print(f'/initialpose {len(resets)} 회')

    keep = [s for s in samples
            if all(abs(s[0] - r) > RESET_GUARD for r in resets)]
    print(f'리셋 +-{RESET_GUARD:.0f} s 를 뺀 상시 거동 표본 {len(keep)} / {len(samples)}')
    t = np.array([s[0] for s in keep])
    x = np.array([s[1] for s in keep])
    y = np.array([s[2] for s in keep])
    yaw = np.unwrap(np.array([s[3] for s in keep]))
    robot_t = np.array([s[0] for s in robot])

    step = np.hypot(np.diff(x), np.diff(y))
    print('\n연속 두 표본 사이 이동:')
    for mult, label in ((0.5, '0.5셀'), (1.0, '1셀'), (2.0, '2셀')):
        th = RESOLUTION * mult
        print(f'  {label}({th*1000:5.0f} mm) 초과 {100*np.mean(step > th):6.2f} %')

    wander, at_robot, turn = [], [], []
    for i in range(0, len(t), 25):
        j = bisect.bisect_left(t, t[i] + MEMORY_WINDOW)
        if j <= i + 1 or j > len(t):
            continue
        wander.append(math.hypot(x[i:j].ptp(), y[i:j].ptp()))
        turn.append(yaw[i:j].ptp())
        if len(robot_t):
            # 창 시작의 로봇 위치(child 좌표)를 고정하고 parent 좌표로 옮긴다.
            k = min(np.searchsorted(robot_t, t[i]), len(robot_t) - 1)
            px, py = robot[k][1], robot[k][2]
            c, s = np.cos(yaw[i:j]), np.sin(yaw[i:j])
            at_robot.append(math.hypot((x[i:j] + c * px - s * py).ptp(),
                                       (y[i:j] + s * px + c * py).ptp()))
    if not wander:
        return
    w = np.sort(np.array(wander))
    print(f'\n{MEMORY_WINDOW:.0f} s 창(memory_seconds) 안의 배회폭, 창 {len(w)} 개:')
    for p in (50, 90, 99):
        v = w[int(len(w) * p / 100) - 1]
        print(f'  {p:2d} 백분위 {v*1000:8.1f} mm = {v/RESOLUTION:6.2f} 셀')
    print(f'  최대      {w[-1]*1000:8.1f} mm = {w[-1]/RESOLUTION:6.2f} 셀')
    worst = []
    if at_robot:
        worst.append(report(f'로봇 위치 xy ({base} 를 창 시작에 고정) 배회폭',
                            at_robot, 1000, 'mm'))
    else:
        print(f'\n{child} -> {base} 가 없어 로봇 위치 배회폭은 건너뛴다.')
    turn = np.array(turn)
    worst.append(report(f'yaw 배회폭 ({ENDPOINT_RANGE:.0f} m 끝점 환산)',
                        turn * ENDPOINT_RANGE, 1000, 'mm'))
    print(f'  (yaw 90 백분위 {math.degrees(np.sort(turn)[int(len(turn) * 0.9) - 1]):.2f} deg)')
    print(f'\n판정: 로봇 위치 xy 와 yaw 끝점 환산 중 큰 쪽의 90 백분위 '
          f'{max(worst)/RESOLUTION:.2f} 셀.')
    print('  1 셀을 넘으면 map 프레임 누적은 성립하지 않는다.')


if __name__ == '__main__':
    main()
