#!/usr/bin/env python3
"""시뮬레이터(Isaac Sim) 연동용 — RViz 클릭 없이 초기위치·목표·arm 을 명령으로 준다.

문제: cartographer 의 map 프레임은 매핑을 시작한 순간 tracking_frame
(mapping_2d.lua: camera_imu_optical_frame) 의 자세로 정해진다. 그래서 시뮬레이터의
world 프레임과 map 프레임 사이 변환 T_world_map 은 사전에 알 수 없고, 매핑 중에
"같은 순간의 두 pose" 로 한 번 재서 지도 옆에 저장해 둬야 한다.

    T_world_map = T_world_base · T_map_base⁻¹

  T_world_base : 시뮬레이터 ground truth  (/ground_truth, PoseStamped, base_footprint)
  T_map_base   : 스택의 측위 결과          (/odometry/map, ekf_carto 출력, map→base_footprint)

사용 순서
  1) 매핑 중(로봇 정지 상태에서):
       sim_nav.py anchor --map isaac_room            # 두 pose 를 N 회 평균해 anchor 저장
     저장 위치: <maps_dir>/<map>/world_map_anchor.yaml  (finish_map 과 같은 폴더)
  2) 측위 모드로 autonomy.launch.py 를 띄운 뒤:
       sim_nav.py initialpose --map isaac_room       # GT 를 map 프레임으로 바꿔 /initialpose
       sim_nav.py goal --map isaac_room --world 5.0 -2.0 90    # world 좌표 목표 (yaw deg)
       sim_nav.py goal --map-xy 3.0 1.0 0                       # map 좌표로 직접
       sim_nav.py arm      /  sim_nav.py disarm                  # /mpc/arm (std_msgs/Bool)
  3) sim_nav.py where --map isaac_room     # GT·측위·anchor 로 계산한 map pose 를 나란히 출력
                                           # → 측위 오차 = |측위 − GT⁻¹변환| 이 검증 지표
  self-test: sim_nav.py selftest           # 변환 합성 검산 (ROS 불필요)

QoS: /initialpose 구독자가 둘이다 — set_initial_pose_node(BEST_EFFORT) 와
rviz_theta_star_node. RELIABLE 발행자는 두 종류 구독자 모두와 호환된다.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# ─────────────────────────── SE(2) 도우미 ────────────────────────────
def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_inv(T):
    R = T[:2, :2]
    t = T[:2, 2]
    Ti = np.eye(3)
    Ti[:2, :2] = R.T
    Ti[:2, 2] = -R.T @ t
    return Ti


def se2_xyyaw(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def mean_pose(samples):
    """(x, y, yaw) 표본 평균. yaw 는 단위벡터 평균으로 wrap 문제를 피한다."""
    a = np.asarray(samples, dtype=float)
    yaw = math.atan2(np.sin(a[:, 2]).mean(), np.cos(a[:, 2]).mean())
    return float(a[:, 0].mean()), float(a[:, 1].mean()), yaw


def anchor_path(maps_dir, map_name):
    return os.path.join(maps_dir, map_name, 'world_map_anchor.yaml')


def default_maps_dir():
    # finish_map / cartographer.launch.py 와 같은 규칙: 소스 트리의 global_planner/maps.
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..', 'planner', 'global_planner', 'maps'))


def load_anchor(maps_dir, map_name):
    p = anchor_path(maps_dir, map_name)
    if yaml is None:
        raise SystemExit('pyyaml 이 없다')
    if not os.path.exists(p):
        raise SystemExit(f'anchor 없음: {p}\n  매핑 중 "sim_nav.py anchor --map {map_name}" 을 먼저 실행할 것')
    with open(p) as f:
        d = yaml.safe_load(f)
    return se2(float(d['x']), float(d['y']), math.radians(float(d['yaw_deg'])))


def selftest():
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        T_wm = se2(*rng.uniform(-10, 10, 2), rng.uniform(-math.pi, math.pi))
        T_mb = se2(*rng.uniform(-10, 10, 2), rng.uniform(-math.pi, math.pi))
        T_wb = T_wm @ T_mb
        est = T_wb @ se2_inv(T_mb)               # anchor 식
        back = se2_inv(est) @ T_wb               # initialpose 식 → T_mb 복원
        worst = max(worst, np.abs(est - T_wm).max(), np.abs(back - T_mb).max())
    print(f'selftest: 200 무작위 포즈, 최대 오차 {worst:.2e}  ({"OK" if worst < 1e-9 else "FAIL"})')
    return 0 if worst < 1e-9 else 1


# ─────────────────────────── ROS 노드 ────────────────────────────
def make_node(name):
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    return rclpy, Node(name)


def wait_subscribers(rclpy, node, pub, timeout=6.0):
    """발행 직후 discovery 가 안 끝나면 메시지가 유실된다 (2026-09-14 finish_map 트리거 실패).
    구독자가 한 명이라도 보일 때까지 기다린다."""
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    n = pub.get_subscription_count()
    if n == 0:
        print(f'경고: {pub.topic_name} 구독자 없음 ({timeout:.0f}s) — 발행은 하지만 아무도 못 받는다')
    return n


def collect_pair(rclpy, node, n, timeout):
    """/ground_truth 와 /odometry/map 을 n 쌍 모아 (gt_list, loc_list) 로 돌려준다."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    gt, loc = [], []
    latest = {'gt': None, 'loc': None}

    def on_gt(m):
        latest['gt'] = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))

    def on_loc(m):
        p = m.pose.pose
        latest['loc'] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if latest['gt'] is not None:
            gt.append(latest['gt'])
            loc.append(latest['loc'])

    node.create_subscription(PoseStamped, '/ground_truth', on_gt, 20)
    node.create_subscription(Odometry, '/odometry/map', on_loc, 20)
    t0 = time.monotonic()
    while len(loc) < n and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(loc) < n:
        raise SystemExit(f'{timeout}s 안에 {len(loc)}/{n} 쌍 — /ground_truth(Isaac bridge) 와 '
                         f'/odometry/map(ekf_carto, cartographer.launch.py) 둘 다 나오는지 확인')
    return gt, loc


def cmd_anchor(args):
    rclpy, node = make_node('sim_nav_anchor')
    gt, loc = collect_pair(rclpy, node, args.samples, args.timeout)
    gtm, locm = mean_pose(gt), mean_pose(loc)
    gsd = np.asarray(gt)[:, :2].std(axis=0).max()
    lsd = np.asarray(loc)[:, :2].std(axis=0).max()
    if max(gsd, lsd) > args.max_std:
        node.get_logger().warn(f'표본 흩어짐 gt {gsd*100:.1f} cm / loc {lsd*100:.1f} cm — 로봇을 세우고 다시')
    T_wm = se2(*gtm) @ se2_inv(se2(*locm))
    x, y, yaw = se2_xyyaw(T_wm)
    p = anchor_path(args.maps_dir, args.map)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, 'w') as f:
        yaml.safe_dump({
            'x': round(x, 5), 'y': round(y, 5), 'yaw_deg': round(math.degrees(yaw), 4),
            'note': 'T_world_map = T_world_base * inv(T_map_base). world = Isaac ground truth, '
                    'map = cartographer. sim_nav.py anchor 가 저장.',
            'samples': args.samples,
            'gt_mean': [round(v, 5) for v in gtm], 'loc_mean': [round(v, 5) for v in locm],
        }, f, allow_unicode=True, sort_keys=False)
    print(f'anchor 저장: {p}\n  T_world_map  x {x:.4f}  y {y:.4f}  yaw {math.degrees(yaw):.3f}°'
          f'\n  gt 평균 {tuple(round(v, 4) for v in gtm)}   loc 평균 {tuple(round(v, 4) for v in locm)}')
    rclpy.shutdown()


def latest_gt(rclpy, node, timeout):
    from geometry_msgs.msg import PoseStamped
    box = {}
    node.create_subscription(PoseStamped, '/ground_truth',
                             lambda m: box.__setitem__('p', (m.pose.position.x, m.pose.position.y,
                                                              yaw_of(m.pose.orientation))), 10)
    t0 = time.monotonic()
    while 'p' not in box and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'p' not in box:
        raise SystemExit('/ground_truth 없음 — Isaac bridge 가 떠 있는지 확인')
    return box['p']


def cmd_initialpose(args):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy, node = make_node('sim_nav_initialpose')
    T_wm = load_anchor(args.maps_dir, args.map)
    gx, gy, gyaw = latest_gt(rclpy, node, args.timeout)
    x, y, yaw = se2_xyyaw(se2_inv(T_wm) @ se2(gx, gy, gyaw))
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    wait_subscribers(rclpy, node, pub)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.25      # RViz 2D Pose Estimate 와 같은 값
    m.pose.covariance[35] = 0.0685
    for _ in range(args.repeat):
        m.header.stamp = node.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.3)
    print(f'/initialpose ← map ({x:.3f}, {y:.3f}, {math.degrees(yaw):.2f}°)'
          f'   [GT world ({gx:.3f}, {gy:.3f}, {math.degrees(gyaw):.2f}°)]')
    rclpy.shutdown()


def cmd_goal(args):
    from geometry_msgs.msg import PoseStamped
    rclpy, node = make_node('sim_nav_goal')
    if args.world is not None:
        T_wm = load_anchor(args.maps_dir, args.map)
        wx, wy, wyaw = args.world[0], args.world[1], math.radians(args.world[2])
        x, y, yaw = se2_xyyaw(se2_inv(T_wm) @ se2(wx, wy, wyaw))
    else:
        x, y, yaw = args.map_xy[0], args.map_xy[1], math.radians(args.map_xy[2])
    pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    wait_subscribers(rclpy, node, pub)
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y = x, y
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    for _ in range(args.repeat):
        m.header.stamp = node.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.3)
    print(f'/goal_pose ← map ({x:.3f}, {y:.3f}, {math.degrees(yaw):.2f}°)')
    rclpy.shutdown()


def cmd_arm(args, value):
    from std_msgs.msg import Bool
    rclpy, node = make_node('sim_nav_arm')
    pub = node.create_publisher(Bool, '/mpc/arm', 10)
    wait_subscribers(rclpy, node, pub)
    for _ in range(3):
        pub.publish(Bool(data=value))
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f'/mpc/arm ← {value}')
    rclpy.shutdown()


def cmd_where(args):
    rclpy, node = make_node('sim_nav_where')
    T_wm = load_anchor(args.maps_dir, args.map)
    gt, loc = collect_pair(rclpy, node, args.samples, args.timeout)
    gtm, locm = mean_pose(gt), mean_pose(loc)
    x, y, yaw = se2_xyyaw(se2_inv(T_wm) @ se2(*gtm))
    ex, ey, eyaw = locm[0] - x, locm[1] - y, wrap(locm[2] - yaw)
    print(f'GT→map   ({x:.3f}, {y:.3f}, {math.degrees(yaw):.2f}°)\n'
          f'측위 map ({locm[0]:.3f}, {locm[1]:.3f}, {math.degrees(locm[2]):.2f}°)\n'
          f'오차     {math.hypot(ex, ey)*100:.1f} cm / {math.degrees(eyaw):.2f}°   ({args.samples} 표본 평균)')
    rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(p, need_map=True):
        p.add_argument('--map', required=need_map, help='맵 이름 (planner/global_planner/maps/<map>)')
        p.add_argument('--maps_dir', default=default_maps_dir())
        p.add_argument('--timeout', type=float, default=15.0)

    p = sub.add_parser('anchor', help='매핑 중 T_world_map 을 재서 저장'); common(p)
    p.add_argument('--samples', type=int, default=50)
    p.add_argument('--max_std', type=float, default=0.02, help='표본 흩어짐 경고 문턱 [m]')
    p = sub.add_parser('initialpose', help='GT → map 으로 바꿔 /initialpose'); common(p)
    p.add_argument('--repeat', type=int, default=3)
    p = sub.add_parser('goal', help='/goal_pose'); common(p, need_map=False)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--world', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'), help='world 좌표 (anchor 필요, --map)')
    g.add_argument('--map-xy', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'), help='map 좌표 직접')
    p.add_argument('--repeat', type=int, default=3)
    sub.add_parser('arm', help='/mpc/arm true')
    sub.add_parser('disarm', help='/mpc/arm false')
    p = sub.add_parser('where', help='GT 기준 측위 오차'); common(p)
    p.add_argument('--samples', type=int, default=20)
    sub.add_parser('selftest')

    a = ap.parse_args(argv)
    if a.cmd == 'selftest':
        return selftest()
    if a.cmd == 'goal' and a.world is not None and not a.map:
        ap.error('--world 는 --map 이 필요하다 (anchor 를 읽는다)')
    return {'anchor': cmd_anchor, 'initialpose': cmd_initialpose, 'goal': cmd_goal,
            'arm': lambda x: cmd_arm(x, True), 'disarm': lambda x: cmd_arm(x, False),
            'where': cmd_where}[a.cmd](a) or 0


if __name__ == '__main__':
    sys.exit(main())
