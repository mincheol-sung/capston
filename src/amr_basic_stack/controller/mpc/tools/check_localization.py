#!/usr/bin/python3
"""측위가 저장 지도에 맞았는지 확인하고, 안 맞았으면 지도에서 위치를 찾아 /initialpose 명령을 만든다 (2026-11-14).

왜: Cartographer 측위는 켜질 때 로봇이 지도 원점 (0, 0, 0°)에 있다고 가정한다. 다른 곳에 놓였으면 멈춰 있는 동안
스스로 못 고친다(localization_2d.lua global_sampling_ratio 0.003, 움직일 때만). 출발 뒤 진짜 위치로 튀어 주행이 깨진다
(2026-11-14 jjm 1회차 2.7 m, 같은 날 밤 smc 판은 무장 전 이 검사로 1.3 m·64° 어긋남을 찾았다).

방법: /scan_matched_points2 (Cartographer 가 지금 포즈로 지도 좌표에 옮긴 라이다 점)가 저장 지도(png)의 벽 칸에 얼마나
겹치는지 센다. /map_carto 가 아니라 원본 png 와 비교한다 — /map_carto 에는 지금 쌓는 새 스캔도 들어 있어 자기와 비교가 된다.

  export ROS_DOMAIN_ID=81 && unset ROS_STATIC_PEERS        # 측위를 띄운 터미널과 같게
  /usr/bin/python3 tools/check_localization.py              # 확인만. 겹침이 낮으면 지도 전체에서 찾아 명령을 출력
  /usr/bin/python3 tools/check_localization.py --apply      # 찾은 위치를 /initialpose 로 넣고 다시 확인

판정 기준(잠정): ±5 cm 겹침 ≥ 60 % 이면 맞음. 2026-11-14 맞았을 때 83 %, 안 맞았을 때(1.3 m 어긋남) 21 %.
  2026-11-21 사용자 결정 70 → 60 %: 같은 자세 (−0.59, +1.49 m, −75°) 에서 11 번 중 10 번이 65~69 % 로 실패 — 탐색 자세 겹침은 늘 80~84 %,
  /initialpose 뒤 Cartographer 가 0.3~0.6° 더 돌아간 자리에 앉으며 떨어짐 (큰 어긋남 아님, 그 자세에서 띄운 184008 도 정상 주행).
  근거 analysis/torque_mpc_real/2026-11-21_square/RESULTS_KO.md 9 절.
--apply 뒤 Cartographer 첫 최적화까지 기다림 (2026-11-23): /initialpose 로 새 trajectory 가 시작되면 그 초기 포즈의 어긋남
  (이 도구의 탐색 오차, 실측 16~112 mm·0.3~1.7°) 은 로컬 스캔 매칭으로는 안 고쳐지고 포즈 그래프 최적화 (optimize_every_n_nodes 20,
  서 있으면 노드 0.5 s 마다 → 약 10 s 뒤) 에서 한 번에 고쳐진다. 그 전에 무장하면 기준이 안 고친 포즈에 묶이고, 출발 약 1 s 뒤 보정이
  /tracked_pose 계단으로 와서 제어기가 가짜 오차 (상태 추정기 넘침 포함 최대 17 cm) 를 쫓았다 (11-23 실차 5 판, 무장이 늦었던 120544 만 정상).
  → /submap_list 에서 새 trajectory 서브맵의 지도 좌표 포즈가 바뀌는 것 (= 최적화가 local_to_map 을 고침) 을 본 뒤 다시 확인하고 끝낸다.
  근거 analysis/torque_mpc_real/2026-11-23_max_compare/RESULTS_KO.md "SLAM 점프 원인".
종료 코드: 0 맞음, 1 측위 결과 없음, 2 찾았지만 안 넣음, 3 지도 어디에도 안 맞음, 4 넣었는데 아직 안 맞음, 5 넣었는데 새 trajectory 가 안 생김
  (보정이 0 이면 — 초기 포즈가 이미 정확 — 최적화가 안 보이므로 최대 30 s 기다린 뒤 정상으로 본다. 2026-11-23 오후 고침.
   보정이 30 s 뒤에 오면 못 잡는다 — 근본 해결은 이 도구의 초기 포즈 정확도)
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
try:
    from cartographer_ros_msgs.msg import SubmapList
except ImportError:                      # 메시지 패키지가 없으면 최적화 기다림 없이 예전처럼
    SubmapList = None

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAP = os.path.normpath(os.path.join(HERE, "../../../planner/global_planner/maps/smoc_lab/smoc_lab.yaml"))


def load_map(path):
    from PIL import Image
    meta = yaml.safe_load(open(path))
    img = np.array(Image.open(os.path.join(os.path.dirname(path), meta["image"])).convert("L")).astype(float) / 255.0
    occ = (img if meta.get("negate", 0) else 1.0 - img) > float(meta["occupied_thresh"])
    return occ[::-1, :], float(meta["resolution"]), float(meta["origin"][0]), float(meta["origin"][1])   # 행 0 = y 작은 쪽


def score(occ, res, ox, oy, p):
    h, w = occ.shape
    ix = ((p[:, 0] - ox) / res).astype(int)
    iy = ((p[:, 1] - oy) / res).astype(int)
    ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    return (occ[iy[ok], ix[ok]].mean() if ok.any() else 0.0), ok.mean()


def transform(p, dx, dy, dth):
    c, s = math.cos(dth), math.sin(dth)
    return p @ np.array([[c, s], [-s, c]]) + [dx, dy]


def search(occ2, occ1, res, ox, oy, p, pose):
    """지금 포즈를 기준으로 점들을 (회전 dth, 이동 d) 옮겨 가장 잘 맞는 곳. 반환 (겹침, 새 포즈 x, y, yaw)."""
    h, w = occ2.shape
    x0, y0, th0 = pose
    q = p - [x0, y0]                      # 추적점 기준으로 옮겨서 돌린다
    sub = q[:: max(1, len(q) // 400)]
    xs = ox + res * np.arange(0, w, 2)    # 10 cm 격자
    ys = oy + res * np.arange(0, h, 2)
    best = (0.0, x0, y0, th0)
    for dth in np.radians(np.arange(-180, 180, 5)):
        r = transform(sub, 0, 0, dth)
        for x in xs:
            ix = ((r[:, 0] + x - ox) / res).astype(int)
            if ix.min() < 0 or ix.max() >= w:
                continue
            for y in ys:
                iy = ((r[:, 1] + y - oy) / res).astype(int)
                if iy.min() < 0 or iy.max() >= h:
                    continue
                sc = occ2[iy, ix].mean()
                if sc > best[0]:
                    best = (sc, x, y, th0 + dth)
    # 다듬기: ±0.2 m (2 cm), ±5° (0.5°)
    _, bx, by, bth = best
    for dth in np.radians(np.arange(-5, 5.01, 0.5)):
        r = transform(q, 0, 0, bth - th0 + dth)
        for x in bx + np.arange(-0.2, 0.201, 0.02):
            for y in by + np.arange(-0.2, 0.201, 0.02):
                sc, _ = score(occ1, res, ox, oy, r + [x, y])
                if sc > best[0]:
                    best = (sc, x, y, bth + dth)
    return best


def optimization_moved(prev, msg, traj_min):
    """/submap_list 한 메시지 → traj_min 이상인 가장 큰 trajectory 에서 이미 본 서브맵이 지도 좌표로 옮겨진 최대 (거리 [m], 각 [rad]).
    서브맵은 그 trajectory 의 로컬 프레임에 고정이라, 지도 좌표 포즈가 바뀌었다 = 포즈 그래프 최적화가 local_to_map 을 고쳤다.
    처음 보는 서브맵은 기록만 한다. prev ({(traj, index): (x, y, yaw)}) 는 갱신된다.
    초기 포즈가 이미 정확하면 첫 최적화의 보정이 0.1 mm 도 안 된다 (11-23 151038·151249: 31.6 s 에 위치 < 0.1 mm) — 각도 같이 보고 문턱은 아주 작게."""
    ids = [e.trajectory_id for e in msg.submap]
    if not ids or max(ids) < traj_min:
        return 0.0, 0.0
    tmax, moved, turned = max(ids), 0.0, 0.0
    for e in msg.submap:
        if e.trajectory_id != tmax:
            continue
        q = e.pose.orientation
        key = (tmax, e.submap_index)
        cur = (e.pose.position.x, e.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if key in prev:
            moved = max(moved, math.hypot(cur[0] - prev[key][0], cur[1] - prev[key][1]))
            turned = max(turned, abs(math.atan2(math.sin(cur[2] - prev[key][2]), math.cos(cur[2] - prev[key][2]))))
        prev[key] = cur
    return moved, turned


class Probe(Node):
    def __init__(self):
        super().__init__("check_localization")
        self.cloud = None
        self.pose = None
        self.create_subscription(PointCloud2, "/scan_matched_points2", self.on_cloud, 5)
        self.create_subscription(PoseStamped, "/tracked_pose", self.on_pose, 10)
        self.submaps = []
        if SubmapList is not None:
            self.create_subscription(SubmapList, "/submap_list", self.submaps.append, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose",
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_cloud(self, m):
        self.cloud = np.array([[q[0], q[1]] for q in pc2.read_points(m, field_names=("x", "y"), skip_nans=True)])

    def on_pose(self, m):
        q = m.pose.orientation
        self.pose = (m.pose.position.x, m.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def max_trajectory(self, sec=3.0):
        """지금 /submap_list 의 가장 큰 trajectory id (못 받으면 None)."""
        t = time.time()
        while time.time() - t < sec and not self.submaps:
            rclpy.spin_once(self, timeout_sec=0.1)
        ids = [e.trajectory_id for m in self.submaps for e in m.submap]
        self.submaps.clear()
        return max(ids) if ids else None

    def wait_optimization(self, sec, traj_min):
        """traj_min 이상 trajectory 의 서브맵이 옮겨질 때까지 (= 첫 최적화의 보정), 최대 sec 초.
        반환 ("moved", 보정 [m], 걸린 [s]) | ("none", 0, sec) (새 trajectory 는 있었는데 sec 동안 안 옮겨짐 = 고칠 게 없었다) | None (새 trajectory 가 안 생김).
        초기 포즈가 정확하면 첫 최적화가 서브맵의 위치·방향을 전혀 안 바꾼다 (11-23 151038·151249: 적용 +12 s 최적화에 x·y·방향 변화 0,
        다음 변화가 +61 s 의 0.9 mm) — 그래서 못 봤다고 실패로 보지 않는다. 첫 최적화는 적용 +12.5 s 쯤인데,
        지도와의 제약을 아직 못 찾으면 보정 없이 지나가고 다음 최적화에서 고친다 (144542: +12.6 s 0 → +24.3 s 47 mm·1.2°) — 그래서 30 s."""
        prev, t, seen = {}, time.time(), False
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            while self.submaps:
                m = self.submaps.pop(0)
                seen = seen or any(e.trajectory_id >= traj_min for e in m.submap)
                moved, turned = optimization_moved(prev, m, traj_min)
                if moved > 1e-6 or turned > 1e-6:          # 최적화 없이는 같은 값이 그대로 다시 온다 — 아주 작은 변화도 최적화
                    return "moved", moved, time.time() - t
        return ("none", 0.0, sec) if seen else None

    def wait(self, sec):
        self.cloud = self.pose = None
        t = time.time()
        while time.time() - t < sec and (self.cloud is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.cloud is not None and self.pose is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=DEFAULT_MAP, help="지도 yaml (png 와 같은 폴더)")
    ap.add_argument("--min", type=float, default=0.60, help="맞았다고 볼 ±5 cm 겹침 비율 (2026-11-21 0.70 → 0.60)")
    ap.add_argument("--apply", action="store_true", help="찾은 위치를 /initialpose 로 넣고 다시 확인")
    a = ap.parse_args()
    occ, res, ox, oy = load_map(a.map)
    occ1, occ2 = binary_dilation(occ, iterations=1), binary_dilation(occ, iterations=2)
    rclpy.init()
    n = Probe()
    try:
        if not n.wait(10.0):
            sys.exit("10 s 동안 /scan_matched_points2·/tracked_pose 를 못 받음 — 측위가 떴는지, ROS_DOMAIN_ID 가 같은지 확인")
        p, pose = n.cloud, n.pose
        sc, inside = score(occ1, res, ox, oy, p)
        print(f"지금 측위 ({pose[0]:+.3f}, {pose[1]:+.3f}) m, {math.degrees(pose[2]):+.1f}°: "
              f"라이다 점 {len(p)} 개, 지도 벽 겹침 ±5 cm {100*sc:.1f} % (지도 안 {100*inside:.0f} %)")
        if sc >= a.min:
            print(f"→ 맞음 (기준 {100*a.min:.0f} %). 무장해도 된다 — 단 10 s 넘게 안정인지도 볼 것")
            return 0
        print("→ 안 맞음. 지도 전체에서 찾는 중 (1~2 분)...")
        bs, bx, by, bth = search(occ2, occ1, res, ox, oy, p, pose)
        print(f"찾은 위치 ({bx:+.2f}, {by:+.2f}) m, {math.degrees(bth):+.1f}°: 겹침 ±5 cm {100*bs:.1f} %")
        if bs < a.min:
            print("★ 지도 어디에도 잘 안 맞음 — 다른 방이거나, 주변이 지도 때와 많이 달라졌다. 위치를 손으로 정해야 한다")
            return 3
        qz, qw = math.sin(bth / 2), math.cos(bth / 2)
        cmd = (f'ros2 topic pub --once --qos-reliability best_effort /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '
               f'"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {bx:.2f}, y: {by:.2f}}}, '
               f'orientation: {{z: {qz:.4f}, w: {qw:.4f}}}}}}}}}"')
        if not a.apply:
            print("넣는 명령 (같은 ROS_DOMAIN_ID 터미널에서):\n  " + cmd + "\n또는 이 도구를 --apply 로 다시 실행")
            return 2
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y = bx, by
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        traj_before = n.max_trajectory() if SubmapList is not None else None
        for _ in range(3):
            n.pub.publish(m)
            time.sleep(0.1)
        if traj_before is None:
            print("/initialpose 보냄. (/submap_list 를 못 받아 Cartographer 최적화는 못 기다림) 5 s 뒤 다시 확인...")
            time.sleep(5.0)
        else:
            print("/initialpose 보냄. Cartographer 보정을 기다린다 (서 있으면 보통 +12.5 s, 최대 30 s) — 그 전에 무장하면 출발 뒤 위치가 튈 수 있다")
            opt = n.wait_optimization(30.0, traj_before + 1)
            if opt is None:
                print("★ 30 s 동안 새 trajectory 가 안 생김 — /initialpose 가 안 먹었다")
                return 5
            if opt[0] == "moved":
                print(f"Cartographer 첫 최적화: 초기 포즈를 {opt[1] * 1e3:.1f} mm 고침 ({opt[2]:.1f} s 기다림). 다시 확인...")
            else:
                print("30 s 동안 서브맵 보정 없음 — 최적화 (보통 +12.5 s, +24 s …) 가 고칠 게 없었다 (초기 포즈가 이미 맞음). 다시 확인...")
        if n.wait(10.0):
            sc2, _ = score(occ1, res, ox, oy, n.cloud)
            x, y, th = n.pose
            print(f"다시: ({x:+.3f}, {y:+.3f}) m, {math.degrees(th):+.1f}°, 겹침 ±5 cm {100*sc2:.1f} % → "
                  + ("맞음" if sc2 >= a.min else "★ 아직 안 맞음"))
            return 0 if sc2 >= a.min else 4
        print("★ /initialpose 뒤 10 s 동안 측위 결과를 못 받음")
        return 1
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
