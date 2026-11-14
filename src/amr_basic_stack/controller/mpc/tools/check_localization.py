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

판정 기준(잠정): ±5 cm 겹침 ≥ 70 % 이면 맞음. 2026-11-14 맞았을 때 83 %, 안 맞았을 때(1.3 m 어긋남) 21 %.
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


class Probe(Node):
    def __init__(self):
        super().__init__("check_localization")
        self.cloud = None
        self.pose = None
        self.create_subscription(PointCloud2, "/scan_matched_points2", self.on_cloud, 5)
        self.create_subscription(PoseStamped, "/tracked_pose", self.on_pose, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose",
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_cloud(self, m):
        self.cloud = np.array([[q[0], q[1]] for q in pc2.read_points(m, field_names=("x", "y"), skip_nans=True)])

    def on_pose(self, m):
        q = m.pose.orientation
        self.pose = (m.pose.position.x, m.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def wait(self, sec):
        self.cloud = self.pose = None
        t = time.time()
        while time.time() - t < sec and (self.cloud is None or self.pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.cloud is not None and self.pose is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=DEFAULT_MAP, help="지도 yaml (png 와 같은 폴더)")
    ap.add_argument("--min", type=float, default=0.70, help="맞았다고 볼 ±5 cm 겹침 비율")
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
            return
        print("→ 안 맞음. 지도 전체에서 찾는 중 (1~2 분)...")
        bs, bx, by, bth = search(occ2, occ1, res, ox, oy, p, pose)
        print(f"찾은 위치 ({bx:+.2f}, {by:+.2f}) m, {math.degrees(bth):+.1f}°: 겹침 ±5 cm {100*bs:.1f} %")
        if bs < a.min:
            print("★ 지도 어디에도 잘 안 맞음 — 다른 방이거나, 주변이 지도 때와 많이 달라졌다. 위치를 손으로 정해야 한다")
            return
        qz, qw = math.sin(bth / 2), math.cos(bth / 2)
        cmd = (f'ros2 topic pub --once --qos-reliability best_effort /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '
               f'"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {bx:.2f}, y: {by:.2f}}}, '
               f'orientation: {{z: {qz:.4f}, w: {qw:.4f}}}}}}}}}"')
        if not a.apply:
            print("넣는 명령 (같은 ROS_DOMAIN_ID 터미널에서):\n  " + cmd + "\n또는 이 도구를 --apply 로 다시 실행")
            return
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y = bx, by
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        for _ in range(3):
            n.pub.publish(m)
            time.sleep(0.1)
        print("/initialpose 보냄. 5 s 뒤 다시 확인...")
        time.sleep(5.0)
        if n.wait(10.0):
            sc2, _ = score(occ1, res, ox, oy, n.cloud)
            x, y, th = n.pose
            print(f"다시: ({x:+.3f}, {y:+.3f}) m, {math.degrees(th):+.1f}°, 겹침 ±5 cm {100*sc2:.1f} % → "
                  + ("맞음" if sc2 >= a.min else "★ 아직 안 맞음"))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
