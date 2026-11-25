#!/usr/bin/python3
"""path following 시뮬 기록기 (2026-11-23) — slip_rec.SlipRec (참값 λ) + 주행 시간·contouring 오차·기준 시각 늦음.
  주행 시간 = 참값 표본 중 기준이 움직이는 (|v_r| > 0.02) 첫 표본 ~ 마지막 표본 시각 차.
  contouring = 참값 (x, y) 에서 /figure8/path (latched, 10 배 선형 보간) 까지 최단 거리 — 기준 이동 중 표본.
  lag_end = /figure8/progress 마지막 t_clock − t_ref [s] (기준 시각이 시계보다 늦은 양, path following 이 아니면 0).
  npz (--npz): 참값 궤적 [t, x, y, 기준 이동 중] 과 경로 — 그림용."""
import argparse, importlib.util, json, math, os, signal, time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from std_msgs.msg import Float64MultiArray

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("slip_rec", os.path.join(HERE, "..", "2026-11-23_load_transfer", "slip_rec.py"))
sr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sr)


class PfRec(sr.SlipRec):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.path_xy, self.prog, self.tr = None, None, []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/figure8/path", self.on_path, latched)
        self.create_subscription(Float64MultiArray, "/figure8/progress", lambda m: setattr(self, "prog", list(m.data)), 20)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_tr, 50)

    def on_path(self, m):
        self.path_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])

    def on_tr(self, m):
        if not self.armed or self.ref is None:
            return
        t = sr.cr.stamp_s(m.header); _, ur = self.ref.at(t)
        self.tr.append([t, m.pose.pose.position.x, m.pose.pose.position.y, float(math.hypot(ur[0], ur[1]) > 0.02)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run"); ap.add_argument("--timeout", type=float, default=150.0); ap.add_argument("--npz", default="")
    a = ap.parse_args()
    rclpy.init()
    node = PfRec(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary(a.label)
    lam = np.array(node.lam, float).reshape(-1, 4); lam = lam[np.all(np.isfinite(lam), axis=1)]
    if len(lam):
        mx = np.abs(lam).max(axis=1)
        s.update(slip_frac10=float(np.mean(mx > 0.1)), slip_frac50=float(np.mean(mx > 0.5)), slip_max=float(mx.max()),
                 slip_rms=float(np.sqrt(np.mean(lam ** 2))))
    tr = np.array(node.tr).reshape(-1, 4)
    mv = tr[:, 3] > 0.5
    if mv.any():
        s["drive_time"] = float(tr[mv, 0].max() - tr[mv, 0].min())
    if node.path_xy is not None and mv.any():
        P = node.path_xy; u = np.linspace(0, len(P) - 1, 10 * (len(P) - 1) + 1)
        Pd = np.stack([np.interp(u, np.arange(len(P)), P[:, 0]), np.interp(u, np.arange(len(P)), P[:, 1])], 1)
        ec = cKDTree(Pd).query(tr[mv, 1:3])[0]
        s.update(cont_rms_mm=1e3 * float(np.sqrt(np.mean(ec ** 2))), cont_max_mm=1e3 * float(ec.max()))
    if node.prog is not None:
        s["lag_end_s"] = float(node.prog[2] - node.prog[1])
    if a.npz:
        np.savez(a.npz, tr=tr, path=node.path_xy if node.path_xy is not None else np.zeros((0, 2)))
    s["timed_out"] = time.monotonic() - t0 >= a.timeout
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
