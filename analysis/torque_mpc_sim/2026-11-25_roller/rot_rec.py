#!/usr/bin/python3
"""롤러 구름 저항 시뮬 기록기 (2026-11-25) — ../2026-11-23_path_follow/pf_rec.py (참값 λ 슬립·경로까지 거리) + 회전 지표.
  회전 속도 비 = 기준 회전 속도가 0.3 rad/s 넘는 표본에서 Σ sgn(ω_r)·ω_참값 / Σ|ω_r|  (실차 MPC 0.74~0.83)
  헤딩 오차 최대 [°], 회전 구간 헤딩 오차 RMS [°]. npz 에 [t, 헤딩 오차, 기준 ω, 참값 ω] 를 남긴다."""
import argparse, importlib.util, json, math, os, signal, time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("pf_rec", os.path.join(HERE, "..", "2026-11-23_path_follow", "pf_rec.py"))
pf = importlib.util.module_from_spec(spec); spec.loader.exec_module(pf)


class RotRec(pf.PfRec):
    def __init__(self, end_after):
        super().__init__(end_after)
        self.rot = []
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_rot, 50)

    def on_rot(self, m):
        if not self.armed or self.ref is None:
            return
        t = pf.sr.cr.stamp_s(m.header)
        Xr, ur = self.ref.at(t)
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z ** 2)
        self.rot.append([t, pf.sr.cr.wrap(yaw - Xr[2]), ur[2], m.twist.twist.angular.z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run"); ap.add_argument("--timeout", type=float, default=150.0); ap.add_argument("--npz", default="")
    a = ap.parse_args()
    rclpy.init()
    node = RotRec(1.5)
    stop = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(f=True))
    t0 = time.monotonic()
    while rclpy.ok() and not stop["f"] and not node.finished() and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    s = node.summary(a.label)
    lam = np.array(node.lam, float).reshape(-1, 4); lam = lam[np.all(np.isfinite(lam), axis=1)]
    if len(lam):
        mx = np.abs(lam).max(axis=1)
        s.update(slip_frac10=float(np.mean(mx > 0.1)), slip_frac50=float(np.mean(mx > 0.5)), slip_max=float(mx.max()))
    tr = np.array(node.tr).reshape(-1, 4)
    mv = tr[:, 3] > 0.5
    if node.path_xy is not None and mv.any():
        P = node.path_xy; u = np.linspace(0, len(P) - 1, 10 * (len(P) - 1) + 1)
        Pd = np.stack([np.interp(u, np.arange(len(P)), P[:, 0]), np.interp(u, np.arange(len(P)), P[:, 1])], 1)
        ec = cKDTree(Pd).query(tr[mv, 1:3])[0]
        s.update(cont_rms_mm=1e3 * float(np.sqrt(np.mean(ec ** 2))), cont_max_mm=1e3 * float(ec.max()))
    R = np.array(node.rot).reshape(-1, 4)
    if len(R):
        s["yaw_max_deg"] = math.degrees(float(np.abs(R[:, 1]).max()))
        rs = np.abs(R[:, 2]) > 0.3
        if rs.any():
            s["rot_ratio"] = float(np.sum(np.sign(R[rs, 2]) * R[rs, 3]) / np.sum(np.abs(R[rs, 2])))
            s["rot_yaw_rms_deg"] = math.degrees(float(np.sqrt(np.mean(R[rs, 1] ** 2))))
    if a.npz:
        np.savez(a.npz, tr=tr, rot=R, path=node.path_xy if node.path_xy is not None else np.zeros((0, 2)))
    s["timed_out"] = time.monotonic() - t0 >= a.timeout
    print(json.dumps(s, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
