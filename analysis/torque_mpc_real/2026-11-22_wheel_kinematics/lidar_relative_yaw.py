#!/usr/bin/python3
"""앞·뒤 라이다 상대 장착 각 (2026-11-22). 로봇이 서 있는 구간의 /scan_front·/scan_rear 를 빔마다 평균 → 실행 TF (/tf_static) 로 base_footprint 에
옮긴 두 점구름을 비교. TF 가 맞으면 두 구름이 겹친다 (상대 yaw 0).
  (1) 2D ICP (점–점, 짝 거리 문턱 줄여 가며): 뒤 → 앞 정렬 (dθ, dx, dy). dθ = 뒤 라이다 yaw 오차 − 앞 라이다 yaw 오차.
  (2) 독립 확인: 앞 구름에서 RANSAC 으로 벽 직선 → 같은 자리의 뒤 점으로 직선 맞춤 → 두 직선 각 차.
두 라이다가 똑같이 돌아가 있으면 (공통 회전) 이 방법으로는 0 으로 보인다.
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-22_wheel_kinematics/lidar_relative_yaw.py bags/2026-11-22/193243 [...]"""
import math, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(b):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/scan_front", "/scan_rear", "/tf_static", "/wheel_states"]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    F, Rr, W, TF = [], [], [], {}
    while r.has_next():
        n, d, t = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tf_static":
            for tr in m.transforms:
                TF[tr.child_frame_id] = (tr.header.frame_id, tr.transform.translation.x, tr.transform.translation.y, quat_yaw(tr.transform.rotation))
        elif n == "/wheel_states":
            W.append((st(m.header), max(abs(v) for v in m.velocity)))
        else:
            (F if n == "/scan_front" else Rr).append((st(m.header), m.angle_min, m.angle_increment, np.array(m.ranges, float)))
    return F, Rr, np.array(W), TF


def to_base(TF, frame):
    x, y, a = 0.0, 0.0, 0.0
    f = frame
    while f in TF and f != "base_footprint":
        par, tx, ty_, ta = TF[f]
        x, y = tx + math.cos(ta) * x - math.sin(ta) * y, ty_ + math.sin(ta) * x + math.cos(ta) * y
        a += ta; f = par
    return x, y, a


def cloud(scans, tf, rmin=0.35, rmax=6.0):
    R = np.vstack([s[3] for s in scans]); R[~np.isfinite(R)] = np.nan
    rg = np.nanmedian(R, 0)                                         # 빔마다 중앙값 (정지)
    ang = scans[0][1] + np.arange(len(rg)) * scans[0][2]
    ok = np.isfinite(rg) & (rg > rmin) & (rg < rmax)
    px, py = rg[ok] * np.cos(ang[ok]), rg[ok] * np.sin(ang[ok])
    x, y, a = tf
    return np.stack([x + math.cos(a) * px - math.sin(a) * py, y + math.sin(a) * px + math.cos(a) * py], 1)


def icp(src, dst):
    tree = cKDTree(dst); th, t = 0.0, np.zeros(2)
    for thr in (0.20, 0.10, 0.05, 0.03, 0.02, 0.02, 0.02):
        for _ in range(10):
            c, s = math.cos(th), math.sin(th)
            S = src @ np.array([[c, s], [-s, c]]) + t
            d, j = tree.query(S); m = d < thr
            A, B = S[m], dst[j[m]]
            ma, mb = A.mean(0), B.mean(0)
            H = (A - ma).T @ (B - mb); dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            cc, ss = math.cos(dth), math.sin(dth); Rm = np.array([[cc, -ss], [ss, cc]])
            t = Rm @ t + (mb - Rm @ ma); th += dth
    return th, t, m.mean(), np.sqrt(np.mean(d[m] ** 2))


def ransac_lines(P, n_lines=4, tol=0.01, min_in=150, rng=np.random.default_rng(0)):
    out, rest = [], P.copy()
    for _ in range(n_lines):
        best = None
        for _ in range(3000):
            i, j = rng.choice(len(rest), 2, replace=False)
            dvec = rest[j] - rest[i]; L = np.linalg.norm(dvec)
            if L < 0.3:
                continue
            nrm = np.array([-dvec[1], dvec[0]]) / L
            inl = np.abs((rest - rest[i]) @ nrm) < tol
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < min_in:
            break
        out.append(rest[best]); rest = rest[~best]
    return out


def line_angle(Q):
    c = Q - Q.mean(0); _, _, vt = np.linalg.svd(c, full_matrices=False)
    return math.atan2(vt[0, 1], vt[0, 0])


if __name__ == "__main__":
    for b in sys.argv[1:]:
        F, Rr, W, TF = load(b)
        tff, tfr = to_base(TF, "laser_front"), to_base(TF, "laser_rear")
        # 정지 창: 바퀴 최대 |ω| < 0.5 rad/s (엔코더 정지 잡음 최대 0.2~0.4) 가 3 s 넘게 이어진 첫 구간 (최대 6 s)
        still = W[:, 1] < 0.5; idx = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
        segs = [(W[a, 0], W[e - 1, 0]) for a, e in zip(idx[::2], idx[1::2]) if W[e - 1, 0] - W[a, 0] > 3.0]
        if not segs:
            print(f"{b}: 정지 구간 없음"); continue
        t0, t1 = segs[0][0] + 0.5, min(segs[0][1] - 0.5, segs[0][0] + 6.5)
        fs = [s for s in F if t0 <= s[0] <= t1]; rs = [s for s in Rr if t0 <= s[0] <= t1]
        Pf, Pr = cloud(fs, tff), cloud(rs, tfr)
        th, t, frac, rms = icp(Pr, Pf)
        # 직선 각 비교 (앞에서 찾은 벽 → 같은 자리 뒤 점)
        lines = ransac_lines(Pf); diffs = []
        for Q in lines:
            a_f = line_angle(Q); nrm = np.array([-math.sin(a_f), math.cos(a_f)]); c0 = Q.mean(0)
            lo, hi = ((Q - c0) @ np.array([math.cos(a_f), math.sin(a_f)])).min(), ((Q - c0) @ np.array([math.cos(a_f), math.sin(a_f)])).max()
            u = (Pr - c0) @ np.array([math.cos(a_f), math.sin(a_f)])
            near = (np.abs((Pr - c0) @ nrm) < 0.04) & (u > lo) & (u < hi)
            if near.sum() > 50:
                a_r = line_angle(Pr[near]); da = (a_r - a_f + math.pi / 2) % math.pi - math.pi / 2
                diffs.append((len(Q), int(near.sum()), math.degrees(da), np.linalg.norm(c0)))
        print(f"== {b}: 정지 {t0:.1f}~{t1:.1f} (스캔 앞 {len(fs)}·뒤 {len(rs)} 장, 점 앞 {len(Pf)}·뒤 {len(Pr)})")
        print(f"   TF: 앞 ({tff[0]:+.4f}, {tff[1]:+.4f}, {math.degrees(tff[2]):+.1f}°), 뒤 ({tfr[0]:+.4f}, {tfr[1]:+.4f}, {math.degrees(tfr[2]):+.1f}°)")
        print(f"   ICP 뒤→앞: dθ {math.degrees(th):+.3f}°, dx {t[0]*1e3:+.1f} mm, dy {t[1]*1e3:+.1f} mm  (짝 {frac*100:.0f} %, 짝 거리 RMS {rms*1e3:.1f} mm)")
        for n_f, n_r, da, dist in diffs:
            print(f"   벽 직선 (앞 {n_f}·뒤 {n_r} 점, 거리 {dist:.2f} m): 뒤 − 앞 각 {da:+.3f}°")
