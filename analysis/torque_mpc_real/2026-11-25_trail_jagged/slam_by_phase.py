#!/usr/bin/python3
"""SLAM 흔들림이 어디서 나나: 꼭짓점 (진행 방향이 도는 중) vs 변, 변 중에서도 앞뒤로 가는 변 vs 옆으로 (크랩) 가는 변 (2026-11-25).
SLAM 이동 − 바퀴 이동 (≈ 0.1 s, 몸체 좌표) 에서 0.5 s 이동중앙값을 뺀 크기 (slam_noise.py 와 같은 값) 를 나눠 본다.
  꼭짓점 = 바퀴 속도 방향 (몸체 좌표) 이 0.6 rad/s 넘게 도는 중이거나 속도 < 0.3 m/s
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/slam_by_phase.py bag [bag ...]"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OFF = np.array([0.1093, 0.017]); R_W = 0.04
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rows = {}
for BAG in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states"])); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S, W = [], []
    while r.has_next():
        n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tracked_pose":
            p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
            S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps))
        else:
            W.append((st(m.header), *m.velocity))
    if len(S) < 50 or len(W) < 50: continue
    S, W = np.array(S), np.array(W)
    tw = W[:, 0]; om = W[:, 1:5]
    vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
    cx = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]); cy = np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1])
    ps = np.unwrap(S[:, 3]); ym = 0.5 * (ps[1:] + ps[:-1]); dx, dy = np.diff(S[:, 1]), np.diff(S[:, 2])
    rx = (np.cos(ym) * dx + np.sin(ym) * dy) - np.diff(np.interp(ts, tw, cx))
    ry = (-np.sin(ym) * dx + np.cos(ym) * dy) - np.diff(np.interp(ts, tw, cy))
    vx, vy = np.interp(tm, tw, vxw), np.interp(tm, tw, vyw); sp = np.hypot(vx, vy)
    dirn = np.unwrap(np.arctan2(vy, vx)); turn = np.abs(np.gradient(dirn, tm))
    def dt_(x):
        out = np.empty_like(x)
        for i in range(len(x)):
            out[i] = x[i] - np.median(x[np.abs(tm - tm[i]) <= 0.25])
        return out
    h = np.hypot(dt_(rx), dt_(ry))
    mov = sp > 0.1
    corner = mov & ((turn > 0.6) | (sp < 0.3)); edge = mov & ~corner
    fwd = edge & (np.abs(vx) >= np.abs(vy)); side = edge & (np.abs(vy) > np.abs(vx))
    kind = os.path.basename(BAG).split("_")[1]
    yr = np.abs(np.diff(ps) / np.diff(ts))                              # SLAM 요 각속도 [rad/s]
    fast = np.where(yr > 1.0)[0]
    since = np.array([tm[i] - tm[fast[fast <= i][-1]] if np.any(fast <= i) else 99.0 for i in range(len(tm))])
    for lo, hi in ((0, 0.3), (0.3, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 99)):
        rows.setdefault((kind, f"요 각속도 {lo}~{hi} rad/s"), []).append(h[mov & (yr >= lo) & (yr < hi)])
    for lab, m in (("변: 빠른 회전 (>1 rad/s) 뒤 0.5 s 안", edge & (since < 0.5)), ("변: 그 뒤", edge & (since >= 0.5))):
        rows.setdefault((kind, lab), []).append(h[m])
    rows.setdefault((kind, "요 각속도 최대·95 % (rad/s)"), []).append(np.array([yr[mov].max(), np.percentile(yr[mov], 95)]))
    # 월드 좌표 가속도 크기 |d(R(ψ) v)/dt| (구심 포함) — Cartographer 외삽 (바퀴 오돔 없이 등속) 이 0.2 s (반쪽 스캔 두 장) 에 어긋나는 크기 ½ a t²
    psi_m = np.interp(tm, ts, ps); vwx = np.cos(psi_m) * vx - np.sin(psi_m) * vy; vwy = np.sin(psi_m) * vx + np.cos(psi_m) * vy
    aw = np.hypot(np.gradient(vwx, tm), np.gradient(vwy, tm))
    for lo, hi in ((0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 99)):
        rows.setdefault(("전체", f"|a_월드| {lo}~{hi} m/s² (½a(0.2 s)² = {500 * lo * 0.04:.0f}~{500 * min(hi, 6) * 0.04:.0f} mm)"), []).append(h[mov & (aw >= lo) & (aw < hi)])
        rows.setdefault((kind, f"|a_월드| {lo}~{hi} m/s²"), []).append(h[mov & (aw >= lo) & (aw < hi)])
    for lab, m in (("꼭짓점", corner), ("변 앞뒤", fwd), ("변 옆 (크랩)", side)):
        rows.setdefault((kind, lab), []).append(h[m])
print("| 경로 | 구간 | 간격 수 | SLAM 흔들림 RMS mm | 95 % mm | > 2 cm 비율 % |\n|---|---|---|---|---|---|")
for (kind, lab), hs in sorted(rows.items()):
    h = np.concatenate(hs)
    if lab.startswith("요 각속도 최대"):
        print(f"| {kind} | {lab} | {len(hs)} 판 | 최대 {h[0::2].max():.2f} | 95 % 중앙값 {np.median(h[1::2]):.2f} | |"); continue
    if len(h) < 5: continue
    print(f"| {kind} | {lab} | {len(h)} | {1e3 * np.sqrt(np.mean(h ** 2)):.1f} | {1e3 * np.percentile(h, 95):.1f} | {100 * np.mean(h > 0.02):.0f} |")
