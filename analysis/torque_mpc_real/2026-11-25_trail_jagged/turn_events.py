#!/usr/bin/python3
"""SLAM 흔들림이 '빠른 방향 전환' 에서만 나나 (2026-11-25). 11-25 실차 판 전부.
SLAM 흔들림 h = SLAM 이동 − 바퀴 이동 (0.1 s, 몸체 좌표) 에서 0.5 s 이동중앙값을 뺀 크기 (slam_noise.py 와 같은 정의).
움직임은 SLAM 과 무관한 센서로 가른다: 요 각속도·요 각가속도 = 자이로 (/imu/base_link, 5 Hz 저역통과 뒤 미분), 진행 속도 = 바퀴.
  A 달리며 방향 전환     |ψ̈| > 4 rad/s² 이고 속도 > 0.3 m/s
  B 서서·천천히 방향 전환 |ψ̈| > 4 이고 속도 ≤ 0.3
  C 꾸준히 빠르게 돎      |ψ̇| > 1 rad/s 이고 |ψ̈| ≤ 4
  D 직진·완만            나머지 중 속도 > 0.1
  E 정지                 속도 ≤ 0.1 이고 |ψ̇| < 0.1
  (|ψ̈| 는 그 SLAM 간격 ±0.1 s 안의 최대)
그리고 '방향 전환 시작' (자이로 |ψ̇| 가 1 rad/s 를 위로 넘는 순간, 그때 속도 > 0.3) 기준 −1~+1.5 s 의 평균 h.
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-25_trail_jagged/turn_events.py bag [bag ...]   → out_turn_events.md"""
import math, os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__))
OFF = np.array([0.1093, 0.017]); R_W = 0.04
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
CLS = ["A 달리며 방향 전환", "A 직후 0.5 s (D·C 중)", "B 서서·천천히 방향 전환", "C 꾸준히 빠르게 돎", "D 직진·완만", "E 정지"]
acc = {}; trig = {}; POOL = []
for BAG in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/tracked_pose", "/wheel_states", "/imu/base_link"]))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    S, W, G = [], [], []
    while r.has_next():
        n, d, _ = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/tracked_pose":
            p = m.pose; ps = yaw_of(p.orientation); c, s = math.cos(ps), math.sin(ps)
            S.append((st(m.header), p.position.x - (c * OFF[0] - s * OFF[1]), p.position.y - (s * OFF[0] + c * OFF[1]), ps))
        elif n == "/wheel_states":
            W.append((st(m.header), *m.velocity))
        else:
            G.append((st(m.header), m.angular_velocity.z))
    if min(len(S), len(W), len(G)) < 50: continue
    S, W, G = map(np.array, (S, W, G))
    kind = os.path.basename(BAG).split("_")[1]
    tw = W[:, 0]; om = W[:, 1:5]
    vxw = R_W / 4 * om.sum(1); vyw = R_W / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3])
    cx = np.concatenate([[0], np.cumsum(vxw[1:] * np.diff(tw))]); cy = np.concatenate([[0], np.cumsum(vyw[1:] * np.diff(tw))])
    tg = G[:, 0]; fg = 1 / np.median(np.diff(tg)); b, a = butter(2, 5 / (fg / 2))
    wz = filtfilt(b, a, G[:, 1]); wdot = np.gradient(wz, tg)
    ts = S[:, 0]; tm = 0.5 * (ts[1:] + ts[:-1])
    ps = np.unwrap(S[:, 3]); ym = 0.5 * (ps[1:] + ps[:-1]); dx, dy = np.diff(S[:, 1]), np.diff(S[:, 2])
    rx = (np.cos(ym) * dx + np.sin(ym) * dy) - np.diff(np.interp(ts, tw, cx))
    ry = (-np.sin(ym) * dx + np.cos(ym) * dy) - np.diff(np.interp(ts, tw, cy))
    dtr = lambda x: np.array([x[i] - np.median(x[np.abs(tm - tm[i]) <= 0.25]) for i in range(len(x))])
    h = np.hypot(dtr(rx), dtr(ry))
    sp = np.hypot(np.interp(tm, tw, vxw), np.interp(tm, tw, vyw))
    yr = np.abs(np.interp(tm, tg, wz))
    ya = np.array([np.abs(wdot[(tg >= t - 0.1) & (tg <= t + 0.1)]).max(initial=0.0) for t in tm])
    A = (ya > 4) & (sp > 0.3); B = (ya > 4) & (sp <= 0.3)
    C = ~A & ~B & (yr > 1); E = ~A & ~B & ~C & (sp <= 0.1) & (yr < 0.1); D = ~A & ~B & ~C & ~E & (sp > 0.1)
    POOL.append(np.c_[ya, yr, sp, h])
    tA = tm[A]
    after = np.array([np.any((tA < t) & (tA >= t - 0.5)) for t in tm]) & (C | D)
    D2 = D & ~after; C2 = C & ~after
    for lab, m in zip(CLS, (A, after, B, C2, D2, E)):
        acc.setdefault((kind, lab), []).append(h[m]); acc.setdefault(("전체", lab), []).append(h[m])
    # 방향 전환 시작 기준 평균
    up = np.where((np.abs(wz[1:]) > 1) & (np.abs(wz[:-1]) <= 1))[0] + 1
    for i in up:
        t0 = tg[i]
        if np.interp(t0, tw, np.hypot(vxw, vyw)) <= 0.3: continue
        rel = tm - t0
        for k in range(-10, 15):
            m = (rel >= k * 0.1) & (rel < (k + 1) * 0.1)
            if m.any(): trig.setdefault((kind, k), []).extend(h[m])
out = ["# SLAM 흔들림 vs 움직임 종류 (11-25 실차 판, 움직임은 자이로·바퀴로 가름)\n",
       "| 경로 | 구간 | SLAM 간격 수 | 흔들림 RMS mm | 95 % mm | > 2 cm 비율 % | 전체 > 2 cm 중 몫 % |", "|---|---|---|---|---|---|---|"]
for kind in ["전체", "sq2p", "sq2", "c8d2"]:
    tot = sum(int((np.concatenate(acc[(kind, l)]) > 0.02).sum()) for l in CLS if (kind, l) in acc)
    for lab in CLS:
        if (kind, lab) not in acc: continue
        x = np.concatenate(acc[(kind, lab)])
        if len(x) < 5: continue
        big = int((x > 0.02).sum())
        out.append(f"| {kind} | {lab} | {len(x)} | {1e3 * np.sqrt(np.mean(x ** 2)):.1f} | {1e3 * np.percentile(x, 95):.1f} | "
                   f"{100 * big / len(x):.1f} | {100 * big / max(tot, 1):.0f} |")
out += ["\n## 방향 전환 시작 (자이로 |ψ̇| 1 rad/s 위로, 속도 > 0.3 m/s) 기준 평균 흔들림 mm (0.1 s 칸)\n",
        "| 경로 | 시작 수 | " + " | ".join(f"{k / 10:+.1f}" for k in range(-10, 15)) + " |", "|---|---|" + "---|" * 25]
for kind in ["sq2p", "sq2", "c8d2"]:
    ks = [k for (kk, k) in trig if kk == kind]
    if not ks: continue
    n0 = len(trig.get((kind, 0), []))
    out.append(f"| {kind} | ~{n0} | " + " | ".join(f"{1e3 * np.mean(trig[(kind, k)]):.0f}" if (kind, k) in trig else "-" for k in range(-10, 15)) + " |")
P = np.vstack(POOL)
out += ["\n## 요 각가속도 |ψ̈| (자이로) × 진행 속도 (바퀴) 별 흔들림 — 전체 판, 칸 = RMS mm / > 2 cm % (간격 수)\n",
        "| |ψ̈| rad/s² | 속도 ≤ 0.1 | 0.1~0.3 | 0.3~0.8 | > 0.8 m/s |", "|---|---|---|---|---|"]
for lo, hi in ((0, 1), (1, 2), (2, 4), (4, 8), (8, 16), (16, 999)):
    cells = []
    for vlo, vhi in ((0, 0.1), (0.1, 0.3), (0.3, 0.8), (0.8, 9)):
        m = (P[:, 0] >= lo) & (P[:, 0] < hi) & (P[:, 2] >= vlo) & (P[:, 2] < vhi)
        x = P[m, 3]
        cells.append(f"{1e3 * np.sqrt(np.mean(x ** 2)):.1f} / {100 * np.mean(x > 0.02):.0f} ({m.sum()})" if m.sum() >= 10 else "-")
    out.append(f"| {lo}~{hi} | " + " | ".join(cells) + " |")
out += ["\n## 꾸준한 회전 (|ψ̈| < 2) 에서 요 각속도별 — 속도 > 0.3 m/s\n", "| |ψ̇| rad/s | 간격 수 | RMS mm | > 2 cm % |", "|---|---|---|---|"]
for lo, hi in ((0, 0.5), (0.5, 1), (1, 2), (2, 4)):
    m = (P[:, 0] < 2) & (P[:, 2] > 0.3) & (P[:, 1] >= lo) & (P[:, 1] < hi); x = P[m, 3]
    if m.sum() >= 10:
        out.append(f"| {lo}~{hi} | {m.sum()} | {1e3 * np.sqrt(np.mean(x ** 2)):.1f} | {100 * np.mean(x > 0.02):.1f} |")
open(os.path.join(H, "out_turn_events.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))
