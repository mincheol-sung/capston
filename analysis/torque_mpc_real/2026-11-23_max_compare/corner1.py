#!/usr/bin/python3
"""사각형 C (121319) 첫 꼭짓점이 안으로 파고드는 이유 — 0.1 s 표.
 e_t·e_n: 기준 속도 방향·그 왼쪽 법선 (반시계 정사각형이라 + e_n = 안쪽). SLAM 은 로봇 중심, MPC 가 본 오차 = /torque_mpc/status ex·ey (추정기 − 기준, 월드).
 τ/τ̄: 바퀴별, U_s/τ̄: 목표 입력, d̂: status 14~19, slip_guard g: /torque_output/slip_guard 의 바퀴별 이득 (1 = 안 줄임)."""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from load import load, ref_at, TINV
B = "/home/smoc/capston/capston/bags/2026-11-23"; TB = np.array([0.1445, 0.1184, 0.1444, 0.1859])
bag = sys.argv[1] if len(sys.argv) > 1 else "121319"
d = load(f"{B}/{bag}"); D = d["D"]; sl, E, S, W, G = d["slam"], d["E"], d["S"], d["W"], d["G"]; a0, a1 = d["arm"]
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}; SG = []
if "/torque_output/slip_guard" in types:
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/torque_output/slip_guard"]))
    while rd.has_next():
        tp, data, tb = rd.read_next(); SG.append(list(deserialize_message(data, get_message(types[tp])).data))
SG = np.array(SG)
ts = np.arange(a0, a0 + 30, 0.01); sp = np.array([np.hypot(*ref_at(D, t)[1][:2]) for t in ts]); tr0 = ts[np.argmax(sp > 0.02)]
nn = lambda A, t: np.argmin(np.abs(A[:, 0] - t))
print(f"{bag}: 기준 출발 +{tr0 - a0:.2f} s 무장 뒤")
hdr = " t   v_r  ω_r | SLAM e_t  e_n(+안)  e_ψ° | MPC 본 e_t  e_n | 추정−SLAM 옆 | τ/τ̄ FL FR RL RR | U_s/τ̄ max | λ FL FR RL RR | g FL FR RL RR | d̂(0..5)"
print(hdr)
for t in np.arange(tr0, tr0 + 3.0, 0.1):
    Xr, ur = ref_at(D, t); c, s = math.cos(Xr[2]), math.sin(Xr[2])
    vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); v = np.linalg.norm(vw)
    if v < 0.02:                                                     # 서 있을 때는 기준 헤딩 방향
        vw = np.array([c, s]); v = 1.0
    tg = vw / v; nr = np.array([-tg[1], tg[0]])
    k = nn(sl, t); ds = sl[k, 1:3] - Xr[:2]
    ks = nn(S, t) if len(S) else None
    ke = nn(E, t); de = np.array([E[ke, 1] - sl[k, 1], E[ke, 2] - sl[k, 2]])
    j0, j1 = nn(sl, t - 0.1), nn(sl, t + 0.1); kg = nn(G, t); kw = nn(W, t)
    va = (sl[j1, 1:3] - sl[j0, 1:3]) / max(sl[j1, 0] - sl[j0, 0], 1e-3); cs_, sn_ = math.cos(sl[k, 3]), math.sin(sl[k, 3])
    wn = TINV @ np.array([cs_ * va[0] + sn_ * va[1], -sn_ * va[0] + cs_ * va[1], G[kg, 1]]); wm = W[kw, 1:5]
    lam = (wm - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(wm), np.abs(wn)), 2.5)
    line = (f"{t - tr0:4.1f} {math.hypot(*ur[:2]):4.2f} {ur[2]:+5.2f} | {ds @ tg * 1e3:+6.0f} {ds @ nr * 1e3:+6.0f} "
            f"{math.degrees(math.atan2(math.sin(sl[k, 3] - Xr[2]), math.cos(sl[k, 3] - Xr[2]))):+5.1f} | ")
    if ks is not None:
        em = S[ks, 1:3]
        line += f"{em @ tg * 1e3:+6.0f} {em @ nr * 1e3:+6.0f} | {de @ nr * 1e3:+5.0f} | " + " ".join(f"{x:+.2f}" for x in S[ks, 4:8] / TB)
        line += f" | {(np.abs(S[ks, 22:26]) / TB).max():.2f} | "
    else:
        line += f"   –      –   | {de @ nr * 1e3:+5.0f} | " + " " * 23 + " |  –   | "
    line += " ".join(f"{x:+.1f}" for x in lam)
    if len(SG):
        kq = nn(SG, t); line += " | " + " ".join(f"{x:.2f}" for x in SG[kq, 5:9])
    if ks is not None:
        line += " | " + " ".join(f"{x:+.2f}" for x in S[ks, 14:20])
    print(line)
