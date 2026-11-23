#!/usr/bin/python3
"""η_s 끔/켬 시뮬 판 확인 (2026-11-23).  /usr/bin/python3 analysis/torque_mpc_sim/2026-11-23_slip_eta_smoke/check.py
  - 노드가 낸 λ (/torque_mpc/slip) 와 slip_logger λ 가 같은가 (같은 정의 — 구현 검산)
  - η_s 가 상자에 곱해졌나 (|τ_i| ≤ /torque_mpc/load 상한), QP fallback
  - 슬립 (slip_logger λ, 10 Hz filtfilt) > 0.25·> 0.5 %, 추종 오차 RMS"""
import os
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__))


def load(lab):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.join(H, "runs", lab), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/torque_mpc/slip", "/torque_mpc/status", "/torque_mpc/load", "/slip_logger/slip"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append(list(deserialize_message(d, ty[n]).data))
    return {k: np.array(v) for k, v in D.items()}


print("| 판 | 켬 | 추종 주기 | λ 노드 vs slip_logger 최대 차 | 곱한 η_s 최소 | η_s<0.5 바퀴-시간 % (곱한 / 계산한) | |τ|>상자 | fallback | λ̃>0.25 % | λ̃>0.5 % | 추종 오차 RMS mm |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for lab in ("eta_off", "eta_on"):
    if not os.path.exists(os.path.join(H, "runs", lab, "metadata.yaml")):
        print(f"| {lab} | (bag 없음) |"); continue
    D = load(lab)
    st, sl, ld, lg = D["/torque_mpc/status"], D["/torque_mpc/slip"], D["/torque_mpc/load"], D["/slip_logger/slip"]
    act = st[:, 20] == 0
    n = min(len(st), len(sl), len(ld))
    st, sl, ld, act = st[:n], sl[:n], ld[:n], act[:n]
    # 노드 λ (sl[:, 2:6]) 와 slip_logger λ (lg[:, 1:5]): 노드 λ 는 바퀴 메시지 시각이 없어 값 분포로 견준다 → 같은 시각 근처 최근접
    tl = lg[:, 0]
    over = np.abs(st[act, 4:8]) - ld[act, 5:9]
    L = np.where(np.isfinite(lg[:, 1:5]), lg[:, 1:5], 0.0)
    bb, aa = butter(1, 10 / (100 / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    t0, t1 = st[act, 0].min(), st[act, 0].max()
    w = (tl >= t0) & (tl <= t1)
    mx = np.abs(Lf[w]).max(1)
    e = np.hypot(st[act, 1], st[act, 2])
    # 노드가 낸 원신호 λ 와 slip_logger λ 의 분포 차 (같은 정의면 백분위가 같다)
    q = np.percentile(np.abs(sl[act, 2:6]), [50, 90, 99]) - np.percentile(np.abs(L[w]), [50, 90, 99])
    print(f"| {lab} | {int(sl[0, 1])} | {act.sum()} | 백분위 50/90/99 차 {np.round(q, 3)} | {sl[act, 14:18].min():.3f} | "
          f"{100 * np.mean(sl[act, 14:18] < 0.5):.1f} / {100 * np.mean(sl[act, 10:14] < 0.5):.1f} | {int((over > 1e-6).sum())} | "
          f"{int(st[act, 11].sum())} | {100 * np.mean(mx > 0.25):.1f} | {100 * np.mean(mx > 0.5):.1f} | {1e3 * np.sqrt(np.mean(e ** 2)):.1f} |")
