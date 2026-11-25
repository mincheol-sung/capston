#!/usr/bin/python3
"""C 가 왜 미끄러지나 — 큰 슬립 순간 (max_i |λ_i| > 0.5, 기준 이동 중·hold 아님) 의 슬립 바퀴 i 에서:
  u_i = |τ_i| / τ̄_i(t)   (τ = MPC 출력 /torque_mpc/status 4~7, τ̄(t) = /torque_mpc/load 의 스텝 0 상한 — h 0 이면 정지 상한)
  → u ≥ 0.9 면 "상한까지 밀었는데 미끄러짐" (상한 > 실제 접지력), u < 0.5 면 "상한 한참 아래서 미끄러짐" (상한 모델 밖).
  기준 불가능 몫: 이동 중 max_i |U_s,i|/τ̄_i(t) > 1 인 시간 % (U_s = 목표 입력, status 22~25 — 기준을 모델대로 따라가는 데 드는 토크).
  슬립 가드: /torque_output/slip_guard [t, σ×4, g×4, 판정 바퀴] — 판정 % (이동 중), 큰 슬립 중 판정 %.
사용: slip_cause.py 폴더 ..."""
import os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm


def topic(bag, name, use_stamp_col0=True):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    if name not in {t.name for t in rd.get_all_topics_and_types()}:
        return None
    rd.set_filter(rosbag2_py.StorageFilter(topics=[name])); X = []
    while rd.has_next():
        _, data, tb = rd.read_next(); m = list(deserialize_message(data, Float64MultiArray).data)
        X.append(m if use_stamp_col0 else [tb * 1e-9] + m)
    return np.array(X) if X else None


print("| 판 | 큰 슬립 % | 슬립 바퀴 u = |τ|/τ̄: <0.5 · 0.5~0.9 · ≥0.9 | u 중앙 | 기준 불가능 % (|U_s|>τ̄) | 큰 슬립 중 기준 불가능 % | 가드 판정 % (큰 슬립 중) |")
for bag in sys.argv[1:]:
    o = cm.series(bag); d = cm.load(f"{cm.B}/{bag}"); S = d["S"]; t_abs = o["t"] + o["t0"]
    mv = o["moving"] & ~o["hold"]
    L = topic(f"{cm.B}/{bag}", "/torque_mpc/load")
    tbar = np.stack([np.interp(t_abs, L[:, 0], L[:, 5 + i]) for i in range(4)], 1) if L is not None else np.tile(cm.TB, (len(t_abs), 1))
    tau = o["tau_mpc"]; us = o["us"]; lam = o["lam"]
    big = mv & (np.abs(lam).max(1) > 0.5); i = np.abs(lam).argmax(1); rows = np.arange(len(i))
    u = (np.abs(tau[rows, i]) / tbar[rows, i])[big]
    infeas = (np.abs(us) / tbar).max(1) > 1.0
    G = topic(f"{cm.B}/{bag}", "/torque_output/slip_guard", use_stamp_col0=False)
    if G is not None:
        flag = np.interp(t_abs, G[:, 0], (G[:, 10] >= 0).astype(float)) > 0.5
        gs = f"{100 * flag[mv].mean():.0f} ({100 * flag[big].mean():.0f})"
    else:
        gs = "–"
    print(f"| {bag} | {100 * big.sum() / mv.sum():.0f} | {100 * np.mean(u < 0.5):.0f} · {100 * np.mean((u >= 0.5) & (u < 0.9)):.0f} · {100 * np.mean(u >= 0.9):.0f} | "
          f"{np.median(u):.2f} | {100 * infeas[mv].mean():.0f} | {100 * infeas[big].mean():.0f} | {gs} |")
