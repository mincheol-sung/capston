#!/usr/bin/python3
"""주행 끝 정렬 — 기준이 멈춘 뒤 (마지막 활성 시각 t_end ~ +5 s, figure8 '유지 5 s') MPC 와 PP 가 남은 오차를 줄이나.
  추정기 오차 Xe (제어기가 보는 값): MPC /torque_mpc/status 1~3, PP /pure_pursuit/status 1~3 → |e_xy| [mm], e_θ [°] 를 t_end, +1, +4.5 s 에.
  hold: MPC status 20 (래치로 DAMPING), PP status 6 = 2 (hold). 래치 진입 = |e_xy| < 1 cm · |e_θ| < 2° · 멈춤 (torque_mpc.yaml hold).
  바퀴: /wheel_states 속도 max |ω| [rad/s] 와 모터 명령 토크 max |τ_motor| (마찰 보상 포함) — t_end+0.5 ~ +4.5 s.
  MPC 순토크 max |τ_mpc| (status 4~7). 바닥 출발 토크 ≈ 0.09 N·m (torque_plant.yaml friction.static_Nm 주석, 정지 마찰 보상은 ω_ref = 0 이면 0).
사용: end_align.py 라벨:폴더 ..."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
from load import load, ref_at
B = "/home/smoc/capston/capston/bags/2026-11-23"
print("| 판 | 제어 | t_end 오차 xy·θ | +1 s | +4.5 s | hold 진입 | 바퀴 max |ω| | 모터 명령 max |τ| | MPC 순토크 max |τ| |")
for arg in sys.argv[1:]:
    lab, bag = arg.split(":", 1)
    d = load(f"{B}/{bag}"); D = d["D"]; a0, a1 = d["arm"]
    tt = np.arange(a0, min(a1, a0 + 90), 0.02)
    sp = np.array([np.hypot(*ref_at(D, x)[1][:2]) + abs(ref_at(D, x)[1][2]) for x in tt])
    act = sp > 0.02; t_end = tt[len(act) - 1 - np.argmax(act[::-1])]
    pp = len(d["S"]) == 0
    X = np.array(d["PS"]) if pp else d["S"]
    at = lambda dt: X[np.searchsorted(X[:, 0], t_end + dt).clip(0, len(X) - 1)]
    fmt = lambda r: f"{1e3 * np.hypot(r[1], r[2]):.0f} mm · {np.degrees(r[3]):+.2f}°"
    w = (X[:, 0] >= t_end) & (X[:, 0] <= t_end + 5.0)
    hold = (X[w, 6] == 2).any() if pp else (X[w, 20] > 0.5).any()
    W = d["W"]; ww = (W[:, 0] >= t_end + 0.5) & (W[:, 0] <= t_end + 4.5)
    om = np.abs(W[ww, 1:5]).max(); tm = np.abs(W[ww, 5:9]).max()
    tmpc = "–" if pp else f"{np.abs(X[(X[:, 0] >= t_end + 0.5) & (X[:, 0] <= t_end + 4.5), 4:8]).max():.3f}"
    print(f"| {lab} {bag} | {'PP' if pp else 'MPC'} | {fmt(at(0))} | {fmt(at(1))} | {fmt(at(4.5))} | {'예' if hold else '아니오'} | "
          f"{om:.2f} | {tm:.3f} | {tmpc} |")
