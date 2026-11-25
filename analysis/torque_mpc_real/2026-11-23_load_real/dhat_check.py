#!/usr/bin/python3
"""d̂ (외란 추정치) 가 슬립에 얼마나 끼었나 — 실차 h 스윕 판들 (disturbance:=true, observer lpf α 0.01, frame body, accel_only,
freeze_on_saturation, acc_max 0.5 m/s², yaw_acc_max 2.0 rad/s²).
  â_d = d̂[3:6] / Ts (몸체 [m/s², m/s², rad/s²], /torque_mpc/status 17~19 칸), Ts 0.01.
  d̂ 몫 토크 Δτ_d = −F⁺ diag(m_e, m_e, I_e) â_d  (식-14 의 −M â_d 항, R 은 몸체라 생략) — 바퀴별, τ̄ (정지 상한) 로 나눔.
  큰 슬립 순간 (max_i |λ_i| > 0.5) 에 가장 큰 슬립 바퀴 i 에서: Δτ_d,i 가 τ_i 와 같은 부호 (= d̂ 가 그 바퀴를 더 민다) 인 몫, |Δτ_d,i| / |τ_i|.
  /torque_mpc/adapt 7 번째 칸 = d̂ 갱신 건너뜀 코드 (3 = 포화 중 멈춤).
사용: dhat_check.py 폴더 ..."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core import fig8_limits as fl
c = fl.plant_constants("/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/config")
M = np.diag([c["me"], c["me"], c["Ie"]]); TS = 0.01; TB = cm.TB
print(f"m_e {c['me']:.2f} kg, I_e {c['Ie']:.4f} kg·m², â_d 상한 0.5 m/s² → 바퀴 토크 {np.abs(c['Fp'] @ M @ [0.5, 0, 0]).max():.4f} N·m (x), "
      f"{np.abs(c['Fp'] @ M @ [0, 0.5, 0]).max():.4f} (y), 2.0 rad/s² → {np.abs(c['Fp'] @ M @ [0, 0, 2.0]).max():.4f}; τ̄ {TB}")
print("| 판 | |â_xy| 평균/최대 [m/s²] | |α_d| 평균/최대 [rad/s²] | 상한 붙음 % | |Δτ_d|/τ̄ RMS · 최대 | 큰 슬립 때 d̂ 가 슬립 바퀴를 더 미는 몫 | 그때 |Δτ_d|/|τ| 중앙 | 갱신 멈춤 % |")
for bag in sys.argv[1:]:
    o = cm.series(bag); d = cm.load(f"{cm.B}/{bag}"); S = d["S"]
    t_abs = o["t"] + o["t0"]; mv = o["moving"] & ~o["hold"]
    ad = np.stack([np.interp(t_abs, S[:, 0], S[:, 17 + k]) for k in range(3)], 1) / TS
    dtau = -(ad @ M.T) @ c["Fp"].T                                  # (N,4)
    axy = np.hypot(ad[:, 0], ad[:, 1])[mv]; ayaw = np.abs(ad[:, 2])[mv]
    clamp = 100 * float(np.mean((axy > 0.49) | (ayaw > 1.98)))
    r = np.abs(dtau[mv]) / TB
    lam = o["lam"][mv]; tau = o["tau_mpc"][mv]; dt = dtau[mv]
    big = np.abs(lam).max(1) > 0.5; i = np.abs(lam).argmax(1); rows = np.arange(len(i))
    same = np.sign(dt[rows, i]) == np.sign(tau[rows, i])
    frac = np.abs(dt[rows, i]) / np.maximum(np.abs(tau[rows, i]), 1e-3)
    sk = ""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Float64MultiArray
        rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{cm.B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
        rd.set_filter(rosbag2_py.StorageFilter(topics=["/torque_mpc/adapt"])); A = []
        while rd.has_next():
            _, data, _ = rd.read_next(); A.append(list(deserialize_message(data, Float64MultiArray).data))
        A = np.array(A); w = (A[:, 0] >= t_abs[0]) & (A[:, 0] <= t_abs[-1]); sk = f"{100 * np.mean(A[w, 6] == 3):.0f}"
    except Exception as e:
        sk = f"? ({e.__class__.__name__})"
    print(f"| {bag} | {axy.mean():.2f} / {axy.max():.2f} | {ayaw.mean():.2f} / {ayaw.max():.2f} | {clamp:.0f} | "
          f"{np.sqrt(np.mean(r ** 2)):.2f} · {r.max():.2f} | {100 * same[big].mean():.0f} % (큰 슬립 {100 * big.mean():.0f} % 중) | "
          f"{np.median(frac[big]):.2f} | {sk} |")
