#!/usr/bin/python3
"""구동계 마찰 보상을 끄고 d̂ 로 대신하면? — 실차 C h 0.2 판 (d̂ 켬) 에서 계산 (2026-11-23, 교수님 "마찰은 d̂ 에 든 외란" 지적)
  τ_f,i(ω) = coulomb_i·tanh(ω_i/0.5) + viscous_i·ω_i   (torque_plant.yaml friction 절 실측값, 보상은 ×0.9)
  몸체 등가 가속도 a_f = M_eff⁻¹ T⁻ᵀ τ_f  (몸체 좌표, TorquePlant.G(0) 와 같은 사상)
  d̂ (lpf α 0.01, 몸체) 가 a_f 만 따라간다고 보면 남는 몫 = a_f − LPF(a_f) (+ acc_max·yaw_acc_max 자르기) — 지금 보상 (×0.9) 이 남기는 몫 0.1·a_f 와 비교
  바퀴 쪽: 마찰 τ_f / τ̄_i, 제동 때 바닥 토크 |τ_m| + τ_f 가 τ̄ 를 넘는 몫 (보상 없이 |τ_m| ≤ τ̄ 만 걸 때)
사용: fric_vs_dhat.py 160416_h020b 161250_h020c"""
import sys
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from std_msgs.msg import Bool, Float64MultiArray
from amr_mpc.torque.model import TorquePlant, wheel_coefficients

B = "/home/smoc/capston/capston/bags/2026-11-23"
CFG = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/config"
P = TorquePlant.from_yaml()
Y = yaml.safe_load(open(f"{CFG}/torque_plant.yaml"))["friction"]
DY = yaml.safe_load(open(f"{CFG}/torque_mpc.yaml"))["disturbance"]
COUL, VISC = wheel_coefficients(Y["coulomb_Nm"]), wheel_coefficients(Y["viscous_Nms_per_rad"])
SCALE, STATIC = float(Y["compensation_scale"]), wheel_coefficients(Y["static_Nm"], "static_Nm")
TAUBAR = np.array([0.1445, 0.1184, 0.1444, 0.1859])          # h 0 상한 (정지 하중), RESULTS 본문 값
Gb = np.linalg.solve(P.M_eff, P.Tinv.T)                       # 몸체 가속도 = Gb τ
Ts, ALPHA, AMAX, YMAX = 0.01, float(DY["alpha"]), float(DY["acc_max"]), float(DY["yaw_acc_max"])


def read(bag):
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=f"{B}/{bag}", storage_id=""), rosbag2_py.ConverterOptions("", ""))
    T = {"/wheel_states": WheelStates, "/torque_mpc/cmd_raw": WheelTorqueCommand, "/torque_mpc/status": Float64MultiArray,
         "/torque_mpc/adapt": Float64MultiArray, "/mpc/armed": Bool}
    rd.set_filter(rosbag2_py.StorageFilter(topics=list(T))); out = {k: [] for k in T}
    while rd.has_next():
        top, d, ts = rd.read_next(); m = deserialize_message(d, T[top]); t = ts * 1e-9
        if top == "/wheel_states":
            out[top].append([t, *m.velocity])
        elif top == "/torque_mpc/cmd_raw":
            out[top].append([t, *m.torque])
        elif top == "/mpc/armed":
            out[top].append([t, float(m.data)])
        else:
            out[top].append([t, *m.data])
    return {k: np.array(v) for k, v in out.items()}


print("| 판 | 이동 표본 | 바퀴 |ω|<0.5 rad/s % (FL/FR/RL/RR) | 부호 바뀜 횟수 | τ_f / τ̄ 중앙·최대 | |a_f,xy| 중앙/95%/최대 [m/s²] | "
      "|α_f| 95%/최대 [rad/s²] | > acc_max % | d̂ 실측 |â_xy| 중앙/95% | 갱신 멈춤 % | 남는 몫 RMS: 지금 0.1·a_f / d̂ 로 [m/s²] | 제동 바닥 토크 > τ̄ % |")
for bag in sys.argv[1:]:
    D = read(bag); W = D["/wheel_states"]; C = D["/torque_mpc/cmd_raw"]; S = D["/torque_mpc/status"]; A = D["/torque_mpc/adapt"]
    arm = D["/mpc/armed"]; on = arm[arm[:, 1] > 0.5, 0]; off = arm[(arm[:, 1] < 0.5) & (arm[:, 0] > on[0]), 0]
    t0, t1 = on[0], (off[0] if len(off) else W[-1, 0])
    w = W[(W[:, 0] >= t0) & (W[:, 0] <= t1)]; t, om = w[:, 0], w[:, 1:5]
    mv = np.abs(om).max(1) > 0.5
    tf = COUL * np.tanh(om / 0.5) + VISC * om                      # 실제 운동 마찰 (보상 ×0.9 전)
    af = tf @ Gb.T                                                  # 몸체 등가 가속도 (x, y, θ)
    # d̂ (lpf) 가 a_f 하나만 본다면 — 바퀴 상태 간격 그대로 α_eff = 1 − (1−α)^(dt/Ts)
    lp = np.zeros_like(af); dt = np.diff(t, prepend=t[0])
    for k in range(1, len(t)):
        a = 1 - (1 - ALPHA) ** (dt[k] / Ts); lp[k] = (1 - a) * lp[k - 1] + a * af[k]
        n = np.hypot(*lp[k, :2])
        if n > AMAX:
            lp[k, :2] *= AMAX / n
        lp[k, 2] = np.clip(lp[k, 2], -YMAX, YMAX)
    res_now, res_dhat = 0.1 * af, af - lp
    # 제동 바닥 토크: 보상 없이 τ_m = τ (MPC 가 |τ| ≤ τ̄) 이면 바닥 토크 ≈ τ_m − τ_f (관성 무시). 지금 MPC τ 를 τ_m 로 본다
    c = C[(C[:, 0] >= t0) & (C[:, 0] <= t1)]; ci = np.clip(np.searchsorted(c[:, 0], t), 0, len(c) - 1); tau = c[ci, 1:5]
    ground = tau - tf; over = (np.abs(ground) > TAUBAR) & (np.abs(tau) <= TAUBAR * 1.001)
    s = S[(S[:, 0] >= t0) & (S[:, 0] <= t1)]; ah = s[:, 17:20] / Ts
    ad = A[(A[:, 0] >= t0) & (A[:, 0] <= t1)]; frz = np.mean(ad[:, -1] == 3) * 100
    zc = [int(np.sum(np.diff(np.sign(om[mv, i][np.abs(om[mv, i]) > 0.2])) != 0)) for i in range(4)]
    low = [100 * np.mean(np.abs(om[mv, i]) < 0.5) for i in range(4)]
    rat = np.abs(tf[mv]) / TAUBAR; nxy = np.hypot(af[mv, 0], af[mv, 1])
    rms = lambda r: np.sqrt(np.mean(np.sum(r[mv, :2] ** 2, 1)))
    print(f"| {bag} | {mv.sum()} | {'/'.join(f'{v:.0f}' for v in low)} | {'/'.join(map(str, zc))} | {np.median(rat):.2f}·{rat.max():.2f} | "
          f"{np.median(nxy):.2f}/{np.percentile(nxy, 95):.2f}/{nxy.max():.2f} | {np.percentile(np.abs(af[mv, 2]), 95):.2f}/{np.abs(af[mv, 2]).max():.2f} | "
          f"{100 * np.mean(nxy > AMAX):.0f} | {np.median(np.hypot(ah[:, 0], ah[:, 1])):.2f}/{np.percentile(np.hypot(ah[:, 0], ah[:, 1]), 95):.2f} | {frz:.0f} | "
          f"{rms(res_now):.3f} / {rms(res_dhat):.3f} | {100 * np.mean(over[mv].any(1)):.1f} |")
print(f"\n상수: M_eff diag {np.round(np.diag(P.M_eff), 4)}, T⁻ᵀ 열 k = {P.k:.4f} m, 보상 ×{SCALE}, 정지 마찰 {STATIC} (보상은 ×{SCALE}, 위 계산에는 안 넣음), "
      f"d̂ α {ALPHA} (시정수 {Ts / ALPHA:.1f} s) · acc_max {AMAX} · yaw_acc_max {YMAX}")
for v in (0.345, 0.69, 1.035):
    om = np.full(4, v / P.r); tf = COUL * np.tanh(om / 0.5) + VISC * om
    print(f"직진 {v:.3f} m/s (ω {om[0]:.1f} rad/s): τ_f {np.round(tf, 4)} N·m = τ̄ 의 {np.round(tf / TAUBAR * 100, 0)} %, a_f,x {(Gb @ tf)[0]:.3f} m/s²")
