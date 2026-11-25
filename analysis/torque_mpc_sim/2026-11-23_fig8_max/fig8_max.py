#!/usr/bin/python3
"""오늘 실차 8자 설정 (114635·114459: circles8 width 2.0, tangent, laps 2, speed_mod_amp 0.5, cycles 2, phase −90°, crossing_blend 0) 에서
speed (순항 V, 변조 뒤 최고 1.5V) 와 accel (출발·정지) 을 바꿔 가며
  ① 부하 예산: figure8_reference 와 같은 peak_load(K_GEOM) ≤ load_budget·V_LIMIT (기동 로그 V_LIMIT 1.55, budget 1.0)
  ② C 마찰 상한: 노드가 보는 앞먹임 max_i |U_s,i| / τ̄_i — U_s = F⁺ R(ψ)ᵀ M (p̈ − w), p̈ = 기준 월드 속도 0.2 s 차분
     (창을 0~0.15 s 앞당긴 넷 중 최악, 2026-11-20 acc_window_shift 뒤의 노드와 같음), τ̄_i = (r/√2)·μ̂_i·N_i·c_s (N_i = 하중 몫·mg, 지금 yaml)
외란 추정 â_d 는 0 으로 둔다 (실행마다 다름)."""
import math, sys
import numpy as np, yaml
MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"; sys.path.insert(0, MPC)
from amr_mpc.core.figure8 import Figure8
from amr_mpc.body import mpc as mpc_mod
P = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml")); geo, body = P["geometry"], P["body"]
r = geo["wheel_radius_m"]; k = 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"]); m = body["mass_kg"]; g = P["model"]["gravity"]
mw = 4 * P["wheel"]["inertia_J_w_kgm2"] / r**2; me = m + mw; Ie = body["inertia_z_kgm2"] + mw * k * k
Fp = np.linalg.pinv(np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r)
share = np.array(body["wheel_load_share"]); mu = np.array(P["surface"]["mu_hat_wheels"])
TB = r / math.sqrt(2) * m * g * P["surface"]["c_s"] * mu * share
V_LIMIT, BUDGET, K_GEOM = 1.55, 1.0, mpc_mod.K_GEOM
T = yaml.safe_load(open(f"{MPC}/config/torque_track.yaml"))["track"]


def make(V, A):
    return Figure8(width=2.0, height=float(T["height"]), speed=V, accel=A, laps=2, yaw_mode="tangent", align_s=float(T["align_s"]),
                   curve="circles", crossing_blend=0.0, speed_mod_amp=0.5, speed_mod_cycles=2.0, speed_mod_phase_deg=-90.0)


def ff_ratio(f, dt=2e-3):
    t = np.arange(0.0, f.T_total + 0.3, dt)
    XU = [f.at(x) for x in t]; X = np.array([a for a, _ in XU]); U = np.array([b for _, b in XU])
    c, s = np.cos(X[:, 2]), np.sin(X[:, 2])
    V = np.stack([c * U[:, 0] - s * U[:, 1], s * U[:, 0] + c * U[:, 1], U[:, 2]], 1)
    h = int(round(0.1 / dt)); worst = None
    for lead in (0.0, 0.05, 0.1, 0.15):
        L = int(round(lead / dt)); pad = 2 * h + L
        Vp = np.vstack([np.repeat(V[:1], pad, 0), V, np.repeat(V[-1:], pad, 0)])
        acc = (Vp[pad + L + h: pad + L + h + len(t)] - Vp[pad + L - h: pad + L - h + len(t)]) / 0.2
        w = np.stack([-mw * V[:, 2] * V[:, 1] / me, mw * V[:, 2] * V[:, 0] / me, 0 * V[:, 0]], 1)
        q = acc - w; F = np.stack([me * (c * q[:, 0] + s * q[:, 1]), me * (-s * q[:, 0] + c * q[:, 1]), Ie * q[:, 2]], 1)
        rat = np.abs(F @ Fp.T) / TB
        worst = rat if worst is None else np.maximum(worst, rat)
    return t, worst, V, U


if __name__ == "__main__":
    print(f"τ̄ = {np.round(TB, 4)} N·m (μ̂ {mu}, 하중 몫 {share}, c_s {P['surface']['c_s']}),  K_GEOM {K_GEOM},  V_LIMIT {V_LIMIT}·budget {BUDGET}")
    f0 = make(0.6, 1.0); run, _ = f0.peak_load(K_GEOM); t, rat, _, _ = ff_ratio(f0)
    print(f"[검산] 오늘 판 speed 0.6 accel 1.0: 부하 {run / V_LIMIT * 100:.0f} % (로그 81 %), 주행 {f0.T_path if hasattr(f0, 'T_path') else f0.T_total:.1f} s, 앞먹임 최대 {rat.max():.2f}")
    print("\n| speed V (최고 1.5V) | accel | 부하 % | 앞먹임 max |U_s|/τ̄ (어느 바퀴, 시각) | 1 넘는 시간 % | 접선 가속 최대 |")
    print("|---|---|---|---|---|---|---|")
    for V in (0.6, 0.65, 0.7, 0.72, 0.74, 0.75, 0.76):
        for A in (1.0, 1.2, 1.4, 1.5):
            f = make(V, A); run, _ = f.peak_load(K_GEOM); t, rat, _, _ = ff_ratio(f)
            mx = rat.max(1); i = int(np.argmax(mx)); w = int(np.argmax(rat[i]))
            print(f"| {V} ({1.5*V:.3f}) | {A} | {run / V_LIMIT * 100:.0f}{' ✗' if run > BUDGET * V_LIMIT else ''} | {mx.max():.2f} ({['FL','FR','RL','RR'][w]}, {t[i]:.1f} s) "
                  f"| {100 * (mx > 1).mean():.1f} | {f.max_tangential_accel():.2f} |")
