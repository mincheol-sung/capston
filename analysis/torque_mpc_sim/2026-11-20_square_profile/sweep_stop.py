#!/usr/bin/python3
"""꼭짓점 정지 (corner_mode stop) 의 rot_s·dwell_s 별 노드 앞먹임 |U_s,i|/τ̄_i (0.2 s 차분, 창 가운데를 t+δ, δ 0~0.15 s 중 최악).
식은 sweep.py 와 같다. 직선 가감속만으로 0.964 (RR) 이므로 그보다 크면 회전·꼭짓점이 더한 것."""
import math, sys
import numpy as np, yaml
MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"; sys.path.insert(0, MPC)
from amr_mpc.core.square import Square
P = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml")); geo, body = P["geometry"], P["body"]
r = geo["wheel_radius_m"]; k = 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"]); m = body["mass_kg"]; g = P["model"]["gravity"]
mw = 4 * P["wheel"]["inertia_J_w_kgm2"] / r**2; me = m + mw; Ie = body["inertia_z_kgm2"] + mw * k * k
Fp = np.linalg.pinv(np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r)
TB = r / math.sqrt(2) * np.array(P["surface"]["mu_hat_wheels"]) * m * g / 4 * P["surface"]["c_s"]
def ratio(sq, lead):
    dt = 1e-3; t = np.arange(0, sq.T_total + 0.3, dt)
    XU = [sq.at(x) for x in t]; X = np.array([a for a, _ in XU]); U = np.array([b for _, b in XU])
    c, s = np.cos(X[:, 2]), np.sin(X[:, 2])
    V = np.stack([c * U[:, 0] - s * U[:, 1], s * U[:, 0] + c * U[:, 1], U[:, 2]], 1)
    h, L = 100, int(round(lead / dt)); pad = 2 * h + L
    Vp = np.vstack([np.repeat(V[:1], pad, 0), V, np.repeat(V[-1:], pad, 0)])
    hi = Vp[pad + L + h: pad + L + h + len(t)]; lo = Vp[pad + L - h: pad + L - h + len(t)]; acc = (hi - lo) / 0.2
    w = np.stack([-mw * V[:, 2] * V[:, 1] / me, mw * V[:, 2] * V[:, 0] / me, 0 * V[:, 0]], 1)
    q = acc - w; f = np.stack([me * (c * q[:, 0] + s * q[:, 1]), me * (-s * q[:, 0] + c * q[:, 1]), Ie * q[:, 2]], 1)
    rat = np.abs(f @ Fp.T) / TB; run = (t > 0) & (t < sq.T_path)
    return rat.max(1)[run], t[run]
print("| rot_s | dwell_s | 주행 시간 | 최대 |U_s|/τ̄ (δ 0~0.15 최악) | 1 넘는 시간 | 회전 중 최대 (순수 회전) |")
for rot in (0.6, 0.8, 1.0, 1.2):
    for dw in (0.0, 0.1, 0.2):
        sq = Square(side=1.5, accel=1.5, v_max=1.5, yaw_mode="tangent", corner_mode="stop", rot_s=rot, dwell_s=dw, align_s=1.5)
        res = [ratio(sq, d) for d in (0.0, 0.05, 0.1, 0.15)]
        mx = max(x.max() for x, _ in res); pct = max(100 * (x > 1).mean() for x, _ in res)
        # 순수 회전 구간 (dwell 안쪽, 창이 병진과 안 겹치는 곳) 의 최대
        x, tt = res[0]; inrot = np.zeros(len(tt), bool)
        for i in range(3):
            t0 = sq.t_start[i] + sq.edges[i].T + dw
            inrot |= (tt > t0 + 0.1) & (tt < t0 + rot - 0.1)
        print(f"| {rot} | {dw} | {sq.T_path:.2f} s | {mx:.2f} | {pct:.1f} % | {x[inrot].max() if inrot.any() else float('nan'):.2f} |")
sq = Square(side=1.5, accel=1.5, v_max=1.5, yaw_mode="fixed", corner_mode="stop", dwell_s=0.2)
x, _ = ratio(sq, 0.0); print(f"| fixed stop dwell 0.2 | - | {sq.T_path:.2f} s | {x.max():.2f} | {100*(x>1).mean():.1f} % | - |")
