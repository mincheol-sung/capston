#!/usr/bin/python3
"""yaw_blend·yaw_mode·accel 를 바꿨을 때 노드가 보는 앞먹임 (0.2 s 차분) |U_s,i|/τ̄_i — profile.py 와 같은 식, 벡터화."""
import math, os, sys
import numpy as np, yaml
MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"; sys.path.insert(0, MPC)
from amr_mpc.core.square import Square
P = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml")); geo, body = P["geometry"], P["body"]
r = geo["wheel_radius_m"]; k = 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"]); m = body["mass_kg"]; g = P["model"]["gravity"]
mw = 4 * P["wheel"]["inertia_J_w_kgm2"] / r**2; me = m + mw; Ie = body["inertia_z_kgm2"] + mw * k * k
Fp = np.linalg.pinv(np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r)
TB = r / math.sqrt(2) * np.array(P["surface"]["mu_hat_wheels"]) * m * g / 4 * P["surface"]["c_s"]
def run(**kw):
    sq = Square(side=1.5, corner_speed=kw.get("vc", 0.1), accel=kw.get("a", 1.5), v_max=1.5, yaw_mode=kw.get("mode", "tangent"),
                yaw_blend=kw.get("b", 0.3), align_s=1.5)
    dt = 1e-3; t = np.arange(0, sq.T_total + 0.3, dt)
    XU = [sq.at(x) for x in t]; X = np.array([a for a, _ in XU]); U = np.array([b for _, b in XU])
    c, s = np.cos(X[:, 2]), np.sin(X[:, 2])
    V = np.stack([c * U[:, 0] - s * U[:, 1], s * U[:, 0] + c * U[:, 1], U[:, 2]], 1)
    h = 100; Vp = np.vstack([np.repeat(V[:1], h, 0), V, np.repeat(V[-1:], h, 0)]); acc = (Vp[2 * h:] - Vp[:-2 * h]) / 0.2
    w = np.stack([-mw * V[:, 2] * V[:, 1] / me, mw * V[:, 2] * V[:, 0] / me, 0 * V[:, 0]], 1)
    q = acc - w; f = np.stack([me * (c * q[:, 0] + s * q[:, 1]), me * (-s * q[:, 0] + c * q[:, 1]), Ie * q[:, 2]], 1)
    rat = np.abs(f @ Fp.T) / TB; mx = rat.max(1); run_ = (t > 0) & (t < sq.T_path)
    return sq.T_path, mx[run_].max(), 100 * (mx[run_] > 1).mean(), np.abs(U[run_, 2]).max()
print("| 경우 | 주행 시간 | 노드 앞먹임 최대 |U_s|/τ̄ | 주행 중 상한 넘는 시간 | 주행 중 |ω| 최대 |")
for lab, kw in [("tangent b 0.3 (지금)", dict(b=0.3)), ("tangent b 0.6", dict(b=0.6)), ("tangent b 0.9", dict(b=0.9)),
                ("tangent b 1.2", dict(b=1.2)), ("tangent b 1.5", dict(b=1.5)), ("fixed (헤딩 고정, 옆걸음)", dict(mode="fixed")),
                ("tangent b 0.3, accel 1.0", dict(b=0.3, a=1.0)), ("tangent b 1.5, accel 1.0", dict(b=1.5, a=1.0))]:
    T, mx, pct, wm = run(**kw)
    print(f"| {lab} | {T:.2f} s | {mx:.2f} | {pct:.1f} % | {wm:.2f} rad/s |")
