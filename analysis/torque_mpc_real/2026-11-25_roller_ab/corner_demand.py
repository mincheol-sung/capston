# 2026-11-26 꺾은선 모서리 통과 기준이 바퀴에 요구하는 바닥 토크 (실차 식별 모델: m_eff 6.785, I_eff 0.113, 롤러 0.88 N) vs 상한
#   요구 = F⁺ · (M·기준 가속 − 롤러 저항(기준 속도))  (몸체 좌표, 바퀴 관성 포함 M), 상한 τ̄_i = (r/√2)·μ 0.33·N_i·c_s 0.8 (바퀴별 하중 몫)
#   yaw_blend (꼭짓점 앞뒤로 헤딩을 돌리는 거리) · 꼭짓점 속도를 바꿔 본다
import math, os, sys
import numpy as np
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core.polyline import Polyline
from amr_mpc.torque.model import RollerResistance, TorquePlant
p = TorquePlant.from_yaml(inertia_z=0.03)
rr = RollerResistance(p, 0.88)
M = p.M_eff; F = np.asarray(p.Tinv, float).T; Fp = np.linalg.pinv(F)
tb = p.r / math.sqrt(2) * 0.33 * p.wheel_load_share * p.mass * p.g * 0.8
print("상한 τ̄ [FL FR RL RR] mN·m", np.round(tb * 1e3, 1), " M_eff diag", np.round(np.diag(M), 3))
def demand(**kw):
    pl = Polyline(corner_mode="pass", accel=1.5, v_max=1.5, **kw)
    dt = 0.002; t = np.arange(0, pl.T_total, dt)
    U = np.array([pl.at(tt)[1] for tt in t])                       # 몸체 속도 [vx, vy, wz]
    dU = np.gradient(U, dt, axis=0)
    acc = dU + np.column_stack([-U[:, 2] * U[:, 1], U[:, 2] * U[:, 0], np.zeros(len(t))])   # 몸체 좌표 관성 가속
    Q = np.array([rr.force(u) for u in U])
    tau = (Fp @ (M @ acc.T - Q.T)).T
    ratio = np.abs(tau) / tb
    rot = np.abs(U[:, 2]) > 0.3
    return dict(wz=np.abs(U[:, 2]).max(), al=np.abs(dU[:, 2]).max(), over=100 * np.mean(ratio.max(1)[rot] > 1.0) if rot.any() else 0, peak=ratio.max(),
                rot_t=rot.sum() * dt, T=pl.T_total)
for yb in (0.3, 0.4, 0.5):
    for cs in (0.1, 0.3):
        try:
            d = demand(yaw_blend=yb, corner_speed=cs)
        except ValueError as e:
            print(f"yaw_blend {yb} corner {cs}: {e}"); continue
        print(f"yaw_blend {yb:.1f} m · 꼭짓점 {cs:.1f} m/s: 최고 회전 속도 {d['wz']:.2f} rad/s · 각가속도 {d['al']:.1f} rad/s², 회전 구간 {d['rot_t']:.2f} s 중 요구가 상한 넘는 시간 {d['over']:.0f} %, "
              f"최대 요구/상한 {d['peak']:.2f}, 한 바퀴 {d['T']:.2f} s")
