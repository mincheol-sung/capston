# 가감속 8자 후보 프로파일: 기준이 요구하는 몸체 가속 (접선 a_t, 구심 v²/R, 요 각가속 κ·a_t) 이 C 의 토크 상한 안에 드는가.
#   C 상한 사용률 u(t) = 1/s_max,  s_max = max s  s.t.  F τ = s·M_eff·[a_t, a_c, α],  |τ_i| ≤ τ̄_i  (힘 분배 LP, 새 μ̂)
#   u ≤ 1 이면 C 가 모델상 기준을 다 낼 수 있다. 부하 검사 V(1+ε)(1 + K/R) ≤ 1.55 도 같이.
import sys, numpy as np
from scipy.optimize import linprog
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core.figure8 import Figure8
r, k, m, Jw, Iz = 0.04, 0.1932, 4.56, 8.9e-4, 0.06
F = np.array([[1,1,1,1],[-1,1,1,-1],[-k,k,-k,k]]) / r
M = np.diag([m + 4*Jw/r**2, m + 4*Jw/r**2, Iz + 4*Jw*k**2/r**2])
tb = 0.25306 * np.array([0.572, 0.468, 0.570, 0.734])
def util(d):
    if np.allclose(d, 0): return 0.0
    res = linprog([0,0,0,0,-1], A_eq=np.hstack([F, -(M @ d).reshape(3,1)]), b_eq=np.zeros(3), bounds=[(-t,t) for t in tb]+[(0,None)])
    return 1.0 / res.x[-1]
def evaluate(W, V, eps, n, phase=-90.0, laps=2, accel=1.0, shape="sin"):
    f = Figure8(width=W, speed=V, accel=accel, curve="circles", yaw_mode="tangent", laps=laps, speed_mod_amp=eps, speed_mod_cycles=n, speed_mod_phase_deg=phase, speed_mod_shape=shape)
    R = W / 4.0
    st = 20 if shape == 'sin' else 10
    t, s, v = f._lap_t[::st], f._lap_s[::st], f._lap_v[::st]
    a_t = np.gradient(v, t)
    lap = f.L / f.laps
    sgn = np.where((s % lap) < lap / 2, 1.0, -1.0)             # 첫 원 / 둘째 원 곡률 부호
    a_c = sgn * v**2 / R; alpha = sgn * a_t / R
    U = np.array([util(np.array([a_t[i], a_c[i], alpha[i]])) for i in range(len(t))])
    load = f.V_peak * (1 + k / R)
    return dict(W=W, V=V, eps=eps, n=n, T=f.T_path, vlo=V*(1-eps), vhi=f.V_peak, at=np.abs(a_t).max(), ac=np.abs(a_c).max(),
                tot=np.hypot(a_t, a_c).max(), u=U.max(), u_frac=(U > 1).mean()*100, load=load/1.55*100, vx=v[np.argmin(np.abs(s % lap))])
if __name__ == "__main__":
    print("W | V | ε | n | 속도 범위 | 주행 s | |a_t| 최대 | |a_c| 최대 | 합 최대 | C 상한 사용률 최대 · >1 시간 | 부하")
    for W in (2.0, 1.6):
        for V in (0.6, 0.7, 0.8):
            for eps in (0.3, 0.5):
                for n in (2, 4, 6):
                    d = evaluate(W, V, eps, n)
                    if d["load"] > 100: continue
                    print(f"{W} | {V} | {eps} | {n} | {d['vlo']:.2f}~{d['vhi']:.2f} | {d['T']:.1f} | {d['at']:.2f} | {d['ac']:.2f} | {d['tot']:.2f} | {d['u']:.2f} · {d['u_frac']:.0f} % | {d['load']:.0f} %")
