"""circles8 (폭 2.0 m, R 0.5 m, yaw 고정, 가감속 0.4 m/s²) 기준이 요구하는 최대 바퀴 토크 vs 속도 → μ̂ 별 최고 속도."""
import math, numpy as np
from amr_mpc.core.figure8 import Figure8
r, k, m, g, Jw, Iz, cs = 0.04, (0.1897 + 0.207) / 2, 4.56, 9.81, 8.9e-4, 0.06, 0.8
mw = 4 * Jw / r ** 2; Meq, Ieq = m + mw, Iz + mw * k * k
P = np.linalg.pinv(((1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]])).T)
EX, EY = 0.97, 0.60                       # 5-2 바닥 효율을 M = m_eq 기준으로
tau_mu = lambda mu: r / math.sqrt(2) * mu * m * g / 4 * cs
def demand(v, ex=EX, ey=EY):
    f = Figure8(width=2.0, speed=v, accel=0.4, yaw_mode="fixed", align_s=0.0, curve="circles")
    t = np.arange(0.0, f.T_total, 0.002)
    u = np.array([f.at(ti)[1] for ti in t])            # 몸체 속도 [vx, vy, ω]
    a = np.gradient(u, t, axis=0)
    F = np.column_stack([Meq * a[:, 0] / ex, Meq * a[:, 1] / ey, Ieq * a[:, 2]])
    return np.abs(P @ F.T).max()
print("검증 (bag 값): ", ", ".join(f"{v} m/s → {demand(v):.4f}" for v in (0.3, 0.5, 0.7)), " (bag 0.0473 / 0.0728 / 0.1299)")
vs = np.arange(0.15, 1.01, 0.01); D = np.array([demand(v) for v in vs])
vload = 0.65 * 1.55 / math.sqrt(2)
print(f"바퀴 속도 부하 한계 (기준 노드 예산 65 %): v ≤ {vload:.3f} m/s")
print("μ̂   | τ̄ N·m  | v_max (필요 ≤ 60 % τ̄, 되먹임 여유) | v_max (필요 ≤ 100 % τ̄, 이론 한계)")
for mu in (0.3, 0.33, 0.4, 0.45, 0.5, 0.6):
    tb = tau_mu(mu)
    v60 = vs[D <= 0.6 * tb].max() if (D <= 0.6 * tb).any() else float('nan')
    v100 = vs[D <= tb].max() if (D <= tb).any() else float('nan')
    print(f"{mu:<4} | {tb:.4f} | {v60:.2f} m/s{'  (속도 한계로 ' + format(min(v60, vload), '.2f') + ')' if v60 > vload else ''}" + " " * 18 + f"| {v100:.2f} m/s{'  (속도 한계로 ' + format(min(v100, vload), '.2f') + ')' if v100 > vload else ''}")

print("\n관측 판의 필요/τ̄ (생성 코드 기준): 175042 0.3·μ̂0.3 → %.0f %% (잘 따라감), 175613 0.5·μ̂0.5 → %.0f %% (잘 따라감), 180104 0.7·μ̂1.0 → %.0f %% (잘 따라감·슬립), "
      "175201 0.5·μ̂0.3 → %.0f %% (34~51 cm), 180204 0.7·μ̂0.3 → %.0f %% (107 cm)" % (
      100 * demand(0.3) / tau_mu(0.3), 100 * demand(0.5) / tau_mu(0.5), 100 * demand(0.7) / tau_mu(1.0), 100 * demand(0.5) / tau_mu(0.3), 100 * demand(0.7) / tau_mu(0.3)))
print("μ̂   | τ̄ N·m  | v_max @70 % (관측상 잘 따라간 범위) | v_max @100 %")
for mu in (0.3, 0.33, 0.4, 0.45, 0.5):
    tb = tau_mu(mu)
    v70 = vs[D <= 0.7 * tb].max(); v100 = vs[D <= tb].max()
    print(f"{mu:<4} | {tb:.4f} | {min(v70, vload):.2f} m/s{' (속도 한계)' if v70 > vload else ''}                         | {min(v100, vload):.2f} m/s{' (속도 한계)' if v100 > vload else ''}")
