#!/usr/bin/python3
"""정사각형 기준 프로파일 (side 1.5, corner 0.1, accel 1.5, v_max 1.5, tangent, yaw_blend 0.3, align 1.5 s) 과
그 기준을 따라가는 데 드는 바퀴 토크 (torque_mpc 의 목표 입력 U_s, 식-14) vs C 마찰 상한.

  U_s = F⁺ R(ψ_r)ᵀ M (p̈_r − w),   F = force_map (torque_mpc.cpp),  M = diag(m+m_w, m+m_w, I_z+m_w k²),
  w = (m_w/m_eff) ω [−ẏ, ẋ, 0]  (use_w 켬).
  p̈_r 두 가지:  exact  = 해석 (접선 가속 ±a, α 는 ω 의 1 ms 차분)
                 diff   = 노드가 실제로 쓰는 0.2 s 차분 (월드 속도, 창 가운데를 t 로 둠 — 노드의 창은 기준열 첫 행부터라
                          0.05~0.15 s 앞당겨진다. 크기는 같고 시각만 밀린다)
  C 상한 τ̄_i = (r/√2) μ̂_i (m g/4) c_s,  B 상한 τ_rated − τ_fric_max.
값은 리포 설정 파일에서 읽는다."""
import csv, math, os, sys
import numpy as np, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
MPC = "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc"
sys.path.insert(0, MPC)
from amr_mpc.core.square import Square, wrap  # noqa: E402

P = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml"))
TM = yaml.safe_load(open(f"{MPC}/config/torque_mpc.yaml"))
g = P["model"]["gravity"]; body = P["body"]; wh = P["wheel"]; geo = P["geometry"]
r = geo["wheel_radius_m"]; k = 0.5 * (geo["wheel_base_m"] + geo["robot_width_m"])
m = body["mass_kg"] + body.get("payload_kg", 0.0); Iz = body["inertia_z_kgm2"]; Jw = wh["inertia_J_w_kgm2"]
mw = 4 * Jw / r**2; m_eff = m + mw; I_eff = Iz + mw * k * k
M = np.diag([m_eff, m_eff, I_eff])
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
Fp = np.linalg.pinv(F)
mu = np.array(P["surface"]["mu_hat_wheels"]); cs_ = P["surface"]["c_s"]
TB_C = r / math.sqrt(2) * mu * m * g / 4 * cs_
TB_B = P["motor"]["tau_rated_Nm"] - P["friction"]["tau_fric_max_Nm"]
ACC_DT = TM["tuning"]["acc_dt"]

SIDE, VC, A, VMAX, BLEND, ALIGN = 1.5, 0.1, 1.5, 1.5, 0.3, 1.5
sq = Square(side=SIDE, corner_speed=VC, accel=A, v_max=VMAX, yaw_mode="tangent", yaw_blend=BLEND, align_s=ALIGN)

dt = 1e-3
t = np.arange(0.0, sq.T_total + 0.3, dt)


def world(tt):
    X, u = sq.at(tt)
    c, s = math.cos(X[2]), math.sin(X[2])
    return X, u, np.array([c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]])


def phase(tt):
    """(변 번호, 접선 가속, 이동 거리, 속력) — 주행 밖이면 (−1, 0, ·, 0)."""
    if tt <= 0.0 or tt >= sq.T_path:
        return -1, 0.0, 0.0, 0.0
    i = int(np.searchsorted(sq.t_start, tt, side="right") - 1); e = sq.edges[i]; tau = tt - sq.t_start[i]
    s, v = e.at(tau)
    at = A if tau < e.t1 else (0.0 if tau < e.t1 + e.t2 else -A)
    return i, at, s, v


X = np.zeros((len(t), 3)); U = np.zeros((len(t), 3)); V = np.zeros((len(t), 3))
EDGE = np.zeros(len(t), int); AT = np.zeros(len(t)); SPD = np.zeros(len(t)); S = np.zeros(len(t))
for n, tt in enumerate(t):
    X[n], U[n], V[n] = world(tt)
    i, at, s, v = phase(tt)
    EDGE[n], AT[n], SPD[n] = i, at, v
    S[n] = (i * SIDE + s) if i >= 0 else (sq.L if tt >= sq.T_path else 0.0)
psi = np.unwrap(X[:, 2])
alpha = np.gradient(U[:, 2], dt)

# exact p̈ (월드): 접선 가속 × 변 방향 (꼭짓점 속도 방향 계단 √2 v_c 는 충격이라 뺀다)
acc_exact = np.zeros((len(t), 3))
for n in range(len(t)):
    if EDGE[n] >= 0:
        acc_exact[n, :2] = AT[n] * sq.dirs[EDGE[n]]
acc_exact[:, 2] = alpha
# 노드가 쓰는 0.2 s 차분
h = int(round(0.5 * ACC_DT / dt))
Vpad = np.vstack([np.repeat(V[:1], h, 0), V, np.repeat(V[-1:], h, 0)])
acc_diff = (Vpad[2 * h:] - Vpad[:-2 * h]) / ACC_DT


def ff(acc):
    out = np.zeros((len(t), 4))
    for n in range(len(t)):
        c, s = math.cos(X[n, 2]), math.sin(X[n, 2])
        Rt = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        w = np.array([-mw * V[n, 2] * V[n, 1] / m_eff, mw * V[n, 2] * V[n, 0] / m_eff, 0.0])
        out[n] = Fp @ (Rt @ (M @ (acc[n] - w)))
    return out


US_E, US_D = ff(acc_exact), ff(acc_diff)
RAT_E, RAT_D = np.abs(US_E) / TB_C, np.abs(US_D) / TB_C

# 꼭짓점 구간 = 헤딩 회전 구간 |S − j·side| ≤ blend/2 (j = 1..3)
corner = np.zeros(len(t), bool)
for j in range(1, sq.n):
    corner |= (np.abs(S - j * SIDE) <= 0.5 * BLEND) & (EDGE >= 0)
run = (t > 0) & (t < sq.T_path)
align = (t >= sq.T_path) & (t <= sq.T_total)

print(f"[설정] m {m} kg, m_w {mw:.3f}, m_eff {m_eff:.3f}, I_z {Iz}, I_eff {I_eff:.4f}, k {k:.4f}, r {r}, g {g}, acc_dt {ACC_DT}")
print(f"[상한] C τ̄ = {np.round(TB_C, 4)} N·m (μ̂ {mu}, c_s {cs_}),  B {TB_B:.2f} N·m")
print(f"[시간] 주행 {sq.T_path:.3f} s + 정렬 {sq.T_align:.2f} s = {sq.T_total:.3f} s, 거리 {sq.L:.1f} m, 평균 속력 {sq.L / sq.T_path:.3f} m/s")
print("| 변 | v_s | v_p | v_e | 가속 t1 (거리) | 등속 t2 (거리) | 감속 t3 (거리) | 변 시간 | 시작 시각 |")
for i, e in enumerate(sq.edges):
    print(f"| {i} | {e.vs:.2f} | {e.vp:.3f} | {e.ve:.2f} | {e.t1:.3f} s ({e.d1:.3f} m) | {e.t2:.4f} s ({e.d2:.4f} m) | "
          f"{e.t3:.3f} s ({e.d3:.3f} m) | {e.T:.3f} s | {sq.t_start[i]:.3f} s |")
# 꼭짓점 하나 (j = 1)
c1 = (np.abs(S - SIDE) <= 0.5 * BLEND) & (EDGE >= 0)
tc = t[c1]
print(f"[꼭짓점 1] 회전 구간 {tc[0]:.3f}~{tc[-1]:.3f} s ({tc[-1] - tc[0]:.3f} s), 진입 속력 {SPD[c1][0]:.3f} m/s, "
      f"꼭짓점 시각 {sq.t_start[1]:.3f} s 속력 {VC}")
print(f"   |ω| 최대 {np.abs(U[c1, 2]).max():.3f} rad/s, |α| 최대 {np.abs(alpha[c1]).max():.2f} rad/s² (꼭짓점에서 접선 가속 부호가 바뀌며 계단)")
print(f"   0.2 s 차분 p̈ 크기 최대 {np.hypot(acc_diff[c1, 0], acc_diff[c1, 1]).max():.3f} m/s², θ̈ 최대 {np.abs(acc_diff[c1, 2]).max():.2f} rad/s²")
print(f"[정렬 회전] ω 최대 {np.abs(U[align, 2]).max():.3f} rad/s, α 최대 {np.abs(alpha[align]).max():.2f} rad/s²")
names = ["FL", "FR", "RL", "RR"]
for lab, R_, US in (("exact", RAT_E, US_E), ("0.2 s 차분 (노드)", RAT_D, US_D)):
    mx = R_.max(1)
    print(f"[U_s/τ̄ — {lab}] 최대 {mx[run].max():.2f} (t {t[run][np.argmax(mx[run])]:.3f} s), "
          f"주행 중 >1 {100 * (mx[run] > 1).mean():.1f} %, 꼭짓점 구간 중 >1 {100 * (mx[corner] > 1).mean():.1f} %, "
          f"꼭짓점 밖 주행 >1 {100 * (mx[run & ~corner] > 1).mean():.1f} %, 정렬 최대 {mx[align].max():.2f}")
    print("    바퀴별 |U_s| 최대 [N·m]: " + ", ".join(f"{nm} {np.abs(US[run, i]).max():.3f} (τ̄ {TB_C[i]:.4f}, ×{np.abs(US[run, i]).max() / TB_C[i]:.2f})" for i, nm in enumerate(names))
          + f";  B 상한 {TB_B:.2f} 대비 최대 {np.abs(US[run]).max() / TB_B * 100:.0f} %")
# 직선 등가속 검산: U_s = m_eff a r / 4
st = run & ~corner & (np.abs(AT) > 0) & (np.abs(U[:, 2]) < 1e-9)
print(f"[검산] 직선 등가속 |U_s| exact {np.abs(US_E[st]).mean():.4f} vs m_eff·a·r/4 {m_eff * A * r / 4:.4f} N·m "
      f"(÷ RR τ̄ {m_eff * A * r / 4 / TB_C[3]:.3f})")

# CSV (10 ms) — 기준 프로파일 그 자체
with open(os.path.join(HERE, "square_profile_v1.5_a1.5.csv"), "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["t", "x", "y", "psi_deg", "vx_body", "vy_body", "omega", "speed", "a_tan", "alpha",
                 "Us_FL", "Us_FR", "Us_RL", "Us_RR", "ratio_max_C"])
    for n in range(0, len(t), 10):
        wr.writerow([f"{t[n]:.3f}", f"{X[n, 0]:.4f}", f"{X[n, 1]:.4f}", f"{math.degrees(psi[n]):.2f}", f"{U[n, 0]:.4f}",
                     f"{U[n, 1]:.4f}", f"{U[n, 2]:.4f}", f"{SPD[n]:.4f}", f"{AT[n]:.2f}", f"{alpha[n]:.3f}",
                     *[f"{x:.4f}" for x in US_D[n]], f"{RAT_D[n].max():.3f}"])
np.savez(os.path.join(HERE, "profile.npz"), t=t, X=X, U=U, psi=psi, alpha=alpha, spd=SPD, at=AT, S=S, corner=corner,
         us_e=US_E, us_d=US_D, rat_e=RAT_E, rat_d=RAT_D, tb=TB_C, t_start=np.array(sq.t_start), T_path=sq.T_path,
         T_total=sq.T_total)
