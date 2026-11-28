"""바퀴 공간 슬립 되먹임 이득 K_ω (4×4) 설계 시제품 — C++ 에 넣기 전에 숫자 확인.
  ① 역기구학: 지금 Δτ = −K_v (R(δ0)Tω − v_e) 를 바퀴 오차 e_ω = ω − J v_e (J = T⁻¹R(δ0)ᵀ) 로 쓰면 K_ω = K_v R(δ0) T
  ② 최적화: 바퀴 슬립 모델  ė = J_w⁻¹Δτ − (J_w⁻¹I + J M_b⁻¹ Jᵀ) C e  를 ZOH 이산화 → DARE → K_ω
python3 -I kw_design.py <npz 폴더>"""
import os
import sys
import numpy as np

r, wb, wd = 0.04, 0.1724, 0.2140
k = (wb + wd) / 2
d0 = np.radians(3.1)
m, Iz, Jw, Ts = 4.56, 0.03, 8.9e-4, 0.01
N = 4.56 * 9.81 * np.array([0.239, 0.239, 0.261, 0.261])
Kv = np.array([[+0.1031, -0.0925, -0.0450], [+0.0925, +0.1031, +0.0450],
               [+0.0925, +0.1031, -0.0450], [+0.1031, -0.0925, +0.0450]])     # 실차 기동 로그 (162411)
Tinv = np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]) / r
T = np.linalg.pinv(Tinv)
c, s = np.cos(d0), np.sin(d0)
R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
J = Tinv @ R.T
W = ("FL", "FR", "RL", "RR")
np.set_printoptions(precision=4, suppress=True, linewidth=140)
print("R T J =", (R @ T @ J).round(12).tolist())

K_ik = Kv @ R @ T
sv = np.linalg.svd(K_ik, compute_uv=False)
null = np.linalg.svd(K_ik)[2][-1]
print("\n① 역기구학 K_ω = K_v R(δ0) T  [N·m per rad/s]\n", K_ik)
print("   특이값", sv.round(6), "→ rank", int((sv > 1e-9).sum()), "  null 방향", (null / np.abs(null).max()).round(3))


def dlqr_slip(q_s, rho, cstiff):
    Mb_inv = np.diag([1 / m, 1 / m, 1 / Iz])
    Gw = J @ Mb_inv @ J.T
    A = -(np.eye(4) / Jw + Gw) @ (cstiff * np.eye(4))
    B = np.eye(4) / Jw
    Mx = np.zeros((8, 8)); Mx[:4, :4] = A * Ts; Mx[:4, 4:] = B * Ts
    from scipy.linalg import expm, solve_discrete_are
    E = expm(Mx); Ad, Bd = E[:4, :4], E[:4, 4:]
    Q = q_s * r ** 2 * np.eye(4); Rr = rho * np.eye(4)
    P = solve_discrete_are(Ad, Bd, Q, Rr)
    K = np.linalg.solve(Rr + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, np.linalg.eigvals(Ad - Bd @ K), np.linalg.eigvals(Ad)


print("\n② 최적화 (DARE, 비용 Σ q_s‖r e‖² + ρ‖Δτ‖², q_s = MPC q_ẋ 2, ρ = MPC r_U 10)")
for v_design in (None, 1.0, 0.5):
    if v_design is None:
        cst, lab = 0.0, "미끄러지는 중 (접촉 힘 포화, C = 0)"
    else:
        cst = 1.26 * N.mean() * r ** 2 / (np.sqrt(2) * v_design)
        lab = f"붙어 구름 (μ–λ 기울기 1.26, v {v_design} m/s → C = {cst:.4f} N·m/(rad/s))"
    K, pcl, pol = dlqr_slip(2.0, 10.0, cst)
    print(f"  {lab}\n", K, "\n   열린 고리 극", np.sort(np.abs(pol)).round(4), " 닫힌 고리 극", np.sort(np.abs(pcl)).round(4),
          " → 가장 느린 시정수", f"{-Ts / np.log(np.abs(pcl).max()) * 1e3:.0f} ms")

# ③ bag 재생: e_ω 를 다시 만들어 ① 이 기록된 Δτ 와 같은지, ② 의 크기
K_lqr = dlqr_slip(2.0, 10.0, 0.0)[0]
for name in sys.argv[2:]:
    A_ = np.load(os.path.join(sys.argv[1], name + ".npz"))
    F, ws, od, arm = A_["sfb"], A_["ws"], A_["od"], A_["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]; t_off = td[td > t_on][0] if (td > t_on).any() else ws[-1, 0]
    t = F[:, 0]; m_ = (t >= t_on) & (t <= t_off) & (F[:, 3] > 0.5)
    iw = np.clip(np.searchsorted(ws[:, 0], t) - 1, 0, len(ws) - 1); io = np.clip(np.searchsorted(od[:, 0], t) - 1, 0, len(od) - 1)
    e_w = ws[iw, 5:9] - od[io, 4:7] @ J.T
    d_log = F[:, 7:11]
    d_ik = -e_w @ K_ik.T * (1.0 if name.endswith("r2") else 0.5)    # r3 은 배율 0.5
    d_new = -e_w @ K_lqr.T
    err = d_ik[m_] - d_log[m_]
    print(f"\n③ {name}: 무장 중 {m_.sum()} 주기")
    print(f"   다시 만든 e_ω 로 ① 계산 − 기록 Δτ : 중앙 |차| {np.median(np.abs(err)):.4f}, 상관 {np.corrcoef(d_ik[m_].ravel(), d_log[m_].ravel())[0, 1]:.4f}")
    print("   e_ω RMS [rad/s] " + "  ".join(f"{W[i]} {np.sqrt(np.mean(e_w[m_, i] ** 2)):.2f}" for i in range(4)))
    print("   기록 Δτ RMS      " + "  ".join(f"{W[i]} {np.sqrt(np.mean(d_log[m_, i] ** 2)):.3f}" for i in range(4)))
    print("   ② Δτ RMS (배율 1) " + "  ".join(f"{W[i]} {np.sqrt(np.mean(d_new[m_, i] ** 2)):.3f}" for i in range(4)) +
          f"   최대 {np.abs(d_new[m_]).max():.3f} N·m")
