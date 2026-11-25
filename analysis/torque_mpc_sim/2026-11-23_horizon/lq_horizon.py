#!/usr/bin/python3
"""토크 MPC 의 지평 (Np, Nc) — 계산: 직선·정지 선형화 (θ = 0, w = 0) 에서 무한 지평 LQ (DARE) 와 유한 지평·move blocking 이득 비교.
모델 (torque_mpc.cpp 와 같음): x = [p; v] (6), x⁺ = A x + B τ,  A = [I, Ts I; 0, I],  B = [½Ts² G; Ts G],  G = M⁻¹ F,  F = (T⁻¹)ᵀ.
비용 (식-16): Σ_{j=1}^{Np} ‖x_j‖²_W + Σ_{j<Nc} [w_Δ ‖τ_j − τ_{j−1}‖² + r_U ‖τ_j‖²]   (terminal cost 없음, Nc 뒤 τ 유지)
무한 지평 (Nc = Np → ∞): z = [x; τ_prev], 입력 Δτ — Q̃ = blkdiag(W, r_U I), R̃ = (w_Δ + r_U) I, 교차항 S = [0; r_U I].
가중치는 torque_mpc.yaml (W, w_dudt 0.01 → w_Δ 100, r_u 10, q_null 0)."""
import numpy as np, scipy.linalg as sl, yaml
Y = yaml.safe_load(open("/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/config/torque_mpc.yaml"))["tuning"]
Ts = Y["Ts"]; W = np.diag(Y["W"]); wD = Y["w_dudt"] / Ts ** 2; rU = Y["r_u"]
r, k, m, Jw, Iz = 0.04, 0.1932, 4.56, 8.9e-4, 0.06
mu = 4 * Jw / r ** 2; M = np.diag([m + mu, m + mu, Iz + mu * k * k])
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r; G = np.linalg.solve(M, F)
A = np.block([[np.eye(3), Ts * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]]); B = np.vstack([0.5 * Ts * Ts * G, Ts * G])
n, mI = 6, 4
Aa = np.block([[A, B], [np.zeros((mI, n)), np.eye(mI)]]); Ba = np.vstack([B, np.eye(mI)])
Qa = sl.block_diag(W, rU * np.eye(mI)); Ra = (wD + rU) * np.eye(mI); Sa = np.vstack([np.zeros((n, mI)), rU * np.eye(mI)])
P = sl.solve_discrete_are(Aa, Ba, Qa, Ra, s=Sa); Kinf = np.linalg.solve(Ra + Ba.T @ P @ Ba, Ba.T @ P @ Aa + Sa.T)
ev = np.linalg.eigvals(Aa - Ba @ Kinf); evc = np.log(ev.astype(complex)) / Ts
print(f"가중치: W {Y['W']}, w_Δ {wD:.0f}, r_U {rU}, Ts {Ts}")
print("무한 지평 LQ closed-loop pole (연속 환산 s = ln λ / Ts, 1/s), 느린 순:")
for s in sorted(evc, key=lambda z: -z.real)[:8]:
    print(f"   {s.real:+8.3f} {s.imag:+8.3f}j   시정수 {-1/s.real:6.3f} s")
tau_dom = -1 / max(evc.real)

# 유한 지평 · move blocking 이득 (batch, z = [x; τ_prev] → 첫 τ_0)
def batch(Np, Nc):
    # τ_j = U[min(j, Nc−1)]; x_j = A^j x0 + Σ_{i<j} A^{j−1−i} B τ_i
    Ax = np.zeros((n * Np, n)); Bu = np.zeros((n * Np, mI * Nc)); Ap = np.eye(n); Phi = np.zeros((n, mI * Nc))
    for j in range(1, Np + 1):
        Phi = A @ Phi; Phi[:, mI * min(j - 1, Nc - 1):mI * min(j - 1, Nc - 1) + mI] += B
        Ap = A @ Ap; Ax[n * (j - 1):n * j] = Ap; Bu[n * (j - 1):n * j] = Phi
    Wb = np.kron(np.eye(Np), W)
    D = np.eye(mI * Nc) - np.eye(mI * Nc, k=-mI)        # Δ = D U − e1 τ_prev
    E1 = np.zeros((mI * Nc, mI)); E1[:mI] = np.eye(mI)
    H = Bu.T @ Wb @ Bu + wD * D.T @ D + rU * np.eye(mI * Nc)
    gx = Bu.T @ Wb @ Ax; gt = -wD * D.T @ E1                  # g = gx x0 + gt τ_prev
    Kz = np.linalg.solve(H, np.hstack([gx, gt]))[:mI]         # τ_0 = −Kz z
    Kd = Kz.copy(); Kd[:, n:] += np.eye(mI)                   # Δτ_0 = τ_0 − τ_prev = −(Kz + [0 I]) z
    return Kd, H, Bu, Wb
print(f"\n유한 지평 이득 K(Np, Nc) 와 K∞ 의 상대 차 ‖K − K∞‖/‖K∞‖, closed-loop 가장 느린 pole (1/s)  [무한 지평 가장 느린 시정수 {tau_dom:.3f} s]")
print("   Np \\ Nc " + "".join(f"{nc:>16d}" for nc in (5, 10, 15, 25, 50, 100)))
for Np in (20, 30, 50, 75, 100, 150, 200):
    row = f"   {Np:4d} ({Np*Ts:.2f} s)"
    for Nc in (5, 10, 15, 25, 50, 100):
        if Nc > Np:
            row += f"{'':>16s}"; continue
        Kd, *_ = batch(Np, Nc)
        e = np.linalg.norm(Kd - Kinf) / np.linalg.norm(Kinf)
        s = np.log(np.linalg.eigvals(Aa - Ba @ Kd).astype(complex)) / Ts
        row += f"  {100*e:5.1f}%  {max(s.real):+6.2f}"
    print(row)

# preview: 미래 기준 x_r,j 가 첫 입력 τ_0 에 주는 영향 (긴 지평, Nc = Np) — j 에 따른 크기
Np = 200; Kd, H, Bu, Wb = batch(Np, Np)
Gr = np.linalg.solve(H, Bu.T @ Wb)[:mI]                     # τ_0 = ... + Gr x_ref (쌓은 것)
mag = np.array([np.linalg.norm(Gr[:, n * j:n * j + 3]) for j in range(Np)])   # 위치 기준의 몫
cum = np.cumsum(mag) / mag.sum()
print(f"\npreview (Np = Nc = 200): 위치 기준 x_r,j 가 τ_0 에 주는 영향의 누적 몫 — 90 % 에 {np.searchsorted(cum, 0.90)+1} 스텝 ({(np.searchsorted(cum,0.90)+1)*Ts:.2f} s), "
      f"95 % 에 {np.searchsorted(cum, 0.95)+1} ({(np.searchsorted(cum,0.95)+1)*Ts:.2f} s), 99 % 에 {np.searchsorted(cum, 0.99)+1} ({(np.searchsorted(cum,0.99)+1)*Ts:.2f} s)")
print("   크기 (최대 대비) j = " + " ".join(f"{j}:{mag[j-1]/mag.max():.2f}" for j in (1, 10, 25, 50, 75, 100, 150)))
