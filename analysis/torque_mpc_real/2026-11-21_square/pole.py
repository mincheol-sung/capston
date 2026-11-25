#!/usr/bin/python3
"""토크 MPC 의 무제약 closed-loop pole — torque_mpc.cpp step() 의 H, g 를 그대로 옮겨 직선 (θ_r = 0, ω_r = 0, U_s 일정) 에서 선형 이득을 만든다.
  예측: x_{i+1} = A x_i + B τ_{min(i, Nc−1)},  A = [I Ts·I; 0 I],  B = [½Ts² G; Ts G],  G = M⁻¹ F,  M = diag(m_eff, m_eff, I_eff),  F = (T⁻¹)ᵀ.
  비용 (torque_mpc.cpp 식-18): Σ_{i=1}^{Np} e_iᵀ W e_i + w_Δ ‖D T − e₁⊗τ_prev‖² + r_U ‖T − U_s‖²  (W 는 스텝마다 같은 대각).
  오차 좌표 δT = T − U_s 에서 δT* = −H⁻¹ (ΦᵀW̄Λ x̃ − w_Δ e₁⊗δτ_prev) → δτ_k = −K_x x̃ + K_τ δτ_prev → 증강 closed loop [x̃; δτ_prev] (10 상태).
  z → s = ln z / Ts. 축 (x, y, θ, 영공간) 은 고유벡터의 몫으로 가른다.
검증: torque_mpc.yaml 2026-11-18 (4) 표 (W [5,5,2.5,2,2,1]) — w_Δ 100·r_U 1 → 가장 빠른 pole 5.2 Hz ζ 0.72, w_Δ 10·r_U 1 → 8.9 Hz ζ 0.80,
      w_Δ 1·r_U 0.1 → 13.2 Hz ζ 0.77, "입력을 아무리 싸게 해도 가장 느린 pole ≈ −1.03"."""
import numpy as np
from scipy.linalg import logm  # noqa: F401  (복소 로그는 np.log 로 충분)

r, k = 0.04, 0.1932; m_eff, I_eff = 6.785, 0.1431
Ts, Np, Nc = 0.01, 50, 5
W = np.array([5.0, 5.0, 2.5, 2.0, 2.0, 1.0])
F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
G = np.diag([1 / m_eff, 1 / m_eff, 1 / I_eff]) @ F
A = np.block([[np.eye(3), Ts * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
B = np.vstack([0.5 * Ts ** 2 * G, Ts * G])
null = np.array([1, -1, -1, 1]) / 2.0                        # F @ null = 0


def gains(w_delta, r_u):
    nU = 4 * Nc
    Phi = np.zeros((6 * Np, nU)); Lam = np.zeros((6 * Np, 6)); Ai = np.eye(6); Pj = np.zeros((6, nU))
    for i in range(Np):
        c = min(i, Nc - 1)
        Pj = A @ Pj; Pj[:, 4 * c:4 * c + 4] += B
        Ai = A @ Ai
        Phi[6 * i:6 * i + 6] = Pj; Lam[6 * i:6 * i + 6] = Ai
    Wb = np.kron(np.eye(Np), np.diag(W))
    Hc = np.zeros((nU, nU))
    for j in range(Nc):
        Hc[4 * j:4 * j + 4, 4 * j:4 * j + 4] = (w_delta * (1.0 if j == Nc - 1 else 2.0) + r_u) * np.eye(4)
        if j + 1 < Nc:
            Hc[4 * j:4 * j + 4, 4 * j + 4:4 * j + 8] = -w_delta * np.eye(4); Hc[4 * j + 4:4 * j + 8, 4 * j:4 * j + 4] = -w_delta * np.eye(4)
    H = Phi.T @ Wb @ Phi + Hc; Hi = np.linalg.inv(H)
    Kx = (Hi @ Phi.T @ Wb @ Lam)[:4]                                # δτ = −Kx x̃ + Kτ δτ_prev
    Kt = w_delta * Hi[:4, :4]
    return Kx, Kt


def poles(w_delta, r_u):
    Kx, Kt = gains(w_delta, r_u)
    Acl = np.block([[A - B @ Kx, B @ Kt], [-Kx, Kt]])
    z, V = np.linalg.eig(Acl); s = np.log(z.astype(complex)) / Ts
    out = []
    for i in range(len(z)):
        v = V[:, i]
        # 축 몫: 위치·속도 성분 (x: 0,3  y: 1,4  θ: 2,5), 영공간: δτ_prev 의 null 방향
        e = np.abs(v[:6]) ** 2; px, py, pt = e[0] + e[3], e[1] + e[4], e[2] + e[5]
        pn = abs(null @ v[6:]) ** 2
        lab = ["x·y", "x·y", "θ", "영공간"][int(np.argmax([px, py, pt, pn]))]   # x·y 는 대칭이라 같은 고윳값 (겹근) — 한 이름으로
        out.append((lab, s[i]))
    return out


def fmt(s):
    wn = abs(s); zeta = -s.real / wn if wn > 0 else 1
    return f"{s.real:+7.2f}{s.imag:+7.2f}j ({wn/(2*np.pi):5.2f} Hz, ζ {zeta:.2f})" if abs(s.imag) > 1e-6 else f"{s.real:+7.2f}         ({wn/(2*np.pi):5.2f} Hz)"


def report(w_delta, r_u, label=""):
    P = poles(w_delta, r_u)
    print(f"-- w_Δ {w_delta:g} · r_U {r_u:g} {label}")
    for ax in ("x·y", "θ", "영공간"):
        ss = sorted({(round(p.real, 3), round(abs(p.imag), 3)) for a, p in P if a == ax})
        print(f"   {ax:3s}: " + "  ".join(fmt(complex(a, b)) for a, b in ss))
    sl = max((p for a, p in P if a == "x·y"), key=lambda p: p.real)       # x·x·y 가장 느린 (실수부가 0 에 가장 가까운)
    fa = max((p for _, p in P), key=lambda p: abs(p))
    return sl, fa


if __name__ == "__main__":
    print("== 검증 (torque_mpc.yaml 2026-11-18 (4) 표)")
    for wd, ru, ref in ((100, 1, "가장 빠른 5.2 Hz ζ 0.72"), (10, 1, "8.9 Hz ζ 0.80"), (1, 0.1, "13.2 Hz ζ 0.77"), (1e-3, 1e-3, "가장 느린 ≈ −1.03")):
        sl, fa = report(wd, ru, f"← 주석값 {ref}")
        print(f"   → 가장 빠른 {fmt(fa)}, 가장 느린 x·y {fmt(sl)}")
    print("== 지금 (w_dudt 0.01 → w_Δ 100, r_U 10) 과 후보")
    res = {}
    for ru in (10, 3, 1, 0.1):
        sl, fa = report(100, ru)
        tau = -1 / sl.real
        res[ru] = (sl, fa)
        print(f"   → x·y 가장 느린 {fmt(sl)} : 시정수 {tau:.2f} s, 0.9 s 뒤 남는 몫 {abs(np.exp(sl*0.9)):.2f} | 가장 빠른 {fmt(fa)}")
    # 옆 0.10 m 오차에 첫 스텝 바퀴 토크 (정지 상태 오차 x̃ = [0, 0.1, 0, 0, 0, 0])
    for ru in (10, 1, 0.1):
        Kx, _ = gains(100, ru); dt = -Kx @ np.array([0, 0.10, 0, 0, 0, 0])
        print(f"   r_U {ru:g}: 옆 0.10 m 오차의 첫 스텝 feedback 토크 {np.round(dt, 4)} N·m → 옆 가속 {(F @ dt)[1] / m_eff:+.2f} m/s²")


def report_W(Wn, w_delta=100, r_u=10):
    global W
    W0 = W; W = np.array(Wn, float)
    P = poles(w_delta, r_u); Kx, _ = gains(w_delta, r_u)
    W = W0
    sl = max((p for a, p in P if a == "x·y"), key=lambda p: p.real); fa = max((p for _, p in P), key=lambda p: abs(p))
    th = max((p for a, p in P if a == "θ"), key=lambda p: p.real)
    dy = -Kx @ np.array([0, 0.10, 0, 0, 0, 0]); dv = -Kx @ np.array([0, 0, 0, 0, 0.30, 0])
    return sl, fa, th, (F @ dy)[1] / m_eff, (F @ dv)[1] / m_eff


if __name__ == "__main__":
    print("== Q₁₁=Q₂₂ (x·y 위치 오차 가중) 후보 (R_Δ = 100 I, R_U = 10 I 그대로)")
    print("   | W | x·y 가장 느린 (시정수) | θ 가장 느린 | 가장 빠른 | 옆 0.10 m 위치 오차 → 첫 스텝 옆 가속 | 옆 0.30 m/s 속도 오차 → 첫 스텝 옆 가속 |")
    for Wn in ([5, 5, 2.5, 2, 2, 1], [20, 20, 2.5, 2, 2, 1], [50, 50, 2.5, 2, 2, 1], [20, 20, 10, 2, 2, 1]):
        sl, fa, th, ay, av = report_W(Wn)
        print(f"   | {Wn} | {sl.real:+.2f} ({-1/sl.real:.2f} s) | {th.real:+.2f} | {fmt(fa)} | {ay:+.2f} m/s² | {av:+.2f} m/s² |")
