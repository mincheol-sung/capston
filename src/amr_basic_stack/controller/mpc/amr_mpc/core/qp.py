#!/usr/bin/env python3
# Wang Ch.2.4 — Hildreth 이차계획법 (교재 QPhild.m 의 numpy 이식)
# ---------------------------------------------------------------
#   min  ½ ηᵀ E η + ηᵀ F      s.t.  M η ≤ γ
#
#   1. 무제약 해 η⁰ = −E⁻¹F. 제약 전부 만족하면 즉시 반환 (활성 아닌 샘플이 대부분).
#   2. 쌍대:  H = M E⁻¹ Mᵀ,  K = γ + M E⁻¹ F.   λ ≥ 0 를 Gauss-Seidel 로 갱신
#        λᵢ ← max(0, −(kᵢ + Σ_{j≠i} hᵢⱼ λⱼ) / hᵢᵢ)        (λ 를 제자리 갱신 → j<i 는 새 값)
#   3. η = η⁰ − E⁻¹ Mᵀ λ
#
#   글자: E, F (primal), M, γ (제약), H, K (dual) — 교재와 동일. 예측행렬은 F_free.
#   반복 상한에 걸려도 그때까지의 λ 로 반환한다 (역행렬 없음 → 안 죽음). 호출자가 위반량을 확인할 것.
#
#   qpfgm(): 같은 dual 을 Nesterov 가속 투영경사로 푼다 (Richter, Jones, Morari 2012, "Computational
#   complexity certification for real-time MPC using a fast gradient method"). 전부 벡터 연산이라
#   파이썬에서 Hildreth 보다 10배 이상 빠르고, 출력 제약처럼 조건수가 나쁜 dual 에서도 수렴한다.
#   운용은 qpfgm, qphild 는 교재 대조·검산용.
#
#   python3 qphild.py     교재 Example 2.10 / 2.11 검산 (두 솔버 모두)
# ---------------------------------------------------------------
import numpy as np


def qphild(E, F, M, gamma, max_iter=100, tol=1e-9, viol_tol=None, lam0=None):
    """반환 (η, 반복수, λ).  반복수 0 = 무제약 해가 이미 가능.
    정지: ‖Δλ‖² < tol  또는  (viol_tol 지정 시) max(Mη − γ) < viol_tol.
    lam0: 직전 호출의 λ (워밍스타트). 제약 수가 같을 때만 쓴다."""
    eta0 = -np.linalg.solve(E, F)
    if (M @ eta0 <= gamma + 1e-12).all():
        return eta0, 0, np.zeros(len(gamma))
    EiMt = np.linalg.solve(E, M.T)                 # E⁻¹Mᵀ  (n × m)
    H = M @ EiMt                                   # dual Hessian (m × m)
    K = gamma - M @ eta0                           # = γ + M E⁻¹ F
    lam = np.zeros(len(gamma)) if lam0 is None or len(lam0) != len(gamma) else lam0.copy()
    h_diag = np.diag(H)
    for it in range(1, max_iter + 1):
        lam_p = lam.copy()
        for i in range(len(lam)):
            w = H[i] @ lam - h_diag[i] * lam[i] + K[i]
            lam[i] = max(0.0, -w / h_diag[i])
        if (lam - lam_p) @ (lam - lam_p) < tol:
            break
        if viol_tol is not None and (M @ (eta0 - EiMt @ lam) - gamma).max() < viol_tol:
            break
    return eta0 - EiMt @ lam, it, lam


def qpfgm(E, F, M, gamma, max_iter=2000, viol_tol=1e-6, lam0=None):
    """dual  min_{λ≥0} ½λᵀHλ + Kᵀλ  을 Nesterov 가속 투영경사로. 반환 (η, 반복수, λ).
    스텝 1/L (L = λ_max(H)), 10 회마다 primal 위반 검사."""
    eta0 = -np.linalg.solve(E, F)
    if (M @ eta0 <= gamma + 1e-12).all():
        return eta0, 0, np.zeros(len(gamma))
    EiMt = np.linalg.solve(E, M.T); H = M @ EiMt; K = gamma - M @ eta0
    alpha = 1.0 / np.linalg.eigvalsh(H).max()
    lam = np.zeros(len(gamma)) if lam0 is None or len(lam0) != len(gamma) else lam0.copy()
    y, t = lam.copy(), 1.0
    for it in range(1, max_iter + 1):
        lam_new = np.maximum(0.0, y - alpha * (H @ y + K))
        t_new = 0.5 * (1 + np.sqrt(1 + 4 * t * t))
        y = lam_new + (t - 1) / t_new * (lam_new - lam)
        lam, t = lam_new, t_new
        if it % 10 == 0 and (M @ (eta0 - EiMt @ lam) - gamma).max() < viol_tol:
            break
    return eta0 - EiMt @ lam, it, lam


def _check(name, E, F, M, g, expect):
    ok = True
    for sname, solver in (("Hildreth", qphild), ("FGM", qpfgm)):
        eta, it, lam = solver(np.array(E, float), np.array(F, float), np.array(M, float), np.array(g, float))
        good = np.allclose(eta, expect, atol=2e-4); ok &= good
        print(f"[{name} / {sname:8s}] η = {np.round(eta, 4)}  기대 {expect}  반복 {it:3d}  λ = {np.round(lam, 4)}   {'OK' if good else '✗'}")
    return ok


if __name__ == "__main__":
    ok = _check("Ex 2.10", [[2, -1], [-1, 1]], [-1, 0], [[-1, 0], [0, -1], [3, 2]], [0, 0, 4], [0.8276, 0.7586])
    ok &= _check("Ex 2.11", [[1, 0], [0, 1]], [-2, -2], [[1, 0], [-1, 0], [0, 1], [0, -1]], [1, 0, 1, 0], [1.0, 1.0])
    raise SystemExit(0 if ok else 1)


# ---------------------------------------------------------------
# Primal active-set (Nocedal & Wright, Numerical Optimization 2nd ed., Algorithm 16.3)
#   min ½ηᵀEη + Fᵀη  s.t.  Mη ≤ γ,   E ≻ 0.
#   작은 dense QP (변수 수십, 제약 백 단위, 활성 소수) 에 적합. 해가 정확(활성집합 위에서 KKT 를 직접 풂)하고
#   working set 을 다음 틱에 warm start 하면 보통 0~3 회로 끝난다.
#   시작점은 feasible 해야 한다 — η₀ = 0 (구조 B: ω_prev 가 한계 안이면 ΔΩ = 0 이 항상 feasible).
# ---------------------------------------------------------------
def qpactive(E, F, M, gamma, max_iter=50, W0=None, eta0=None, tol=1e-10):
    """반환 (η, 반복수, λ(전체 길이), W(활성 인덱스 리스트)).  eta0 는 feasible 점이어야 한다."""
    n, m = E.shape[0], M.shape[0]
    eta = np.zeros(n) if eta0 is None else np.array(eta0, float)
    if (M @ eta - gamma).max() > 1e-9:
        eta = np.zeros(n)                                  # 시작점이 infeasible 이면 0 으로
    W = [i for i in (W0 or []) if 0 <= i < m]
    # warm-start working set 이 현재 점에서 활성이 아니면 버린다 (아래 EQP 가 잘못된 등식을 강제하지 않게)
    W = [i for i in W if abs(M[i] @ eta - gamma[i]) < 1e-6]
    lam_full = np.zeros(m)
    for it in range(1, max_iter + 1):
        k = len(W)
        if k:
            MW = M[W]
            K = np.zeros((n + k, n + k)); K[:n, :n] = E; K[:n, n:] = MW.T; K[n:, :n] = MW
            rhs = np.concatenate([-F, gamma[W]])
            try:
                sol = np.linalg.solve(K, rhs)
            except np.linalg.LinAlgError:                   # 종속 행 → 최소자승
                sol = np.linalg.lstsq(K, rhs, rcond=None)[0]
            eta_star, lam_W = sol[:n], sol[n:]
        else:
            eta_star, lam_W = np.linalg.solve(E, -F), np.zeros(0)
        p = eta_star - eta
        if np.abs(p).max() < tol:
            if k == 0 or lam_W.min() >= -tol:               # KKT 만족
                lam_full[:] = 0; lam_full[W] = np.maximum(lam_W, 0)
                return eta, it, lam_full, W
            W.pop(int(np.argmin(lam_W)))                    # 음수 승수 제약을 뺀다
            continue
        # 비활성 제약에 대한 ratio test
        alpha, block = 1.0, None
        Mp = M @ p; slack = gamma - M @ eta
        for i in range(m):
            if i in W or Mp[i] <= 1e-12:
                continue
            a = slack[i] / Mp[i]
            if a < alpha:
                alpha, block = a, i
        eta = eta + alpha * p
        if block is not None:
            W.append(block)
    lam_full[:] = 0
    if len(W):
        lam_full[W] = np.maximum(lam_W, 0)
    return eta, max_iter, lam_full, W
