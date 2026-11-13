#!/usr/bin/env python3
# 구조 B — 바퀴 각속도 입력 MPC.  body/mpc.py(몸체속도 입력)와 병행한다.
# ---------------------------------------------------------------
# Wang Ch.1.3–1.4 예측·최적화 + Ch.2 제약.  결정변수 ΔΩ = [Δω(k); …; Δω(k+Nc−1)] (4·Nc)
#   Y = F_free x + Φ ΔΩ,   E = 2(ΦᵀQ̄Φ + R̄),   F = 2ΦᵀQ̄F_free x,   ΔΩ* = −E⁻¹F
#   제약 (M η ≤ γ, 행 정규화):
#     바퀴 속도  |ω_i(k+j)| ≤ W_MAX,   ω(k+j) = ω_prev − Σ_{l≤j} Δω_l    →  2·4·Nc 행 (box — L1 대리식 불필요)
#     바퀴 가속  |Δω_i| ≤ ALPHA_MAX·Ts                                   →  2·4·Nc 행
#   솔버: 기본 primal active-set (core/qp.qpactive, warm start). Hildreth·FGM 도 선택 가능 (plant_wheel.yaml tuning.solver).
#         상한 걸리면 바퀴별 클립 폴백.
#
# ★ null-space 벌점 (2026-09-10, bags/mpc_chain_20260910_161323).  T_FWD 는 3×4 라 nullity 1 —
#   n = [1,1,−1,−1]/2 (앞 두 바퀴 vs 뒤 두 바퀴) 방향의 바퀴속도는 몸체를 전혀 못 움직인다 (T_FWD n = 0).
#   비용은 y = X_e 와 Δω 뿐이라 이 방향은 어느 항에도 안 보인다:
#     Φ (n⊗e_j) = 0  →  F = 2ΦᵀQ̄F_free x 는 null 성분이 0,   E (n⊗e_j) = 2 r_dw (n⊗e_j)
#   즉 무제약 해 −E⁻¹F 는 null 성분을 절대 만들지 않지만 **지우지도 않는다**. incremental form 이
#   ω_cmd = ω_prev − Δω 로 입력에 적분기를 심어 두었으므로, 바퀴별 box·rate 제약이 활성일 때 KKT 항
#   Mᵀλ (M 의 행이 e_i 라 null 성분을 가짐) 로 한 번 들어온 nᵀω 는 제약이 풀린 뒤에도 그대로 남는다 —
#   출력에서 안 보이는 극 1 (undetectable integrator). 실차 hold 구간 29 곳에서 |nᵀω_cmd| 중앙값
#   0.2~19 rad/s 가 상수로 유지됐고 (표준편차 ≤ 0.08), 바퀴는 그 속도로 돌며 롤러만 미끄러졌다.
#   미끄러지는 동안 IMU 자이로 1σ 153 mrad/s (null 0.2 rad/s 구간은 11) — 몸체가 요 방향으로 흔들리고
#   MPC 가 그걸 되먹여 "속도 명령 0 인데 오차 부호가 계속 뒤집히는" 증상이 됐다.
#   고침: 예측 지평의 nᵀω(k+j) 에 q_null 을 건다 (plant_wheel.yaml tuning.q_null).
#     ω(k+j) = ω_prev − S_j ΔΩ,  N_j = nᵀ S_{min(j,Nc−1)}   (Nc 이후는 Δω = 0 이라 상수)
#     J += q_null Σ_j (nᵀω_prev − N_j ΔΩ)²  →  E += 2 q_null NᵀN,   F += −2 q_null Nᵀ1 · (nᵀω_prev)
#   null 모드가 폐루프 극 1 − δ₀/c 로 감쇠한다 (q_null 선정 근거는 yaml 주석). 추종 경로는 안 건드린다 —
#   기준 ω_r = T_INV u_r 은 pseudo-inverse 라 null 성분이 원래 0 이다.
#
# 오차 관례 (r − y):  X_e = X_r − X,  ω_e = ω_r − ω,  ω_r = T⁻¹ u_r,  z = ω_r − ω_prev,
#                    ω_cmd = ω_prev − Δω_e(k)
#
#   python3 -m amr_mpc.wheel.mpc    원궤적 폐루프 시뮬레이션 자기검사 (controller/mpc 에서)
# ---------------------------------------------------------------
import numpy as np

from amr_mpc.wheel import model as wm
from amr_mpc.core.observer import VelocityObserver
from amr_mpc.core.qp import qpfgm, qphild, qpactive

_T = wm.CFG["tuning"]


def clip_wheels(w, w_max=None):
    """바퀴별 한계. 드라이버 limit_and_apply_inversion 과 같은 비례 축소 (방향 유지)."""
    w_max = wm.W_MAX if w_max is None else w_max
    m = np.abs(w).max()
    return w * (w_max / m) if m > w_max else w


class WheelMPC:
    def __init__(self, Ts=None, Np=None, Nc=None, q=None, r=None,
                 w_max=None, dw_max=None, qp=True, solver=None, max_iter=None, k_obs=None, q_null=None):
        self.Ts = float(_T["Ts"] if Ts is None else Ts)
        self.Np = int(_T["Np"] if Np is None else Np)
        self.Nc = int(_T["Nc"] if Nc is None else Nc)
        q = (_T["q_pos"], _T["q_pos"], _T["q_yaw"]) if q is None else q
        r = (float(_T["r_dw"]),) * 4 if r is None else r
        self.w_max = wm.W_MAX if w_max is None else float(w_max)
        self.dw_max = (wm.ALPHA_MAX * self.Ts) if dw_max is None else float(dw_max)
        self.qp = qp
        solver = str(_T.get("solver", "active")) if solver is None else solver
        self.solver_name = solver
        self.solver = {"fgm": qpfgm, "hildreth": qphild, "active": qpactive}[solver]
        self.W = []; self.eta_prev = None                     # active-set warm start (working set + 이전 해)
        self.max_iter = int(_T.get("qp_max_iter", 100)) if max_iter is None else int(max_iter)
        self.nu, self.p = 4, 3
        self.a, self.b, self.d = wm.wheel_block(self.Ts)
        # 바퀴 각속도 관측기 (Wang Ch.4). 아래 x_m 이 ω_e 를 1차 차분하므로 측정잡음이 √2 배로
        # 커진 채 ΔΩ 로 나온다 — 구조 A 와 같은 문제. 근거는 core/observer.py 머리주석.
        self.obs = VelocityObserver(self.a, self.b, self.d,
                                    float(_T["k_obs"]) if k_obs is None else k_obs)
        self.n_m = wm.plant(0.0, 0.0, 0.0, self.Ts)[0].shape[0]
        self.n = self.n_m + self.p
        self.Qbar = np.diag(np.tile(q, self.Np))
        self.Rbar = np.diag(np.tile(r, self.Nc))
        # 제약 상수부: S_j 는 Σ_{l≤j} Δω_l 을 뽑는 4×4Nc
        nU = self.nu * self.Nc
        self.S = [np.hstack([np.tile(np.eye(4), j + 1), np.zeros((4, 4 * (self.Nc - j - 1)))]) for j in range(self.Nc)]
        rows, rhs_const, rhs_wprev = [], [], []          # rhs = rhs_const + rhs_wprev @ ω_prev
        for j in range(self.Nc):
            for i in range(4):
                s = self.S[j][i]                          # ω_i(k+j) = ω_prev_i − s·ΔΩ
                rows.append(-s); rhs_const.append(self.w_max); rhs_wprev.append(-np.eye(4)[i])   #  ω ≤ W  → −sΔΩ ≤ W − ω_prev
                rows.append(s);  rhs_const.append(self.w_max); rhs_wprev.append(np.eye(4)[i])    # −ω ≤ W →  sΔΩ ≤ W + ω_prev
        for j in range(self.Nc):
            for i in range(4):
                e = np.zeros(nU); e[4 * j + i] = 1.0
                rows.append(e);  rhs_const.append(self.dw_max); rhs_wprev.append(np.zeros(4))
                rows.append(-e); rhs_const.append(self.dw_max); rhs_wprev.append(np.zeros(4))
        M = np.array(rows); nrm = np.linalg.norm(M, axis=1)
        self.M = M / nrm[:, None]; self.g0 = np.array(rhs_const) / nrm; self.gW = np.array(rhs_wprev) / nrm[:, None]
        # null-space 벌점 (머리주석). N_j = nᵀ S_{min(j,Nc−1)}, Np 행.  E·F 에 더할 상수부를 미리 만든다.
        self.q_null = float(_T.get("q_null", 0.0)) if q_null is None else float(q_null)
        self.n_dir = wm.NULL_DIR / np.linalg.norm(wm.NULL_DIR)
        N = np.array([self.n_dir @ self.S[min(j, self.Nc - 1)] for j in range(self.Np)])
        self._E_null = 2.0 * self.q_null * (N.T @ N)
        self._F_null = -2.0 * self.q_null * (N.T @ np.ones(self.Np))    # × (nᵀ ω_prev) 를 step 에서 곱한다
        self.xm_prev = None; self.lam = None
        self.stats = dict(iters=0, active=0, fallback=0)
        self._init_augment_buffers()

    def reset(self):
        self.xm_prev = None; self.lam = None; self.W = []; self.eta_prev = None
        self.obs.reset()          # 기준열 끊김 뒤 재시작 — 명령 이력이 끊겼으니 추정도 버린다

    def _init_augment_buffers(self):
        """증강 A, B, C 의 상수부를 미리 채운다 (body/mpc.py 와 같은 수법).
        d=1 구조에서 reference 에 의존하는 블록은 A_k (3×3) 와 B_k = B_k3·T (3×4) 뿐이고, 같은 값이
        Δx_m 행과 y 행 두 군데에 들어간다. B, C 는 상수. augment() 는 이 구조의 기준 구현이고 여기는
        그 캐시 — 기동 시 무작위 reference 20개로 일치를 검증한다."""
        if self.d != 1:
            self._Abuf = None
            return
        n, p, nu = self.n_m, self.p, self.nu                 # 11, 3, 4
        A = np.zeros((n + p, n + p)); B = np.zeros((n + p, nu))
        A[3:7, 3:7] = self.a                                 # ω_e(k+1) = a′ ω_e + b′ z
        A[3:7, 7:11] = self.b
        A[n:, n:] = np.eye(p)                                # y(k+1) = y(k) + C_m A_m Δx_m
        B[7:11, :] = np.eye(nu)                              # z(k+1) = ω_e,cmd(k)
        self._Abuf, self._Bbuf = A, B
        self._Cbuf = np.hstack([np.zeros((p, n)), np.eye(p)])
        rng = np.random.default_rng(0)
        for _ in range(20):
            th, vx, vy = rng.uniform(-np.pi, np.pi), *rng.uniform(-1, 1, 2)
            Aa, Ba, Ca = wm.augment(th, vx, vy, self.Ts); Af, Bf, Cf = self._augment_fast(th, vx, vy)
            if not (np.allclose(Aa, Af) and np.allclose(Ba, Bf) and np.allclose(Ca, Cf)):
                raise AssertionError("증강 fast path 가 wheel.model.augment() 와 어긋난다 — plant() 구조를 바꿨으면 여기도 고쳐라")

    def _augment_fast(self, theta_r, vxr, vyr):
        A_k, B_k3 = wm.kinematic_jacobians(theta_r, vxr, vyr, self.Ts)
        B_k = B_k3 @ wm.T_FWD
        A = self._Abuf
        A[0:3, 0:3] = A_k; A[0:3, 3:7] = B_k
        A[self.n_m:, 0:3] = A_k; A[self.n_m:, 3:7] = B_k
        return A.copy(), self._Bbuf, self._Cbuf

    def _F_Phi(self, ref):
        """ref[i] = (θ_r, v_xr, v_yr).  LTV 누적곱.  C = [0 I] 라 C·(·) 는 마지막 p 행 슬라이스."""
        Np, Nc, p, nu, n = self.Np, self.Nc, self.p, self.nu, self.n_m
        F_free = np.zeros((p * Np, self.n)); Phi = np.zeros((p * Np, nu * Nc))
        acc = np.eye(self.n); cols = [np.zeros((self.n, nu)) for _ in range(Nc)]
        for i in range(Np):
            r = ref[min(i, len(ref) - 1)]
            A, B, C = self._augment_fast(*r) if self._Abuf is not None else wm.augment(*r, self.Ts)
            cols = [A @ c for c in cols]
            cols[min(i, Nc - 1)] = cols[min(i, Nc - 1)] + B
            acc = A @ acc
            F_free[p*i:p*i+p, :] = acc[n:, :]                # C @ acc
            for j in range(Nc):
                Phi[p*i:p*i+p, nu*j:nu*j+nu] = cols[j][n:, :]   # C @ cols[j]
        return F_free, Phi

    def step(self, X, w_meas, w_prev, ref, Xr, ur):
        """현재 포즈 X, 측정 바퀴속도 w_meas(4), 직전 바퀴 명령 w_prev(4) → 다음 바퀴 명령 (4).
        ref: [(θ_r, v_xr, v_yr)]×Np, Xr: 현재 기준 포즈, ur: 현재 기준 몸체속도."""
        w_r = wm.T_INV @ np.asarray(ur, float)
        Xe = np.asarray(Xr, float) - np.asarray(X, float); Xe[2] = np.arctan2(np.sin(Xe[2]), np.cos(Xe[2]))
        # 측정 w_meas 를 그대로 쓰면 아래 Δx_m 이 그걸 차분해 잡음을 √2 배로 키운다.
        we = w_r - self.obs.update(w_meas)
        xm = np.concatenate([Xe, we, w_r - np.asarray(w_prev, float)]) if self.d == 1 else np.concatenate([Xe, we])
        if self.xm_prev is None:
            self.xm_prev = xm.copy()
        x = np.concatenate([xm - self.xm_prev, Xe]); self.xm_prev = xm
        F_free, Phi = self._F_Phi(ref)
        E = 2 * (Phi.T @ self.Qbar @ Phi + self.Rbar) + self._E_null
        F = 2 * (Phi.T @ self.Qbar @ F_free @ x) + self._F_null * (self.n_dir @ np.asarray(w_prev, float))
        if not self.qp:
            dw = np.linalg.solve(E, -F)[:4]
            w_c = clip_wheels(np.asarray(w_prev, float) - dw, self.w_max)
            self.obs.push(w_c)                               # 관측기 예측에 쓸 실제 인가 명령
            return w_c
        gamma = self.g0 + self.gW @ np.asarray(w_prev, float)
        if self.solver_name == "active":
            eta, it, self.lam, self.W = qpactive(E, F, self.M, gamma, max_iter=self.max_iter, W0=self.W, eta0=self.eta_prev)
            self.eta_prev = eta
        else:
            eta, it, self.lam = self.solver(E, F, self.M, gamma, max_iter=self.max_iter, viol_tol=1e-6, lam0=self.lam)
        w = np.asarray(w_prev, float) - eta[:4]
        w_c = clip_wheels(w, self.w_max)
        self.stats = dict(iters=it, active=int((self.lam > 0).sum()), fallback=int(not np.allclose(w, w_c)))
        self.obs.push(w_c)                                   # 관측기 예측에 쓸 실제 인가 명령
        return w_c


# ---------- 자기검사: 원궤적 폐루프 (바퀴 플랜트) ----------
def demo(verbose=True, speed=0.5, radius=1.0, T=16.0):
    f_kin = wm.f
    Ts = float(_T["Ts"]); omega = speed / radius
    m = WheelMPC(); a, b, d = wm.wheel_block(Ts)

    def reference(t):
        th = omega * t
        return (np.array([radius*np.sin(th), radius*(1-np.cos(th)), th]), np.array([speed, 0.0, omega]))

    X = np.array([0.0, -0.15, 0.0]); w = np.zeros(4); w_prev = np.zeros(4); hist = [np.zeros(4)]
    errs, wmax, its, fb = [], [], [], []
    for k in range(int(T / Ts)):
        t = k * Ts; Xr, ur = reference(t)
        ref = [(reference(t + i*Ts)[0][2], *reference(t + i*Ts)[1][:2]) for i in range(m.Np)]
        wc = m.step(X, w, w_prev, ref, Xr, ur); hist.append(wc); w_prev = wc
        w = a @ w + b @ hist[max(0, len(hist) - 1 - d)]            # 바퀴별 FOPDT
        v = wm.T_FWD @ w                                             # 순기구학
        X = X + Ts * f_kin(X, v); X[2] = np.arctan2(np.sin(X[2]), np.cos(X[2]))
        errs.append(np.linalg.norm((X - reference(t + Ts)[0])[:2])); wmax.append(np.abs(wc).max())
        its.append(m.stats["iters"]); fb.append(m.stats["fallback"])
    errs = np.array(errs); n0 = int(4.0 / Ts)
    ok = errs[n0:].max() < 0.02 and max(wmax) <= wm.W_MAX + 1e-9
    if verbose:
        print(f"원궤적 R={radius} m, v={speed} m/s, {T:.0f} s, 초기오차 15 cm  (바퀴 입력 MPC, 상태 {m.n_m}+3)")
        print(f"  수렴 후 위치오차   RMS {np.sqrt(np.mean(errs[n0:]**2))*100:.2f} cm   최대 {errs[n0:].max()*100:.2f} cm")
        print(f"  1 cm 진입          {np.argmax(errs < 0.01)*Ts:.2f} s")
        print(f"  바퀴속도 최대      {max(wmax):.2f} / {wm.W_MAX:.0f} rad/s  ({max(wmax)/wm.W_MAX*100:.0f} %)")
        print(f"  QP 반복 최대 {max(its)}  폴백 {sum(fb)} 스텝")
        print(f"\n  {'OK' if ok else '✗ 발산 또는 제약 위반'}")
    return ok, errs


if __name__ == "__main__":
    raise SystemExit(0 if demo()[0] else 1)
