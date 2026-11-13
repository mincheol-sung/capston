#!/usr/bin/env python3
# WP4 — 궤적추종 MPC.  Wang, "MPC ... Using MATLAB" Ch.1.3–1.4 형식
# ---------------------------------------------------------------
# Wang Ch.1.3 예측:      Y = F_free·x(k_i) + Φ·ΔU        (교재의 예측 F. QP 의 F 와 글자 충돌이라 F_free)
# Wang Ch.1.4 비용/해:   J = (R_s − Y)ᵀ Q̄ (R_s − Y) + ΔUᵀ R̄ ΔU
# Wang Ch.2 QP 표준형:   J = ½ ΔUᵀ E ΔU + ΔUᵀ F + const
#                        E = 2(ΦᵀQ̄Φ + R̄),   F = −2 ΦᵀQ̄ (R_s − F_free·x)
#                        무제약 해  ΔU = −E⁻¹F   (2 는 약분되지만 Ex 2.14 등 교재 숫자와 대조하려고 그대로 둔다)
#
# 우리는 오차좌표계라 목표가 0 이다  ->  R_s = 0,  F = 2 ΦᵀQ̄ F_free x.
# 글자 규약: F_free(예측) / E, F(QP primal) / M, gamma(제약) / H, K(Hildreth dual — 아직 미구현).
#            ΦᵀQ̄Φ + R̄ 를 H 라 부르지 않는다 — H 는 dual Hessian 의 자리.
#
# ★ Wang Ch.1 은 LTI 전제라 F, Φ 를 A 의 거듭제곱으로 만든다. 우리는 A 가 기준궤적을
#   따라 매 스텝 바뀌는 LTV 이므로 거듭제곱 대신 루프로 누적한다. 구조는 동일하다.
#
# Wang Ch.2 제약 (qphild.py 의 Hildreth 로 풂).  결정변수 η = [ΔU (3Nc) ; ε (3)]
#   입력 진폭  |v_x|+|v_y|+k|ω| ≤ V_LIMIT     u(k+j) = u_prev − Σ_{i≤j} Δu_i  →  L1 다면체 8면 × Nc 행
#   입력 변화  |Δu_k| ≤ du_max_k              옵션. 축별 (스칼라 또는 [vx,vy,wz]), 6 × Nc 행.
#                                             du_max = a_max·Ts 로 주면 명령 가속도 제한이다 (실제 가속도는 τ 로 한 번 더 완만).
#   출력 연성  |y(k+i)| ≤ y_max + ε_i, ε_i ≥ 0   옵션.  초기 오차가 한계 밖이면 비가능이라 스텝별 슬랙 ε_i 를
#                                             비용에 ρ·Σε_i² 로 넣는다 (Maciejowski 3.4).  슬랙을 지평 전체가
#                                             공유하면 Φ 행이 0 인 앞 스텝(지연) 때문에 ε 이 커져 제약이 무력해진다.
#                                             앞 3 스텝(Δu 가 y 에 못 닿음)은 제외, out_every 로 행 솎음.
#   솔버: solver='fgm' (Nesterov 가속 투영경사, 기본) 또는 'hildreth' (교재). 출력 제약이 붙으면 Hildreth 는
#   1000 회에도 못 맞춘다 (dual 조건수 1e4). 어느 쪽이든 상한(plant.yaml tuning.qp_max_iter)에 걸리면
#   그때까지의 λ 로 반환 → 마지막에 clip_twist 로 첫 스텝 가능성 보장, stats['fallback'] 에 기록.
#   상한은 주기를 지키는 쪽으로 낮게 잡았다 — 포화 영역에서 반복을 늘려도 추종이 안 나아진다.
#   qp=False 면 1장 무제약 해 + 클립 (이전 동작).
#
#   python3 -m amr_mpc.body.mpc     원궤적 폐루프 시뮬레이션 자기검사   (ROS 배선은 nodes/body_tracking.py)
# ---------------------------------------------------------------
import itertools
import numpy as np
from amr_mpc.core import config as params
from amr_mpc.body.model import augment, kinematic_jacobians, plant, velocity_block, f
from amr_mpc.core.observer import VelocityObserver
from amr_mpc.core.qp import qphild, qpfgm

# ── 기하·제약: config/plant.yaml 이 유일한 출처 (geometry, limits 절) ──
_G, _L = params.section("geometry"), params.section("limits")
R_WHEEL = float(_G["wheel_radius_m"])
W_MAX   = float(_L["max_wheel_rad_per_sec"])   # base.yaml 과 같게. set_wheel_limit() 로 런타임 동기
K_GEOM  = (float(_G["wheel_base_m"]) + float(_G["robot_width_m"])) / 2.0
V_LIMIT = R_WHEEL * W_MAX                      # |vx|+|vy|+K_GEOM·|w| <= V_LIMIT


# ── 튜닝·안전: config/plant.yaml 의 tuning, safety 절 ──
# 노드·demo·자기시험이 전부 이 값을 참조하므로 "라이브러리 기본값과 실제로 뜨는 값이 다른"
# 상황이 안 생긴다. 2026-09-03 이전에는 갈라져 있었다 — MPC 기본 r=(1,1,0.3) 인데 노드는
# (1,1,1) 을 넘기고, du_max 는 mpc_tracking_node 만 걸어서 자기시험은 무제약으로 돌았다.
_T, _S = params.section("tuning"), params.section("safety")
DEFAULT_TS    = float(_T["Ts"])
DEFAULT_NP    = int(_T["Np"])
DEFAULT_NC    = int(_T["Nc"])
DEFAULT_Q     = (float(_T["q_pos"]), float(_T["q_pos"]), float(_T["q_yaw"]))
DEFAULT_R     = (float(_T["r_du"]),) * 3
DEFAULT_K_OBS = float(_T["k_obs"])             # 속도 관측기 gain. 1.0 이면 측정 그대로 (구 동작)
DEFAULT_QP_MAX_ITER = int(_T["qp_max_iter"])   # 상한 도달 시 폴백. plant.yaml 주석에 근거
DEFAULT_OUT_EVERY   = int(_T["out_every"])    # 출력 제약을 몇 스텝마다 걸까
DEFAULT_RHO         = float(_T["slack_rho"])  # 슬랙 벌점 ρ (비용에 ρ·Σε²)
DEFAULT_Y_MAX       = np.array([float(v) for v in _L["max_pose_error"]])   # 포즈오차 연성 한계
DEFAULT_ACCEL = tuple(float(v) for v in _L["max_cmd_accel"])
LOAD_BUDGET   = float(_L["load_budget"])
ODOM_TIMEOUT  = float(_S["odom_timeout"])
REF_TIMEOUT   = float(_S["ref_timeout"])

_ACCEL_DEFAULT = object()              # du_max=None(제약 없음) 과 "기본값 쓰기" 를 구분하는 표식


def du_max_from_accel(accel, Ts):
    """명령 가속도 한계를 |Δu| 한계로. 0 이하인 축은 제약 없음(inf)."""
    a = np.asarray(accel, float)
    return np.where(a > 0.0, a * Ts, np.inf)


def set_wheel_limit(w_max):
    """드라이버 한계를 런타임에 맞춘다 (mpc_node 파라미터). clip_twist·MPC 기본 u_l1 이 따라간다."""
    global W_MAX, V_LIMIT
    W_MAX = float(w_max); V_LIMIT = R_WHEEL * W_MAX


def clip_twist(u):
    """메카넘 입력 제약. 방향은 유지하고 크기만 줄인다 (드라이버와 같은 방식)."""
    load = abs(u[0]) + abs(u[1]) + K_GEOM * abs(u[2])
    return u * (V_LIMIT / load) if load > V_LIMIT else u


class MPC:
    def __init__(self, Ts=DEFAULT_TS, Np=DEFAULT_NP, Nc=DEFAULT_NC,
                 q=DEFAULT_Q, r=DEFAULT_R,
                 qp=True, u_l1=None, du_max=_ACCEL_DEFAULT, y_max=_ACCEL_DEFAULT,
                 rho=None, out_every=None, solver="fgm", max_iter=None, k_obs=None):
        # du_max, y_max: 생략하면 plant.yaml 값.  명시적으로 None 을 넘기면 그 제약 없음.
        if du_max is _ACCEL_DEFAULT:
            du_max = du_max_from_accel(DEFAULT_ACCEL, Ts)
        if y_max is _ACCEL_DEFAULT:
            y_max = DEFAULT_Y_MAX
        if rho is None:
            rho = DEFAULT_RHO
        if out_every is None:
            out_every = DEFAULT_OUT_EVERY
        self.Ts, self.Np, self.Nc = Ts, Np, Nc
        self.qp, self.u_l1, self.du_max, self.y_max = qp, (V_LIMIT if u_l1 is None else u_l1), du_max, y_max
        self.rho, self.out_every = rho, out_every
        self.solver = qpfgm if solver == "fgm" else qphild
        self.max_iter = DEFAULT_QP_MAX_ITER if max_iter is None else max_iter
        self.lam = None                                      # Hildreth 워밍스타트
        self.stats = dict(iters=0, active=0, fallback=0)      # 진단용
        # 출력 제약을 거는 예측 스텝.  시작 인덱스는 Δu → y 의 **상대차수(relative degree)** 로 정한다.
        #
        #   Φ 의 (i, j) 블록 = ∂y(k+i+1)/∂Δu(k+j) = C·A^(i−j)·B   (j ≤ i, 아니면 0)
        #   따라서 j=0 열의 첫 비영 블록이 상대차수를 준다. 증강 구조로 직접 전개하면:
        #     C = [0  I],   B = [B_m ; C_m B_m],   A = [[A_m, 0], [C_m A_m, I]]
        #     C·B     = C_m B_m               = [I 0 0]·[0;0;I]       = 0        (i=0 행 0)
        #     C·A·B   = C_m A_m B_m           = [I 0 0]·[0;b;0]       = 0        (i=1 행 0)
        #     C·A²·B  = C_m A_m² B_m          = [I 0 0]·[B_k b; a b; 0] = B_k·b ≠ 0   (i=2 첫 비영)
        #   즉 y(k+3) 이 Δu(k) 의 영향을 받는 첫 출력이고 상대차수는 d+2 다. 물리적 분해:
        #     +1  지연 상태 z(k+1) = u_e(k)          ← d=1 (θ=40 ms)
        #     +1  z → v_e (FOPDT 이산화의 b)
        #     +1  v_e → X_e (기구학 적분의 B_k)
        #   d≠1 이면 지연 단이 그만큼 늘어나 상대차수도 d+2 로 따라간다.
        #
        # 첫 d+1(=2) 행은 Φ 가 **정확히 0** 이므로 제약을 걸면 결정변수가 없는 순수 상태검사가 되어
        # 어떤 ΔU 로도 못 고친다 (슬랙만 커진다). 그래서 반드시 제외한다.
        # 첫 유효행 i=d+1=2 는 걸 수 있으나 ‖Φ 행‖ 이 지평 끝의 1/250 (2.4e−2 vs 6.0) 이라
        # 조작 여력이 사실상 없어 역시 슬랙만 키운다 — 실용적 판단으로 상대차수 d+2 부터 건다.
        start = velocity_block(Ts)[2] + 2       # self.d 는 아래에서 설정된다. velocity_block 은 메모이즈
        self.out_steps = list(range(start, Np, out_every)) if y_max is not None else []
        self.out_axes = [k for k in range(3) if y_max is not None and np.isfinite(y_max[k])]  # 유한 한계인 축만
        self.n_eps = len(self.out_steps) * len(self.out_axes)  # 스텝별·축별 슬랙 수
        # 제약 조립에 쓰는 상수 행렬
        self.W = np.diag([1.0, 1.0, K_GEOM])
        self.SIGNS = np.array(list(itertools.product([1, -1], repeat=3)), float)   # L1 볼의 8 면
        self.S = [np.hstack([np.tile(np.eye(3), j + 1), np.zeros((3, 3 * (Nc - j - 1)))])
                  for j in range(Nc)]                        # Σ_{i≤j} Δu_i 를 뽑는 3×3Nc
        self.a, self.b, self.d = velocity_block(Ts)
        # 속도 관측기 (Wang Ch.4). step() 에 들어오는 v 는 측정이고, Δx_m 이 그걸 1차 차분하므로
        # 잡음이 √2 배로 커진 채 ΔU 로 나온다. 예측기와 같은 (a, b, d) 를 넘겨 모델을 공유한다.
        self.obs = VelocityObserver(self.a, self.b, self.d,
                                    DEFAULT_K_OBS if k_obs is None else k_obs)
        self.n_m = plant(0.0, 0.0, 0.0, Ts)[0].shape[0]      # 플랜트 차원
        self._init_augment_buffers()
        self._init_constraint_buffers()
        self.Qbar = np.diag(np.tile(q, Np))                  # 출력(포즈오차) 가중
        self.Rbar = np.diag(np.tile(r, Nc))                  # Δu 가중
        self.xm_prev = None                                  # Δx_m 계산용

    def _init_augment_buffers(self):
        """증강 A, B, C 의 상수부를 미리 채운다.

        d=1 구조에서 기준궤적에 의존하는 것은 A 의 A_k, B_k 두 블록뿐이고 (그것도 두 군데에
        같은 값이 들어간다) B, C 는 아예 상수다. 그런데 augment() 는 매 호출마다
        velocity_block(Ts) 를 다시 풀고 np.block 으로 12×12 를 새로 조립한다 —
        지평 하나당 Np=20 회니 낭비가 크다 (측정: _F_Phi 1.10 ms 중 0.82 ms).

        구조를 여기 한 번 더 적는 대가로, __init__ 에서 augment() 와 일치하는지 검증한다.
        model.py 가 읽는 쪽 기준 구현이고 이건 그걸 캐싱한 것 — 어긋나면 기동 시 즉시 죽는다."""
        if self.d != 1:
            self._Abuf = None                                # d≠1 이면 fast path 없이 augment() 를 쓴다
            return
        n, p = self.n_m, 3
        A = np.zeros((n + p, n + p)); B = np.zeros((n + p, p))
        A[3:6, 3:6] = self.a                                 # v_e(k+1) = a v_e + b z
        A[3:6, 6:9] = self.b
        A[n:, n:] = np.eye(p)                                # y(k+1) = y(k) + C_m A_m Δx_m
        B[6:9, :] = np.eye(p)                                # z(k+1) = u_e(k)
        self._Abuf, self._Bbuf = A, B
        self._Cbuf = np.hstack([np.zeros((p, n)), np.eye(p)])
        # 검증: 무작위 기준궤적에서 augment() 와 같아야 한다
        rng = np.random.default_rng(0)
        for _ in range(20):
            th, vx, vy = rng.uniform(-np.pi, np.pi), *rng.uniform(-1, 1, 2)
            Aa, Ba, Ca = augment(th, vx, vy, self.Ts)
            Af, Bf, Cf = self._augment_fast(th, vx, vy)
            if not (np.allclose(Aa, Af) and np.allclose(Ba, Bf) and np.allclose(Ca, Cf)):
                raise AssertionError("증강 fast path 가 model.augment() 와 어긋난다 — "
                                     "plant() 구조가 바뀌었으면 _init_augment_buffers 도 고쳐라")

    def _augment_fast(self, theta_r, vxr, vyr):
        """augment() 와 같은 (A, B, C). 바뀌는 두 블록만 덮어쓴다. A 는 버퍼 재사용이 아니라
        복사해 돌려준다 — _F_Phi 가 A 를 누적곱에 쓰므로 별개 객체여야 한다."""
        A_k, B_k = kinematic_jacobians(theta_r, vxr, vyr, self.Ts)
        A = self._Abuf
        A[0:3, 0:3] = A_k; A[0:3, 3:6] = B_k                 # Δx_m 의 기구학 행
        A[self.n_m:, 0:3] = A_k; A[self.n_m:, 3:6] = B_k     # y 행 = C_m A_m
        return A.copy(), self._Bbuf, self._Cbuf

    def _F_Phi(self, ref):
        """Wang Ch.1.3 의 F_free, Φ. LTV 라 거듭제곱 대신 누적한다.
        ref[i] = (theta_r, vxr, vyr) — 기준궤적이라 지평 전체를 미리 안다."""
        Np, Nc = self.Np, self.Nc
        p = 3                                                 # 출력 차원
        F_free = np.zeros((p * Np, self.n_m + p))
        Phi = np.zeros((p * Np, 3 * Nc))
        acc = np.eye(self.n_m + p)                            # A_{i-1}···A_0
        cols = [np.zeros((self.n_m + p, 3)) for _ in range(Nc)]
        for i in range(Np):
            r = ref[min(i, len(ref) - 1)]
            A, B, C = self._augment_fast(*r) if self._Abuf is not None else augment(*r, self.Ts)
            cols = [A @ c for c in cols]
            # Nc 이후: 마지막 Δu 를 계속 재적용한다 (u 가 램프한다).
            # ★ Wang Ch.1.3 은 Nc 이후 Δu=0 으로 두어 u 를 유지한다 — 이건 의도적 이탈이다.
            #   대가: Φ 의 마지막 열이 부풀려진다 (열 노름 j=0..4 → 2.73/2.48/2.25/2.02/10.09).
            #         Nc 이후 예측 입력에는 L1 제약이 안 걸리므로(self.S 가 Nc 개) 그 구간의
            #         예측 입력은 제약 밖으로 나갈 수 있다. 실행되는 것은 Δu(k) 뿐이라 안전하지만
            #         최적화기가 보는 미래는 실제로 안 할 램프다.
            #   이득: 마지막 증분이 비싸 보여 최적화기가 아껴 쓴다 → 명령이 부드러워진다.
            #         폐루프 시뮬(R=1 m, 0.25 m/s, 초기오차 15 cm) 동일 조건 비교:
            #           repeat(현재)  Δu 부호반전 30.2 %,  QP 반복 60
            #           wang(교재)    Δu 부호반전 40.9 %,  QP 반복 70   ← 추종·부하는 동일
            #         떨림이 4 할 늘어나므로 교재 관례로 되돌리지 않는다.
            #   되돌리려면 이 줄을 `if i < Nc: cols[i] = cols[i] + B` 로 바꾸면 된다.
            cols[min(i, Nc - 1)] = cols[min(i, Nc - 1)] + B
            acc = A @ acc
            F_free[p*i:p*i+p, :] = C @ acc
            for j in range(Nc):
                Phi[p*i:p*i+p, 3*j:3*j+3] = C @ cols[j]
        return F_free, Phi

    def reset(self):
        self.xm_prev = None
        self.lam = None
        self.obs.reset()          # 기준열 끊김 뒤 재시작 — 명령 이력이 끊겼으니 추정도 버린다

    def _init_constraint_buffers(self):
        """M η ≤ γ 중 상수부를 미리 채운다.

        100 행 중 80 행이 상수다 — L1 의 계수 −(sᵀW S_j) (rhs 만 u_prev 의존), |Δu| 행 전부
        (계수·rhs 둘 다), 그리고 ε ≥ 0 행. 매 틱 바뀌는 것은 ±Φ 행 2·n_eps 개뿐인데
        (Φ 가 기준궤적 의존) 기존 구현은 100 행을 파이썬 리스트로 새로 쌓았다.
        행 순서는 예전과 동일하게 유지한다 — 워밍스타트 λ 가 행 순서에 묶여 있다."""
        nU, ne = 3 * self.Nc, self.n_eps
        n = nU + ne
        M0, g0 = [], []
        l1_c = []
        if self.u_l1 is not None:                            # L1: 계수 상수, rhs = u_l1 − c·u_prev
            for S in self.S:
                for sgn in self.SIGNS:
                    c = sgn @ self.W
                    M0.append(np.concatenate([-(c @ S), np.zeros(ne)])); g0.append(0.0); l1_c.append(c)
        self._l1_c = np.array(l1_c) if l1_c else np.zeros((0, 3))
        self._n_l1 = len(l1_c)
        if self.du_max is not None:                          # |Δu|: 전부 상수
            dmax = np.broadcast_to(np.asarray(self.du_max, float), (3,))
            for j in range(self.Nc):
                for k in range(3):
                    if not np.isfinite(dmax[k]):
                        continue
                    e = np.zeros(n); e[3 * j + k] = 1
                    M0.append(e.copy()); g0.append(dmax[k])
                    M0.append(-e);       g0.append(dmax[k])
        self._n_fixed = len(M0)
        idx = []                                             # 출력 행: (+Φ, −Φ, −ε) 3 개씩 끼워 넣는다
        if self.y_max is not None:
            for nn, i in enumerate(self.out_steps):
                for a, k in enumerate(self.out_axes):
                    col = nU + len(self.out_axes) * nn + a
                    r = np.zeros(n); r[col] = -1
                    M0.append(r.copy()); g0.append(0.0)      # +Φ 행 (계수는 매 틱)
                    M0.append(r.copy()); g0.append(0.0)      # −Φ 행 (계수는 매 틱)
                    r2 = np.zeros(n); r2[col] = -1
                    M0.append(r2);      g0.append(0.0)       # −ε ≤ 0 (상수)
                    idx.append((3 * i + k, k))
        self._M0 = np.array(M0) if M0 else np.zeros((0, n))
        self._g0 = np.array(g0) if g0 else np.zeros(0)
        self._out_idx = np.array([a for a, _ in idx], int)    # Φ / fx 에서 뽑을 행
        self._out_ax  = np.array([b for _, b in idx], int)    # y_max 축
        self._nU = nU

    def _constraints(self, u_prev, F_free, Phi, x):
        """Wang Ch.2 의 M η ≤ γ.  η = [ΔU ; ε].  반환 (M, γ) — 행은 단위 노름으로 정규화 (Hildreth 조건수).
        상수부는 _init_constraint_buffers 가 채워뒀고 여기서는 바뀌는 부분만 덮어쓴다."""
        nU = self._nU
        M = self._M0.copy(); gamma = self._g0.copy()
        if self._n_l1:                                       # L1 rhs 만 u_prev 의존 (계수는 상수)
            gamma[:self._n_l1] = self.u_l1 - self._l1_c @ u_prev
        if self.n_eps:                                       # 출력 행: ±Φ 계수와 rhs 를 덮어쓴다
            base = self._n_fixed
            P = Phi[self._out_idx]                           # (n_eps, nU)
            fx = (F_free @ x)[self._out_idx]
            ym = np.asarray(self.y_max, float)[self._out_ax]
            M[base + 0::3, :nU] = P
            M[base + 1::3, :nU] = -P
            gamma[base + 0::3] = ym - fx
            gamma[base + 1::3] = ym + fx
        nrm = np.linalg.norm(M, axis=1); return M / nrm[:, None], gamma / nrm

    def step(self, X, v, u_prev, ref, Xr, ur):
        """현재 포즈 X, 몸체속도 v, 직전 명령 u_prev -> 다음 명령 u.

        오차 관례는 고전 제어 (Wang 의 R_s − Y 와 같은 방향):
            X_e = X_r − X,   v_e = v_r − v,   u_e = u_r − u      (v_r ≈ u_r 로 근사)
        오차 동역학이 선형·동차라 model.py 의 A, B 는 관례와 무관하다.
        부호는 오차계로 들어가는 여기와 나가는 return 에서만 나타난다."""
        Xe = Xr - X
        Xe[2] = np.arctan2(np.sin(Xe[2]), np.cos(Xe[2]))      # 각도 랩
        # 측정 v 를 그대로 쓰면 아래 Δx_m 이 그걸 차분해 잡음을 √2 배로 키운다 (core/observer.py).
        ve = ur - self.obs.update(v)
        # 오차좌표 플랜트 상태 x_m.  d=1 이면 지연 상태 z = u_e(k−1) 이 붙는다.
        xm = np.concatenate([Xe, ve, ur - u_prev]) if self.d == 1 \
             else np.concatenate([Xe, ve])
        if self.xm_prev is None:
            self.xm_prev = xm.copy()                          # 첫 스텝은 Δx_m = 0
        x = np.concatenate([xm - self.xm_prev, Xe])           # Wang 의 x = [Δx_m ; y]
        self.xm_prev = xm

        F_free, Phi = self._F_Phi(ref)
        E = 2 * (Phi.T @ self.Qbar @ Phi + self.Rbar)         # Wang QP 의 E (primal Hessian)
        F = 2 * (Phi.T @ self.Qbar @ F_free @ x)              # Wang QP 의 F (R_s = 0)
        if not self.qp:
            du = np.linalg.solve(E, -F)[:3]                   # 1장: ΔU = −E⁻¹F, 첫 이동만
            u_c = clip_twist(u_prev - du)                     # u = u_r − u_e 이므로 −Δu_e
            self.obs.push(u_c)                                # 관측기 예측에 쓸 실제 인가 명령
            return u_c
        # 2장: 슬랙 ε 을 붙인 η = [ΔU ; ε],  E_η = blkdiag(E, 2ρI),  F_η = [F ; 0]
        nU, ne = 3 * self.Nc, self.n_eps
        E_eta = np.zeros((nU + ne, nU + ne)); E_eta[:nU, :nU] = E; E_eta[nU:, nU:] = 2 * self.rho * np.eye(ne)
        F_eta = np.concatenate([F, np.zeros(ne)])
        M, gamma = self._constraints(u_prev, F_free, Phi, x)
        eta, it, self.lam = self.solver(E_eta, F_eta, M, gamma, max_iter=self.max_iter, viol_tol=1e-6, lam0=self.lam)
        u = u_prev - eta[:3]
        u_c = clip_twist(u)                                  # 폴백: 상한에 걸려 위반이 남아도 첫 스텝은 가능하게
        self.stats = dict(iters=it, active=int((self.lam > 0).sum()), fallback=int(not np.allclose(u, u_c)))
        self.obs.push(u_c)                                   # 관측기 예측에 쓸 실제 인가 명령
        return u_c


# ---------- 자기검사: 원궤적 폐루프 시뮬레이션 ----------

def demo(verbose=True):
    Ts, T = 0.04, 20.0
    radius, speed = 1.0, 0.15
    omega = speed / radius
    mpc = MPC(Ts=Ts)
    a, b, d = velocity_block(Ts)

    def reference(t):
        th = omega * t
        return (np.array([radius*np.sin(th), radius*(1-np.cos(th)), th]),
                np.array([speed, 0.0, omega]))

    X = np.array([0.0, -0.15, 0.0])          # 일부러 15cm 벗어난 곳에서 시작
    v = np.zeros(3); u_prev = np.zeros(3); u_hist = [np.zeros(3)]
    errs, loads = [], []
    for k in range(int(T / Ts)):
        t = k * Ts
        Xr, ur = reference(t)
        ref = [(reference(t + i*Ts)[0][2], *reference(t + i*Ts)[1][:2])
               for i in range(mpc.Np)]
        u = mpc.step(X, v, u_prev, ref, Xr, ur)
        u_hist.append(u); u_prev = u
        # 플랜트: 식별한 속도루프 + 정확한 기구학
        v = a @ v + b @ u_hist[max(0, len(u_hist)-1-d)]
        X = X + Ts * f(X, v)
        X[2] = np.arctan2(np.sin(X[2]), np.cos(X[2]))
        errs.append(np.linalg.norm((X - reference(t + Ts)[0])[:2]))   # X 는 이제 t+Ts 시점
        loads.append(abs(u[0]) + abs(u[1]) + K_GEOM*abs(u[2]))

    errs = np.array(errs); n0 = int(4.0 / Ts)
    ok = errs[n0:].max() < 0.02 and max(loads) <= V_LIMIT + 1e-9
    if verbose:
        print(f"원궤적 R={radius}m, v={speed}m/s, {T:.0f}s, 초기오차 15cm\n")
        print(f"  수렴 후 위치오차   RMS {errs[n0:].mean()*100:.2f} cm"
              f"   최대 {errs[n0:].max()*100:.2f} cm")
        print(f"  초기오차 수렴시간  {np.argmax(errs < 0.01)*Ts:.2f} s  (1cm 이하 진입)")
        print(f"  입력 부하 최대     {max(loads):.3f} / {V_LIMIT:.2f}"
              f"  ({max(loads)/V_LIMIT*100:.0f}%)")
        print(f"\n  {'OK — 수렴하고 제약을 지킨다' if ok else '✗ 발산하거나 제약 위반'}")
    return ok, errs, np.array(loads)


if __name__ == "__main__":
    raise SystemExit(0 if demo()[0] else 1)
