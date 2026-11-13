#!/usr/bin/env python3
# 구조 B — 바퀴 각속도 입력 모델.  body/model.py(몸체속도 입력)와 병행한다.
# ---------------------------------------------------------------
# Upper controller (MPC) 가 kinematics T 를 소유하고, Lower (RoboClaw wheel PI + 모터) 는 바퀴별 FOPDT.
#
#   kinematics   Ẋ = R(θ) v,     v = T ω          T: 3×4 forward kinematics (T⁻¹ 의 pseudo-inverse)
#   actuator     τ_i ω̇_i + ω_i = K_i ω_cmd,i(t − θ_d)      i = FL, FR, RL, RR
#   dead time    z = ω_e(k−1)                       (lower loop 통신·PI 지연을 상태로 실현. 토크 단계에서 제거 예정)
#
# 오차좌표 (r − y 관례):  X_e = X_r − X,  ω_e = ω_r − ω,  ω_r = T⁻¹ u_r,  ω_e,cmd = ω_r − ω_cmd
#   x_m = [X_e (3); ω_e (4); z (4)] = 11,   입력 ω_e,cmd (4),   출력 X_e (3)
#   A_m = [[A_k, B_k T, 0], [0, a', b'], [0, 0, 0]],   B_m = [0; 0; I₄],   C_m = [I₃ 0 0]
#   Wang 증강 x = [Δx_m; X_e] = 14
#
# 몸체 단위 모델과의 관계: ω_e = T⁻¹ v_e 로 두면 B_k T T⁻¹ = B_k 라 kinematics 블록이 일치한다.
#   차이는 (1) 제약이 바퀴별 box, (2) 4 − 3 = 1 개의 내부 자유도 (네 바퀴가 서로 상쇄하는 조합) 가
#   출력에 안 보이는 모드로 생긴다. 플랜트 상태 ω_e 의 null 성분은 a' 로 감쇠하니 x_m 자체는 detectable 이
#   맞지만, incremental form 의 입력 적분기 ω_cmd = ω_prev − Δω 는 x_m 밖에 있고 null 방향으로 극 1 이다 —
#   제약이 한 번 심어 놓은 nᵀω_prev 를 아무 항도 지우지 않는다 (2026-09-10 실차 hold 에서 확인).
#   그래서 wheel/mpc.py 가 nᵀω 에 q_null 을 따로 건다 (plant_wheel.yaml tuning.q_null).
#
#   python3 -m amr_mpc.wheel.model   자기검사 (controller/mpc 에서)
# ---------------------------------------------------------------
import numpy as np

from amr_mpc.core import config
from amr_mpc.core.kinematics import WHEELS, f, kinematic_jacobians, mecanum_matrices   # noqa: F401

CFG = config.load("plant_wheel.yaml")
_M, _G, _L, _T = CFG["model"], CFG["geometry"], CFG["limits"], CFG["tuning"]
TAU = np.array([float(_M["tau"][w]) for w in WHEELS])
K_DC = np.array([float(_M["k_dc"][w]) for w in WHEELS])
THETA = float(_M["theta"])
R_W = float(_G["wheel_radius_m"])
W_MAX = float(_L["max_wheel_rad_per_sec"])
ALPHA_MAX = float(_L["max_wheel_accel"])
T_INV, T_FWD, NULL_DIR, K_GEOM = mecanum_matrices(R_W, float(_G["wheel_base_m"]), float(_G["robot_width_m"]))


def wheel_block(Ts):
    """바퀴별 FOPDT 의 ZOH 이산화. (a′ 4×4 diag, b′ 4×4 diag, d)"""
    a = np.exp(-Ts / TAU)
    b = K_DC * (1.0 - a)
    d = int(round(THETA / Ts))
    if abs(THETA / Ts - d) > 1e-9:
        raise ValueError(f"theta/Ts = {THETA/Ts:.3f} 가 정수가 아니다 (Ts 를 theta 의 약수로)")
    return np.diag(a), np.diag(b), d


def plant(theta_r, vxr, vyr, Ts):
    """Wang 의 (A_m, B_m, C_m) — 바퀴 입력판. x_m = [X_e; ω_e; z] (11), 입력 ω_e,cmd (4)."""
    a, b, d = wheel_block(Ts)
    A_k, B_k3 = kinematic_jacobians(theta_r, vxr, vyr, Ts)     # 3×3, 3×3 (몸체속도 → 포즈)
    B_k = B_k3 @ T_FWD                                          # 3×4 (바퀴속도 → 포즈)
    Z34, Z43, Z44, Z33 = np.zeros((3, 4)), np.zeros((4, 3)), np.zeros((4, 4)), np.zeros((3, 3))
    I3, I4 = np.eye(3), np.eye(4)
    if d == 0:
        A_m = np.block([[A_k, B_k], [Z43, a]]); B_m = np.vstack([Z34, b]); C_m = np.hstack([I3, Z34])
    elif d == 1:
        A_m = np.block([[A_k, B_k, Z34],
                        [Z43, a,   b],
                        [Z43, Z44, Z44]])
        B_m = np.vstack([Z34, Z44, I4])
        C_m = np.hstack([I3, Z34, Z34])
    else:
        raise ValueError(f"d={d}: 지연 상태를 {d}단으로 늘려야 한다")
    return A_m, B_m, C_m


def augment(theta_r, vxr, vyr, Ts):
    """Wang Ch.1.2 증강. x = [Δx_m; y] (14), 입력 Δω (4), 출력 X_e (3)."""
    A_m, B_m, C_m = plant(theta_r, vxr, vyr, Ts)
    n, p = A_m.shape[0], C_m.shape[0]
    A = np.block([[A_m, np.zeros((n, p))], [C_m @ A_m, np.eye(p)]])
    B = np.vstack([B_m, C_m @ B_m])
    C = np.hstack([np.zeros((p, n)), np.eye(p)])
    return A, B, C


# ---------- 자기검사 ----------
def _check():
    Ts = float(_T["Ts"])
    print(f"plant_wheel.yaml: {CFG['_path']}")
    print(f"τ = {np.round(TAU*1000,1)} ms   K = {np.round(K_DC,4)}   θ = {THETA*1000:.0f} ms   r = {R_W}  k = {K_GEOM:.4f}")
    a, b, d = wheel_block(Ts)
    print(f"a′ = {np.round(np.diag(a),4)}   b′ = {np.round(np.diag(b),4)}   d = {d}")
    ok = True
    e1 = np.abs(T_FWD @ T_INV - np.eye(3)).max(); print(f"[1] T T⁻¹ = I₃   오차 {e1:.1e}"); ok &= e1 < 1e-12
    e2 = np.abs(T_FWD @ NULL_DIR).max(); print(f"[2] T·null_dir = 0   {e2:.1e}"); ok &= e2 < 1e-12
    # 몸체 모델과 kinematics 블록 일치: B_k T T⁻¹ == B_k3
    A_k, B_k3 = kinematic_jacobians(0.3, 0.5, 0.0, Ts); e3 = np.abs(B_k3 @ T_FWD @ T_INV - B_k3).max()
    print(f"[3] B_k T T⁻¹ = B_k(몸체)   {e3:.1e}"); ok &= e3 < 1e-12
    A_m, B_m, C_m = plant(0.3, 0.5, 0.0, Ts); A, B, C = augment(0.3, 0.5, 0.0, Ts)
    print(f"[4] 차원  x_m {A_m.shape[0]}  x {A.shape[0]}  입력 {B.shape[1]}  출력 {C.shape[0]}")
    ev = np.sort(np.abs(np.linalg.eigvals(A_m))); print(f"    σ(A_m) |λ| = {np.round(ev,3)}")
    # 증강계 vs 원 플랜트 20스텝
    rng = np.random.default_rng(0)
    xm_prev = rng.normal(size=11) * 0.1; ue_prev = rng.normal(size=4) * 0.5
    xm = A_m @ xm_prev + B_m @ ue_prev                      # 정합한 한 스텝 이력 (Δx_m 정의에 필요)
    ys, ya = [], []; x = np.concatenate([xm - xm_prev, C_m @ xm])
    for k in range(20):
        ue = rng.normal(size=4) * 0.5; du = ue - ue_prev; ue_prev = ue
        xm = A_m @ xm + B_m @ ue; ys.append(C_m @ xm)
        x = A @ x + B @ du; ya.append(C @ x)
    e4 = np.abs(np.array(ys) - np.array(ya)).max(); print(f"[5] 증강계 vs 원 플랜트 20스텝   {e4:.1e}"); ok &= e4 < 1e-10
    # 가제어·가관측
    n = A.shape[0]; Cc = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
    Ob = np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])
    rc, ro = np.linalg.matrix_rank(Cc, tol=1e-9), np.linalg.matrix_rank(Ob, tol=1e-9)
    print(f"[6] controllability {rc}/{n}   observability {ro}/{n}   (잃는 모드: 지연 λ=0 과 내부 자유도 — 둘 다 스스로 감쇠 → detectable)")
    print("\n전부 통과" if ok else "\n✗ 실패")
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if _check() else 1)
