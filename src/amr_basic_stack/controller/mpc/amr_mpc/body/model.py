#!/usr/bin/env python3
# WP4 예측모델 — Wang, "MPC System Design and Implementation Using MATLAB" Ch.1 형식
# ---------------------------------------------------------------
# Wang Ch.1.2 의 적분기 내장 증강모델을 그대로 쓴다.
#
#   플랜트   x_m(k+1) = A_m x_m(k) + B_m u(k),   y(k) = C_m x_m(k)
#   증분     Δx_m(k) = x_m(k) − x_m(k−1),        Δu(k) = u(k) − u(k−1)
#
#   증강     x(k) = [ Δx_m(k) ; y(k) ]
#            ⎡Δx_m(k+1)⎤   ⎡ A_m     0 ⎤⎡Δx_m(k)⎤   ⎡  B_m  ⎤
#            ⎢         ⎥ = ⎢           ⎥⎢       ⎥ + ⎢       ⎥ Δu(k)
#            ⎣ y(k+1)  ⎦   ⎣C_m·A_m  I ⎦⎣ y(k)  ⎦   ⎣C_m·B_m⎦
#            y(k) = [0  I]·x(k)
#
# 우리 플랜트는 오차좌표계다 (Tang 논문 Ⅱ장). 기준궤적 주위로 선형화하면 계수가
# 기준궤적에만 의존하므로 지평 전체가 미리 확정된다 (LTV).
#   X_e(k+1) = A_k X_e(k) + B_k v_e(k)      기구학
#   v_e(k+1) = a  v_e(k)  + b  u_e(k−d)     식별한 속도루프 (FOPDT)
# 오차좌표이므로 Wang 의 설정치 R_s 는 0 이 된다.
#
# theta=40ms, Ts=40ms 라 d=1 (정확히 한 스텝, 올림 없음). 드라이버 20ms 의 정수배.
# Wang 형식은 u(k−1) 을 상태로 갖지 않으므로
# 지연 상태 z(k)=u_e(k−1) 을 플랜트에 넣는다 (x_m 9차원 -> 증강 x 12차원).
#
#   python3 -m amr_mpc.body.model [지상_로그.csv]      자기검사 (controller/mpc 에서)
# ---------------------------------------------------------------
import csv, sys
import numpy as np

from amr_mpc.core import config as params
from amr_mpc.core.kinematics import f, kinematic_jacobians   # noqa: F401 (mpc.py 가 여기서 import)

# 식별 결과는 config/plant.yaml 의 model 절이 유일한 출처다. 재피팅하면 그 세 줄만 바꾼다.
# 값의 근거(2차 극점, τ·θ 비유일성, θ 고정 이유)도 그 파일 주석에 있다.
_M    = params.section("model")
TAU   = dict(_M["tau"])
K_DC  = dict(_M["k_dc"])
THETA = float(_M["theta"])
AXES  = ("vx", "vy", "wz")


_VB_CACHE = {}


def velocity_block(Ts):
    """FOPDT 를 ZOH 이산화. a, b 는 축별 대각, d 는 지연 스텝수.
    Ts 만 의존하므로 캐싱한다 (MPC 가 지평마다 Np 회 부른다). 반환 배열은 쓰기 금지."""
    hit = _VB_CACHE.get(Ts)
    if hit is None:
        a = np.diag([np.exp(-Ts / TAU[x]) for x in AXES])
        b = np.diag([K_DC[x] * (1 - np.exp(-Ts / TAU[x])) for x in AXES])
        a.flags.writeable = False; b.flags.writeable = False
        hit = _VB_CACHE[Ts] = (a, b, int(round(THETA / Ts)))
    return hit






def plant(theta_r, vxr, vyr, Ts):
    """Wang 의 (A_m, B_m, C_m). 오차좌표 플랜트, 출력은 포즈오차 X_e."""
    a, b, d = velocity_block(Ts)
    A_k, B_k = kinematic_jacobians(theta_r, vxr, vyr, Ts)
    Z, I = np.zeros((3, 3)), np.eye(3)
    if d == 0:
        A_m = np.block([[A_k, B_k],                   # x_m = [X_e ; v_e]
                        [Z,   a]])
        B_m = np.vstack([Z, b])
        C_m = np.hstack([I, Z])
    elif d == 1:
        A_m = np.block([[A_k, B_k, Z],                # x_m = [X_e ; v_e ; z]
                        [Z,   a,   b],                #   z(k)   = u_e(k−1)
                        [Z,   Z,   Z]])               #   z(k+1) = u_e(k)
        B_m = np.vstack([Z, Z, I])
        C_m = np.hstack([I, Z, Z])
    else:
        raise ValueError(f"d={d}: 지연 상태를 {d}단으로 늘려야 한다")
    return A_m, B_m, C_m


def augment(theta_r, vxr, vyr, Ts):
    """Wang Ch.1.2 증강모델 (A, B, C). x = [Δx_m ; y]."""
    A_m, B_m, C_m = plant(theta_r, vxr, vyr, Ts)
    n, p = A_m.shape[0], C_m.shape[0]
    A = np.block([[A_m,       np.zeros((n, p))],
                  [C_m @ A_m, np.eye(p)]])
    B = np.vstack([B_m, C_m @ B_m])
    C = np.hstack([np.zeros((p, n)), np.eye(p)])
    return A, B, C


# ---------- 자기검사 ----------

def check_jacobian():
    """해석적 A_k, B_k 를 중심차분과 대조. 부호·전치 실수를 잡는다."""
    rng = np.random.default_rng(0); Ts, h, worst = 0.05, 1e-6, 0.0
    for _ in range(300):
        th = rng.uniform(-np.pi, np.pi)
        vxr, vyr = rng.uniform(-0.3, 0.3, 2)
        X, u = np.array([0.0, 0.0, th]), np.array([vxr, vyr, rng.uniform(-1, 1)])
        Afd, Bfd = np.zeros((3, 3)), np.zeros((3, 3))
        for i in range(3):
            e = np.zeros(3); e[i] = h
            Afd[:, i] = (f(X + e, u) - f(X - e, u)) / (2 * h)
            Bfd[:, i] = (f(X, u + e) - f(X, u - e)) / (2 * h)
        A_k, B_k = kinematic_jacobians(th, vxr, vyr, Ts)
        worst = max(worst, abs((np.eye(3) + Ts * Afd) - A_k).max(),
                    abs(Ts * Bfd - B_k).max())
    print(f"[1] 기구학 야코비안 vs 중심차분   최대오차 {worst:.2e}"
          f"   {'OK' if worst < 1e-9 else '✗ 불일치'}")
    return worst < 1e-9


def check_augment():
    """증강모델이 원래 플랜트와 같은 y(k+1) 을 주는지 (Wang Ch.1.2 유도 검증)."""
    Ts = 0.05
    A_m, B_m, C_m = plant(0.3, 0.2, 0.05, Ts)
    A, B, C = augment(0.3, 0.2, 0.05, Ts)
    rng = np.random.default_rng(1); worst = 0.0
    for _ in range(200):
        n = A_m.shape[0]
        xm_prev, xm = rng.normal(size=n), rng.normal(size=n)
        u_prev, du = rng.normal(size=3), rng.normal(size=3)
        # 증강 경로
        x = np.concatenate([xm - xm_prev, C_m @ xm])
        y_aug = C @ (A @ x + B @ du)
        # 원 플랜트 경로: u(k) = u(k−1) + Δu(k)
        y_dir = C_m @ (A_m @ xm + B_m @ (u_prev + du))
        # 두 경로는 A_m·xm_prev + B_m·u_prev = xm 일 때 일치한다 (일관된 궤적)
        xm_consistent = A_m @ xm_prev + B_m @ u_prev
        x2 = np.concatenate([xm_consistent - xm_prev, C_m @ xm_consistent])
        y_aug2 = C @ (A @ x2 + B @ du)
        y_dir2 = C_m @ (A_m @ xm_consistent + B_m @ (u_prev + du))
        worst = max(worst, abs(y_aug2 - y_dir2).max())
    print(f"[2] 증강모델 vs 원 플랜트 (Wang Ch.1.2)   최대오차 {worst:.2e}"
          f"   {'OK' if worst < 1e-11 else '✗ 불일치'}")
    return worst < 1e-11


def check_openloop(path, Np=40):
    """지상 로그로 속도블록의 다단 개루프 예측 오차. 피드백 없이 Np 스텝 굴린다."""
    rows = list(csv.DictReader(open(path)))
    def col(n):
        o = np.empty(len(rows))
        for i, r in enumerate(rows):
            try: o[i] = float(r[n])
            except (TypeError, ValueError): o[i] = np.nan
        return np.nan_to_num(o)
    t, ax = col("t_s"), col("active").astype(int)
    Ts = float(np.median(np.diff(t)))
    a, b, d = velocity_block(Ts)
    U = np.column_stack([col("cvx"), col("cvy"), col("cw")])
    V = np.column_stack([col("ovx"), col("ovy"), col("ow")])

    print(f"[3] 개루프 예측 ({Np}스텝 = {Np*Ts:.2f}s, 피드백 없음)   d={d}")
    print(f"    {'축':>5}{'RMS':>12}{'최대':>10}{'명령 대비':>11}")
    ok = True
    for j, name in enumerate(AXES):
        idx = np.where(ax == j)[0]
        idx = idx[(idx > 0) & (idx < len(t) - Np)]
        errs = []
        for k in idx[::10]:
            v = V[k].copy()
            for i in range(Np):
                src = k + i - d
                v = a @ v + b @ (U[src] if src >= 0 else np.zeros(3))
            errs.append(v[j] - V[k + Np, j])
        errs = np.array(errs)
        rel = np.sqrt(np.mean(errs ** 2)) / np.abs(U[idx, j]).max() * 100
        ok &= rel < 10.0
        print(f"    {name:>5}{np.sqrt(np.mean(errs**2)):>12.4f}"
              f"{np.abs(errs).max():>10.4f}{rel:>10.1f}%")
    return ok


if __name__ == "__main__":
    Ts = 0.04
    a, b, d = velocity_block(Ts)
    n_m = plant(0.0, 0.2, 0.0, Ts)[0].shape[0]
    A, B, C = augment(theta_r=0.0, vxr=0.2, vyr=0.0, Ts=Ts)
    print(f"Ts={Ts}  d={d}   x_m {n_m}차원  ->  증강 x {A.shape[0]}차원\n")
    print("a(대각) =", np.round(np.diag(a), 4), "   b(대각) =", np.round(np.diag(b), 4))
    print()
    ok = check_jacobian(); ok = check_augment() and ok
    if len(sys.argv) > 1:
        ok = check_openloop(sys.argv[1]) and ok
    print("\n" + ("전부 통과" if ok else "⚠ 실패 항목 있음"))
