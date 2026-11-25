#!/usr/bin/python3
"""d̂ 와 ρ 결합 Kalman (torque_mpc.cpp joint_kalman_update 와 같은 식) 을 실차 8자 bag 의 잔차에 오프라인으로 돌린다 (RESULTS 20 절).
  잔차 r_k = x_k − f(x_{k−1}, τ_{k−1}) 는 공칭 모델과 실제의 차라 그때 제어기 (ρ 없음) 와 무관하다 → 이 bag 으로 "켰다면 무엇을 추정했을지" 를 본다.
  상태 z = [d_x, d_y, η] (몸체), 측정 = 몸체 잔차 [r_p,x; r_p,y; r_v,x; r_v,y] = H z + v,  H 의 η 열 = [½Ts²; Ts] ⊗ g,
  g = (c_n/m_eq) R(θ̂_k)ᵀ n̂_c,  c_n = n̂_c·M(p̈_r − w_r)  (직전 주기 기준, |ω_r| > 0.3·v_r > 0.2·|c_n| > 0.3 아니면 0).
  건너뛰기: 상태 간격 Ts ± 2 ms, skip_xy 1 mm, 포화 (|τ| ≥ 0.98 τ̄ 또는 |U_s| > τ̄). 예측 (P += Q) 은 매 상태마다.
  비교: d̂ 만 (축별 정상상태 Kalman, ρ 없음) 의 몸체 â_d,y 를 왼원·오른원 따로.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/residual_joint_kf.py [bag ...]"""
import os, math, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "residual_offline.py")).read()
g0 = {"__file__": os.path.join(HERE, "residual_offline.py"), "__name__": "ro"}
exec(src[:src.index("for b in (sys.argv")], g0)
cs, w, WS, read_odom = g0["cs"], g0["w"], g0["WS"], g0["read_odom"]
Ts, F, Minv, m, mu, m_eff, I_eff = g0["Ts"], g0["F"], g0["Minv"], g0["m"], g0["mu"], g0["m_eff"], g0["I_eff"]
TB = np.array([0.1230, 0.1253, 0.1298, 0.1187])                 # C 상한 (compare_all.py 와 같음)
KQ_TH, R_POS, R_VEL = 0.8, np.array([1.6e-3, 1.6e-3]), np.array([0.017, 0.029])   # yaml disturbance.kalman_* 기본
ACC_MAX, RHO_MIN, RHO_MAX, P0, PD0 = 0.5, 0.4, 1.2, 0.04, 0.09


def steady_gain(q_rate, rp, rv):
    q = q_rate ** 2 * Ts; H = np.array([0.5 * Ts * Ts, Ts]); R = np.diag([rp ** 2, rv ** 2]); P = q
    for _ in range(200000):
        Pm = P + q; S = np.outer(H, H) * Pm + R; K = Pm * H @ np.linalg.inv(S); Pn = (1 - K @ H) * Pm
        if abs(Pn - P) <= 1e-14 * P:
            break
        P = Pn
    return K


def prepare(b):
    path = f"{WS}/bags/2026-11-21/{b}"; D = cs.load(path); S, E, P, A = w.load(path)
    O = read_odom(path); t = O[:, 0]; th = np.unwrap(O[:, 3])
    c, s = np.cos(th), np.sin(th); X = np.c_[O[:, 1], O[:, 2], th, c * O[:, 4] - s * O[:, 5], s * O[:, 4] + c * O[:, 5], O[:, 6]]
    ts = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]
    rt = np.array([a_ for a_, _ in D["ref"]])

    def ref_at(x):
        i = np.searchsorted(rt, x, side="right") - 1
        if i < 0:
            return np.zeros(3), np.zeros(3)
        Xr, ur = D["ref"][i][1].at(x)[:2]
        return np.asarray(Xr, float), np.asarray(ur, float)

    def xdot_w(x):
        Xr, ur = ref_at(x); c_, s_ = math.cos(Xr[2]), math.sin(Xr[2])
        return np.array([c_ * ur[0] - s_ * ur[1], s_ * ur[0] + c_ * ur[1], ur[2]])

    rows = []
    for kk in range(1, len(t)):
        row = dict(t=t[kk], ok=False, g=np.zeros(2), om=0.0, vr=0.0, armed=False)
        dt = t[kk] - t[kk - 1]
        j = np.searchsorted(ts, t[kk - 1] + 1e-4)
        Xd0 = xdot_w(t[kk - 1]); vr = math.hypot(Xd0[0], Xd0[1]); row.update(om=Xd0[2], vr=vr)
        row["armed"] = bool(cs.armed_mask(np.array([t[kk]]), A)[0]) and vr > 0.02
        if abs(dt - Ts) > 0.002 or j >= len(ts) or ts[j] > t[kk]:
            rows.append(row); continue
        x0 = X[kk - 1]; th0 = x0[2]
        R3 = np.array([[math.cos(th0), -math.sin(th0), 0], [math.sin(th0), math.cos(th0), 0], [0, 0, 1]])
        a = Minv @ R3 @ F @ tau[j] + Minv @ np.array([mu * x0[5] * (-x0[4]), mu * x0[5] * x0[3], 0.0])
        pred = np.r_[x0[:3] + Ts * x0[3:] + 0.5 * Ts * Ts * a, x0[3:] + Ts * a]
        d = X[kk] - pred; d[2] = math.atan2(math.sin(d[2]), math.cos(d[2]))
        if max(abs(d[0]), abs(d[1])) > 1e-3:                       # skip_xy
            rows.append(row); continue
        jp = max(j - 1, 0)                                          # τ_{k−1} 의 앞 주기 (포화 판단은 직전 τ·U_s)
        sat = (np.abs(tau[j]) >= 0.98 * TB).any() or (np.abs(us[j]) > TB).any()
        row["sat"] = sat
        if sat and os.environ.get("FREEZE", "1") == "1":            # FREEZE=0 이면 포화 표본도 쓴다 (비교용)
            rows.append(row); continue
        ck, sk = math.cos(X[kk, 2]), math.sin(X[kk, 2])
        db = np.array([ck * d[0] + sk * d[1], -sk * d[0] + ck * d[1], ck * d[3] + sk * d[4], -sk * d[3] + ck * d[4]])
        # g: 직전 주기 기준의 c_n·n̂_c 를 θ̂_k 로 몸체에
        gb = np.zeros(2)
        if abs(Xd0[2]) > 0.3 and vr > 0.2:
            sg = 1.0 if Xd0[2] > 0 else -1.0
            n_c = np.array([-sg * Xd0[1] / vr, sg * Xd0[0] / vr])
            acc = (xdot_w(t[kk - 1] + Ts) - xdot_w(t[kk - 1] - Ts)) / (2 * Ts)
            wv = np.array([-mu * Xd0[2] * Xd0[1] / m_eff, mu * Xd0[2] * Xd0[0] / m_eff])   # use_w 켬
            c_n = n_c @ (m_eff * (acc[:2] - wv))
            if abs(c_n) > 0.3:
                gb = (c_n / m_eff) * np.array([ck * n_c[0] + sk * n_c[1], -sk * n_c[0] + ck * n_c[1]])
        row.update(ok=True, db=db, g=gb)
        rows.append(row)
    return rows


def run_joint(rows, q_xy, q_eta):
    z = np.array([0.0, 0.0, 0.0]); P = np.diag([PD0, PD0, P0]); hp = 0.5 * Ts * Ts
    Rm = np.diag(np.r_[R_POS ** 2, R_VEL ** 2]); Q = Ts * np.array([q_xy ** 2, q_xy ** 2, q_eta ** 2])
    out = []
    for r in rows:
        P = P + np.diag(Q)
        if P[2, 2] > 10 * P0:
            f = math.sqrt(10 * P0 / P[2, 2]); P[2, :] *= f; P[:, 2] *= f
        if r["ok"]:
            gx, gy = r["g"]
            H = np.array([[hp, 0, hp * gx], [0, hp, hp * gy], [Ts, 0, Ts * gx], [0, Ts, Ts * gy]])
            S = H @ P @ H.T + Rm; K = P @ H.T @ np.linalg.inv(S)
            z = z + K @ (r["db"] - H @ z); IKH = np.eye(3) - K @ H; P = IKH @ P @ IKH.T + K @ Rm @ K.T
            z[2] = min(max(z[2], 1 - RHO_MAX), 1 - RHO_MIN)
            n = math.hypot(z[0], z[1])
            if n > ACC_MAX:
                z[:2] *= ACC_MAX / n
        out.append((r["t"], z[0], z[1], 1 - z[2], math.sqrt(P[2, 2])))
    return np.array(out)


def run_dhat_only(rows, q_xy):
    K = [steady_gain(q_xy, R_POS[0], R_VEL[0]), steady_gain(q_xy, R_POS[1], R_VEL[1])]; a = np.zeros(2); out = []
    for r in rows:
        if r["ok"]:
            for ax in range(2):
                a[ax] += K[ax][0] * (r["db"][ax] - 0.5 * Ts * Ts * a[ax]) + K[ax][1] * (r["db"][ax + 2] - Ts * a[ax])
            n = math.hypot(*a)
            if n > ACC_MAX:
                a *= ACC_MAX / n
        out.append((r["t"], a[0], a[1]))
    return np.array(out)


for b in (sys.argv[1:] or ["212502", "212105"]):
    rows = prepare(b)
    om = np.array([r["om"] for r in rows]); vr = np.array([r["vr"] for r in rows]); arm = np.array([r["armed"] for r in rows])
    circ = arm & (vr > 0.3) & (np.abs(om) > 0.3); L, Rr = circ & (om > 0), circ & (om < 0)
    gn = np.array([np.linalg.norm(r["g"]) for r in rows])
    t_arm = np.array([r["t"] for r in rows])[arm]
    sat_c = np.array([r.get("sat", False) for r in rows])
    print(f"   (원 위 포화 표본 {100 * np.mean(sat_c[circ]):.0f} % — 왼원 {100 * np.mean(sat_c[L]):.0f} %, 오른원 {100 * np.mean(sat_c[Rr]):.0f} %;  FREEZE={os.environ.get('FREEZE', '1')})")
    print(f"== {b}  주행 {t_arm[-1] - t_arm[0]:.1f} s,  원 위 |g| = c_n/m_eq 평균 {gn[circ].mean():.3f} m/s² (v_r ω_r 평균 {np.mean(vr[circ] * np.abs(om[circ])):.3f}),"
          f"  측정 쓴 비율 {100 * np.mean([r['ok'] for r in np.array(rows)[arm]]):.0f} %")
    for q_xy in (0.3, 0.1):
        Dd = run_dhat_only(rows, q_xy)
        print(f"   d̂ 만 (q_xy {q_xy}):  몸체 â_d,y 왼원 {Dd[L, 2].mean():+.3f}  오른원 {Dd[Rr, 2].mean():+.3f} m/s²")
        for q_eta in (0.01, 0.03, 0.1):
            J = run_joint(rows, q_xy, q_eta)
            half = arm & (np.arange(len(rows)) > np.flatnonzero(arm)[len(np.flatnonzero(arm)) // 2])
            print(f"   결합 (q_xy {q_xy}, q_η {q_eta:.2f}):  ρ̂ 끝 {J[arm, 3][-1]:.3f}  뒤쪽 절반 평균 {J[half, 3].mean():.3f} ± {J[half, 3].std():.3f}"
                  f" (σ_η 끝 {J[arm, 4][-1]:.3f})   몸체 d_y 왼원 {J[L, 2].mean():+.3f}  오른원 {J[Rr, 2].mean():+.3f}   d_x {J[circ, 1].mean():+.3f}")
    # batch least squares (같은 잔차·같은 건너뛰기): 원 위 몸체 속도 잔차/Ts = d + η g,  d = [d_x, d_y] 일정 — 사전값·수렴 속도 없는 추정
    sel = [i for i, r in enumerate(rows) if r["ok"] and circ[i] and np.linalg.norm(r["g"]) > 0]
    A_ = []; y_ = []
    for i in sel:
        gx, gy = rows[i]["g"]; db = rows[i]["db"]
        A_ += [[1, 0, gx], [0, 1, gy]]; y_ += [db[2] / Ts, db[3] / Ts]
    A_ = np.array(A_); y_ = np.array(y_); cf, *_ = np.linalg.lstsq(A_, y_, rcond=None)
    res = y_ - A_ @ cf
    # 표준오차: 성분별 잔차 분산으로 가중 (x·y 잡음이 다르다), 이웃 표본 상관은 무시 → 실제보다 작게 나온다
    sx, sy = res[0::2].std(), res[1::2].std(); Wd = np.tile([1 / sx ** 2, 1 / sy ** 2], len(sel))
    cov = np.linalg.inv(A_.T @ (A_ * Wd[:, None])); se = np.sqrt(np.diag(cov))
    gm = np.mean([np.linalg.norm(rows[i]["g"]) for i in sel])
    print(f"   batch LS (원 위 {len(sel)} 표본):  η {cf[2]:+.3f} ± {se[2]:.3f} → ρ {1 - cf[2]:.3f}   d_x {cf[0]:+.3f}  d_y {cf[1]:+.3f} m/s²"
          f"   (η g = {cf[2] * gm:+.3f} m/s² @ |g| {gm:.3f};  표준오차는 표본 상관 무시 — 하한)")
