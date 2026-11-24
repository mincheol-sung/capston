"""figure8·circles8 기준에서 낼 수 있는 최대 speed·accel (2026-11-23). launch speed:=max·accel:=max 가 쓴다.

두 제약을 다 지키는 가장 큰 값 (speed 0.01 m/s, accel 0.05 m/s² 격자, 내림):
  ① 부하 예산 — figure8_reference 와 같은 Figure8.peak_load(K_GEOM) (주행·정렬 둘 다) ≤ load_budget × r × max_wheel_rad_per_sec
  ② C 마찰 상한 — torque_mpc 가 보는 목표 입력 U_s = F⁺ R(ψ_r)ᵀ M (p̈_r − w) 가 max_i |U_s,i| / τ̄_i ≤ max_ff_ratio (기본 1)
     p̈_r 은 노드처럼 기준 월드 속도의 acc_dt (0.2 s) 차분이고, 노드 창이 기준열 첫 행부터라 0~0.15 s 앞당겨지므로 넷 중 최악을 쓴다.
     τ̄_i = (r/√2)·μ̂_i·N_i·c_s, N_i = 하중 몫_i·m g (torque_mpc.cpp tau_friction_limits 와 같은 식). 외란 추정 â_d 는 0 으로 둔다.
     torque_mpc cap_on total (바퀴 토크 상한, 기본) 기준이다. cap_on ground 이면 바닥 토크가 바퀴 관성 몫만큼 작아 이 값은 보수 쪽.
A·B·C 가 같은 기준을 받게, 제어기 종류와 상관없이 늘 C 상한으로 계산한다 (B·PP 는 상한이 더 크다).
검산: 2026-11-23 실차 circles8 (width 2.0, speed_mod 0.5 × 2, 위상 −90°) → speed 0.65 · accel 1.3 (analysis/torque_mpc_sim/2026-11-23_fig8_max).

지그재그 (zigzag_max, 2026-11-24): 직선마다 몸체 x 방향 가감속뿐이라 정사각형 변과 같은 직선 가속 한계 (line_accel_max) 를 쓴다.
  꼭짓점 제자리 회전 (zigzag_rot_s) 앞먹임은 info['ff'] 로 알려만 준다.

정사각형 (square_max, 2026-11-23 사용자 "1.5 1.5 로 했더니 돌아갔잖아"): 8자와 기준이 다르다.
  꼭짓점 (pass) 헤딩 회전은 앞먹임이 상한의 2 배 넘게 나와도 짧게 지나간다 — 실차 114214 (accel 1.5, v_max 1.5) 는 꼭짓점 시간의 49 % 포화에도
  완주, SLAM 위치 오차 RMS 45 mm. 그래서 꼭짓점은 빼고, 계속 걸리는 변 위 직선 가속만 상한 안에 둔다:
    accel = 0.05 내림( max_ff_ratio · 4·min_i τ̄_i / (m_eff·r) )   (직선 등가속의 U_s 는 네 바퀴 모두 m_eff·a·r/4, w = 0)
    speed:=max → square_v_max 0 (= load_budget·V_LIMIT 에서 자름, figure8_reference 의 기존 뜻). 꼭짓점 앞먹임은 기동 로그에 알려만 준다.
  8자는 등속 원에서 상한이 계속 걸려 (11-21 실차 1.1 m/s 원 앞먹임 1.05 → fault) 전체 구간 기준 그대로.

  python3 -m amr_mpc.core.fig8_limits --width 2.0 --speed_mod_amp 0.5 --speed_mod_phase_deg -90   # 미리 보기
"""
import math
import os

import numpy as np
import yaml

from amr_mpc.core.figure8 import Figure8

V_STEP, A_STEP = 0.01, 0.05
LEADS = (0.0, 0.05, 0.1, 0.15)


def plant_constants(config_dir, payload_kg=0.0, mu_hat=None, mu_hat_wheels=None, wheel_load_share=None, use_w=None,
                    com_height=None, load_min_frac=None, load_weighted_alloc=None, load_dynamic_cap=None):
    """torque_plant.yaml·torque_mpc.yaml → 앞먹임·상한 계산에 드는 값. 인자를 주면 yaml 대신 (launch 인자와 같은 뜻).
    com_height 등 = torque_mpc.yaml load_transfer (하중 이동, 2026-11-23) — com_height > 0 이면 ff_ratio 가 노드처럼 바퀴별 N 으로 상한·배분."""
    with open(os.path.join(config_dir, "torque_plant.yaml")) as f:
        P = yaml.safe_load(f)
    with open(os.path.join(config_dir, "torque_mpc.yaml")) as f:
        M = yaml.safe_load(f)
    geo, body, surf = P["geometry"], P["body"], P["surface"]
    r = float(geo["wheel_radius_m"]); k = 0.5 * (float(geo["wheel_base_m"]) + float(geo["robot_width_m"]))
    m = float(body["mass_kg"]) + float(payload_kg)
    mw = 4.0 * float(P["wheel"]["inertia_J_w_kgm2"]) / r ** 2
    share = np.array(wheel_load_share if wheel_load_share is not None else body["wheel_load_share"], float)
    mu0 = float(mu_hat if mu_hat is not None else surf["mu_hat"])
    muw = np.array(mu_hat_wheels if mu_hat_wheels is not None else surf.get("mu_hat_wheels", [-1.0] * 4), float)
    mu = np.where(muw > 0.0, muw, mu0)
    tau_bar = r / math.sqrt(2.0) * m * float(P["model"]["gravity"]) * float(surf["c_s"]) * mu * share
    if use_w is None:
        use_w = bool(M.get("model_terms", {}).get("use_w", False))
    lt = M.get("load_transfer", {})
    pick = lambda v, key, dflt: v if v is not None else lt.get(key, dflt)
    F = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]], float) / r          # 몸체 힘 = F τ (torque_mpc force_map)
    return dict(r=r, k=k, m=m, mw=mw, me=m + mw, Ie=float(body["inertia_z_kgm2"]) + mw * k * k, tau_bar=tau_bar,
                use_w=bool(use_w), acc_dt=float(M["tuning"]["acc_dt"]), F=F, Fp=np.linalg.pinv(F),
                n0=share * m * float(P["model"]["gravity"]), L=float(geo["wheel_base_m"]), W=float(geo["robot_width_m"]),
                com_height=float(pick(com_height, "com_height_m", 0.0)), min_frac=float(pick(load_min_frac, "min_frac", 0.2)),
                weighted_alloc=bool(pick(load_weighted_alloc, "weighted_alloc", True)),
                dynamic_cap=bool(pick(load_dynamic_cap, "dynamic_cap", True)))


def ff_ratio(fig, c, dt=0.01):
    """기준 전체에서 max_t max_i |U_s,i(t)| / τ̄_i (노드 차분 창 위치 넷 중 최악)."""
    t = np.arange(0.0, fig.T_total + 0.3, dt)
    XU = [fig.at(x) for x in t]
    X = np.array([a for a, _ in XU]); U = np.array([b for _, b in XU])
    cs, sn = np.cos(X[:, 2]), np.sin(X[:, 2])
    V = np.stack([cs * U[:, 0] - sn * U[:, 1], sn * U[:, 0] + cs * U[:, 1], U[:, 2]], 1)
    w = np.zeros_like(V)
    if c["use_w"]:
        w[:, 0], w[:, 1] = -c["mw"] * V[:, 2] * V[:, 1] / c["me"], c["mw"] * V[:, 2] * V[:, 0] / c["me"]
    h = int(round(0.5 * c["acc_dt"] / dt)); worst = 0.0
    for lead in LEADS:
        L = int(round(lead / dt)); pad = 2 * h + L
        Vp = np.vstack([np.repeat(V[:1], pad, 0), V, np.repeat(V[-1:], pad, 0)])
        acc = (Vp[pad + L + h: pad + L + h + len(t)] - Vp[pad + L - h: pad + L - h + len(t)]) / c["acc_dt"]
        q = acc - w
        F = np.stack([c["me"] * (cs * q[:, 0] + sn * q[:, 1]), c["me"] * (-sn * q[:, 0] + cs * q[:, 1]), c["Ie"] * q[:, 2]], 1)
        worst = max(worst, float(_ratio(F, acc, cs, sn, c).max()))
    return worst


def _ratio(Fb, acc, cs, sn, c):
    """몸체 힘 Fb (n,3) → max_i |U_s,i| / 상한_i (n,). 하중 이동 (c com_height > 0) 이면 torque_mpc.cpp step 과 같은 식:
    N_i = 몫_i m g − s_x m a_x h/(2L) − s_y m a_y h/(2W) (a = 기준 관성 가속도의 몸체 성분, ≥ min_frac × 정지), cap_i = τ̄_i N_i/N_i,0,
    배분 weighted_alloc → U = W⁻¹Fᵀ(FW⁻¹Fᵀ)⁻¹ Fb (W⁻¹ = diag(cap²)), 분모 dynamic_cap → cap_i (아니면 정지 τ̄_i)."""
    tb = c["tau_bar"]
    if not c.get("com_height", 0.0) > 0.0:
        return (np.abs(Fb @ c["Fp"].T) / tb).max(1)
    ax, ay = cs * acc[:, 0] + sn * acc[:, 1], -sn * acc[:, 0] + cs * acc[:, 1]
    sx, sy = np.array([1.0, 1.0, -1.0, -1.0]), np.array([1.0, -1.0, 1.0, -1.0])
    h, m, n0 = c["com_height"], c["m"], c["n0"]
    N = n0 - np.outer(m * ax * h / (2.0 * c["L"]), sx) - np.outer(m * ay * h / (2.0 * c["W"]), sy)
    N = np.maximum(N, c["min_frac"] * n0)
    cap = tb * N / n0
    if c["weighted_alloc"]:
        Fm = c["F"]; wi = cap ** 2                                        # (n,4)
        A = np.einsum("ij,nj,kj->nik", Fm, wi, Fm)                        # F W⁻¹ Fᵀ (n,3,3)
        lam = np.linalg.solve(A, Fb[:, :, None])[:, :, 0]                 # (n,3)
        U = wi * (lam @ Fm)                                               # W⁻¹ Fᵀ λ (n,4)
    else:
        U = Fb @ c["Fp"].T
    return (np.abs(U) / (cap if c["dynamic_cap"] else tb)).max(1)


def _largest(ok, lo, hi):
    """ok(n) 가 참인 가장 큰 정수 n ∈ [lo, hi] (ok 는 n 에 대해 단조 감소라고 본다). 없으면 None."""
    if hi < lo or not ok(lo):
        return None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid - 1)
    return lo


def max_speed_accel(make, c, load_cap, k_geom, speed=None, accel=None, max_ff_ratio=1.0, a_probe=0.3, a_hi=3.0):
    """make(speed, accel) → Figure8. speed·accel 중 None 인 것을 최대로 정한다. → (speed, accel, 정보 dict).
    둘 다 None 이면 speed 를 먼저 (accel = a_probe 로) 최대로, 그 speed 에서 accel 을 최대로."""
    def load_ok(f):
        run, spin = f.peak_load(k_geom)
        return run <= load_cap and spin <= load_cap

    def ok(v, a):
        f = make(v, a)
        return load_ok(f) and ff_ratio(f, c) <= max_ff_ratio

    a_try = accel if accel is not None else a_probe
    if speed is None:
        run1, _ = make(1.0, a_try).peak_load(k_geom)                 # 주행 부하는 speed 에 선형 → 위 끝
        n = _largest(lambda n: ok(n * V_STEP, a_try), 1, int(math.floor(load_cap / run1 / V_STEP + 1e-9)))
        if n is None:
            raise ValueError(f"accel {a_try} 에서 speed {V_STEP} 도 상한 안에 못 든다 (부하 예산·C 앞먹임)")
        speed = n * V_STEP
    if accel is None:
        n = _largest(lambda n: ok(speed, n * A_STEP), 1, int(round(a_hi / A_STEP)))
        if n is None:
            raise ValueError(f"speed {speed} 에서 accel {A_STEP} 도 상한 안에 못 든다")
        accel = n * A_STEP
    f = make(speed, accel); run, spin = f.peak_load(k_geom)
    return speed, accel, dict(ff=ff_ratio(f, c), load=run / load_cap, spin_load=spin / load_cap, T=f.T_total, tau_bar=c["tau_bar"],
                              com_height=c.get("com_height", 0.0))


def line_accel_max(c, max_ff_ratio=1.0):
    """몸체 x 방향 직선 등가속의 C 상한 안 최대 가속도 (0.05 내림). U_s 는 네 바퀴 모두 m_eff·a·r/4 (w = 0) → 가장 약한 τ̄_i 가 정한다."""
    a_lim = max_ff_ratio * 4.0 * float(np.min(c["tau_bar"])) / (c["me"] * c["r"])
    accel = math.floor(a_lim / A_STEP + 1e-9) * A_STEP
    if accel <= 0.0:
        raise ValueError(f"직선 가속 한계 {a_lim:.3f} m/s² — 0.05 보다 작다 (τ̄ 확인)")
    return accel


def zigzag_make(track, accel, v_max):
    """launch _track 값 (zigzag_* 키) → Zigzag."""
    from amr_mpc.core.zigzag import Zigzag
    return Zigzag(n=float(track["zigzag_n"]), length=float(track["zigzag_length"]), angle_deg=float(track["zigzag_angle_deg"]),
                  accel=accel, v_max=v_max, profile=str(track["zigzag_profile"]), rot_s=float(track["zigzag_rot_s"]),
                  dwell_s=float(track["zigzag_dwell_s"]), yaw_mode=str(track["yaw_mode"]),
                  spin_rate=float(track.get("spin_rate", 1.0)), align_s=float(track.get("align_s", 0.0)))


def zigzag_max(c, load_cap, track, accel=None, v_max=0.0, max_ff_ratio=1.0, k_geom=None):
    """지그재그 (2026-11-24): (직선 최고 속도, accel, 정보). accel None 이면 직선 가속 한계 line_accel_max.
    v_max (launch speed) ≤ 0 이면 load_cap 에서 자른다 (figure8_reference 와 같은 뜻). 꼭짓점 제자리 회전 앞먹임은 info['ff'] 에 같이 (전체 최대).
    yaw_mode fixed·spin (2026-11-24 (9)) 은 몸체 가속이 x 축만이 아니라 (옆걸음 |cos β|+|sin β| 배, spin 은 요 가속 κ_s·a 가 더해짐)
    직선 식이 바퀴 토크를 낮춰 본다 → accel None 이면 경로 전체 앞먹임 ff_ratio ≤ max_ff_ratio·부하 예산 안의 최대를 0.05 격자로 찾는다."""
    vm = v_max if v_max > 0.0 else load_cap
    if accel is None and str(track["yaw_mode"]) != "tangent":
        k = k_geom if k_geom is not None else c["k"]

        def ok(n):
            zz = zigzag_make(track, n * A_STEP, vm)
            run, spin = zz.peak_load(k)
            return run <= load_cap and spin <= load_cap and ff_ratio(zz, c) <= max_ff_ratio
        n = _largest(ok, 1, int(round(3.0 / A_STEP)))
        if n is None:
            raise ValueError(f"zigzag {track['yaw_mode']}: accel {A_STEP} 도 상한 안에 못 든다 (부하 예산·C 앞먹임)")
        accel = n * A_STEP
    if accel is None:
        accel = line_accel_max(c, max_ff_ratio)
    z = zigzag_make(track, accel, vm)
    run, _ = z.peak_load(k_geom if k_geom is not None else c["k"])
    line_ff = c["me"] * max(g.a_peak for g in z.legs) * c["r"] / 4.0 / float(np.min(c["tau_bar"]))
    return z.V, accel, dict(ff=ff_ratio(z, c), line_ff=line_ff, load=run / load_cap, T=z.T_total, tau_bar=c["tau_bar"])


def square_max(c, load_cap, track, accel=None, max_ff_ratio=1.0, k_geom=None):
    """정사각형: (변 최고 속도, accel, 정보). accel None 이면 변 직선 가속 한계 (위 식), 변 최고 = min(√(v_c² + a·L), load_cap)."""
    from amr_mpc.core.square import Square
    if accel is None:
        accel = line_accel_max(c, max_ff_ratio)
    vmax = float(track.get("square_v_max", 0.0)) or load_cap
    sq = Square(side=float(track["square_side"]), corner_speed=float(track["square_corner_speed"]), accel=accel, v_max=vmax,
                laps=float(track["laps"]), yaw_mode=str(track["yaw_mode"]), yaw_blend=float(track["square_yaw_blend"]),
                align_s=float(track["align_s"]), corner_mode=str(track.get("square_corner_mode", "pass")),
                rot_s=float(track.get("square_rot_s", 1.0)), dwell_s=float(track.get("square_dwell_s", 0.1)))
    run, spin = sq.peak_load(k_geom if k_geom is not None else c["k"])
    edge_ff = c["me"] * accel * c["r"] / 4.0 / float(np.min(c["tau_bar"]))
    return max(e.vp for e in sq.edges), accel, dict(ff=ff_ratio(sq, c), edge_ff=edge_ff, load=run / load_cap,
                                                     spin_load=spin / load_cap, T=sq.T_total, tau_bar=c["tau_bar"])


def figure8_maker(track):
    """launch _track 이 넘기는 값 (figure8_reference 파라미터와 같은 이름) → make(speed, accel)."""
    curve = "circles" if track["shape"] == "circles8" else "gerono"

    def make(v, a):
        return Figure8(width=float(track["width"]), height=float(track["height"]), speed=v, accel=a, laps=float(track["laps"]),
                       yaw_mode=str(track["yaw_mode"]), align_s=float(track["align_s"]), curve=curve,
                       crossing_blend=float(track["crossing_blend_m"]), spin_rate=float(track["spin_rate"]),
                       speed_mod_amp=float(track["speed_mod_amp"]), speed_mod_cycles=float(track["speed_mod_cycles"]),
                       speed_mod_phase_deg=float(track["speed_mod_phase_deg"]), speed_mod_shape=str(track["speed_mod_shape"]))
    return make


if __name__ == "__main__":
    import argparse
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.normpath(os.path.join(here, "..", "..", "config"))
    with open(os.path.join(cfg, "torque_track.yaml")) as fh:
        tr = yaml.safe_load(fh)["track"]
    ap = argparse.ArgumentParser(description="figure8·circles8 의 낼 수 있는 최대 speed·accel (launch speed:=max accel:=max 와 같은 계산)")
    for key in ("shape", "width", "height", "laps", "yaw_mode", "align_s", "crossing_blend_m", "spin_rate",
                "speed_mod_amp", "speed_mod_cycles", "speed_mod_phase_deg", "speed_mod_shape",
                "square_side", "square_corner_speed", "square_yaw_blend", "square_corner_mode", "square_rot_s", "square_dwell_s",
                "zigzag_n", "zigzag_length", "zigzag_angle_deg", "zigzag_profile", "zigzag_rot_s", "zigzag_dwell_s"):
        ap.add_argument(f"--{key}", default=tr.get(key))
    ap.add_argument("--load_budget", type=float, default=1.0)
    ap.add_argument("--max_wheel_rad_per_sec", type=float, default=370.0 * 2 * math.pi / 60.0)
    ap.add_argument("--max_ff_ratio", type=float, default=1.0)
    ap.add_argument("--com_height", type=float, default=None, help="하중 이동 무게중심 높이 [m] (비우면 torque_mpc.yaml load_transfer)")
    ap.add_argument("--speed", type=float, default=None, help="고정 speed (비우면 최대)")
    ap.add_argument("--accel", type=float, default=None, help="고정 accel (비우면 최대)")
    a = ap.parse_args()
    track = {k: getattr(a, k) for k in vars(a)}
    c = plant_constants(cfg, com_height=a.com_height)
    k_geom = c["k"]
    cap = a.load_budget * c["r"] * a.max_wheel_rad_per_sec
    if track["shape"] == "zigzag":
        vp, acc, info = zigzag_max(c, cap, track, accel=a.accel, v_max=a.speed or 0.0, max_ff_ratio=a.max_ff_ratio, k_geom=k_geom)
        print(f"zigzag {track['zigzag_n']} × {track['zigzag_length']} m, 사이각 {track['zigzag_angle_deg']}°, {track['zigzag_profile']}: "
              f"yaw {track['yaw_mode']}, accel {acc:.2f} → 직선 최고 {vp:.2f} m/s (상한 {f'speed {a.speed:g}' if a.speed else f'부하 예산 {cap:.2f}'})  ("
              + (f"직선 앞먹임 {info['line_ff']:.3f} ≤ {a.max_ff_ratio}, 꼭짓점 회전 포함 최대 {info['ff']:.2f}, " if track["yaw_mode"] == "tangent"
                 else f"경로 전체 앞먹임 {info['ff']:.3f} ≤ {a.max_ff_ratio}, ") +
              f"부하 {info['load'] * 100:.0f} %, 기준 {info['T']:.1f} s)")
    elif track["shape"] == "square":
        track["square_v_max"] = 0.0
        vp, acc, info = square_max(c, cap, track, max_ff_ratio=a.max_ff_ratio, k_geom=k_geom)
        print(f"square side {track['square_side']} ({track['square_corner_mode']}): accel:={acc:.2f} square_v_max:=0 → 변 최고 {vp:.2f} m/s  "
              f"(변 직선 앞먹임 {info['edge_ff']:.3f} ≤ {a.max_ff_ratio}, 꼭짓점 포함 최대 {info['ff']:.2f}, 부하 {info['load'] * 100:.0f} %, 기준 {info['T']:.1f} s)")
    else:
        v, acc, info = max_speed_accel(figure8_maker(track), c, cap, k_geom, speed=a.speed, accel=a.accel, max_ff_ratio=a.max_ff_ratio)
        print(f"{track['shape']} width {track['width']} (h {c['com_height']:g}): speed:={v:.2f} accel:={acc:.2f}  "
              f"(C 앞먹임 {info['ff']:.3f} ≤ {a.max_ff_ratio}, 부하 {info['load'] * 100:.0f} %, 기준 {info['T']:.1f} s, τ̄ {np.round(info['tau_bar'], 4)})")
