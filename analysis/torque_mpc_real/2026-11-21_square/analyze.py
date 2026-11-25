#!/usr/bin/python3
"""실차 정사각형 (2026-11-21 18:38~18:40) — 퓨어 퍼슛 (룩어헤드 상수 0.30 m) vs MPC C (μ̂ × 0.9/0.8 = c_s 0.9 상당, feedforward 창 고친 뒤).
비교용으로 11-20 판 (퓨어 퍼슛 171640 룩어헤드 속도 비례, C 190506 c_s 0.8 · feedforward 버그) 을 같이 낸다.

잣대 (11-20 `../2026-11-20_square/plot_newmu.py` 와 같음 + lag · λ 추가)
  위치 = /tracked_pose (IMU 점 → 로봇 중심), 기준 = /dra_mppi/reference_trajectory, 정사각형 로컬 좌표 = 주행 시작 순간의 기준 포즈.
  contour error = 정사각형 변까지 거리. 시간 추종·e_t·e_c·yaw·슬립 RMS x·y·τ·Δτ = tools/compare_slam.py analyse().
  lag = s_robot − s_ref (arc length, `../2026-11-20_why_c/lag_arclen.py` 와 같은 정의, 여기서는 SLAM 중심으로 — PP 에는 /torque_mpc/status 가 없다).
  λ = /slip_logger/slip 의 λ_FL..RR (추정기 v̂ 기준 ω_nom, slip_logger.py). λ > 0 헛돎, λ < 0 끌림. 0.5 m/s 에서 잡음 바닥 ±0.1.
  feedforward 비율 ρ · 포화 (C 만) = `../2026-11-20_why_c/verify_ff.py`·`phase.py` 와 같은 식.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/analyze.py"""
import importlib.util, json, math, os
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
WS = "/home/smoc/capston/capston"
MPC = f"{WS}/src/amr_basic_stack/controller/mpc"
WHY = f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", f"{WHY}/why_c.py"); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
spec3 = importlib.util.spec_from_file_location("la", f"{WHY}/lag_arclen.py")

# (bag, 이름, 색, 선, C 상한 τ̄ 또는 None)
BAGS = [("2026-11-21/184008", "PP (룩어헤드 0.30 m)", "#1baf7a", "-", None),
        ("2026-11-21/183827", "MPC C1 (c_s 0.9, Q11=Q22=5)", "#eb6834", "-", np.array([0.1230, 0.1253, 0.1298, 0.1187])),
        ("2026-11-21/190553", "MPC C2 (c_s 0.9, Q11=Q22=20)", "#6250d6", "-", np.array([0.1230, 0.1253, 0.1298, 0.1187])),
        ("2026-11-20/171640", "11-20 PP (룩어헤드 비례)", "#8fd3b9", "--", None),
        ("2026-11-20/190506", "11-20 MPC C (c_s 0.8, ff 버그)", "#f4ae90", "--", np.array([0.1093, 0.1113, 0.1154, 0.1055]))]
SIDE = 1.5
CORNERS = np.array([(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE)])
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2, REF = "#0b0b0b", "#52514e", "#a9a8a2"
est = yaml.safe_load(open(f"{MPC}/config/torque_estimator.yaml"))["estimator"]
plant = yaml.safe_load(open(f"{MPC}/config/torque_plant.yaml"))
off = (float(est["imu_offset_x"]), float(est["imu_offset_y"]))
r_w = plant["geometry"]["wheel_radius_m"]
k_geo = (plant["geometry"]["wheel_base_m"] + plant["geometry"]["robot_width_m"]) / 2
m_eff = plant["body"]["mass_kg"] + 4 * plant["wheel"]["inertia_J_w_kgm2"] / r_w ** 2


def contour(px, py):
    def seg(ax, ay, bx, by):
        ux, uy = bx - ax, by - ay
        t = np.clip(((px - ax) * ux + (py - ay) * uy) / (ux * ux + uy * uy), 0, 1)
        return np.hypot(px - ax - t * ux, py - ay - t * uy)
    P = [(0, 0), (SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]
    return np.min([seg(*P[i], *P[i + 1]) for i in range(4)], axis=0)


def project(px, py, X, Y, Sc, s0, lo=-0.3, hi=0.6):
    j = np.flatnonzero((Sc >= s0 + lo) & (Sc <= s0 + hi))
    if len(j) < 2:
        return s0
    best, bs = 1e9, s0
    for k in j[:-1]:
        ax, ay, dx, dy = X[k], Y[k], X[k + 1] - X[k], Y[k + 1] - Y[k]; L2 = dx * dx + dy * dy
        u = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        d = (ax + u * dx - px) ** 2 + (ay + u * dy - py) ** 2
        if d < best:
            best, bs = d, Sc[k] + u * math.sqrt(L2)
    return bs


def read_topic(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in rd.get_all_topics_and_types()}
    rd.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while rd.has_next():
        _, data, _ = rd.read_next(); out.append(list(deserialize_message(data, get_message(types[topic])).data))
    return np.array(out)


def run(bag, tb):
    path = f"{WS}/bags/{bag}"
    D = cs.load(path)
    P = np.array(D["pose"]); xc, yc = cs.imu_to_center(P[:, 1], P[:, 2], P[:, 3], off)
    rt = np.array([a for a, _ in D["ref"]])
    ref_at = lambda x: D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1].at(x)
    rows = []
    for t, x, y, th in zip(P[:, 0], xc, yc, P[:, 3]):
        if np.searchsorted(rt, t, side="right") - 1 >= 0:
            Xr, ur = ref_at(t)[:2]; rows.append([t, x, y, th, *Xr, *ur])
    a = np.array(rows); t = a[:, 0]
    sp = np.hypot(a[:, 7], a[:, 8]); arm = cs.armed_mask(t, D["armed"]); mv = arm & (sp > 0.02)
    i0, i1 = np.flatnonzero(mv)[0], np.flatnonzero(mv)[-1]; o = a[i0, 4:7]; T0 = t[i0]
    c, s = math.cos(o[2]), math.sin(o[2])
    loc = lambda X, Y: (c * (X - o[0]) + s * (Y - o[1]), -s * (X - o[0]) + c * (Y - o[1]))
    lx, ly = loc(a[:, 1], a[:, 2]); rx, ry = loc(a[:, 4], a[:, 5])
    win = arm & (t >= T0 - 0.5) & (t <= t[i1] + 2.0)
    d = contour(lx, ly)
    # 슬립 크기 (compare_slam 과 같은 식) — 11-20 표와 이어 보기용
    W = np.array(D["wheel"]); q = r_w / 4.0
    wv = np.stack([q * W[:, 1:5].sum(1), q * (-W[:, 1] + W[:, 2] + W[:, 3] - W[:, 4])], 1)
    ts, h = P[:, 0], 0.1
    sv = [(np.interp(ts + h, ts, arr) - np.interp(ts - h, ts, arr)) / (2 * h) for arr in (xc, yc)]
    cc, ss = np.cos(P[:, 3]), np.sin(P[:, 3])
    bx, by = cc * sv[0] + ss * sv[1], -ss * sv[0] + cc * sv[1]
    slipmag = np.hypot(np.interp(ts, W[:, 0], wv[:, 0]) - bx, np.interp(ts, W[:, 0], wv[:, 1]) - by)
    Tq = np.array(D["tau"]); Tq = Tq[(Tq[:, 1] != 1)]
    # 기준 조밀 표본 (100 Hz) → arc length, 꼭짓점 (|ω_r| > 0.3)
    td = np.arange(T0, t[i1] + 0.5, 0.01)
    RD = np.array([np.r_[ref_at(x)[0], ref_at(x)[1]] for x in td])
    Xd, Yd = loc(RD[:, 0], RD[:, 1]); Sc = np.r_[0.0, np.cumsum(np.hypot(np.diff(Xd), np.diff(Yd)))]
    corner_d = np.abs(RD[:, 5]) > 0.3; vd_ref = np.hypot(RD[:, 3], RD[:, 4])
    # lag (SLAM 중심)
    sel = (t >= T0) & (t <= t[i1] + 0.5); lag = np.full(len(t), np.nan); s_prev = 0.0
    for kk in np.flatnonzero(sel):
        s_prev = project(lx[kk], ly[kk], Xd, Yd, Sc, s_prev); lag[kk] = s_prev - np.interp(t[kk], td, Sc)
    tc0 = td[np.argmax(corner_d)]                                         # 첫 꼭짓점 회전 구간 진입 = 변 0 끝
    lag_side0 = float(np.interp(tc0, t[sel], lag[sel]))
    # 변별 최고 속도 (SLAM 차분 속도, 꼭짓점 0.3 m 밖) 와 그 위치 (변 시작에서 거리)
    spd = np.hypot(bx, by); plx, ply = loc(xc, yc) if False else (np.interp(ts, t, lx), np.interp(ts, t, ly))
    mvs = (ts >= T0) & (ts <= t[i1])
    side_peak = []
    for n in range(4):
        A0, B0 = CORNERS[n], CORNERS[(n + 1) % 4]; u = (B0 - A0) / SIDE
        along = (plx - A0[0]) * u[0] + (ply - A0[1]) * u[1]; perp = -(plx - A0[0]) * u[1] + (ply - A0[1]) * u[0]
        m_ = mvs & (along > 0.3) & (along < SIDE - 0.3) & (np.abs(perp) < 0.3)
        if m_.any():
            j = np.flatnonzero(m_)[np.argmax(spd[m_])]; side_peak.append((float(spd[j]), float(along[j])))
    # λ (slip_logger, 추정기 v̂ 기준)
    L = read_topic(path, "/slip_logger/slip")
    L = L[(L[:, 0] >= T0) & (L[:, 0] <= t[i1])]; L = L[np.all(np.isfinite(L[:, 1:5]), axis=1)]
    lam = L[:, 1:5]; al = np.abs(lam); mx = al.max(1); big = mx > 0.5
    sgn = np.sign(lam[np.arange(len(lam)), al.argmax(1)])
    px, py = np.interp(L[:, 0], t, lx), np.interp(L[:, 0], t, ly)
    near = np.min(np.hypot(px[:, None] - CORNERS[None, :, 0], py[:, None] - CORNERS[None, :, 1]), axis=1) < 0.3
    lam_m = dict(big=100 * float(big.mean()), slip01=100 * float((mx > 0.1).mean()), lmax=float(mx.max()),
                 lrms=float(np.sqrt(np.mean(lam ** 2))), spin=100 * float((sgn[big] > 0).mean()) if big.any() else float("nan"),
                 near=100 * float(near[big].mean()) if big.any() else float("nan"),
                 per_wheel_max=[float(x) for x in al.max(0)], per_wheel_big=[100 * float((al[:, i] > 0.5).mean()) for i in range(4)])
    # 기준 속력 > 0.3 m/s 구간만 — 꼭짓점 저속에서는 분모 max(|ω|, |ω_nom|, ω_ε 2.5 rad/s) 가 작아 v̂ 잡음이 λ 를 키운다
    fast = np.interp(L[:, 0], td, vd_ref) > 0.3; bf = big[fast]
    lam_m.update(big_fast=100 * float(bf.mean()), spin_fast=100 * float((sgn[fast][bf] > 0).mean()) if bf.any() else float("nan"),
                 rms_fast=float(np.sqrt(np.mean(lam[fast] ** 2))), share_fast=100 * float(fast.mean()))
    out = dict(t=t - T0, lx=lx, ly=ly, rx=rx, ry=ry, mv=mv, win=win, contour=d, lag=lag, lag_side0=lag_side0,
               ts=ts - T0, speed=spd, slipmag=slipmag, tt=Tq[:, 0] - T0, tau=np.abs(Tq[:, 2:6]).max(axis=1),
               td=td - T0, vref=vd_ref, lt=L[:, 0] - T0, lammax=mx * np.where(sgn >= 0, 1, -1), lam=lam_m, side_peak=side_peak,
               cs=cs.analyse(path, off, r_w, k_geo))
    # C: feedforward 비율 ρ (직선 가속), 포화·U_s 초과 (구간별)
    if tb is not None:
        S, E, Pp, A = w.load(path)
        tS = S[:, 0]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5
        runm = cs.armed_mask(tS, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
        a_true, om, v_r = [], [], []
        for x in tS:
            R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]
            v1 = np.hypot(*R_.at(x + 0.15)[1][:2]); v0 = np.hypot(*R_.at(x + 0.05)[1][:2])
            a_true.append((v1 - v0) / 0.1); u_ = R_.at(x)[1]; om.append(abs(u_[2])); v_r.append(np.hypot(u_[0], u_[1]))
        a_true, om = np.array(a_true), np.array(om)
        sel_ff = runm & (a_true > 1.3) & (om < 0.05) & (np.abs(us).max(1) > 0.01)
        rho = np.abs(us[sel_ff]).mean(1) / (m_eff * a_true[sel_ff] * r_w / 4)
        sat = (np.abs(tau) / tb).max(1) >= 0.98; ffo = (np.abs(us) / tb).max(1) > 1.0
        vd2 = np.gradient(np.array(v_r), tS); corner = om > 0.3
        ph = {"가속": ~corner & (vd2 > 0.2), "감속": ~corner & (vd2 < -0.2), "꼭짓점": corner, "그 밖": ~corner & (vd2 >= -0.2) & (vd2 <= 0.2)}
        out["c"] = dict(rho=float(rho.mean()), rho_n=int(sel_ff.sum()), sat=100 * float(sat[runm].mean()), ffo=100 * float(ffo[runm].mean()),
                        phase={k: dict(share=100 * float((runm & m).sum() / runm.sum()), sat=100 * float(sat[runm & m].mean()), ffo=100 * float(ffo[runm & m].mean()))
                               for k, m in ph.items()})
    return out


R = {b: run(b, tb) for b, _, _, _, tb in BAGS}
NAME = {b: n for b, n, *_ in BAGS}; COL = {b: c for b, _, c, *_ in BAGS}; LS = {b: l for b, _, _, l, _ in BAGS}
NEW = ["2026-11-21/184008", "2026-11-21/183827", "2026-11-21/190553"]; OLD = ["2026-11-20/171640", "2026-11-20/190506"]

# 표
summ = {}
for b in R:
    r = R[b]; c2 = r["cs"]; L = r["lag"][np.isfinite(r["lag"])]
    summ[b] = dict(name=NAME[b], contour=1e3 * float(np.sqrt(np.mean(r["contour"][r["mv"]] ** 2))), contour_max=1e3 * float(r["contour"][r["mv"]].max()),
                   lag_mean=1e3 * float(L.mean()), lag_min=1e3 * float(L.min()), lag_side0=1e3 * r["lag_side0"],
                   side_peak=r["side_peak"], lam=r["lam"], c=r.get("c"),
                   **{k: c2[k] for k in ("rms", "et_mean", "ec", "yaw", "slip_x", "slip_y", "tau_max", "dtau", "dur")})
json.dump(summ, open(os.path.join(HERE, "summary.json"), "w"), ensure_ascii=False, indent=1)
print("| 판 | contour RMS / 최대 [mm] | lag 평균 / 최저 / 변0 끝 [mm] | 시간 추종 RMS | yaw RMS ° | Δτ RMS | τ max | 슬립 RMS x·y |")
for b, s in summ.items():
    print(f"| {s['name']} | {s['contour']:.0f} / {s['contour_max']:.0f} | {s['lag_mean']:+.0f} / {s['lag_min']:+.0f} / {s['lag_side0']:+.0f} | {1e3*s['rms']:.0f} | "
          f"{s['yaw']:.1f} | {s['dtau']:.4f} | {s['tau_max']:.2f} | {s['slip_x']:.3f} · {s['slip_y']:.3f} |")
print("| 판 | 큰 슬립 % (max|λ|>0.5) 전체 / v_ref>0.3 | >0.1 % | λ 최대 | λ RMS 전체 / v_ref>0.3 | 큰 슬립 중 헛돎 % 전체 / v_ref>0.3 | 큰 슬립 중 꼭짓점 0.3 m 안 % | 바퀴별 |λ| 최대 FL/FR/RL/RR | 바퀴별 >0.5 % |")
for b, s in summ.items():
    l = s["lam"]
    print(f"| {s['name']} | {l['big']:.1f} / {l['big_fast']:.1f} | {l['slip01']:.1f} | {l['lmax']:.2f} | {l['lrms']:.3f} / {l['rms_fast']:.3f} | {l['spin']:.0f} / {l['spin_fast']:.0f} | {l['near']:.0f} | "
          f"{' / '.join(f'{x:.2f}' for x in l['per_wheel_max'])} | {' / '.join(f'{x:.1f}' for x in l['per_wheel_big'])} |")
print("변별 최고 속도 [m/s] (변 시작에서 거리 m):")
for b, s in summ.items():
    print(f"   {s['name']}: " + "  ".join(f"{v:.2f} ({p:.2f})" for v, p in s["side_peak"]))
for b, s in summ.items():
    if s["c"]:
        c = s["c"]
        print(f"{s['name']}: ρ {c['rho']:.2f} (n {c['rho_n']}), 포화 {c['sat']:.0f} %, U_s 초과 {c['ffo']:.0f} % | " +
              "  ".join(f"{k} {v['share']:.0f}%: 포화 {v['sat']:.0f} / 초과 {v['ffo']:.0f}" for k, v in c["phase"].items()))

# 그림 1: 궤적
fig, axes = plt.subplots(1, 2, figsize=(13, 6.6))
for ax, sel, title in ((axes[0], NEW, "11-21: PP (룩어헤드 0.30 m) vs MPC C1 (Q11=Q22=5) vs C2 (Q11=Q22=20)"),
                       (axes[1], OLD, "11-20: PP (룩어헤드 비례) vs MPC C (c_s 0.8, feedforward 버그)")):
    ax.plot([0, SIDE, SIDE, 0, 0], [0, 0, SIDE, SIDE, 0], color=REF, lw=7, alpha=0.45, solid_capstyle="round")
    for b in sel:
        r = R[b]; wm = r["win"]; s = summ[b]
        ax.plot(r["lx"][wm], r["ly"][wm], color=COL[b], lw=2.0, ls=LS[b],
                label=f"{NAME[b]}\n  contour {s['contour']:.0f} mm · yaw {s['yaw']:.1f}° · 큰 슬립 {s['lam']['big_fast']:.1f} %")
    ax.set_aspect("equal"); ax.set_xlim(-0.45, 2.0); ax.set_ylim(-0.45, 2.0)
    ax.set_title(title, loc="left", color=INK, fontsize=10.5); ax.legend(loc="center", frameon=False, fontsize=8.8)
    ax.set_xlabel("출발 꼭짓점 기준 x [m] (첫 변 방향)"); ax.set_ylabel("y [m]")
fig.suptitle("실차 정사각형 1.5 m · accel 1.5 m/s² · v_max 1.5 m/s · tangent — SLAM 포즈 (로봇 중심) 궤적", x=0.01, ha="left", color=INK, fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(HERE, "fig_traj.png"), dpi=130); plt.close(fig)

# 그림 2: 시계열 — 속도 / contour / lag / λ / 토크 (11-21 두 판)
fig, ax = plt.subplots(5, 1, figsize=(11, 12.5), sharex=True)
T1 = 9.5
for b in NEW:
    r = R[b]; ww = (r["ts"] > -0.5) & (r["ts"] < T1); wt = (r["t"] > -0.5) & (r["t"] < T1)
    wl = (r["lt"] > -0.5) & (r["lt"] < T1); wq = (r["tt"] > -0.5) & (r["tt"] < T1)
    ax[0].plot(r["ts"][ww], r["speed"][ww], color=COL[b], lw=1.7, label=NAME[b])
    ax[1].plot(r["t"][wt], 1e3 * r["contour"][wt], color=COL[b], lw=1.7)
    ax[2].plot(r["t"][wt], 1e3 * r["lag"][wt], color=COL[b], lw=1.7)
    ax[3].plot(r["lt"][wl], r["lammax"][wl], color=COL[b], lw=1.0)
    ax[4].plot(r["tt"][wq], r["tau"][wq], color=COL[b], lw=1.2)
r = R[NEW[1]]; wd = r["td"] < T1
ax[0].plot(r["td"][wd], r["vref"][wd], color=INK2, lw=1, ls=":", label="기준 속력")
ax[0].set_ylabel("속력 [m/s]\n(SLAM 차분)"); ax[0].legend(loc="upper right", frameon=False, fontsize=8.5, ncol=3)
ax[1].set_ylabel("contour error [mm]\n(정사각형 변까지)")
ax[2].set_ylabel("lag [mm]\ns_robot − s_ref"); ax[2].axhline(0, color=INK2, lw=0.8)
ax[3].set_ylabel("λ (|λ| 최대 바퀴, 부호 유지)\n+ 헛돎 / − 끌림")
for yv in (0.5, -0.5):
    ax[3].axhline(yv, color=INK2, lw=0.8, ls=":")
ax[3].text(T1 - 0.1, 0.55, "|λ| 0.5", color=INK2, fontsize=8.5, ha="right")
ax[4].set_ylabel("최대 바퀴 토크 [N·m]\n(최종 명령, 마찰 보상 포함)"); ax[4].set_xlabel("주행 시작 뒤 시간 [s]")
ax[0].set_title("11-21 실차 — PP vs MPC C (각 1 판)", loc="left", color=INK)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_timeseries.png"), dpi=130); plt.close(fig)

# 그림 3: 요약 small multiples — 단위가 달라 패널을 나눈다
mets = [("contour RMS [mm]", lambda s: s["contour"]), ("yaw 오차 RMS [°]", lambda s: s["yaw"]),
        ("큰 슬립 시간 [%]\nmax|λ| > 0.5, v_ref > 0.3 m/s", lambda s: s["lam"]["big_fast"]), ("토크 떨림 Δτ RMS [N·m]", lambda s: s["dtau"]),
        ("lag 평균 [mm]\n(− 뒤처짐)", lambda s: s["lag_mean"])]
order = ["2026-11-20/171640", "2026-11-21/184008", "2026-11-20/190506", "2026-11-21/183827", "2026-11-21/190553"]
fig, axs = plt.subplots(1, len(mets), figsize=(16.5, 4.2))
for ax, (lab, f) in zip(axs, mets):
    vals = [f(summ[b]) for b in order]
    bars = ax.bar(range(len(order)), vals, color=[COL[b] for b in order], width=0.7, edgecolor="#fcfcfb", linewidth=2)
    for i, b in enumerate(order):
        if LS[b] == "--":
            bars[i].set_hatch("///"); bars[i].set_edgecolor("#fcfcfb")
        ax.text(i, vals[i], f"{vals[i]:.3f}" if abs(vals[i]) < 1 else f"{vals[i]:.0f}" if abs(vals[i]) >= 10 else f"{vals[i]:.1f}",
                ha="center", va="bottom" if vals[i] >= 0 else "top", color=INK, fontsize=8.5)
    ax.set_xticks(range(len(order))); ax.set_xticklabels(["PP\n11-20", "PP\n11-21", "C\n11-20", "C1\n11-21", "C2\n11-21"], fontsize=8.5)
    ax.set_title(lab, loc="left", color=INK, fontsize=9.5); ax.grid(axis="x", visible=False); ax.axhline(0, color=INK2, lw=0.8)
fig.suptitle("실차 정사각형 요약 — 빗금 = 11-20 판 (PP 룩어헤드 비례 · C c_s 0.8 + feedforward 버그), 진한 색 = 11-21 판", x=0.01, ha="left", color=INK, fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "fig_summary.png"), dpi=130); plt.close(fig)
