#!/usr/bin/python3
"""C 가 왜 경로에 안 붙나 — MPC 출력 τ 와 feedforward U_s 를 몸체 힘으로 나눠 feedback 이 어느 축에 가는지, 그리고 옆 오차에 얼마나 세게 반응하는지.
  힘 변환 (torque_mpc.hpp force_map, (T⁻¹)ᵀ):  [F_x, F_y, M_z] = (1/r) [1 1 1 1; −1 1 1 −1; −k k −k k] τ   (바퀴 순서 FL FR RL RR, k = 0.1932)
  feedback ΔF = F(τ) − F(U_s), 몸체 → 월드 (추정기 헤딩) → 경로 좌표 (기준 속도 방향 t̂, 왼쪽 법선 n̂).
  오차 (MPC 가 보는 값, status Xe = 기준 − 추정기): e_t = −Xe·t̂ (− 뒤처짐), e_n = −Xe·n̂ (+ 경로 왼쪽), e_θ = −Xe_θ.
  가속 환산: a = ΔF / m_eff (6.785 kg), α = ΔM_z / I_eff (0.1431 kg·m²).
  회귀 (포화 아닌 표본, 꼭짓점 밖 |ω_r| < 0.3, v_ref > 0.3): a_n = −k_p e_n − k_d ė_n, a_t = −k_p e_t − k_d ė_t → 등가 이득과 2차 pole s² + k_d s + k_p.
  SLAM 포즈 튐: 연속 /tracked_pose 사이 이동이 추정기 이동과 3 cm 넘게 다른 시각.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/decomp.py"""
import importlib.util, json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
WS = "/home/smoc/capston/capston"; MPC = f"{WS}/src/amr_basic_stack/controller/mpc"; WHY = f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec = importlib.util.spec_from_file_location("cs", f"{MPC}/tools/compare_slam.py"); cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2 = importlib.util.spec_from_file_location("w", f"{WHY}/why_c.py"); w = importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
r, k = 0.04, 0.1932; m_eff, I_eff = 6.785, 0.1431
FM = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-k, k, -k, k]]) / r
BAGS = [("2026-11-21/183827", "C1 11-21 (c_s 0.9, Q11=Q22=5)", np.array([0.1230, 0.1253, 0.1298, 0.1187]), "#eb6834"),
        ("2026-11-21/190553", "C2 11-21 (c_s 0.9, Q11=Q22=20)", np.array([0.1230, 0.1253, 0.1298, 0.1187]), "#6250d6"),
        ("2026-11-20/190506", "C 11-20 (c_s 0.8, ff 버그)", np.array([0.1093, 0.1113, 0.1154, 0.1055]), "#f4ae90")]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 10, "axes.unicode_minus": False,
                     "axes.edgecolor": "#b8b7b0", "axes.labelcolor": "#52514e", "xtick.color": "#52514e",
                     "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6,
                     "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb"})
INK, INK2 = "#0b0b0b", "#52514e"


def smooth_grad(x, t, h=5):
    """±h 표본 중앙 차분 (100 Hz → ±50 ms)."""
    g = np.full(len(x), np.nan)
    g[h:-h] = (x[2 * h:] - x[:-2 * h]) / (t[2 * h:] - t[:-2 * h])
    return g


def fit(a, e, ed):
    ok = np.isfinite(a) & np.isfinite(e) & np.isfinite(ed)
    X = np.c_[-e[ok], -ed[ok]]; c, *_ = np.linalg.lstsq(X, a[ok], rcond=None)
    pred = X @ c; r2 = 1 - np.sum((a[ok] - pred) ** 2) / np.sum((a[ok] - a[ok].mean()) ** 2)
    kp, kd = c; poles = np.roots([1, kd, kp])
    return dict(kp=float(kp), kd=float(kd), r2=float(r2), n=int(ok.sum()), poles=[complex(p) for p in poles])


out = {}
for bag, name, tb, col in BAGS:
    path = f"{WS}/bags/{bag}"
    S, E, P, A = w.load(path); D = cs.load(path)
    t = S[:, 0]; Xe = S[:, 1:4]; tau = S[:, 4:8]; us = S[:, 22:26]; hold = S[:, 20] > 0.5
    run = cs.armed_mask(t, A) & ~hold & (np.abs(tau).max(1) > 1e-6)
    rt = np.array([a for a, _ in D["ref"]])
    th = np.interp(t, E[:, 0], np.unwrap(E[:, 3]))                       # 추정기 헤딩
    tv, vr, om = [], [], []
    for x in t:
        R_ = D["ref"][max(0, np.searchsorted(rt, x, side="right") - 1)][1]; Xr, ur = R_.at(x)[:2]
        c, s = math.cos(Xr[2]), math.sin(Xr[2]); vw = np.array([c * ur[0] - s * ur[1], s * ur[0] + c * ur[1]]); n = np.linalg.norm(vw)
        tv.append(vw / max(n, 1e-9)); vr.append(n); om.append(ur[2])
    tv = np.array(tv); nv = np.c_[-tv[:, 1], tv[:, 0]]; vr = np.array(vr); om = np.array(om)
    i0 = np.flatnonzero(run & (vr > 0.02))[0]; T0 = t[i0]
    e_t = -(Xe[:, :2] * tv).sum(1); e_n = -(Xe[:, :2] * nv).sum(1); e_th = -Xe[:, 2]
    dF = (FM @ (tau - us).T).T                                             # 몸체 [F_x, F_y, M_z] feedback
    c, s = np.cos(th), np.sin(th); Fw = np.c_[c * dF[:, 0] - s * dF[:, 1], s * dF[:, 0] + c * dF[:, 1]]
    a_t = (Fw * tv).sum(1) / m_eff; a_n = (Fw * nv).sum(1) / m_eff; alpha = dF[:, 2] / I_eff
    ratio = np.abs(tau) / tb; sat = ratio.max(1) >= 0.98
    # 회귀
    sel = run & ~sat & (np.abs(om) < 0.3) & (vr > 0.3) & (t > T0)
    fits = {ax: fit(a_, e_, smooth_grad(e_, t)) for ax, a_, e_ in (("n", np.where(sel, a_n, np.nan), np.where(sel, e_n, np.nan)),
                                                               ("t", np.where(sel, a_t, np.nan), np.where(sel, e_t, np.nan)))}
    ft = fit(np.where(sel, alpha, np.nan), np.where(sel, e_th, np.nan), smooth_grad(np.where(sel, e_th, np.nan), t))
    fits["θ"] = ft
    # 변 0 시계열
    rows = []
    for tt in np.arange(0.0, 1.9, 0.1):
        kk = i0 + np.searchsorted(t[i0:] - T0, tt); sl = slice(kk, kk + 10)
        satw = "".join(n_ for n_, j in zip(("FL", "FR", "RL", "RR"), range(4)) if (ratio[sl, j] >= 0.98).mean() > 0.5) or "-"
        rows.append(dict(t=float(tt), v_r=float(vr[kk]), e_t=1e3 * float(e_t[sl].mean()), e_n=1e3 * float(e_n[sl].mean()),
                         e_th=float(np.degrees(e_th[sl].mean())), a_t=float(a_t[sl].mean()), a_n=float(a_n[sl].mean()),
                         alpha=float(alpha[sl].mean()), sat=100 * float(sat[sl].mean()), satw=satw))
    # SLAM 포즈 튐 — 연속 /tracked_pose 이동 vs 추정기 이동
    Pp = np.array(D["pose"]); xc, yc = cs.imu_to_center(Pp[:, 1], Pp[:, 2], Pp[:, 3], w.off)
    ex, ey = np.interp(Pp[:, 0], E[:, 0], E[:, 1]), np.interp(Pp[:, 0], E[:, 0], E[:, 2])
    dS = np.c_[np.diff(xc), np.diff(yc)]; dE = np.c_[np.diff(ex), np.diff(ey)]; jump = np.linalg.norm(dS - dE, axis=1)
    tj = Pp[1:, 0] - T0; jm = (jump > 0.03) & (tj > -0.5) & (tj < 9.5)
    out[bag] = dict(name=name, fits=fits, side0=rows, jumps=[(float(a), 1e3 * float(b)) for a, b in zip(tj[jm], jump[jm])],
                    sat_run=100 * float(sat[run & (t > T0) & (vr > 0.02)].mean()),
                    en_rms_straight=1e3 * float(np.sqrt(np.nanmean(e_n[run & (np.abs(om) < 0.3) & (vr > 0.3)] ** 2))),
                    _ts=dict(t=t - T0, e_n=e_n, a_n=a_n, e_t=e_t, a_t=a_t, sat=sat, run=run, vr=vr))

for bag, o in out.items():
    print(f"== {o['name']}  포화 (주행) {o['sat_run']:.0f} %, 직선 e_n RMS {o['en_rms_straight']:.0f} mm")
    for ax, f in o["fits"].items():
        ps = ", ".join(f"{p.real:+.2f}{p.imag:+.2f}j" if abs(p.imag) > 1e-6 else f"{p.real:+.2f}" for p in f["poles"])
        unit = "rad/s² per rad" if ax == "θ" else "m/s² per m"
        print(f"   {ax}: k_p {f['kp']:.2f} [1/s²] k_d {f['kd']:.2f} [1/s]  (R² {f['r2']:.2f}, n {f['n']})  pole {ps}")
    print("   변 0:  t  v_r | e_t e_n [mm] e_θ [°] | feedback a_t a_n [m/s²] α [rad/s²] | 포화 % (바퀴)")
    for rw in o["side0"]:
        print(f"   {rw['t']:3.1f} {rw['v_r']:.2f} | {rw['e_t']:+5.0f} {rw['e_n']:+5.0f} {rw['e_th']:+5.1f} | {rw['a_t']:+.2f} {rw['a_n']:+.2f} {rw['alpha']:+6.1f} | {rw['sat']:3.0f} {rw['satw']}")
    print("   SLAM 튐 (> 3 cm): " + (", ".join(f"t {a:.2f} s {b:.0f} mm" for a, b in o["jumps"]) or "없음"))

# 그림: 옆 오차 e_n 과 옆 feedback a_n, 진행 e_t 와 a_t (11-21 C)
for bag_f, name_f, _, _ in BAGS[:2]:
  o = out[bag_f]["_ts"]; wv = (o["t"] > -0.3) & (o["t"] < 8.0) & o["run"]
  fig, ax = plt.subplots(3, 1, figsize=(11, 8.2), sharex=True)
  ax[0].plot(o["t"][wv], 1e3 * o["e_n"][wv], color="#eb6834", lw=1.5, label="옆 오차 e_n (+ 경로 왼쪽)")
  ax[0].plot(o["t"][wv], 1e3 * o["e_t"][wv], color="#6250d6", lw=1.5, label="진행 오차 e_t (− 뒤처짐)")
  ax[0].axhline(0, color=INK2, lw=0.8); ax[0].set_ylabel("MPC 가 보는 오차 [mm]"); ax[0].legend(loc="lower left", frameon=False, fontsize=9)
  ax[1].plot(o["t"][wv], o["a_n"][wv], color="#eb6834", lw=1.2, label="옆 feedback a_n")
  ax[1].plot(o["t"][wv], o["a_t"][wv], color="#6250d6", lw=1.2, label="진행 feedback a_t")
  ax[1].axhline(0, color=INK2, lw=0.8); ax[1].set_ylabel("feedback 가속 [m/s²]\n(F(τ) − F(U_s)) / m_eff"); ax[1].legend(loc="lower left", frameon=False, fontsize=9)
  ax[2].fill_between(o["t"][wv], 0, o["sat"][wv].astype(float), step="mid", color="#a9a8a2", alpha=0.6, lw=0)
  ax[2].plot(o["t"][wv], o["vr"][wv] / 1.5, color=INK2, lw=1, ls=":", label="기준 속력 / 1.5")
  ax[2].set_ylabel("포화 (회색 = 어느 바퀴든\n|τ| ≥ 0.98 τ̄)"); ax[2].set_yticks([0, 1]); ax[2].legend(loc="upper right", frameon=False, fontsize=9)
  ax[2].set_xlabel("주행 시작 뒤 시간 [s]")
  ax[0].set_title(f"{name_f} {bag_f[-6:]} — MPC 가 보는 오차와 그에 대한 feedback (경로 좌표)", loc="left", color=INK)
  fig.tight_layout(); fig.savefig(os.path.join(HERE, "fig_feedback.png" if bag_f.endswith("183827") else f"fig_feedback_{bag_f[-6:]}.png"), dpi=130); plt.close(fig)
json.dump({b: {k_: v for k_, v in o_.items() if k_ != "_ts"} for b, o_ in out.items()}, open(os.path.join(HERE, "decomp.json"), "w"),
          ensure_ascii=False, indent=1, default=lambda z: [z.real, z.imag] if isinstance(z, complex) else str(z))
