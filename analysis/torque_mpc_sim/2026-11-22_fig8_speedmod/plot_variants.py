# speed_mod_shape × speed_mod_phase_deg 조합별 트랙 그림 (circles8 width 2.0, V 0.6, ε 0.5, n 2, accel 1.0, 2 바퀴)
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.collections import LineCollection
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(FONT); plt.rcParams["font.family"] = fm.FontProperties(fname=FONT).get_name(); plt.rcParams["axes.unicode_minus"] = False
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core.figure8 import Figure8
combos = [("sin", 0.0), ("sin", -90.0), ("square", 0.0), ("square", -90.0)]
fig, ax = plt.subplots(len(combos), 2, figsize=(14, 5.2 * len(combos)), gridspec_kw={"width_ratios": [1, 1.4]})
for r, (shape, ph) in enumerate(combos):
    f = Figure8(width=2.0, speed=0.6, accel=1.0, curve="circles", yaw_mode="tangent", laps=2, speed_mod_amp=0.5,
                speed_mod_cycles=2, speed_mod_phase_deg=ph, speed_mod_shape=shape)
    lap = f.L / f.laps
    s, v, t = f._lap_s, f._lap_v, f._lap_t
    a = np.gradient(v, t)
    k = (s <= lap)                                        # 첫 바퀴만 경로에 칠한다 (둘째 바퀴도 같다)
    idx = np.linspace(0, k.sum() - 1, 700).astype(int)
    sg, vg = s[k][idx], v[k][idx]
    P = np.array([f._point(np.interp(x, f.arc_tab, f.s_tab)) for x in sg])
    A = ax[r, 0]
    lc = LineCollection(np.stack([P[:-1], P[1:]], 1), cmap="viridis", linewidths=5, norm=plt.Normalize(0.0, 0.95))
    lc.set_array(vg[:-1]); A.add_collection(lc)
    fig.colorbar(lc, ax=A, fraction=0.045).set_label("기준 속도 [m/s] (첫 바퀴)")
    A.add_patch(plt.Rectangle((-0.13, -0.1), 0.26, 0.2, fill=False, ec="r", lw=1.5))
    A.annotate("", xy=(0.25, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="r", lw=2))
    for x in (lap * 0.06, lap * 0.56):
        i = np.argmin(np.abs(sg - x)); d = P[min(i + 3, len(P) - 1)] - P[i]
        A.annotate("", xy=P[i] + d * 6, xytext=P[i], arrowprops=dict(arrowstyle="-|>", color="k", lw=1.5))
    vc = v[np.argmin(np.abs(s - lap / 2))]
    A.text(0.06, 0.08, f"교차점 {vc:.2f} m/s", fontsize=9)
    A.set_aspect("equal"); A.set_xlim(-0.9, 0.9); A.set_ylim(-1.25, 1.25); A.grid(alpha=.3)
    A.set_xlabel("로컬 x [m] (출발 때 로봇 앞)"); A.set_ylabel("로컬 y [m] (왼쪽)")
    A.set_title(f"speed_mod_shape:={shape}   speed_mod_phase_deg:={ph:g}")
    B = ax[r, 1]
    B.plot(s, v, "k", lw=1.6, label="기준 속도 v")
    B2 = B.twinx(); B2.plot(s, a, color="tab:orange", lw=1, alpha=0.8, label="접선 가속 dv/dt"); B2.set_ylim(-1.6, 1.6)
    B2.set_ylabel("접선 가속 [m/s²]", color="tab:orange")
    for j in range(int(2 * f.laps) + 1): B.axvline(j * lap / 2, color="r", ls=":", lw=1)
    B.set_ylim(-0.02, 1.0); B.grid(alpha=.3); B.set_xlabel("호길이 σ [m] (한 바퀴 6.28 m)"); B.set_ylabel("속도 [m/s]")
    cru = (t > 1.0) & (t < f.T_path - 1.0)
    B.set_title(f"주행 {f.T_path:.1f} s · 순항 중 |dv/dt| 최대 {np.abs(a[cru]).max():.2f} m/s² · 빨간 점선 = 교차점")
    if r == 0:
        h1, l1 = B.get_legend_handles_labels(); h2, l2 = B2.get_legend_handles_labels(); B.legend(h1 + h2, l1 + l2, fontsize=8, loc="lower right")
fig.suptitle("circles8 width 2.0 · speed 0.6 · speed_mod_amp 0.5 · speed_mod_cycles 2 · accel 1.0 · laps 2", fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.985)); fig.savefig(sys.argv[1], dpi=90); print(sys.argv[1])
