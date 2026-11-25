# P1 트랙 그림: circles8 width 2.0 (R 0.5), 출발 = 'a' 누른 로봇 자리 (로컬 x = 앞, y = 왼쪽), 색 = 기준 속도
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.collections import LineCollection
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams["font.family"] = fm.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc").get_name()
plt.rcParams["axes.unicode_minus"] = False
sys.path.insert(0, "/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc")
from amr_mpc.core.figure8 import Figure8
f = Figure8(width=2.0, speed=0.6, accel=1.0, curve="circles", yaw_mode="tangent", laps=2, speed_mod_amp=0.5, speed_mod_cycles=2, speed_mod_phase_deg=-90)
lap = f.L / f.laps
sg = np.linspace(0, lap, 600)
P = np.array([f._point(np.interp(x, f.arc_tab, f.s_tab)) for x in sg]); V = np.array([f._v_target(x) for x in sg])
fig, ax = plt.subplots(1, 2, figsize=(14, 6.2), gridspec_kw={"width_ratios": [1.15, 1]})
A = ax[0]
seg = np.stack([P[:-1], P[1:]], 1); lc = LineCollection(seg, cmap="viridis", linewidths=5); lc.set_array(V[:-1]); A.add_collection(lc)
cb = fig.colorbar(lc, ax=A, fraction=0.04); cb.set_label("기준 속도 [m/s]")
for x, lab in ((lap*0.06, "① 왼쪽 원 (반시계)"), (lap*0.56, "② 오른쪽 원 (시계)")):
    i = np.argmin(np.abs(sg - x)); d = P[i+3] - P[i]
    A.annotate("", xy=P[i]+d*6, xytext=P[i], arrowprops=dict(arrowstyle="-|>", color="k", lw=1.6))
A.text(0.0, 1.08, "가장 빠름 0.90 m/s", ha="center"); A.text(0.0, -1.12, "가장 빠름 0.90 m/s", ha="center")
A.text(0.08, 0.06, "출발·교차점\n가장 느림 0.30 m/s", fontsize=9)
A.add_patch(plt.Rectangle((-0.13, -0.1), 0.26, 0.2, fill=False, ec="r", lw=1.5)); A.annotate("", xy=(0.25, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="r", lw=2))
A.text(-0.13, -0.2, "로봇 ('a' 누른 자리·방향)", color="r", fontsize=9, va="top")
A.text(0.55, 0.5, "가속 → 감속", fontsize=9, color="dimgray"); A.text(-0.95, 0.5, "감속 → 교차점", fontsize=9, color="dimgray")
A.set_aspect("equal"); A.set_xlim(-0.9, 0.9); A.set_ylim(-1.3, 1.3); A.grid(alpha=.3)
A.set_xlabel("로컬 x [m] (출발 때 로봇 앞)"); A.set_ylabel("로컬 y [m] (출발 때 로봇 왼쪽)")
A.set_title("circles8 width 2.0 — 지름 1.0 m 원 두 개, 2 바퀴")
B = ax[1]
t, s, v = f._lap_t, f._lap_s, f._lap_v
B.plot(s, v, "k", lw=1.5, label="실제 기준 속도 v(σ)")
for k in range(int(2*f.laps)+1): B.axvline(k*lap/2, color="r", ls=":", lw=1)
B.text(0.05, 0.95, "빨간 점선 = 교차점", color="r", transform=B.transAxes, fontsize=9)
B.set_xlabel("호길이 σ [m] (한 바퀴 6.28 m)"); B.set_ylabel("속도 [m/s]"); B.grid(alpha=.3)
B.set_title(f"속도 v = 0.6·(1 − 0.5·cos(2π·2·σ/6.28)),  주행 {f.T_path:.1f} s, 가감속 구간 8 개")
fig.tight_layout(); fig.savefig(sys.argv[1], dpi=100); print(sys.argv[1])
