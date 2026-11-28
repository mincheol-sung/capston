# 2026-11-26 구동 방향 보정 A/B 경로 (../2026-11-25_roller_ab/plot_paths_rr.py 복사) — 2 줄 (끔 · 켬) × 판, 판마다 지표.
#   ../2026-11-25_compare_paths/plot_paths.py 의 함수 (SLAM 중심 위치·기준열 보간·지표·슬립) 를 그대로 가져다 쓴다 (그 파일의 그리기 부분은 실행 안 함).
#   아래 줄: 끔·켬 3 판씩 겹친 큰 그림 + 꼭짓점 확대 (판 사이 차이가 cm 단위라 전체 그림에서 안 보인다)
#   source install/setup.bash 뒤  /usr/bin/python3 plot_paths_rr.py  → paths_dy.png
import os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "2026-11-25_compare_paths", "plot_paths.py")
src = open(SRC).read()
ns = {"__file__": SRC}
exec(compile(src[:src.index("\nfor sc in SCEN:")], SRC, "exec"), ns)
ns["BAGS"] = "/home/smoc/capston/capston/bags/2026-11-27/"
ns["HERE"] = HERE                                        # slam_cache 를 이 폴더에
ns["REAL"] = os.path.dirname(HERE)
plt, SURF, INK, INK2, GRID, CRIT = ns["plt"], ns["SURF"], ns["INK"], ns["INK2"], ns["GRID"], ns["CRIT"]
OFFC, ONC = "#8a8986", "#2a78d6"
GCOL = ["#8a8986", "#2a78d6", "#1baf7a", "#eb6834"]                          # 끔 = 회색 (예전), 켬 = 파랑
import glob as _g
B = "/home/smoc/capston/capston/bags/2026-11-27/"
EXCLUDE = set()
_pick = lambda key: sorted(os.path.basename(b) for b in _g.glob(B + f"*_vs_{key}_r*") if not b.endswith(".tmp") and os.path.basename(b) not in EXCLUDE)
import sys as _s
KEYS = _s.argv[1:] or ["w", "e"]          # 예: plot_paths_dy.py on a b → 세 묶음
NAMES = {"w": "바퀴 속도 (w)", "e": "추정기 속도 (e, 기본)"}
G = {NAMES[k]: _pick(k)[:3] for k in KEYS if _pick(k)}
FOLDER = os.path.basename(HERE)
data = {g: [ns["run_data"](FOLDER, lab) for lab in labs] for g, labs in G.items()}
allx = [v for g in G for d in data[g] for v in (d.tx.min(), d.tx.max(), d.px.min(), d.px.max())]
ally = [v for g in G for d in data[g] for v in (d.ty.min(), d.ty.max(), d.py.min(), d.py.max())]
pad = 0.12
xl, yl = (min(allx) - pad, max(allx) + pad), (min(ally) - pad, max(ally) + pad)

NR = len(G) + 1
fig = plt.figure(figsize=(17, 5.8 * NR + 0.5), facecolor=SURF)
gs = fig.add_gridspec(NR, 3, height_ratios=[1] * len(G) + [1.05], hspace=0.62, wspace=0.18, left=0.045, right=0.99, top=0.93, bottom=0.04)


def style(ax, xl, yl):
    ax.set_facecolor(SURF); ax.set_xlim(xl); ax.set_ylim(yl); ax.set_aspect("equal")
    ax.grid(color=GRID, lw=0.6); [s.set_color(GRID) for s in ax.spines.values()]
    ax.tick_params(colors=INK2, labelsize=8); ax.set_xlabel("x [m]", color=INK2, fontsize=9); ax.set_ylabel("y [m]", color=INK2, fontsize=9)


for r, (g, labs) in enumerate(G.items()):
    col = GCOL[r % len(GCOL)]
    for k, (lab, d) in enumerate(zip(labs, data[g])):
        ax = fig.add_subplot(gs[r, k]); style(ax, xl, yl)
        ax.plot(d.px, d.py, color=INK2, lw=1.3, ls=(0, (4, 3)), label="기준 경로", zorder=2)
        ax.plot(d.tx, d.ty, color=col, lw=1.8, label="실제 (SLAM)", zorder=3)
        ax.plot(d.bx, d.by, ls="none", marker="x", ms=3.5, mew=0.9, color=CRIT, alpha=0.6, zorder=4, label="슬립 > 50 % (10 Hz LPF)")
        ax.plot(0, 0, marker="o", ms=7, mfc=SURF, mec=INK, mew=1.4, zorder=5, label="출발 (출발 헤딩 = +x)")
        M = d.M
        ax.set_title(f"{g.split(' (')[0]} · 판 {k + 1}   {lab[:6]}", color=INK, fontsize=11.5, fontweight="bold", loc="left", pad=6)
        ax.text(0.0, -0.14, f"위치 오차 RMS {M['exy_rms'] * 100:.1f} cm · 최대 {M['exy_max'] * 100:.1f} cm\n"
                            f"경로까지 RMS {M['dpath_rms'] * 100:.1f} cm · 진행 방향 RMS {M['along_rms'] * 100:.1f} cm\n"
                            f"헤딩 오차 RMS {M['eth_rms']:.2f}°   슬립 > 50 % {d.s50:.1f} %",
                transform=ax.transAxes, va="top", ha="left", color=INK, fontsize=9.5, linespacing=1.45)
        if r == 0 and k == 0:
            h0, l0 = ax.get_legend_handles_labels()

# 아래 줄: 겹친 전체 + 꼭짓점 확대 두 곳 (첫 꼭짓점 = 출발 회전 뒤 첫 모서리, 2 m 직선 끝)
P = data[next(iter(G))][0]
corners = [(P.px[i], P.py[i]) for i in range(len(P.px))]
def nearest_corner(x, y):
    j = int(np.argmin([(cx - x) ** 2 + (cy - y) ** 2 for cx, cy in corners])); return corners[j]
views = [("전체 — 묶음별 겹침", xl, yl)]
for (tx, ty), name in (((0.0, -1.0), "첫 모서리 (0, −1) 확대"), ((1.0, 1.0), "2 m 직선 끝 (1, 1) 확대")):
    cx, cy = nearest_corner(tx, ty)
    views.append((name, (cx - 0.22, cx + 0.22), (cy - 0.22, cy + 0.22)))
for k, (name, vx, vy) in enumerate(views):
    ax = fig.add_subplot(gs[NR - 1, k]); style(ax, vx, vy)
    ax.plot(P.px, P.py, color=INK2, lw=1.3, ls=(0, (4, 3)), zorder=2)
    for r, g in enumerate(G):
        for d in data[g]:
            ax.plot(d.tx, d.ty, color=GCOL[r % len(GCOL)], lw=1.6, alpha=0.85, zorder=3 + r)
    ax.set_title(name, color=INK, fontsize=11.5, fontweight="bold", loc="left", pad=6)
    if k == 0:
        for r, g in enumerate(G):
            ax.plot([], [], color=GCOL[r % len(GCOL)], lw=1.8, label=f"{g.split(' (')[0]} ({len(G[g])} 판)")
        ax.legend(loc="lower right", fontsize=9, frameon=False)

fmtm = lambda g, key, s=100: f"{np.mean([d.M[key] * s for d in data[g]]):.1f}"
sumtxt = "   |   ".join(f"{g.split(' (')[0]}: 위치 RMS {fmtm(g, 'exy_rms')} cm · 경로까지 {fmtm(g, 'dpath_rms')} cm · 헤딩 {np.mean([d.M['eth_rms'] for d in data[g]]):.2f}° · 슬립 > 50 % {np.mean([d.s50 for d in data[g]]):.1f} %" for g in G)
fig.suptitle("MPC 속도 출처 실차 A/B — 꺾은선 · 모서리 통과 · 가속 1.5 m/s² · 최고 1.5 m/s (2026-11-27, 롤러 항·구동 방향 보정 기본 켬, 속도 출처만 다름)",
             color=INK, fontsize=14, x=0.01, ha="left", y=0.995, fontweight="bold")
fig.text(0.01, 0.972, "SLAM 위치 (로봇 중심, 메시지 시각), 각 판의 출발 포즈 기준 [m]. 지표도 SLAM 위치로. 평균 — " + sumtxt, color=INK2, fontsize=9, ha="left")
fig.legend(h0, l0, loc="upper left", bbox_to_anchor=(0.005, 0.962), ncol=len(h0), fontsize=9, frameon=False, handlelength=2.2)
out = os.path.join(HERE, "paths_vs.png")
fig.savefig(out, dpi=130, facecolor=SURF)
print(out)
