#!/usr/bin/python3
"""2026-11-23 자동 실차 17 판 (real_run.sh, 15:56~16:38) 궤적 — 판마다 한 칸, 겹치지 않게. → fig_all_traj.png
각 칸: 그 판 기준의 출발 포즈 좌표로 옮긴 기준 (점선) 과 SLAM 로봇 중심 궤적 (compare.series 와 같은 구간 = 기준 활성),
빨간 점 = 큰 슬립 (max_i |λ_i| > 0.5, 기준 이동 중). 제목: 시간 순 번호 · 폴더 · 설정 / 위치 RMS·최대 · yaw RMS · 큰 슬립 %.
163807_f_c020b 는 배터리 방전으로 녹화만 시작된 빈 판이라 뺐다."""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "..", "2026-11-23_max_compare"))
import compare as cm
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 8.5, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#52514e", "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
                     "axes.unicode_minus": False})
INK, LIM = "#0b0b0b", "#d03b3b"
COL = {"C": "#1baf7a", "PP": "#2a78d6", "nod": "#eb6834", "nd": "#8a5cc7", "fC": "#0e6b4a", "fPP": "#1a4f99"}
RUNS = [("155643_h010", "C h 0.10", "C"), ("155956_h015", "C h 0.15", "C"), ("160137_h000", "C h 0 (하중 이동 끔)", "C"),
        ("160416_h020b", "C h 0.20", "C"), ("160536_h015b", "C h 0.15", "C"), ("160715_h025", "C h 0.25", "C"),
        ("160843_h000c", "C h 0 (하중 이동 끔)", "C"), ("161024_h010b", "C h 0.10", "C"), ("161137_h025b", "C h 0.25", "C"),
        ("161250_h020c", "C h 0.20", "C"), ("161536_h015c", "C h 0.15", "C"), ("161649_h020nd", "C h 0.20 상한 고정", "nd"),
        ("161802_ppb", "A 퓨어 퍼슛", "PP"), ("162745_h020nod", "C h 0.20 $\\hat{d}$ 끔", "nod"), ("162858_h020nodb", "C h 0.20 $\\hat{d}$ 끔", "nod"),
        ("163519_f_c020", "C h 0.20 · 마찰 안 기준", "fC"), ("163644_f_pp", "A 퓨어 퍼슛 · 마찰 안 기준", "fPP")]
NPZ = os.path.join(HERE, "all_traj.npz")
if os.path.exists(NPZ):
    z = dict(np.load(NPZ, allow_pickle=True))
else:
    z = {}
    for bag, _, _ in RUNS:
        o = cm.series(bag); r = cm.metrics(o, "fig8")
        z.update({f"{bag}_x": o["x"], f"{bag}_y": o["y"], f"{bag}_ref": o["ref"], f"{bag}_lam": o["lam"], f"{bag}_mv": o["moving"],
                  f"{bag}_m": np.array([r["e_rms"], r["e_max"], r["h_rms"], r["big"]])})
    np.savez(NPZ, **z)


def local(bag, x, y):
    r0 = z[f"{bag}_ref"][0]; c0, s0 = np.cos(r0[2]), np.sin(r0[2]); dx, dy = x - r0[0], y - r0[1]
    return c0 * dx + s0 * dy, -s0 * dx + c0 * dy


nc, nr = 5, 4
fig, ax = plt.subplots(nr, nc, figsize=(16.5, 5.6 * nr), gridspec_kw=dict(wspace=0.12, hspace=0.22))
fig.subplots_adjust(top=0.955, bottom=0.03, left=0.05, right=0.99)
P = []
for bag, _, _ in RUNS:
    P.append(local(bag, z[f"{bag}_x"], z[f"{bag}_y"])); P.append(local(bag, z[f"{bag}_ref"][:, 0], z[f"{bag}_ref"][:, 1]))
xs = np.concatenate([p[0] for p in P]); ys = np.concatenate([p[1] for p in P])
xl = (xs.min() - 0.05, xs.max() + 0.05); yl = (ys.min() - 0.05, ys.max() + 0.05)
for k, (bag, name, kind) in enumerate(RUNS):
    a = ax.flat[k]
    rx, ry = local(bag, z[f"{bag}_ref"][:, 0], z[f"{bag}_ref"][:, 1]); lx, ly = local(bag, z[f"{bag}_x"], z[f"{bag}_y"])
    a.plot(rx, ry, color=INK, lw=0.8, ls="--")
    a.plot(lx, ly, color=COL[kind], lw=1.3)
    big = (np.abs(z[f"{bag}_lam"]).max(1) > 0.5) & z[f"{bag}_mv"]
    a.plot(lx[big], ly[big], ".", color=LIM, ms=2.6)
    a.plot(lx[0], ly[0], "o", color=INK, ms=3.5)
    e_rms, e_max, h_rms, bg = z[f"{bag}_m"]
    a.set_title(f"{k + 1}. {bag}  {name}\nRMS {e_rms:.0f} · 최대 {e_max:.0f} mm · yaw {h_rms:.1f}° · 큰 슬립 {bg:.0f} %",
                loc="left", fontsize=8.3, color=INK)
    a.set_aspect("equal"); a.set_xlim(*xl); a.set_ylim(*yl)
    if k % nc == 0:
        a.set_ylabel("왼쪽 [m]")
    if k >= len(RUNS) - nc:
        a.set_xlabel("출발 헤딩 방향 [m]")
for a in ax.flat[len(RUNS):]:
    a.axis("off")
note = ax.flat[len(RUNS)]
note.text(0.0, 0.95, "읽는 법\n"
          "· 점선 = 기준, 실선 = SLAM 로봇 중심\n· 빨간 점 = 큰 슬립 (max_i |λ_i| > 0.5)\n· 검은 점 = 출발\n"
          "· 좌표 = 판마다 기준의 출발 포즈 (위치·헤딩)\n\n"
          "기준\n· 1~15: speed 0.69 · accel 3.0 계단 변조\n   (0.35 ↔ 1.04 m/s, 마찰 밖 — 앞먹임 최대 3.6 × τ̄)\n"
          "· 16·17: speed 0.55 · accel 0.6 계단 변조\n   (0.28 ↔ 0.83 m/s, 마찰 안 — 0.997 × τ̄, h 0.2)\n\n"
          "색: 초록 C · 파랑 PP · 주황 $\\hat{d}$ 끔\n      보라 상한 고정 (가중 배분만)\n      진한 초록·진한 파랑 = 마찰 안 기준",
          transform=note.transAxes, va="top", fontsize=8.5, color=INK, linespacing=1.45)
fig.suptitle("실차 2026-11-23 자동 17 판 (real_run.sh, 15:56~16:38) — circles8 지름 0.8 m · 2 바퀴, 모터 38~41 °C",
             x=0.05, y=0.985, ha="left", fontsize=12, color=INK)
fig.savefig(os.path.join(HERE, "fig_all_traj.png"), dpi=120, bbox_inches="tight"); print("saved", xl, yl)
