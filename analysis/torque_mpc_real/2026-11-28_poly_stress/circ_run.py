"""2026-11-28 원 8 자 (circles8) 판 분석 — 바퀴별·판 전체 추종 오차, 안쪽/바깥 치우침, 슬립, 경로 그림.
source install/setup.bash 뒤  /usr/bin/python3 circ_run.py <대상 bag> [비교 bag ...]
  bag 읽기·지표 정의는 poly_run.py 와 같다 (그 파일 앞부분을 그대로 불러 쓴다). 더한 것:
  · 안쪽 치우침 = 옆 오차 × sgn(기준 요 속도)  (+ = 로봇이 원 안쪽). 옆 + = 로봇이 기준 왼쪽 → 왼쪽으로 도는 원 (ω > 0) 에서는 왼쪽 = 안쪽.
  · 바퀴 경계 = 기준 호 길이를 laps 로 등분 (q_laps.py 와 같음). 교차점 창 = 기준 요 속도 부호가 바뀌는 시각 ±0.3 s.
  · SLAM 튐 = SLAM 중심 포즈 걸음 속도 > 3 m/s.
그림: <대상>_path.png · <대상>_time.png · circ_compare.png"""
import math
import os
import re
import sys
import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
exec(compile(open(os.path.join(HERE, "poly_run.py")).read().split("# ── 글꼴·색")[0].replace("BAGS = [p.rstrip(\"/\") for p in sys.argv[1:]]", ""), "poly_run_head", "exec"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

BAGS = [p.rstrip("/") for p in sys.argv[1:]]


def circ_args(path):
    A = args_of(path)
    a = open(os.path.join(path, "run_info", "args.txt")).read()
    g = lambda k, d="": (re.search(rf"\b{k}:=(\S+)", a) or [None, d])[1]
    A.update(speed=g("speed"), width=g("width", "3.0"), laps=float(g("laps", "1")))
    return A


def analyze(path):
    name = os.path.basename(path); A = circ_args(path); D = read(path)
    tr, R, od, arm = D["tr"], D["R"], D["od"], D["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]; t_off = td[td > t_on][0] if (td > t_on).any() else od[-1, 0]
    t = od[:, 0]; msk = (t >= tr[0]) & (t <= tr[-1]) & (t >= t_on - 0.05); t, X = t[msk], od[msk]
    xr, yr_, ub, vb, wr = (np.interp(t, tr, R[:, i]) for i in (0, 1, 3, 4, 5))
    th_r = np.interp(t, tr, np.unwrap(R[:, 2])); th = X[:, 3]
    vxr = np.cos(th_r) * ub - np.sin(th_r) * vb; vyr = np.sin(th_r) * ub + np.cos(th_r) * vb
    vx = np.cos(th) * X[:, 4] - np.sin(th) * X[:, 5]; vy = np.sin(th) * X[:, 4] + np.cos(th) * X[:, 5]
    E = np.column_stack([xr - X[:, 1], yr_ - X[:, 2], wrap(th_r - th), vxr - vx, vyr - vy, wr - X[:, 6]])
    vref = np.hypot(vxr, vyr); win = (vref > 0.05) & (t <= t_off)
    tx, ty = vxr / np.maximum(vref, 1e-6), vyr / np.maximum(vref, 1e-6)
    along = E[:, 0] * tx + E[:, 1] * ty; cross = -(E[:, 0] * -ty + E[:, 1] * tx)
    inside = cross * np.sign(wr)
    epos = np.hypot(E[:, 0], E[:, 1])
    vr_grid = np.hypot(R[:, 3], R[:, 4]); s_grid = np.concatenate([[0.0], np.cumsum(0.5 * (vr_grid[1:] + vr_grid[:-1]) * np.diff(tr))])
    laps = A["laps"]; Ltot = s_grid[-1]
    lap_of = lambda tt: np.clip(np.floor(laps * np.interp(tt, tr, s_grid) / Ltot), 0, laps - 1).astype(int) + 1
    lap_e = lap_of(t)
    # 교차점: 기준 요 속도 부호 바뀜
    wsg = np.sign(R[:, 5]); flip = tr[1:][wsg[1:] * wsg[:-1] < 0]                    # 요 속도 부호가 + ↔ − 로 바뀐 점만 (출발 0 → ± 는 뺀다)
    near_x = np.zeros_like(t, bool)
    for f in flip:
        near_x |= np.abs(t - f) < 0.3
    # SLAM 중심
    SL = D["sl"]; c, s = np.cos(SL[:, 3]), np.sin(SL[:, 3])
    sx = SL[:, 1] - (c * IMU_OFF[0] - s * IMU_OFF[1]); sy = SL[:, 2] - (s * IMU_OFF[0] + c * IMU_OFF[1])
    ks = (SL[:, 0] >= tr[0]) & (SL[:, 0] <= tr[-1]) & (SL[:, 0] >= t_on) & (SL[:, 0] <= t_off)
    ts_ = SL[ks, 0]; kk = np.interp(ts_, t, vref) > 0.05
    es = np.hypot(np.interp(ts_, tr, R[:, 0]) - sx[ks], np.interp(ts_, tr, R[:, 1]) - sy[ks])
    stepv = np.hypot(np.diff(sx[ks]), np.diff(sy[ks])) / np.maximum(np.diff(ts_), 1e-3); jumps = ts_[1:][stepv > 3.0]
    # 슬립
    S = D["slip"]; tsl = S[:, 0]; L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(tsl)))), L, axis=0)
    sp = np.hypot(np.interp(tsl, od[:, 0], od[:, 4]), np.interp(tsl, od[:, 0], od[:, 5]))
    ws = (tsl >= t_on) & (tsl <= t_off) & (sp > 0.05)
    A4 = np.abs(Lf[ws]); imax = A4.argmax(1); big = A4.max(1) > 0.5
    sgn = np.sign(Lf[ws][np.arange(len(imax)), imax])
    nx_s = np.zeros(ws.sum(), bool)
    for f in flip:
        nx_s |= np.abs(tsl[ws] - f) < 0.3
    F = D["sfb"]
    if F is not None and np.isfinite(F[:, 18]).any():
        tf = F[:, 0]; wf = (tf >= t_on) & (tf <= t_off) & (np.interp(tf, od[:, 0], np.hypot(od[:, 4], od[:, 5])) > 0.05)
        vslip = 100 * R_W * rms(F[wf, 18:22]); dtau = 1e3 * rms(F[wf, 6:10])
    else:
        vslip = dtau = np.nan
    M = D["mot"]; t_mv = t[win]; wm = (M[:, 0] >= t_mv[0]) & (M[:, 0] <= t_mv[-1])
    done = float(np.interp(t_off, tr, vr_grid)) < 0.05
    vest = np.hypot(X[:, 4], X[:, 5])
    per_lap = []
    for k in range(1, int(laps) + 1):
        e = win & (lap_e == k)
        if e.sum() > 50:
            per_lap.append(dict(lap=k, pos=100 * rms(epos[e]), cross_r=100 * rms(cross[e]), inside=100 * float(np.mean(inside[e])),
                                along=100 * float(np.mean(along[e])), head=float(np.degrees(rms(E[e, 2])))))
    ccw, cw = win & (wr > 0.3), win & (wr < -0.3)
    return dict(name=name, A=A, done=done, armed=t_off - t_on, faults=[f for tb, f in D["flt"] if tb >= t_on - 1], jumps=jumps - t_on,
                pos=100 * rms(epos[win]), pos_max=100 * float(epos[win].max()), cross_r=100 * rms(cross[win]), inside=100 * float(np.mean(inside[win])),
                inside_ccw=100 * float(np.mean(inside[ccw])), inside_cw=100 * float(np.mean(inside[cw])),
                along=100 * float(np.mean(along[win])), along_r=100 * rms(along[win]), head=float(np.degrees(rms(E[win, 2]))),
                head_max=float(np.degrees(np.abs(E[win, 2]).max())), vel=100 * rms(np.hypot(E[win, 3], E[win, 4])),
                pos_x=100 * rms(epos[win & near_x]), pos_nx=100 * rms(epos[win & ~near_x]),
                slam_pos=100 * rms(es[kk]), slam_max=100 * float(es[kk].max()),
                m50=100 * float(big.mean()), m25=100 * float(np.mean(A4.max(1) > 0.25)), p50=100 * (A4 > 0.5).mean(0), lrms=np.sqrt(np.mean(Lf[ws] ** 2, 0)),
                spin=100 * float(np.mean(sgn[big] > 0)) if big.any() else np.nan,
                big_x=100 * float(np.mean(nx_s[big])) if big.any() else np.nan, x_share=100 * float(nx_s.mean()),
                vslip=vslip, dtau=dtau, dtick=1e3 * rms(np.diff(M[wm, 1:5], axis=0)),
                vmax=float(vest[win].max()), vmean=float(vest[win & (vref > 0.9 * vref.max())].mean()), vref_max=float(vref.max()), per_lap=per_lap,
                _t=t - t_on, _win=win, _X=X, _epos=epos, _inside=inside, _along=along, _vref=vref, _ts=ts_ - t_on, _es=es, _sx=sx[ks], _sy=sy[ks],
                _tl=tsl - t_on, _Lf=Lf, _ws=ws, _R=R, _big_t=tsl[ws][big] - t_on, _flip=flip - t_on, _vest=vest, _wr=wr)


def show(r):
    A = r["A"]
    print(f"{r['name']}: circles8 width {A['width']} · speed {A['speed']} · accel {A['accel']} · laps {A['laps']:g} · W {A['W']} · 되먹임 {A['sfb']}"
          f" · 무장 {r['armed']:.1f} s · 끝까지 {'예' if r['done'] else '아니오'} · fault {r['faults'] or '없음'} · SLAM 튐 {len(r['jumps'])}"
          + (" @ " + ", ".join(f"{j:.1f}" for j in r['jumps']) + " s" if len(r['jumps']) else ""))
    print(f"  추정기: 위치 RMS {r['pos']:.2f} cm (최대 {r['pos_max']:.1f}) · 옆 RMS {r['cross_r']:.2f} · 안쪽 치우침 평균 {r['inside']:+.2f} (왼쪽 원 {r['inside_ccw']:+.2f} / 오른쪽 원 {r['inside_cw']:+.2f})"
          f" · 진행 평균 {r['along']:+.2f} (RMS {r['along_r']:.2f}) · 헤딩 RMS {r['head']:.2f}° (최대 {r['head_max']:.1f}) · 속도 RMS {r['vel']:.1f} cm/s")
    print(f"         교차점 ±0.3 s 위치 RMS {r['pos_x']:.2f} / 나머지 {r['pos_nx']:.2f} cm · 기준 {r['vref_max']:.3f} m/s 에 추정기 평균 {r['vmean']:.3f} · 최고 {r['vmax']:.2f} m/s")
    print(f"  SLAM 중심: 위치 RMS {r['slam_pos']:.2f} cm (최대 {r['slam_max']:.1f})")
    for L in r["per_lap"]:
        print(f"    바퀴 {L['lap']}: 위치 {L['pos']:.2f} · 옆 RMS {L['cross_r']:.2f} · 안쪽 {L['inside']:+.2f} · 진행 {L['along']:+.2f} cm · 헤딩 {L['head']:.2f}°")
    print(f"  슬립: 큰 슬립 (>0.5) {r['m50']:.1f} % · 중간 (>0.25) {r['m25']:.1f} % · 바퀴별 >0.5 " + " ".join(f"{w} {p:.1f}" for w, p in zip(WH, r['p50']))
          + " · λ RMS " + " ".join(f"{w} {x:.2f}" for w, x in zip(WH, r['lrms'])))
    print(f"        큰 슬립 중 헛돎 {r['spin']:.0f} % / 끌림 {100 - r['spin']:.0f} % · 큰 슬립의 {r['big_x']:.0f} % 가 교차점 ±0.3 s (시간 몫 {r['x_share']:.0f} %)"
          f" · 미끄럼 속도 RMS {r['vslip']:.2f} cm/s · 보정 토크 RMS {r['dtau']:.1f} mN·m · 모터 명령 떨림 {r['dtick']:.1f} mN·m")


fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
if fp:
    font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
C_REF, C_EST, C_SLAM, C_BIG = "#8c959f", "#0969da", "#1a7f37", "#cf222e"
WC = ["#0969da", "#bf8700", "#8250df", "#cf222e"]
plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})

res = []
for p in BAGS:
    r = analyze(p); show(r); res.append(r)
T = res[0]; tag = T["name"].split("_")[0]; A = T["A"]

fig, axs = plt.subplots(1, 2, figsize=(14, 6.4))
ax = axs[0]; w = T["_win"]
ax.plot(T["_R"][:, 0], T["_R"][:, 1], color=C_REF, lw=6, alpha=0.35, solid_capstyle="round", label="기준 경로")
ax.plot(T["_X"][w, 1], T["_X"][w, 2], color=C_EST, lw=1.0, label="추정기 (3 바퀴)")
ax.plot(T["_sx"], T["_sy"], color=C_SLAM, lw=0.8, ls="--", label="SLAM (로봇 중심)")
bt = T["_big_t"]
if len(bt):
    ax.scatter(np.interp(bt, T["_t"], T["_X"][:, 1]), np.interp(bt, T["_t"], T["_X"][:, 2]), s=8, color=C_BIG, zorder=5,
               label=f"큰 슬립 (네 바퀴 중 최대 |λ| > 0.5) {T['m50']:.0f} %")
ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.5); ax.set_xlabel("x [m] (map)"); ax.set_ylabel("y [m] (map)")
ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.09), fontsize=8.5, frameon=False, ncol=2)
ax.set_title(f"{T['name']} 경로 — 위치 RMS 추정기 {T['pos']:.1f} cm · SLAM {T['slam_pos']:.1f} cm", loc="left", fontsize=10.5, color=INK)
ax = axs[1]
sc = ax.scatter(T["_X"][w, 1], T["_X"][w, 2], c=100 * T["_inside"][w], s=4, cmap="coolwarm", vmin=-10, vmax=10)
ax.plot(T["_R"][:, 0], T["_R"][:, 1], color=C_REF, lw=0.8)
cb = fig.colorbar(sc, ax=ax, shrink=0.8); cb.set_label("안쪽 치우침 [cm] (+ 원 안쪽, − 바깥)")
ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.5); ax.set_xlabel("x [m]")
ax.set_title(f"안쪽/바깥 치우침 — 왼쪽으로 도는 원 {T['inside_ccw']:+.1f} cm · 오른쪽으로 도는 원 {T['inside_cw']:+.1f} cm", loc="left", fontsize=10.5, color=INK)
fig.text(0.01, 0.995, f"circles8 width {A['width']} (반지름 0.75 m) · speed {A['speed']} m/s · accel {A['accel']} m/s² · laps {A['laps']:g} · friction · W [{A['W']}] · 슬립 되먹임 {A['sfb']}",
         fontsize=10, color=INK2, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(os.path.join(HERE, f"{tag}_path.png"), dpi=130); plt.close(fig)

fig, axs = plt.subplots(4, 1, figsize=(12, 9.5), sharex=True)
tt = T["_t"]; w = T["_win"]
for ax in axs:
    for f in T["_flip"]:
        ax.axvspan(f - 0.3, f + 0.3, color="#eaeef2", lw=0)
    ax.grid(color=GRID, lw=0.5)
ax = axs[0]
ax.plot(tt[w], T["_vref"][w], color=C_REF, lw=2, label="기준 속력")
ax.plot(tt[w], T["_vest"][w], color=C_EST, lw=1, label="추정기 속력")
ax.set_ylabel("속력 [m/s]"); ax.legend(fontsize=8.5, frameon=False, loc="lower right", ncol=2)
ax.set_title(f"{T['name']} — 회색 띠 = 교차점 (기준 요 속도 부호가 바뀌는 시각 ±0.3 s)", loc="left", fontsize=10.5, color=INK)
ax = axs[1]
ax.plot(tt[w], 100 * T["_epos"][w], color=C_EST, lw=1.2, label=f"추정기 |오차| (RMS {T['pos']:.1f})")
ax.plot(T["_ts"], 100 * T["_es"], color=C_SLAM, lw=1.0, ls="--", label=f"SLAM 중심 |오차| (RMS {T['slam_pos']:.1f})")
ax.set_ylabel("위치 오차 [cm]"); ax.legend(fontsize=8.5, frameon=False, loc="upper left", ncol=2)
ax = axs[2]
ax.plot(tt[w], 100 * T["_inside"][w], color="#8250df", lw=1.1, label="안쪽 치우침 (+ 원 안쪽)")
ax.plot(tt[w], 100 * T["_along"][w], color="#bf8700", lw=1.1, label="진행 (+ 뒤처짐)")
ax.axhline(0, color=INK2, lw=0.6); ax.set_ylabel("경로 좌표 오차 [cm]"); ax.legend(fontsize=8.5, frameon=False, loc="upper left", ncol=2)
ax = axs[3]; tl = T["_tl"]; wsl = T["_ws"]
for i in range(4):
    ax.plot(tl[wsl], T["_Lf"][wsl, i], color=WC[i], lw=0.8, label=f"{WH[i]} (>0.5 {T['p50'][i]:.0f} %)")
for y in (0.5, -0.5):
    ax.axhline(y, color=C_BIG, lw=0.7, ls=":")
ax.axhline(0, color=INK2, lw=0.5); ax.set_ylim(-2.05, 2.05)
ax.set_ylabel("λ, 10 Hz LPF (+ 헛돎, − 끌림)"); ax.set_xlabel("무장 뒤 시간 [s]")
ax.legend(fontsize=8.5, frameon=False, loc="upper left", ncol=4)
ax.set_xlim(tt[w][0] - 0.5, tt[w][-1] + 0.5)
fig.tight_layout(); fig.savefig(os.path.join(HERE, f"{tag}_time.png"), dpi=130); plt.close(fig)

if len(res) > 1:
    keys = [("pos", "위치 RMS 추정기 [cm]"), ("slam_pos", "위치 RMS SLAM 중심 [cm]"), ("inside", "안쪽 치우침 평균 [cm] (+ 안쪽)"),
            ("head", "헤딩 RMS [°]"), ("m50", "큰 슬립 (>0.5) [%]"), ("vslip", "미끄럼 속도 RMS [cm/s]")]
    fig, axs = plt.subplots(2, 3, figsize=(14, 6.8))
    labs = [f"{r['name'].split('_')[0]}\nW{r['A']['W'].split(',')[0]}" for r in res]
    cols = [C_BIG if i == 0 else ("#54aeff" if float(r["A"]["speed"]) < 0.9 else C_EST) for i, r in enumerate(res)]
    for ax, (k, ttl) in zip(axs.ravel(), keys):
        v = [r[k] for r in res]
        ax.bar(range(len(res)), [0 if not np.isfinite(x) else x for x in v], color=cols, width=0.65)
        for i, x in enumerate(v):
            ax.text(i, (x if np.isfinite(x) else 0), "-" if not np.isfinite(x) else f"{x:.1f}", ha="center", va="bottom" if (not np.isfinite(x) or x >= 0) else "top", fontsize=7.5, color=INK2)
        ax.axhline(0, color=INK2, lw=0.5)
        ax.set_xticks(range(len(res))); ax.set_xticklabels(labs, fontsize=7.5); ax.set_title(ttl, loc="left", fontsize=10, color=INK)
        ax.grid(axis="y", color=GRID, lw=0.5)
    fig.text(0.01, 0.995, "원 8 자 비교 (모두 friction · 슬립 되먹임 lqr 1.0) — 빨강 = 오늘 R 0.75 m · 1.23 m/s · 가감속 1.5 (구심 2.02 m/s²) · 3 바퀴,\n진한 파랑 = 11-27 R 0.5 · 1.0 m/s · 가감속 1.0 (구심 2.00) · 1 바퀴, 옅은 파랑 = 11-27 R 0.5 · 0.8 m/s · 가감속 1.0 (구심 1.28) · 3 바퀴",
             fontsize=10, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(os.path.join(HERE, "circ_compare.png"), dpi=130); plt.close(fig)
print("그림:", ", ".join(os.path.join(HERE, f) for f in (f"{tag}_path.png", f"{tag}_time.png") + (("circ_compare.png",) if len(res) > 1 else ())))
