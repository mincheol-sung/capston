#!/usr/bin/python3
"""경로 위 슬립 지도 (2026-11-27): 8자·꺾은선·지그재그 × 고정 PP·어댑티브 PP·MPC, 11-26 저녁 비교 판 (각 2~3 판).
  슬립률 λ = slip_logger (추정기 속도 기준, 분모 아래 한계 2.5 rad/s), 바퀴별 10 Hz filtfilt → |λ̃|.
  바퀴 속도 max(|ω|, |ω_nom|) < 10 rad/s (0.4 m/s) 인 바퀴는 뺀다 (잡음 바닥이 25 % 를 넘는 저속, 2026-11-23_slip_lpf 5 절).
  시각마다 남은 바퀴 중 가장 큰 |λ̃| → 그 시각의 '기준 궤적 위치' (같은 시각의 기준점, 무장 순간 기준 자세로 맞춘 좌표) 에 쌓는다.
  기준 경로를 10 cm 칸으로 나눠 칸마다 '그 칸을 지난 시간 중 25 % (또는 50 %) 를 넘은 비율' 을 색으로.
  구간 종류 (기준 궤적): 저속 (< 0.4 m/s) · 방향 전환 (|요 각속도| > 0.5 rad/s) · 가속 (> 0.3 m/s²) · 감속 · 등속.
  nice -n 19 /usr/bin/python3 analysis/torque_mpc_real/2026-11-27_slip_map/slip_map.py   → fig_slip25.png, fig_slip50.png, out_sections.md"""
import math, os
import numpy as np, rosbag2_py
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(H, "../../.."))
B = os.path.join(ROOT, "bags/2026-11-26")
RUNS = {   # (경로, 제어기) → bag (30 s 아래로 멈춘 판·velocity_source wheels 판 뺌)
    ("8자 (circles8 1.0 m/s)", "고정 PP"): ["185321_s2_pp_r1", "185437_s2_pp_r2", "185532_s2_pp_r3"],
    ("8자 (circles8 1.0 m/s)", "어댑티브 PP"): ["185636_s2_adpp_r1", "185736_s2_adpp_r2", "185834_s2_adpp_r3"],
    ("8자 (circles8 1.0 m/s)", "MPC"): ["184937_s2_mpc_r2", "185209_s2_mpc_r3", "185937_s2_mpc_r4"],
    ("꺾은선 (polyline 1.5 m/s, 꼭짓점 지나감)", "고정 PP"): ["201404_s2pl_pp_r1", "201704_s2pl_pp_r2", "202049_s2pl_pp_r3"],
    ("꺾은선 (polyline 1.5 m/s, 꼭짓점 지나감)", "어댑티브 PP"): ["201501_s2pl_adpp_r1", "201804_s2pl_adpp_r2", "202150_s2pl_adpp_r3"],
    ("꺾은선 (polyline 1.5 m/s, 꼭짓점 지나감)", "MPC"): ["201259_s2pl_mpc_r1", "201604_s2pl_mpc_r2", "201944_s2pl_mpc_r3"],
    ("지그재그 (zigzag 30°·9 변, 꼭짓점 지나감)", "고정 PP"): ["190718_s2zz_pp_r1", "195945_s2zz_pp_r2", "200815_s2zz_pp_r3"],
    ("지그재그 (zigzag 30°·9 변, 꼭짓점 지나감)", "어댑티브 PP"): ["200337_s2zz_adpp_r2", "200952_s2zz_adpp_r3"],
    ("지그재그 (zigzag 30°·9 변, 꼭짓점 지나감)", "MPC"): ["190531_s2zz_mpc_r1", "191148_s2zz_mpc_r2", "200634_s2zz_mpc_r3"],
}
SHAPES = list(dict.fromkeys(k[0] for k in RUNS)); CTRLS = ["고정 PP", "어댑티브 PP", "MPC"]
W_GATE, FC, BIN = 10.0, 10.0, 0.10
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9


def load(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.join(B, bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/slip_logger/slip", "/wheel_states", "/dra_mppi/reference_trajectory", "/mpc/armed"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    SL, WS, REF, ARM = [], {}, {}, []
    while r.has_next():
        n, d, tb = r.read_next(); m = deserialize_message(d, ty[n])
        if n == "/slip_logger/slip" and len(m.data) >= 9:
            SL.append(m.data[:9])
        elif n == "/wheel_states":
            WS[round(st(m.header), 6)] = list(m.velocity)
        elif n == "/dra_mppi/reference_trajectory":
            t0 = st(m.header)
            for k, q in enumerate(m.references):                     # 뒤에 온 메시지가 덮어쓴다 (최신 기준)
                REF[round(t0 + k * m.dt, 3)] = (q.x, q.y, q.yaw, q.vx, q.vy, q.yaw_rate)
        else:
            ARM.append((tb * 1e-9, m.data))
    SL = np.array(SL)
    om = np.array([WS.get(round(t, 6), [np.nan] * 4) for t in SL[:, 0]])
    rt = np.array(sorted(REF)); R = np.array([REF[t] for t in rt])
    on = [t for t, a in ARM if a]; off = [t for t, a in ARM if not a and on and t > on[0]]
    return SL, om, rt, R, on[0], (off[0] if off else SL[-1, 0])


def samples(bag):
    SL, om, rt, R, ta, tz = load(bag)
    t = SL[:, 0]; lam = SL[:, 1:5]; wn = SL[:, 5:9]
    good = np.isfinite(lam) & np.isfinite(om)
    fs = 1 / np.median(np.diff(t)); b, a = butter(1, FC / (fs / 2))
    lf = filtfilt(b, a, np.where(good, lam, 0.0), axis=0)
    gate = good & (np.maximum(np.abs(np.nan_to_num(om)), np.abs(wn)) >= W_GATE)
    w = (t >= ta) & (t <= tz)
    # 무장 순간의 기준 자세로 맞춘 좌표
    x0, y0, p0 = (np.interp(ta, rt, R[:, i]) for i in (0, 1, 2))
    c, s = math.cos(p0), math.sin(p0)
    gx, gy = np.interp(t, rt, R[:, 0]) - x0, np.interp(t, rt, R[:, 1]) - y0
    lx, ly = c * gx + s * gy, -s * gx + c * gy
    sp = np.hypot(np.interp(t, rt, R[:, 3]), np.interp(t, rt, R[:, 4])); yr = np.abs(np.interp(t, rt, R[:, 5]))
    acc = np.gradient(sp, t); acc = np.convolve(acc, np.ones(21) / 21, mode="same")
    kind = np.where(sp < 0.4, 0, np.where(yr > 0.5, 1, np.where(acc > 0.3, 2, np.where(acc < -0.3, 3, 4))))
    al = np.where(gate, np.abs(lf), -1.0); j = al.argmax(1)
    mx = al[np.arange(len(t)), j]; sg = np.sign(lf[np.arange(len(t)), j])
    # 기준 경로 (그림용): 무장 구간 기준점, 맞춘 좌표
    m2 = (rt >= ta) & (rt <= tz)
    pg = np.c_[c * (R[m2, 0] - x0) + s * (R[m2, 1] - y0), -s * (R[m2, 0] - x0) + c * (R[m2, 1] - y0)]
    return dict(x=lx[w], y=ly[w], mx=mx[w], sg=sg[w], kind=kind[w], path=pg)


def resample(P, step):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])]


KINDS = ["저속", "방향 전환", "가속", "감속", "등속"]
res, geo = {}, {}
for (shape, ctrl), bags in RUNS.items():
    for bag in bags:
        S = samples(bag)
        if shape not in geo:
            geo[shape] = resample(S["path"], BIN)
        V = geo[shape]
        idx = np.empty(len(S["x"]), int)
        for k0 in range(0, len(idx), 2000):
            P = np.c_[S["x"][k0:k0 + 2000], S["y"][k0:k0 + 2000]]
            idx[k0:k0 + 2000] = ((P[:, None, :] - V[None]) ** 2).sum(2).argmin(1)
        res.setdefault((shape, ctrl), []).append((idx, S["mx"], S["sg"], S["kind"]))
        print(bag, len(idx), f"25 % 넘음 {100 * np.mean(S['mx'][S['mx'] >= 0] > 0.25):.1f} %", flush=True)

blues = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]
cmap = ListedColormap(blues); bounds = np.linspace(0, 0.6, len(blues)); norm = BoundaryNorm(bounds, cmap.N, extend="max")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.edgecolor": "#c3c2b7", "axes.labelcolor": "#52514e",
                     "xtick.color": "#52514e", "ytick.color": "#52514e", "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})
out = ["# 경로 구간별 슬립 (11-26 저녁 비교 판, 바퀴 속도 0.4 m/s 아래 뺌)\n"]
for thr, fn in ((0.25, "fig_slip25.png"), (0.5, "fig_slip50.png")):
    fig, axes = plt.subplots(3, 3, figsize=(15, 14), constrained_layout=True)
    for r_, shape in enumerate(SHAPES):
        V = geo[shape]
        for c_, ctrl in enumerate(CTRLS):
            ax = axes[r_, c_]; runs = res[(shape, ctrl)]
            n = np.zeros(len(V)); hit = np.zeros(len(V)); low = np.zeros(len(V))
            for idx, mx, sg, kind in runs:
                np.add.at(n, idx[mx >= 0], 1); np.add.at(hit, idx[mx > thr], 1); np.add.at(low, idx[mx < 0], 1)
            frac = np.where(n >= 5, hit / np.maximum(n, 1), np.nan)
            seg = np.stack([V[:-1], V[1:]], 1)
            ok = np.isfinite(frac[:-1])
            ax.plot(V[:, 0], V[:, 1], color="#c3c2b7", lw=1.2, ls=(0, (2, 2)), zorder=1)            # 저속·표본 적음 = 회색 점선
            lc = LineCollection(seg[ok], colors=cmap(norm(np.clip(frac[:-1][ok], 0, 0.6))), linewidths=5, capstyle="round", zorder=2)
            ax.add_collection(lc)
            ax.plot(*V[0], "o", ms=9, mfc="#fcfcfb", mec="#0b0b0b", mew=1.5, zorder=3)
            ax.annotate("", V[3], V[0], arrowprops=dict(arrowstyle="-|>,head_width=0.4,head_length=0.8", color="#0b0b0b", lw=2), zorder=4)
            per = [100 * np.mean(mx[mx >= 0] > thr) for idx, mx, sg, kind in runs]
            ax.set_title(f"{ctrl} · {len(runs)}판 · {int(thr * 100)} % 넘은 시간 중앙값 {np.median(per):.1f} %", fontsize=11, color="#0b0b0b", loc="left")
            ax.set_aspect("equal"); ax.grid(color="#e8e7e2", lw=0.6); ax.tick_params(labelsize=8)
            if c_ == 0:
                ax.set_ylabel(f"{shape}\n\ny [m] (무장 자세 기준)", fontsize=11, color="#0b0b0b")
            if r_ == 2:
                ax.set_xlabel("x [m]", fontsize=9)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    cb = fig.colorbar(sm, ax=axes, shrink=0.5, pad=0.01, extend="max", format=lambda v, _: f"{v * 100:.0f} %")
    cb.set_label(f"그 10 cm 칸을 지난 시간 중 슬립률 {int(thr * 100)} % 를 넘은 비율 (바퀴 4 개 중 최대, 10 Hz LPF)", color="#52514e")
    fig.suptitle(f"경로 위 슬립 — 슬립률 {int(thr * 100)} % 넘음 (진할수록 자주)   ○ 출발 · 화살표 진행 방향 · 회색 점선 = 저속 (0.4 m/s 아래, 잡음이 커서 뺌)",
                 fontsize=13, color="#0b0b0b", x=0.01, ha="left")
    fig.savefig(os.path.join(H, fn), dpi=110); plt.close(fig)
    # 구간 종류별 표
    out += [f"\n## 슬립률 {int(thr * 100)} % 넘은 시간 비율 — 구간 종류별 (판별 중앙값, 헛돎 몫은 넘은 것 중 구동 쪽)\n",
            "| 경로 | 제어기 | " + " | ".join(KINDS[1:]) + " | 전체 |", "|---|---|" + "---|" * len(KINDS)]
    for shape in SHAPES:
        for ctrl in CTRLS:
            cells = []
            for kk in range(1, len(KINDS)):
                v = [100 * np.mean(mx[(mx >= 0) & (kind == kk)] > thr) for idx, mx, sg, kind in res[(shape, ctrl)] if ((mx >= 0) & (kind == kk)).sum() > 20]
                sp_ = [100 * np.mean(sg[(mx > thr) & (kind == kk)] > 0) for idx, mx, sg, kind in res[(shape, ctrl)] if ((mx > thr) & (kind == kk)).sum() > 5]
                cells.append(f"{np.median(v):.1f} (헛돎 {np.median(sp_):.0f})" if v and sp_ else (f"{np.median(v):.1f}" if v else "-"))
            tot = [100 * np.mean(mx[mx >= 0] > thr) for idx, mx, sg, kind in res[(shape, ctrl)]]
            out.append(f"| {shape.split(' ')[0]} | {ctrl} | " + " | ".join(cells) + f" | {np.median(tot):.1f} |")
open(os.path.join(H, "out_sections.md"), "w").write("\n".join(out) + "\n")
print("\n".join(out))
