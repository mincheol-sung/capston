# 2026-11-25 실차 비교 시나리오별 경로 그림 — 시나리오마다 한 장, 제어기마다 서브플롯, 판 (r1~r3) 겹쳐 그림 + 지표
#   source install/setup.bash 뒤  /usr/bin/python3 plot_paths.py   → scen_*.png
#   좌표: 각 판의 출발 포즈 (무장 순간 추정기 포즈 = origin_mode robot 의 기준 원점) 기준 [m]. 실제 = 추정기 위치 (제어기가 보는 값, 로봇 중심).
#   기준 경로 = 그 판 bag 의 /figure8/path. × 표시 = 10 Hz LPF 슬립 max_i|λ̃_i| > 0.5 인 곳 (11-23 회의 지표).
#   지표 = analyze.py (11-22 PP·MPC 비교 스크립트) 의 위치 오차 RMS · 경로까지 거리 RMS · 헤딩 오차 RMS + 10 Hz LPF 슬립 > 25 % · > 50 % 시간, 판 평균 (최소~최대)
import json, os, sys
from types import SimpleNamespace
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from scipy.signal import butter, filtfilt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__)); REAL = os.path.dirname(HERE)
BAGS = "/home/smoc/capston/capston/bags/2026-11-25/"
for f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"):
    font_manager.fontManager.addfont(f)        # ttc 의 첫 얼굴 이름이 "Noto Sans CJK JP" 로 잡힌다 — 한글 글리프도 들어 있다
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 10})
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e6e5e1", "#fcfcfb"
RUNC = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]        # 판 1~4 (기본 팔레트 슬롯 1~4, 고정 순서)
CRIT = "#d03b3b"                                            # 상태색 (critical) — 큰 슬립 표시에만

SCEN = [
    dict(key="pass_a15", folder="2026-11-25_polyline_pass_real",
         title="꺾은선 (정사각형 두 개) · 모서리 통과 (꼭짓점 0.1 m/s) · 가속 1.5 m/s² · 최고 1.5 m/s",
         groups={"MPC (바퀴 속도)": ["154715_sq2p_mpcw_r1", "155359_sq2p_mpcw_r2", "155804_sq2p_mpcw_r3"],
                 "PP (룩어헤드 고정 0.30 m)": ["154855_sq2p_pp_r1", "155511_sq2p_pp_r2", "155933_sq2p_pp_r3"],
                 "어댑티브 PP": ["155018_sq2p_adpp_r1", "155632_sq2p_adpp_r2", "160100_sq2p_adpp_r3"]}),
    dict(key="stop_a20", folder="2026-11-25_polyline_a20_real",
         title="꺾은선 (정사각형 두 개) · 꼭짓점 정지 · 가속 2.0 m/s² (직진 가속 상한 밖)",
         groups={"MPC (바퀴 속도)": ["151542_sq2_a20_mpcw_r1", "152823_sq2_a20_mpcw_r2", "152926_sq2_a20_mpcw_r3"],
                 "PP (룩어헤드 고정 0.30 m)": ["151644_sq2_a20_pp_r1", "153051_sq2_a20_pp_r2", "153213_sq2_a20_pp_r3"],
                 "어댑티브 PP": ["151817_sq2_a20_adpp_r1", "153342_sq2_a20_adpp_r2", "153500_sq2_a20_adpp_r3"]}),
    dict(key="stop_a115", folder="2026-11-25_polyline_real",
         title="꺾은선 (정사각형 두 개) · 꼭짓점 정지 · 가속 1.15 m/s² (상한 안)",
         groups={"MPC (추정기 속도)": ["143238_sq2_mpc_r1"], "MPC (바퀴 속도)": ["144114_sq2_mpc_wheels_r1", "144240_sq2_mpc_wheels_r2"],
                 "PP (룩어헤드 고정 0.30 m)": ["143351_sq2_pp_r1"]}),
    dict(key="c8_d1", folder="2026-11-25_circles8_d1_real",
         title="원 두 개 8자 · 원 지름 1 m · 속도 1.0 m/s · 가속 1.0 m/s² (구심 가속 2.0 > 상한 1.75~1.83, 스트레스)",
         groups={"MPC (추정기 속도)": ["145538_c8d2_mpc_r1"], "MPC (바퀴 속도)": ["145703_c8d2_mpcw_r1"],
                 "PP (룩어헤드 고정 0.30 m)": ["145812_c8d2_pp_r1"], "어댑티브 PP": ["145912_c8d2_pp_r1"]}),
]


def bag_path(lab):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAGS + lab, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/figure8/path"])); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    last = None
    while r.has_next():
        n, d, _ = r.read_next(); last = deserialize_message(d, ty[n])
    return np.array([[p.pose.position.x, p.pose.position.y] for p in last.poses])


def run_data(folder, lab):
    A = np.load(os.path.join(REAL, folder, lab + ".npz")); arm = A["arm"]
    t_on = arm[arm[:, 1] > 0.5, 0][0]; td = arm[arm[:, 1] < 0.5, 0]
    t_off = td[td > t_on][0] if (td > t_on).any() else A["ws"][-1, 0]
    od = A["od"]; i0 = np.searchsorted(od[:, 0], t_on)
    x0, y0, a0 = od[i0, 1], od[i0, 2], od[i0, 3]
    c, s = np.cos(a0), np.sin(a0)
    loc = lambda X, Y: (c * (X - x0) + s * (Y - y0), -s * (X - x0) + c * (Y - y0))
    w = (od[:, 0] >= t_on) & (od[:, 0] <= t_off)
    tx, ty = loc(od[w, 1], od[w, 2])
    S = A["slip"]; t = S[:, 0]; L = np.where(np.isfinite(S[:, 2:6]), S[:, 2:6], 0.0)
    fs = 1 / np.median(np.diff(t)); bb, aa = butter(1, 10.0 / (fs / 2)); Lf = filtfilt(bb, aa, L, axis=0)
    sp = np.hypot(np.interp(t, od[:, 0], od[:, 4]), np.interp(t, od[:, 0], od[:, 5]))
    win = (t >= t_on) & (t <= t_off) & (sp > 0.05); mx = np.abs(Lf).max(1)
    big = win & (mx > 0.5)
    bx, by = loc(np.interp(t[big], od[:, 0], od[:, 1]), np.interp(t[big], od[:, 0], od[:, 2]))
    P = bag_path(lab); px, py = loc(P[:, 0], P[:, 1])
    return SimpleNamespace(tx=tx, ty=ty, bx=bx, by=by, px=px, py=py, s25=100 * np.mean(mx[win] > 0.25), s50=100 * np.mean(mx[win] > 0.5))


def metrics(folder, labs):
    cache = os.path.join(HERE, "metrics_cache.json")
    C = json.load(open(cache)) if os.path.exists(cache) else {}
    sys.path.insert(0, os.path.join(REAL, folder))
    import importlib; AZ = importlib.import_module("analyze"); sys.path.pop(0)
    out = []
    for lab in labs:
        if lab not in C:
            r, _ = AZ.analyze(BAGS + lab)
            C[lab] = {k: float(r[k]) for k in ("exy_rms", "dpath_rms", "eth_rms", "exy_max")}
            json.dump(C, open(cache, "w"), indent=1)
        out.append(C[lab])
    sys.modules.pop("analyze", None)
    return out


def fmt(v, unit, d=1):
    v = np.array(v)
    return f"{v.mean():.{d}f}{unit}" if len(v) == 1 else f"{v.mean():.{d}f}{unit} ({v.min():.{d}f}~{v.max():.{d}f})"


for sc in SCEN:
    G = sc["groups"]; n = len(G)
    allx, ally = [], []
    data = {g: [run_data(sc["folder"], lab) for lab in labs] for g, labs in G.items()}
    for g in G:
        for d in data[g]:
            allx += [d.tx.min(), d.tx.max(), d.px.min(), d.px.max()]; ally += [d.ty.min(), d.ty.max(), d.py.min(), d.py.max()]
    pad = 0.15
    xl, yl = (min(allx) - pad, max(allx) + pad), (min(ally) - pad, max(ally) + pad)
    w_ax = max(3.2, 4.4 * (xl[1] - xl[0]) / (yl[1] - yl[0]))         # 그림 칸 너비 = 높이 4.4 in × 가로세로 비 (빈 칸이 안 생기게)
    fig, axes = plt.subplots(1, n, figsize=((w_ax + 0.8) * n + 0.3, 6.8), facecolor=SURF)
    for ax, (g, labs) in zip(np.atleast_1d(axes), G.items()):
        ax.set_facecolor(SURF)
        d0 = data[g][0]
        ax.plot(d0.px, d0.py, color=INK2, lw=1.4, ls=(0, (4, 3)), label="기준 경로", zorder=2)
        for k, d in enumerate(data[g]):
            ax.plot(d.tx, d.ty, color=RUNC[k], lw=1.6, alpha=0.9, label=f"판 {k + 1}", zorder=3)
            ax.plot(d.bx, d.by, ls="none", marker="x", ms=3.5, mew=0.9, color=CRIT, alpha=0.55, zorder=4,
                    label="슬립 > 50 % (10 Hz LPF)" if k == 0 else None)
        ax.plot(0, 0, marker="o", ms=7, mfc=SURF, mec=INK, mew=1.4, zorder=5, label="출발 (출발 헤딩 = +x)")
        ax.set_xlim(xl); ax.set_ylim(yl); ax.set_aspect("equal")
        ax.grid(color=GRID, lw=0.6); [s.set_color(GRID) for s in ax.spines.values()]
        ax.tick_params(colors=INK2, labelsize=8); ax.set_xlabel("x [m]", color=INK2, fontsize=9); ax.set_ylabel("y [m]", color=INK2, fontsize=9)
        M = metrics(sc["folder"], labs)
        txt = (f"위치 오차 RMS {fmt([m['exy_rms'] * 100 for m in M], ' cm')}\n"
               f"위치 오차 최대 {fmt([m['exy_max'] * 100 for m in M], ' cm')}\n"
               f"경로까지 거리 RMS {fmt([m['dpath_rms'] * 100 for m in M], ' cm')}\n"
               f"헤딩 오차 RMS {fmt([m['eth_rms'] for m in M], '°')}\n"
               f"슬립 > 50 % 시간 {fmt([d.s50 for d in data[g]], ' %')}\n"
               f"슬립 > 25 % 시간 {fmt([d.s25 for d in data[g]], ' %')}")
        ax.set_title(f"{g}  ·  {len(labs)} 판", color=INK, fontsize=12, fontweight="bold", loc="left", pad=8)
        ax.text(0.0, -0.13, txt, transform=ax.transAxes, va="top", ha="left", color=INK, fontsize=9.5, linespacing=1.45)
    fig.suptitle(sc["title"], color=INK, fontsize=13, x=0.01, ha="left", va="top", y=0.99, fontweight="bold")
    fig.text(0.01, 0.94, va="top", s="추정기 위치 (로봇 중심), 각 판의 출발 포즈 기준 좌표 [m]. 지표는 판 평균 (괄호: 판 사이 최소~최대). 2026-11-25 실차",
             color=INK2, fontsize=9, ha="left")
    h, l = max((a.get_legend_handles_labels() for a in np.atleast_1d(axes)), key=lambda hl: len(hl[0]))
    order = sorted(range(len(l)), key=lambda i: (0 if l[i] == "기준 경로" else 1 if l[i].startswith("판") else 2 if l[i].startswith("슬립") else 3, l[i]))
    h, l = [h[i] for i in order], [l[i] for i in order]
    fig.legend(h, l, loc="upper left", bbox_to_anchor=(0.005, 0.91), ncol=len(h), fontsize=8.5, frameon=False, handlelength=2.2)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.82, bottom=0.28, wspace=0.25)
    out = os.path.join(HERE, f"scen_{sc['key']}.png"); fig.savefig(out, dpi=150, facecolor=SURF); plt.close(fig)
    print(out)
