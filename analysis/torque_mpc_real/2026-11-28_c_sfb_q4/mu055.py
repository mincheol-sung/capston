"""2026-11-28 20:01 bag 200153 — S1 원 MPC 세 조건 번갈아 2 판씩: (μ̂ 0.41, 되먹임 1.0) · (0.55, 1.0) · (0.55, 1.4) + PP 1 판. 200047 = 전원 재투입 뒤 PP 1 판.
배터리 교체·모터 전원 재투입 뒤 (194358 끝에 네 모터 에러 255). 판마다 모터 건강: ak40 "빠진 바퀴" 경고 수, 모터가 잰 |τ| / 명령 |τ|.
비교: 192447_mu041 (μ̂ 0.41 · 0.33 3 판씩, PP 1) · 183517 seq (μ̂ 0.33 MPC 5 · PP 5). 지표는 mu041.run_rows 와 같다.
nice -n 19 /usr/bin/python3 mu055.py → mu055_out.txt, mu055.png"""
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy import stats

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
import seq_s1_cause as C  # noqa: E402
import sfb_modes as SM  # noqa: E402
from bag_runs import runs  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

BAGS = [("20:01", S.B + "2026-11-28/200153"), ("20:00", S.B + "2026-11-28/200047"), ("19:24", S.B + "2026-11-28/192447_mu041"), ("18:35", S.SEQ)]
T33 = SM.TBAR.copy()


def health(bag):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    tp = ["/rosout", "/wheel_states", "/wheel_torque_cmd"]; r.set_filter(rosbag2_py.StorageFilter(topics=tp)); ty = {n: get_message(have[n]) for n in tp}
    miss, err, ws, cmd = [], [], [], []
    while r.has_next():
        n, d, ns = r.read_next(); t = ns * 1e-9; m = deserialize_message(d, ty[n])
        if n == "/rosout":
            if m.name == "ak40_can_node" and "빠진" in m.msg:
                miss.append(t)
            elif m.name == "ak40_can_node" and ("에러" in m.msg or "실패" in m.msg):
                err.append(t)
        elif n == "/wheel_states":
            ws.append([t, *m.torque])
        else:
            cmd.append([t, *m.torque])
    return np.array(miss), np.array(err), np.array(ws), np.array(cmd)


def rows_of(tag, bag):
    RS = runs(bag); D = S.read(bag); F = C.read_sfb(bag); CAP = SM.read_cap(bag); J, _ = C.fit_J(D); miss, err, W, M = health(bag); out = []
    for r in RS:
        if not r["armed"]:
            continue
        (t_on, t_off), = r["armed"]; c = r["controller"].upper(); live = r["mpc"] or {}
        mu, sc = live.get("mu_hat", 0.33), live.get("slip_fb_scale", 1.0)
        m = S.metrics(D, t_on, t_off)
        SM.TBAR = T33 * mu / 0.33
        e = SM.one(D, F, t_on, t_off, CAP) if c == "MPC" else SM.one(D, None, t_on, t_off)
        bf = C.body_force(F, J, m, t_on) if c == "MPC" else None
        SM.TBAR = T33
        kw = (W[:, 0] >= t_on) & (W[:, 0] <= t_off); km = (M[:, 0] >= t_on) & (M[:, 0] <= t_off)
        m.update(tag=tag, run=r["run"], ctrl=c, mu=mu, sc=sc, sat=e["sat"], bf=bf, ew=e["ew"],
                 miss=int(((miss >= t_on) & (miss <= t_off)).sum()) if miss.size else 0, err=int(((err >= t_on) & (err <= t_off)).sum()) if err.size else 0,
                 tq=float(np.abs(W[kw, 1:5]).mean() / max(np.abs(M[km, 1:5]).mean(), 1e-9)))
        out.append(m)
    return out


if __name__ == "__main__":
    out = open(os.path.join(HERE, "mu055_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    rows = [m for tag, b in BAGS for m in rows_of(tag, b)]
    P("| bag | 판 | 제어기 | μ̂ | 되먹임 | 모터 °C | 빠진 바퀴 경고 | 모터 에러 | 잰/명령 |τ| | 위치 RMS cm | 최대 | 진행 (+뒤) | 안쪽 | 헤딩 ° | 미끄럼 속도 cm/s | >0.2 m/s % | 큰 슬립 % | 헛돎 % | 떨림 mN·m | 바닥 상한 붙음 % | 구심 τ_MPC / Δτ / 낸 τ N |")
    P("|" + "---|" * 21)
    for m in rows:
        bf = m["bf"]
        P(f"| {m['tag']} | {m['run']} | {m['ctrl']} | {m['mu']:.2f} | {m['sc']:g} | {m['temp']:.1f} | {m['miss']} | {m['err']} | {m['tq']:.2f} | {m['pos']:.1f} | {m['pos_max']:.1f} | "
          f"{m['along']:+.1f} | {m['inside']:+.1f} | {m['head']:.1f} | {m['sv']:.1f} | {m['sv20']:.1f} | {m['m50']:.1f} | {m['spin']:.0f} | {m['dtick']:.1f} | "
          + (f"{m['sat']:.0f} | {bf['mpc'][1]:+.2f} / {bf['dtau'][1]:+.2f} / {bf['out'][1]:+.2f} |" if bf else "- | - |"))
    G = {}
    for m in rows:
        k = (m["ctrl"], m["mu"], m["sc"]) if m["ctrl"] == "MPC" else ("PP" if m["ctrl"] == "PP" else m["ctrl"], None, None)
        G.setdefault(k, []).append(m)
    order = [("MPC", 0.33, 1.0), ("MPC", 0.41, 1.0), ("MPC", 0.55, 1.0), ("MPC", 0.55, 1.4), ("PP", None, None), ("ADPP", None, None)]
    names = {k: (f"MPC μ̂ {k[1]:.2f} · 되먹임 {k[2]:g}" if k[0] == "MPC" else k[0]) for k in order}
    KEYS = [("pos", "위치 RMS cm"), ("pos_max", "위치 최대 cm"), ("along", "진행 cm (+뒤)"), ("inside", "안쪽 cm"), ("cross_r", "옆 RMS cm"), ("head", "헤딩 RMS °"),
            ("sv", "미끄럼 속도 cm/s"), ("sv20", "미끄럼 > 0.2 m/s %"), ("m50", "큰 슬립 %"), ("dtick", "떨림 mN·m"), ("sat", "바닥 상한 붙음 %"), ("temp", "모터 °C")]
    P("\n## 조건별 평균 (최소~최대) — 18:35 · 19:24 · 20:00 · 20:01 bag 합침")
    P("| 지표 | " + " | ".join(f"{names[k]} (n {len(G.get(k, []))})" for k in order) + " |")
    P("|---|" + "---|" * len(order))
    for key, lab in KEYS:
        cells = []
        for k in order:
            a = np.array([m[key] for m in G.get(k, [])], float)
            cells.append("-" if not a.size or not np.isfinite(a).any() else f"{np.nanmean(a):.1f}" + (f" ({np.nanmin(a):.1f}~{np.nanmax(a):.1f})" if a.size > 1 else ""))
        P(f"| {lab} | " + " | ".join(cells) + " |")
    P("\n## 20:01 bag 안에서만 (같은 배터리·같은 세션, 번갈아) — 조건별 판")
    for k in order[1:5]:
        v = [m for m in G.get(k, []) if m["tag"] == "20:01"]
        if v:
            P(f"- {names[k]}: " + " · ".join(f"판 {m['run']} 위치 {m['pos']:.1f} / 미끄럼 {m['sv']:.1f} / 큰 슬립 {m['m50']:.1f}" for m in v))
    P("\n## 두 조건씩 Welch t (모든 bag 합침)")
    for a_, b_ in [(order[3], order[1]), (order[2], order[1]), (order[3], order[2]), (order[3], order[4]), (order[1], order[0])]:
        A, Bv = G.get(a_, []), G.get(b_, [])
        if len(A) > 1 and len(Bv) > 1:
            s = []
            for key, lab in KEYS[:9]:
                x, y = np.array([m[key] for m in A]), np.array([m[key] for m in Bv])
                s.append(f"{lab} {x.mean() - y.mean():+.1f} (p {stats.ttest_ind(x, y, equal_var=False).pvalue:.3f})")
            P(f"- {names[a_]} − {names[b_]}: " + " · ".join(s))
    P("\n## 구심 힘 (최고 속력 구간, N) — τ_MPC · Δτ · 낸 τ (요구 m v²/R 9.12)")
    for k in order[:4]:
        v = [m for m in G.get(k, []) if m["bf"]]
        if v:
            b = np.mean([[m["bf"]["mpc"][1], m["bf"]["dtau"][1], m["bf"]["out"][1]] for m in v], 0)
            P(f"- {names[k]}: {b[0]:+.2f} · {b[1]:+.2f} · {b[2]:+.2f} · 상한 붙음 {np.mean([m['sat'] for m in v]):.0f} %")
    out.close()

    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    COL = {order[0]: "#86b6ef", order[1]: "#2a78d6", order[2]: "#6250d6", order[3]: "#1baf7a", order[4]: "#eb6834"}
    LAB = {order[0]: "MPC mu_hat 0.33 · sfb 1.0", order[1]: "MPC mu_hat 0.41 · sfb 1.0", order[2]: "MPC mu_hat 0.55 · sfb 1.0", order[3]: "MPC mu_hat 0.55 · sfb 1.4", order[4]: "PP"}
    fig, ax = plt.subplots(figsize=(9, 6))
    for k in order[:5]:
        v = G.get(k, [])
        if not v:
            continue
        x, y = [m["sv"] for m in v], [m["pos"] for m in v]
        ax.scatter(x, y, s=60, color=COL[k], alpha=0.85, label=f"{LAB[k]} (n {len(v)})", zorder=3)
        ax.scatter([np.mean(x)], [np.mean(y)], s=220, marker="X", color=COL[k], edgecolors="white", linewidths=1.5, zorder=4)
    ax.set_xlabel("미끄럼 속도 RMS [cm/s] (왼쪽 = 슬립 적음)"); ax.set_ylabel("위치 RMS [cm] (SLAM 로봇 중심, 아래 = 좋음)")
    ax.grid(color=GRID, lw=0.5); ax.legend(fontsize=9, frameon=False, loc="upper right")
    ax.set_title("S1 원 (반지름 0.5 m · 1.0 m/s · blend 0.6) — 판별 점, X = 조건 평균. 11-28 18:35 · 19:24 · 20:00 · 20:01 bag", loc="left", fontsize=10.5, color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "mu055.png"), dpi=130); plt.close(fig)
    print("그림:", os.path.join(HERE, "mu055.png"))
