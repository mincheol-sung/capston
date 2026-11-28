"""seq_s1.py 뒤속 — 11-28 S1 에서 MPC 위치가 11-26 보다 나빠진 (5.1 → 12.3 cm) 몫을 W ×4 와 슬립 되먹임으로 가른다.
1) 11-27 같은 원 (반지름 0.5 · 1.0 m/s · 가속 1.0 · blend 0 · 1 바퀴) 의 MPC W ×1 되먹임 끔 / W ×1 되먹임 1.0 / 11-28 W ×4 되먹임 1.0 + PP·ADPP 를 seq_s1.metrics 로.
   162239 (11-27 발산 판 — MPC 멈춤 → 제동 → 과속) 은 뺀다.
2) /torque_mpc/slip_fb [t, 켬, fresh, e ×3, Δτ ×4 (6~9), τ_MPC ×4 (10~13), 낸 τ ×4 (14~17), e_ω ×4 (18~21)] 로
   구동 방향 몫 = Σ_i τ_i · sgn(ω_nom,i) (ω_nom = /slip_logger/slip 6~9 열, 바퀴가 굴러가는 방향으로 민 토크 합 [N·m]) — Δτ 와 τ_MPC 각각.
   되먹임 끈 판도 Δτ 는 계산·기록된다 (더하지만 않음).
3) 진행 오차 시간축: 무장 뒤 시간별 판 평균 (11-28 세 제어기 · 11-26 MPC), SLAM 중심 접선 속력.
/usr/bin/python3 seq_s1_cause.py → seq_s1_cause_out.txt + seq_s1_*.png"""
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import seq_s1 as S  # noqa: E402
from bag_runs import runs  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

B27 = [("MPC W×1 되먹임 끔", "2026-11-27", ["162411_sfb_on_r2", "165139_sfbw_lqr025_r3"]),
       ("MPC W×1 되먹임 1.0", "2026-11-27", ["165723_sfbw_lqr1_r1", "192519_yarr_mpc", "193234_yarr_mpc", "193851_yarr_mpc"]),
       ("MPC W×4 되먹임 1.0", "2026-11-28", ["170146_c8_mpc_r1"]),
       ("PP", "2026-11-27", ["165935_pp_r1", "173001_pp_r1"]), ("PP", "2026-11-28", ["170339_c8_pp_r1"]),
       ("ADPP", "2026-11-27", ["173512_pp_r1"]), ("ADPP", "2026-11-28", ["170436_c8_adpp_r1"])]


def read_sfb(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    have = {t.name: t.type for t in r.get_all_topics_and_types()}
    if "/torque_mpc/slip_fb" not in have:
        return None
    r.set_filter(rosbag2_py.StorageFilter(topics=["/torque_mpc/slip_fb"]))
    ty = get_message(have["/torque_mpc/slip_fb"]); F = []
    while r.has_next():
        _, data, _ = r.read_next(); d = list(deserialize_message(data, ty).data); F.append(d[:22] + [np.nan] * (22 - len(d)))
    return np.array(F)


def fit_J(D):
    """ω_nom (slip 6~9 열) = J · [v_x, v_y, ω_z] (추정기 몸체 속도) 최소제곱 — Jᵀτ = 몸체 일반화 힘 [N, N, N·m] (가상 일: τ·ω = (Jᵀτ)·v)."""
    Sl, od = D["slip"], D["od"]; ok = np.isfinite(Sl[:, 5:9]).all(1)
    X = np.column_stack([np.interp(Sl[ok, 0], od[:, 0], od[:, i]) for i in (4, 5, 6)]); Y = Sl[ok, 5:9]
    mv = np.abs(X).sum(1) > 0.05
    J, res, *_ = np.linalg.lstsq(X[mv], Y[mv], rcond=None)
    return J.T, float(np.sqrt(np.mean((X[mv] @ J - Y[mv]) ** 2)))


def body_force(F, J, m, t_on, steady=True):
    """기준 접선 (진행) · 원 중심 쪽 (구심) 힘 평균 [N]: τ_MPC, Δτ, 낸 τ. steady → 기준 속력 > 0.9 최고 구간."""
    ts, vref = m["_ts"] + t_on, m["_vref"]
    R = m["_R"]; k = (F[:, 0] >= ts[0]) & (F[:, 0] <= ts[-1])
    if steady:
        k &= np.interp(F[:, 0], ts, vref) > 0.9 * vref.max()
    ub, vb, wr = (np.interp(F[k, 0], m["_trR"], R[:, i]) for i in (3, 4, 5))
    n_ = np.hypot(ub, vb); tx, ty = ub / np.maximum(n_, 1e-6), vb / np.maximum(n_, 1e-6); nx, ny = -ty * np.sign(wr), tx * np.sign(wr)
    out = {}
    for lab, cols in (("mpc", slice(10, 14)), ("dtau", slice(6, 10)), ("out", slice(14, 18))):
        Q = F[k, cols] @ J          # (N×4)(4×3) = Jᵀτ 행별
        out[lab] = (float(np.mean(Q[:, 0] * tx + Q[:, 1] * ty)), float(np.mean(Q[:, 0] * nx + Q[:, 1] * ny)), float(np.mean(Q[:, 2] * np.sign(wr))))
    return out


def drive_share(F, D, t_on, t_off, steady=None):
    """구동 방향 토크 합 평균 [N·m]: Δτ, τ_MPC, 낸 τ. steady = (ts, vref) 주면 기준 속력 > 0.9 최고 구간만."""
    Sl = D["slip"]; k = (F[:, 0] >= t_on) & (F[:, 0] <= t_off)
    if steady is not None:
        ts, vref = steady; k &= np.interp(F[:, 0], ts, vref) > 0.9 * vref.max()
    sg = np.sign(np.column_stack([np.interp(F[k, 0], Sl[:, 0], Sl[:, 5 + i]) for i in range(4)]))
    d = (F[k, 6:10] * sg).sum(1); m = (F[k, 10:14] * sg).sum(1); o = (F[k, 14:18] * sg).sum(1)
    return dict(dtau=float(d.mean()), tmpc=float(m.mean()), tout=float(o.mean()), cut=100 * float(-d.mean() / m.mean()), on=float(np.nanmean(F[k, 1])))


if __name__ == "__main__":
    out = open(os.path.join(HERE, "seq_s1_cause_out.txt"), "w")

    def P(*a):
        print(*a); print(*a, file=out)

    # ── 1) 11-27 blend 0 묶음
    P("## 1) 11-27·11-28 blend 0 원 (반지름 0.5 · 1.0 m/s · 1 바퀴) — 같은 지표")
    P("| 묶음 | 판 | 위치 RMS cm | 최대 | 진행 평균 (+뒤) | 안쪽 | 헤딩 ° | 큰 슬립 % | 헛돎 % | 미끄럼 속도 cm/s | 구동 방향 Δτ / τ_MPC [N·m] (깎은 %) |")
    P("|---|---|---|---|---|---|---|---|---|---|---|")
    G27 = {}; BF = []
    for lab, day, names in B27:
        for n in names:
            p = S.find(day, n); (rr,) = runs(p)[:1]; t_on, t_off = rr["armed"][0]
            D = S.read(p); m = S.metrics(D, t_on, t_off); F = read_sfb(p) if lab.startswith("MPC") else None
            ds = drive_share(F, D, t_on, t_off) if F is not None else None
            if F is not None:
                J, jr = fit_J(D); bf = body_force(F, J, m, t_on); BF.append((lab, n[:6], bf, jr))
            G27.setdefault(lab, []).append(m)
            P(f"| {lab} | {day[5:]} {n[:6]} | {m['pos']:.1f} | {m['pos_max']:.1f} | {m['along']:+.1f} | {m['inside']:+.1f} | {m['head']:.1f} | {m['m50']:.1f} | {m['spin']:.0f} | {m['sv']:.1f} | "
              + (f"{ds['dtau']:+.3f} / {ds['tmpc']:+.3f} ({ds['cut']:+.0f} %, 켬 {ds['on']:.0f})" if ds else "-") + " |")
    P("\n묶음 평균: " + " · ".join(f"{k} (n {len(v)}) 위치 {np.mean([m['pos'] for m in v]):.1f} · 진행 {np.mean([m['along'] for m in v]):+.1f} · 미끄럼 {np.mean([m['sv'] for m in v]):.1f}" for k, v in G27.items()))

    P("\n## 1b) 최고 속력 구간 몸체 힘 (Jᵀτ, J = 추정기 몸체 속도 → ω_nom 최소제곱) — 진행 방향 / 원 중심 쪽 [N] · 요 모멘트 (회전 쪽 +) [N·m]")
    for lab, n, bf, jr in BF:
        P(f"- {lab} {n}: τ_MPC 진행 {bf['mpc'][0]:+.2f} · 구심 {bf['mpc'][1]:+.2f} · 요 {bf['mpc'][2]:+.3f} | Δτ 진행 {bf['dtau'][0]:+.2f} · 구심 {bf['dtau'][1]:+.2f} · 요 {bf['dtau'][2]:+.3f}"
          f" | 낸 τ 진행 {bf['out'][0]:+.2f} · 구심 {bf['out'][1]:+.2f} (J 맞춤 잔차 {jr:.2f} rad/s)")

    # ── 2) 11-28 seq MPC 판 · 11-26 MPC 판 구동 방향 몫
    RS = runs(S.SEQ); D = S.read(S.SEQ); F = read_sfb(S.SEQ); J28, jr28 = fit_J(D)
    P(f"J (11-28 seq 맞춤, 잔차 {jr28:.2f} rad/s) 행 FL FR RL RR, 열 v_x v_y ω_z:\n" + np.array2string(J28, precision=2))
    P("\n## 2) 11-28 seq MPC 판 — 슬립 되먹임이 구동 방향 토크에서 뺀 몫 (창 전체 / 최고 속력 구간)")
    rows28 = {"MPC": [], "PP": [], "ADPP": []}
    for r in RS:
        (t_on, t_off), = r["armed"]; m = S.metrics(D, t_on, t_off); c = r["controller"].upper(); rows28[c].append(m)
        if c == "MPC":
            a = drive_share(F, D, t_on, t_off); b = drive_share(F, D, t_on, t_off, steady=(m["_ts"] + t_on, m["_vref"]))
            bf = body_force(F, J28, m, t_on)
            P(f"  몸체 힘 (최고 속력 구간): τ_MPC 진행 {bf['mpc'][0]:+.2f} · 구심 {bf['mpc'][1]:+.2f} N · 요 {bf['mpc'][2]:+.3f} | Δτ 진행 {bf['dtau'][0]:+.2f} · 구심 {bf['dtau'][1]:+.2f} · 요 {bf['dtau'][2]:+.3f}")
            P(f"- 판 {r['run']} (회차 {r['seq']['round']}): 전체 Δτ {a['dtau']:+.3f} / τ_MPC {a['tmpc']:+.3f} / 낸 τ {a['tout']:+.3f} N·m → 깎은 {a['cut']:.0f} % · "
              f"최고 속력 구간 Δτ {b['dtau']:+.3f} / τ_MPC {b['tmpc']:+.3f} → {b['cut']:.0f} % · 진행 {m['along']:+.1f} cm")
    rows26 = []
    for n in S.OLD["MPC"]:
        p = S.find("2026-11-26", n); (rr,) = runs(p); t_on, t_off = rr["armed"][0]; Dd = S.read(p); rows26.append(S.metrics(Dd, t_on, t_off))

    # ── 3) 진행 오차 시간축 + 그림
    fp = [f for f in font_manager.findSystemFonts() if "NotoSansCJK" in f and "Regular" in f]
    if fp:
        font_manager.fontManager.addfont(fp[0]); plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp[0]).get_name()
    INK, INK2, GRID = "#1f2328", "#57606a", "#d0d7de"
    COL = {"MPC": "#2a78d6", "PP": "#eb6834", "ADPP": "#1baf7a", "MPC 11-26": "#2a78d6"}
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
    tg = np.arange(0.0, 7.6, 0.02)

    def prof(rows, key):
        Y = []
        for m in rows:
            w = m["_win"]; t0 = m["_ts"][w][0]; Y.append(np.interp(tg, m["_ts"][w] - t0, m[key][w], left=np.nan, right=np.nan))
        return np.nanmean(Y, 0), np.nanmin(Y, 0), np.nanmax(Y, 0)

    P("\n## 3) 진행 오차 (+ 뒤처짐) 판 평균 — 출발 (기준 속력 > 0.05) 뒤 1 · 2 · 3 · 4 · 5 · 6 · 7 s [cm]")
    groups = [("MPC 11-28 (W×4 · 되먹임 1.0)", rows28["MPC"], "MPC", "-"), ("PP 11-28", rows28["PP"], "PP", "-"), ("ADPP 11-28", rows28["ADPP"], "ADPP", "-"),
              ("MPC 11-26 (W×1 · 되먹임 없음)", rows26, "MPC 11-26", "--")]
    for lab, rows, _, _ in groups:
        mu = prof(rows, "_along")[0]
        P(f"- {lab}: " + " · ".join(f"{100 * mu[np.searchsorted(tg, s)]:+.1f}" for s in (1, 2, 3, 4, 5, 6, 7)))

    fig, axs = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True)
    m0 = rows28["MPC"][0]
    for f in m0["_flip"] - m0["_ts"][m0["_win"]][0]:
        for ax in axs:
            ax.axvspan(f - 0.3, f + 0.3, color="#eaeef2", lw=0)
    for lab, rows, ck, ls in groups:
        for ax, key in zip(axs, ("_along", "_inside")):
            mu, lo, hi = prof(rows, key)
            ax.plot(tg, 100 * mu, color=COL[ck], lw=2, ls=ls, label=f"{lab} (n {len(rows)})")
            if ls == "-":
                ax.fill_between(tg, 100 * lo, 100 * hi, color=COL[ck], alpha=0.12, lw=0)
    axs[0].set_ylabel("진행 오차 [cm] (+ 로봇이 뒤처짐)"); axs[1].set_ylabel("안쪽 치우침 [cm] (+ 원 안쪽)")
    axs[1].set_xlabel("출발 뒤 시간 [s] (기준 속력 > 0.05 m/s 부터)")
    for ax in axs:
        ax.axhline(0, color=INK2, lw=0.6); ax.grid(color=GRID, lw=0.5)
    axs[0].legend(fontsize=9, frameon=False, loc="upper left", ncol=2)
    axs[0].set_title("S1 원 8 자 (반지름 0.5 m · 1.0 m/s · 가속 1.0 · blend 0.6) — 판 평균 선, 띠 = 최소~최대, 회색 = 교차점 ±0.3 s", loc="left", fontsize=10.5, color=INK)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "seq_s1_time.png"), dpi=130); plt.close(fig)

    # 판별 지표 점 그림 (11-28 채운 점, 11-26 빈 점)
    S28 = {c: rows28[c] for c in ("MPC", "PP", "ADPP")}
    import pickle
    allrows = pickle.load(open(os.path.join(os.environ.get("SEQ_PKL_DIR", HERE), "seq_s1_rows.pkl"), "rb"))
    keys = [("pos", "위치 RMS [cm] (SLAM 중심)"), ("along", "진행 평균 [cm] (+ 뒤처짐)"), ("head", "헤딩 RMS [°]"),
            ("sv", "미끄럼 속도 RMS [cm/s]"), ("m50", "큰 슬립 [%] (|λ| > 0.5)"), ("dtick", "모터 명령 떨림 [mN·m]")]
    fig, axs = plt.subplots(2, 3, figsize=(13, 6.6))
    for ax, (k, ttl) in zip(axs.ravel(), keys):
        for i, c in enumerate(("MPC", "PP", "ADPP")):
            for day, dx, face in (("11-28", -0.13, True), ("11-26", 0.13, False)):
                v = np.array([m[k] for m in allrows if m["day"] == day and m["ctrl"] == c])
                ax.scatter(np.full(len(v), i + dx), v, s=46, facecolors=COL[c] if face else "white", edgecolors=COL[c], linewidths=1.6, zorder=3)
                ax.plot([i + dx - 0.09, i + dx + 0.09], [v.mean()] * 2, color=INK, lw=1.4, zorder=4)
        ax.set_xticks([0, 1, 2]); ax.set_xticklabels(["MPC", "PP", "ADPP"]); ax.set_xlim(-0.5, 2.5)
        ax.set_title(ttl, loc="left", fontsize=10, color=INK); ax.grid(axis="y", color=GRID, lw=0.5)
    axs[0, 0].scatter([], [], s=46, color=INK2, label="11-28 seq (채운 점, 5 판씩)")
    axs[0, 0].scatter([], [], s=46, facecolors="white", edgecolors=INK2, label="11-26 같은 경로 (빈 점)")
    axs[0, 0].legend(fontsize=8.5, frameon=False, loc="upper right")
    fig.text(0.01, 0.995, "S1 원 8 자 세 제어기 — 11-28 MPC = W ×4 · 슬립 되먹임 1.0, 11-26 MPC = W ×1 · 되먹임 없음 (그 밖의 파라미터 같음). 가로 막대 = 평균",
             fontsize=10, color=INK, va="top")
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(HERE, "seq_s1_dots.png"), dpi=130); plt.close(fig)

    # 경로: 각 묶음에서 위치 RMS 가 중앙값인 판
    fig, axs = plt.subplots(1, 4, figsize=(17, 4.9))
    for ax, (lab, rows, ck, _) in zip(axs, groups):
        m = sorted(rows, key=lambda q: q["pos"])[len(rows) // 2]
        ax.plot(m["_R"][:, 0], m["_R"][:, 1], color="#8c959f", lw=6, alpha=0.35, solid_capstyle="round", label="기준")
        ax.plot(m["_sx"], m["_sy"], color=COL[ck], lw=1.4, label="SLAM 로봇 중심")
        ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.5)
        ax.set_title(f"{lab}\n중앙값 판: 위치 {m['pos']:.1f} cm · 진행 {m['along']:+.1f} · 미끄럼 {m['sv']:.1f} cm/s", loc="left", fontsize=9.5, color=INK)
    axs[0].legend(fontsize=8.5, frameon=False, loc="lower left")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "seq_s1_paths.png"), dpi=130); plt.close(fig)
    out.close()
    print("그림:", ", ".join(os.path.join(HERE, f) for f in ("seq_s1_dots.png", "seq_s1_time.png", "seq_s1_paths.png")))
