"""C버전 (MPC + limit_mode:=friction), 0.7 m/s circles8, mu_hat 0.3/0.4 중 완주한 판의 슬립률 λ 비교 (2026-11-17).
λ 정의·bag 읽기는 controller/mpc/tools/plot_slip.py 를 그대로 쓴다. 주행 창 = arm 구간 중 |v̂| > 0.05 m/s 인 첫~끝 행.
  /usr/bin/python3 slip_compare_c07.py [--out 그림.png]
"""
import argparse, os, sys
import numpy as np
ROOT = "/home/smoc/capston/capston"
sys.path.insert(0, os.path.join(ROOT, "src/amr_basic_stack/controller/mpc/tools"))
from plot_slip import read_bag, arrays, armed_spans, smooth, WHEELS  # noqa: E402

RUNS = [  # (bag, 라벨)
    ("2026-11-16/180204", "180204  μ̂0.3 fixed"),
    ("2026-11-16/184235", "184235  μ̂0.4 fixed"),
    ("2026-11-16/190123", "190123  μ̂0.4 tangent"),
    ("2026-11-16/190958", "190958  μ̂0.4 tangent use_w"),
    ("2026-11-16/190851", "190851  μ̂0.4 tangent use_w dist"),
    ("2026-11-16/191134", "191134  μ̂0.4 tangent use_w dist α1"),
]


def load(bag):
    S, W, O, A = arrays(read_bag(os.path.join(ROOT, "bags", bag)))
    spans = armed_spans(A, S[-1, 0])
    v = np.interp(S[:, 0], O[:, 0], np.hypot(O[:, 1], O[:, 2]))
    arm = np.zeros(len(S), bool)
    for s, e in spans:
        arm |= (S[:, 0] >= s) & (S[:, 0] <= e)
    mov = np.flatnonzero(arm & (v > 0.05))
    t0, t1 = S[mov[0], 0], S[mov[-1], 0]
    win = (S[:, 0] >= t0 - 0.5) & (S[:, 0] <= t1 + 0.5)
    n_sm = max(1, int(round(0.1 / np.median(np.diff(S[:, 0])))))
    return dict(t=S[win, 0] - t0, lam=S[win, 1:5], v=v[win], sel=(arm & (v > 0.05))[win], n_sm=n_sm, dur=t1 - t0)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default=os.path.join(
        ROOT, "analysis/torque_mpc_real/2026-11-16_circles8/slip_compare_C_0.7.png"))
    a = ap.parse_args()
    D = [(lab, load(b)) for b, lab in RUNS]
    print("판 | 주행 s | 바퀴별 RMS λ (FL FR RL RR) | 네 바퀴 |λ|>0.1 %")
    for lab, d in D:
        L = d["lam"][d["sel"]]
        rms = np.sqrt(np.nanmean(L ** 2, axis=0)); frac = 100 * np.nanmean(np.abs(L) > 0.1)
        d["rms"], d["frac"] = rms, frac
        print(f"{lab:36s} {d['dur']:5.1f}  " + " ".join(f"{x:.3f}" for x in rms) + f"   {frac:5.1f}")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [n for n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic") if n in have] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    n = len(D)
    fig = plt.figure(figsize=(16, 2.6 * n + 3.2))
    gs = fig.add_gridspec(n + 1, 3, width_ratios=[2.2, 1, 0.01], height_ratios=[1] * n + [1.3])
    tmax = max(d["t"][-1] for _, d in D)
    for i, (lab, d) in enumerate(D):
        ax = fig.add_subplot(gs[i, 0]); ax2 = ax.twinx()
        ax2.plot(d["t"], d["v"], color="0.55", lw=.9); ax2.set_ylim(0, 1.6); ax2.set_ylabel("|v̂| [m/s]", color="0.45")
        for k, w in enumerate(WHEELS):
            ax.plot(d["t"], smooth(d["lam"][:, k], d["n_sm"]), lw=.9, color=f"C{k}", label=w)
        for y in (.1, -.1):
            ax.axhline(y, color="k", ls=":", lw=.6)
        ax.set_ylim(-1.2, 1.2); ax.set_xlim(-0.5, tmax); ax.set_ylabel("λ (0.1 s 평균)")
        ax.set_title(f"{lab}   — RMS λ {np.mean(d['rms']):.2f}, |λ|>0.1 {d['frac']:.0f} %", loc="left", fontsize=10)
        ax.set_zorder(ax2.get_zorder() + 1); ax.patch.set_visible(False)
        if i == 0:
            ax.legend(loc="upper right", ncol=4, fontsize=8)
        if i == n - 1:
            ax.set_xlabel("출발(|v̂|>0.05) 기준 시간 [s]")
        sc = fig.add_subplot(gs[i, 1])
        for k, w in enumerate(WHEELS):
            sc.scatter(d["v"][d["sel"]], d["lam"][d["sel"], k], s=2, alpha=.3, color=f"C{k}")
        sc.set_xlim(0, 1.2); sc.set_ylim(-1.2, 1.2); sc.set_ylabel("λ")
        if i == n - 1:
            sc.set_xlabel("|v̂| [m/s]")
        if i == 0:
            sc.set_title("λ vs |v̂| (주행 행)", fontsize=10)
    bx = fig.add_subplot(gs[n, :2])
    x = np.arange(n); wbar = 0.2
    for k, w in enumerate(WHEELS):
        bx.bar(x + (k - 1.5) * wbar, [d["rms"][k] for _, d in D], wbar, color=f"C{k}", label=w)
    bx.set_xticks(x); bx.set_xticklabels([lab for lab, _ in D], fontsize=9)
    bx.set_ylabel("RMS λ (주행 행)"); bx.legend(ncol=4, fontsize=8); bx.grid(axis="y", lw=.3)
    fig.suptitle("C버전 (MPC + friction 상한) 0.7 m/s circles8 — 완주한 판의 슬립률 λ", fontsize=13)
    fig.tight_layout(); fig.savefig(a.out, dpi=80)
    print("그림:", a.out)


if __name__ == "__main__":
    main()
