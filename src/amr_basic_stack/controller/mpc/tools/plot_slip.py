#!/usr/bin/python3
"""슬립률 λ 와 속도를 한 그림에 (2026-11-16). bag 에서 읽는다 (real_all bag:=, amrrec).

  /usr/bin/python3 tools/plot_slip.py <bag 폴더> [--out 그림.png] [--smooth 0.1]

그림 (가로축 = 처음 arm 한 시각 기준 [s], 회색 띠 = arm 구간):
  첫 줄   왼쪽: 차체 속도 v̂ (/state_estimator/odom 몸체 v_x, v_y, |v|)   오른쪽: λ vs |v̂| 산점도 (arm 구간, 바퀴별)
  FL~RR   왼쪽: 바퀴 각속도 ω (/wheel_states) 와 ω_nom = T⁻¹ v̂ (/slip_logger/slip)   오른쪽: λ (점선 ±0.1 = 추정 잡음 바닥)
슬립률 정의는 nodes/slip_logger.py:  λ = (ω − ω_nom)·sgn(ω_nom) / max(|ω|, |ω_nom|, ω_ε),  ω_ε = 2.5 rad/s
  λ > 0 헛돎(바퀴가 몸체보다 빠름), λ < 0 끌림. v̂ 오차가 그대로 λ 오차가 된다 (slip_logger 머리주석 "한계").
"""
import argparse
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WHEELS = ("FL", "FR", "RL", "RR")
TOPICS = ("/slip_logger/slip", "/wheel_states", "/state_estimator/odom", "/mpc/armed")


def read_bag(path):
    """토픽별 (bag 수신 시각 [s], 메시지) 목록."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in TOPICS}
    while r.has_next():
        name, data, t = r.read_next()
        out[name].append((t * 1e-9, deserialize_message(data, types[name])))
    return out


def arrays(raw):
    S = np.array([[t, *m.data[1:9]] for t, m in raw["/slip_logger/slip"]])          # t, λ×4, ω_nom×4
    W = np.array([[t, *m.velocity] for t, m in raw["/wheel_states"]])                # t, ω×4
    O = np.array([[t, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z]
                  for t, m in raw["/state_estimator/odom"]])                         # t, v_x, v_y, ω_z
    A = [(t, bool(m.data)) for t, m in raw["/mpc/armed"]]
    return S, W, O, A


def armed_spans(A, t_end):
    spans, start = [], None
    for t, on in A:
        if on and start is None:
            start = t
        elif not on and start is not None:
            spans.append((start, t)); start = None
    if start is not None:
        spans.append((start, t_end))
    return spans


def smooth(x, n):
    """NaN 을 빼고 가운데 이동평균 (n 표본)."""
    if n <= 1:
        return x
    ok = np.isfinite(x)
    k = np.ones(n)
    num = np.convolve(np.where(ok, x, 0.0), k, "same")
    den = np.convolve(ok.astype(float), k, "same")
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(den > 0, num / den, np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out", default="", help="그림 경로 (기본 <bag>/slip_speed.png)")
    ap.add_argument("--smooth", type=float, default=0.1, help="λ 이동평균 창 [s] (0 = 끔)")
    a = ap.parse_args()
    S, W, O, A = arrays(read_bag(a.bag))
    if len(S) == 0:
        raise SystemExit("/slip_logger/slip 가 bag 에 없다 — use_mpc:=true 로 띄웠는지 확인")
    spans = armed_spans(A, S[-1, 0])
    t0 = spans[0][0] if spans else S[0, 0]
    speed = np.hypot(O[:, 1], O[:, 2])
    v_at = np.interp(S[:, 0], O[:, 0], speed)
    in_arm = np.zeros(len(S), bool)
    for s, e in spans:
        in_arm |= (S[:, 0] >= s) & (S[:, 0] <= e)
    n_sm = max(1, int(round(a.smooth / np.median(np.diff(S[:, 0]))))) if a.smooth > 0 else 1

    print(f"{a.bag}: slip {len(S)} 행, arm 구간 {[(round(s - t0, 1), round(e - t0, 1)) for s, e in spans]} s")
    sel = in_arm & (v_at > 0.05)
    print(f"  arm 이고 |v̂| > 0.05 m/s 인 행 {sel.sum()}  |  바퀴: NaN %, 평균 λ, RMS λ, |λ| > 0.1 비율 %")
    for i, w in enumerate(WHEELS):
        lam = S[sel, 1 + i]; ok = np.isfinite(lam)
        print(f"  {w}: {100 * np.mean(~ok):4.1f}  {np.mean(lam[ok]):+.3f}  {np.sqrt(np.mean(lam[ok] ** 2)):.3f}"
              f"  {100 * np.mean(np.abs(lam[ok]) > 0.1):5.1f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}       # 한글: NUC 은 Noto CJK (TTC 라 이름이 JP 로 잡힌다)
    plt.rcParams["font.family"] = [n for n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic") if n in have] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig = plt.figure(figsize=(15, 13))
    gs = fig.add_gridspec(5, 2, width_ratios=[1, 1])
    ax_v = fig.add_subplot(gs[0, 0])
    ax_sc = fig.add_subplot(gs[0, 1])
    ax_w = [fig.add_subplot(gs[1 + i, 0], sharex=ax_v) for i in range(4)]
    ax_l = [fig.add_subplot(gs[1 + i, 1], sharex=ax_v) for i in range(4)]
    for ax in [ax_v, *ax_w, *ax_l]:
        for s, e in spans:
            ax.axvspan(s - t0, e - t0, color="0.9", zorder=0)
    ax_v.plot(O[:, 0] - t0, O[:, 1], lw=.8, label="v_x (몸체)")
    ax_v.plot(O[:, 0] - t0, O[:, 2], lw=.8, label="v_y (몸체)")
    ax_v.plot(O[:, 0] - t0, speed, "k", lw=1, label="|v̂|")
    ax_v.set_ylabel("m/s"); ax_v.legend(loc="upper right", fontsize=8); ax_v.set_title("차체 속도 (추정기)")
    for i, w in enumerate(WHEELS):
        ax_w[i].plot(W[:, 0] - t0, W[:, 1 + i], lw=.8, label="ω (측정)")
        ax_w[i].plot(S[:, 0] - t0, S[:, 5 + i], lw=.8, label="ω_nom = T⁻¹v̂")
        ax_w[i].set_ylabel(f"{w} [rad/s]"); ax_w[i].legend(loc="upper right", fontsize=8)
        ax_l[i].plot(S[:, 0] - t0, S[:, 1 + i], color="0.6", lw=.4)
        ax_l[i].plot(S[:, 0] - t0, smooth(S[:, 1 + i], n_sm), "C3", lw=1,
                     label=f"λ ({a.smooth:.2f} s 평균)" if n_sm > 1 else "λ")
        for y in (0.1, -0.1):
            ax_l[i].axhline(y, color="k", ls=":", lw=.6)
        ax_l[i].set_ylim(-1.2, 1.2); ax_l[i].set_ylabel(f"{w} λ"); ax_l[i].legend(loc="upper right", fontsize=8)
        ax_sc.scatter(v_at[in_arm], S[in_arm, 1 + i], s=2, alpha=.3, label=w)
    ax_w[-1].set_xlabel("arm 기준 시간 [s]"); ax_l[-1].set_xlabel("arm 기준 시간 [s]")
    ax_sc.set_xlabel("|v̂| [m/s]"); ax_sc.set_ylabel("λ"); ax_sc.set_ylim(-1.2, 1.2)
    ax_sc.set_title("λ vs 차체 속도 (arm 구간)"); ax_sc.legend(loc="upper right", fontsize=8, markerscale=4)
    fig.suptitle(os.path.basename(os.path.normpath(a.bag)))
    out = a.out or os.path.join(a.bag, "slip_speed.png")
    fig.tight_layout(); fig.savefig(out, dpi=80)
    print(f"  그림: {out}")


if __name__ == "__main__":
    main()
