#!/usr/bin/python3
"""슬립 되먹임 (2026-11-27, stack_changelog 2026-11-27 (2)) 의 두 입력을 bag 에서 나란히 본다.

  nominal  x̂_w = R(δ0)·T ω  — 바퀴 각속도 /wheel_states 로 만든 무슬립 몸체 속도 [v_x, v_y, ω_z]
  feedback x_e              — 지금 MPC 가 쓰는 피드백 /state_estimator/odom twist (노드의 est_vb_)
  e = x̂_w − x_e,  Δτ = −K_v e  (켜면 τ = τ_MPC + Δτ)

  cd ~/capston/capston && source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-27_sfb/nominal_vs_feedback.py bags/2026-11-27/161922_sfb_on_r1 [bag ...] [--show]

계산 (노드와 같은 식, torque_mpc.hpp wheel_body_velocity · torque_mpc.cpp correct_wheel_velocity)
  T      (r/4)[1 1 1 1; −1 1 1 −1; (−1 1 −1 1)/k],  k = (wheel_base + width)/2.  r·k 는 run_info plant_file, δ0 는 run_info 의 drive_yaw_offset_deg
  R(δ0)  T ω 의 xy 두 성분에 left-multiply (ω_z 행은 그대로). 선회 미끄럼 보정 slip_coef 는 안 쓴다 — 노드 e 와 같게
  e 재계산  추정기 메시지가 들어온 시각마다, 그때까지 들어온 가장 최근 바퀴 메시지로 x̂_w (zero-order hold) — 노드의 on_state 시점과 같다.
            /torque_mpc/slip_fb 의 e 와 대조해 식이 노드와 같은지 확인한다
  지연    x̂_w(t) 와 x_e(t + L) 의 차 RMS 를 최소로 하는 L (±0.3 s, 5 ms 격자). L > 0 이면 x_e 가 늦다.
          수신 시각 (제어기가 보는 것) 과 header.stamp (측정 시각) 두 가지
  지연 몫  e RMS (L = 0) 와 x_e 를 L 만큼 당긴 뒤 RMS 의 차 → e 가운데 시간 어긋남이 만든 몫. 남는 것이 슬립·모델 차이
  영공간  T 의 null space n = [1, 1, −1, −1] (앞바퀴 둘 − 뒷바퀴 둘). T n = 0 이라 e 에는 안 보인다.
          s_null = r·(ω_FL + ω_FR − ω_RL − ω_RR)/4 [m/s] 로 그 크기를 같이 적는다
창      무장 (/mpc/armed) ∩ 움직임 (|x_e,xy| > 0.05 m/s 또는 |ω_z,e| > 0.1 rad/s)
"""
import os
import sys

import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_PLANT = os.path.join(HERE, "../../../src/amr_basic_stack/controller/mpc/config/torque_plant.yaml")
TOPICS = ("/wheel_states", "/state_estimator/odom", "/torque_mpc/slip_fb", "/mpc/armed")
AXES = ("v_x", "v_y", "ω_z")
UNITS = ("m/s", "m/s", "rad/s")
WHEELS = ("FL", "FR", "RL", "RR")
V_MOVE, W_MOVE = 0.05, 0.1
LAG_MAX, LAG_DT = 0.3, 0.005
# dataviz 기본 categorical 순서 (blue, orange, aqua, yellow)
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def run_params(bag):
    """r, k, δ0 [rad], slip_fb 켬 — run_info 가 없으면 소스 torque_plant.yaml 값."""
    pf = os.path.join(bag, "run_info/params/torque_mpc_node.yaml")
    p = (yaml.safe_load(open(pf)) or {}) if os.path.exists(pf) else {}     # 일찍 끊긴 판은 빈 파일
    p = p.get("/torque_mpc_node", {}).get("ros__parameters")
    if p:
        plant = yaml.safe_load(open(p["plant_file"]))
        d0, on, src = p.get("drive_yaw_offset_deg", 0.0), p.get("slip_fb_enable", False), "run_info"
    else:
        plant = yaml.safe_load(open(SRC_PLANT))
        d0, on, src = plant.get("drive", {}).get("yaw_offset_deg", 0.0), None, "소스 torque_plant.yaml (run_info 없음)"
    g = plant["geometry"]
    r, k = g["wheel_radius_m"], 0.5 * (g["wheel_base_m"] + g["robot_width_m"])
    return r, k, np.radians(d0), on, src


def read(bag):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types() if t.name in TOPICS}
    rd.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    d = {k: [] for k in TOPICS}
    while rd.has_next():
        name, data, ns = rd.read_next()
        m = deserialize_message(data, types[name])
        t = ns * 1e-9
        if name == "/wheel_states":
            d[name].append([t, stamp(m.header), *m.velocity[:4]])
        elif name == "/state_estimator/odom":
            tw = m.twist.twist
            d[name].append([t, stamp(m.header), tw.linear.x, tw.linear.y, tw.angular.z])
        elif name == "/torque_mpc/slip_fb":
            d[name].append([t, *m.data])
        else:
            d[name].append([t, float(m.data)])
    return {k: np.array(v, float) for k, v in d.items()}


def wheel_to_body(om, r, k, d0):
    """x̂_w = R(δ0)·T ω  (om: N×4 → N×3)."""
    T = 0.25 * r * np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / k, 1 / k, -1 / k, 1 / k]])
    v = om @ T.T
    c, s = np.cos(d0), np.sin(d0)
    return np.column_stack([c * v[:, 0] - s * v[:, 1], s * v[:, 0] + c * v[:, 1], v[:, 2]])


def hold(t_src, t_q):
    """t_q 마다 t_src ≤ t_q 인 마지막 인덱스 (없으면 −1)."""
    return np.searchsorted(t_src, t_q, side="right") - 1


def best_lag(t_w, xw, t_e, xe, t_mask, mask):
    """x̂_w(t) − x_e(t + L) RMS 최소 L. t_mask·mask 로 창을 고른다 (격자 위에서)."""
    grid = np.arange(max(t_w[0], t_e[0]) + LAG_MAX, min(t_w[-1], t_e[-1]) - LAG_MAX, LAG_DT)
    sel = np.interp(grid, t_mask, mask.astype(float)) > 0.5
    a = np.interp(grid[sel], t_w, xw)
    lags = np.arange(-LAG_MAX, LAG_MAX + 1e-9, LAG_DT)
    rms = np.array([np.sqrt(np.mean((a - np.interp(grid[sel] + L, t_e, xe)) ** 2)) for L in lags])
    i = int(np.argmin(rms))
    return lags[i], rms[int(np.argmin(np.abs(lags)))], rms[i]


def analyze(bag, show):
    bag = bag.rstrip("/")
    name = os.path.basename(bag)
    r, k, d0, on, src = run_params(bag)
    d = read(bag)
    ws, od, sf, arm = d["/wheel_states"], d["/state_estimator/odom"], d["/torque_mpc/slip_fb"], d["/mpc/armed"]
    if len(ws) == 0 or len(od) == 0:
        print(f"{name}: /wheel_states 또는 /state_estimator/odom 없음 — 건너뜀")
        return
    t0 = ws[0, 0]
    tw, tw_h, om = ws[:, 0] - t0, ws[:, 1] - t0, ws[:, 2:6]
    te, te_h, xe = od[:, 0] - t0, od[:, 1] - t0, od[:, 2:5]
    xw = wheel_to_body(om, r, k, d0)

    # 창: 무장 ∩ 움직임 (추정기 시각 위에서)
    armed = np.zeros(len(te), bool)
    if len(arm):
        j = hold(arm[:, 0] - t0, te)
        armed = (j >= 0) & (arm[np.maximum(j, 0), 1] > 0.5)
    moving = (np.hypot(xe[:, 0], xe[:, 1]) > V_MOVE) | (np.abs(xe[:, 2]) > W_MOVE)
    win = armed & moving

    # 노드와 같은 시점의 e: 추정기 메시지마다 가장 최근 바퀴 (수신 시각 ZOH)
    jw = hold(tw, te)
    ok = jw >= 0
    e = np.full_like(xe, np.nan)
    e[ok] = xw[jw[ok]] - xe[ok]
    s_null = r * (om[:, 0] + om[:, 1] - om[:, 2] - om[:, 3]) / 4.0
    w_on_wheels = np.interp(tw, te, win.astype(float)) > 0.5

    print(f"\n=== {name}  (기하 {src}: r {r:.4f} m, k {k:.4f} m, δ0 {np.degrees(d0):+.2f}°, slip_fb {'켬' if on else '끔' if on is not None else '?'})")
    print(f"  창 (무장 ∩ 움직임) {win.sum() / 100:.1f} s  / 무장 {armed.sum() / 100:.1f} s  / 전체 {te[-1]:.1f} s   (추정기 100 Hz 가정의 초)")
    if win.sum() < 50:
        print("  창이 0.5 s 미만 — 지표 생략")
    else:
        ew = e[win & ok]
        print("  축     x̂_w RMS   x_e RMS   기울기  e 평균    e RMS    | 지연 L (수신)  e RMS → L 보정 뒤  지연 몫 | 지연 L (stamp)")
        for i in range(3):
            a, b = xw[jw[win & ok], i], xe[win & ok, i]
            slope = a @ b / (b @ b)                                     # x̂_w ≈ slope·x_e (원점 지나는 최소제곱)
            L, r0, rL = best_lag(tw, xw[:, i], te, xe[:, i], te, win)
            Lh, _, _ = best_lag(tw_h, xw[:, i], te_h, xe[:, i], te, win)
            share = 1 - (rL / r0) ** 2 if r0 > 0 else np.nan        # RMS² (분산) 기준 몫
            print(f"  {AXES[i]:4s} {np.sqrt(np.mean(a ** 2)):8.4f}  {np.sqrt(np.mean(b ** 2)):8.4f}  {slope:6.3f}"
                  f"  {ew[:, i].mean():+8.4f}  {np.sqrt(np.mean(ew[:, i] ** 2)):8.4f}  | {L * 1e3:+7.0f} ms     "
                  f"{r0:.4f} → {rL:.4f}   {share * 100:5.1f} %  | {Lh * 1e3:+7.0f} ms   [{UNITS[i]}]")
        print(f"  영공간 s_null RMS {np.sqrt(np.mean(s_null[w_on_wheels] ** 2)):.4f} m/s  (앞 − 뒤 바퀴 표면 속도 차의 절반, e 에 안 보임)")

    # 노드 /torque_mpc/slip_fb 와 대조
    if len(sf):
        tf = sf[:, 0] - t0
        j = hold(te, tf)                          # slip_fb 직전에 받은 추정기 메시지의 e
        good = (j >= 0) & (sf[:, 3] > 0.5)        # sfb_fresh
        diff = np.abs(sf[good, 4:7] - e[j[good]])
        print(f"  노드 slip_fb 대조: {good.sum()} 표본 (fresh {good.mean() * 100:.1f} %), |e_노드 − e_재계산| 중앙 "
              + " ".join(f"{np.nanmedian(diff[:, i]):.1e}" for i in range(3))
              + "  · 99 % " + " ".join(f"{np.nanpercentile(diff[:, i], 99):.1e}" for i in range(3)))
        wf = np.interp(tf, te, win.astype(float)) > 0.5
        if wf.sum() >= 50:
            dt, tm = sf[wf, 7:11], sf[wf, 11:15]
            print("  창 안 RMS [mN·m]   Δτ " + " ".join(f"{WHEELS[i]} {np.sqrt(np.mean(dt[:, i] ** 2)) * 1e3:5.1f}" for i in range(4))
                  + "   τ_MPC " + " ".join(f"{WHEELS[i]} {np.sqrt(np.mean(tm[:, i] ** 2)) * 1e3:5.1f}" for i in range(4)))
    else:
        print("  /torque_mpc/slip_fb 없음 (11-27 이전 bag 이거나 PP 판) — 재계산 e 만")

    plot(name, te, xe, tw, xw, e, armed, sf, t0, show)


def plot(name, te, xe, tw, xw, e, armed, sf, t0, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [f for f in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic") if f in have] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    n = 4 if len(sf) else 3
    fig, axs = plt.subplots(n, 1, sharex=True, figsize=(13, 2.4 * n + 0.9), layout="constrained")
    # 무장 안 된 구간은 회색 바탕
    edges = np.flatnonzero(np.diff(np.r_[0, (~armed).astype(int), 0]))
    for i in range(3):
        ax = axs[i]
        ax.plot(tw, xw[:, i], color=C1, lw=1.0, label="x̂_w = R(δ0)·T ω  (바퀴 각속도 nominal)")
        ax.plot(te, xe[:, i], color=C2, lw=1.0, label="x_e  (추정기 피드백)")
        ax.plot(te, e[:, i], color="0.35", lw=0.7, label="e = x̂_w − x_e")
        ax.set_ylabel(f"{AXES[i]} [{UNITS[i]}]")
        ax.axhline(0, color="0.8", lw=0.6, zorder=0)
        ax.grid(alpha=0.25, lw=0.5)
    axs[0].legend(loc="lower left", bbox_to_anchor=(0.0, 1.0), fontsize=9, ncol=3, frameon=False)   # 축 위 바깥 (matplotlib 3.6)
    if len(sf):
        ax = axs[3]
        for i, c in enumerate((C1, C2, C3, C4)):
            ax.plot(sf[:, 0] - t0, sf[:, 7 + i] * 1e3, color=c, lw=0.9, label=WHEELS[i])
        ax.set_ylabel("Δτ = −K_v e [mN·m]")
        ax.axhline(0, color="0.8", lw=0.6, zorder=0)
        ax.grid(alpha=0.25, lw=0.5)
        ax.legend(loc="upper left", fontsize=8, ncol=4, frameon=False)
    for ax in axs:
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(te[a], te[min(b, len(te) - 1)], color="0.92", zorder=-1, lw=0)
    if armed.any():                                  # 측위 기다리는 구간은 잘라 무장 구간 ±1 s 만
        axs[-1].set_xlim(te[armed][0] - 1.0, te[armed][-1] + 1.0)
    axs[-1].set_xlabel("t [s] (첫 바퀴 메시지 기준, 회색 = 무장 전·후)")
    fig.suptitle(f"{name} — 바퀴 각속도 nominal vs 추정기 피드백", fontsize=11)
    out = os.path.join(HERE, f"{name}_nominal_vs_fb.png")
    fig.savefig(out, dpi=110)
    print(f"  그림 {out}")
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--show"]
    if not args:
        sys.exit(__doc__)
    for b in args:
        analyze(b, "--show" in sys.argv)
