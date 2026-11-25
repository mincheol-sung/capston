#!/usr/bin/python3
"""추정기 kf vs slam — 시뮬 참값으로 비교 (2026-11-21). runs/<라벨>/ bag 을 읽는다.

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_sim/2026-11-21_est_kf_vs_slam/analyze.py

창: 무장 뒤 /torque_mpc/status 가 나오는 동안 (기준열이 도는 동안), 참값 속도 > 0.05 m/s 인 "주행" 구간.
  추종   참값 포즈 vs 그 시각 기준(기준열 첫 행 = stamp + dt 시각, 보간): |e_xy|·헤딩.  경로 거리 = 참값 위치 → /figure8/path 꺾은선 최단 거리
  추정기 /state_estimator/odom vs 참값 (/sim/ground_truth, 둘 다 로봇 중심·몸체 속도): 위치·헤딩·속도·각속도 오차,
         v_x 지연 = 추정 v_x 와 참 v_x 의 상호상관이 가장 큰 시간 차
  토크   /torque_mpc/cmd_raw (마찰 보상 전 순토크): 떨림 = τ − 0.2 s 이동평균 의 RMS, 틱간 Δτ RMS, 상한 닿음 = |τ_i| ≥ 0.98 τ̄_i
         (τ̄_i 는 run 로그의 "쓰는 τ̄ [...]")
  슬립   /slip_logger/slip 의 참값 λ (인덱스 9~12): 큰 슬립 = max_i|λ_i| > 0.5 시간 %, 미끄럼 > 0.1 %, λ 최대, λ RMS,
         큰 슬립 중 헛돎(λ > 0) 몫 — 부호로 헛돎·끌림을 나눈다 (메모리 analysis-include-slip)
출력: 표 (stdout), summary.json, fig_est_kf_vs_slam.png
"""
import glob
import json
import math
import os
import re

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def read(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in types}
    while r.has_next():
        n, d, t = r.read_next()
        D[n].append((t * 1e-9, deserialize_message(d, types[n])))
    return D


def odom_arr(L):
    return np.array([[st(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation),
                      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in L])


def tau_bar_from_log(lab):
    try:
        txt = open(os.path.join(RUNS, lab + ".log"), errors="replace").read()
        g = re.findall(r"쓰는 τ̄ \[([0-9.\s]+)\]", txt)
        if g:
            return np.array([float(x) for x in g[-1].split()])
        a = re.findall(r"상한 τ̄ ([0-9.]+) N·m", txt)          # 비교군 A (wheel_pi_node): 네 바퀴 같은 값
        return np.full(4, float(a[-1])) if a else None
    except OSError:
        return None


def met(lab):
    b = os.path.join(RUNS, lab)
    if not os.path.exists(os.path.join(b, "metadata.yaml")):
        return None
    D = read(b)
    G, E = odom_arr(D["/sim/ground_truth"]), odom_arr(D["/state_estimator/odom"])
    arm = next((t for t, m in D.get("/mpc/armed", []) if m.data), None)
    is_a = bool(D.get("/pure_pursuit/status")) and not D.get("/torque_mpc/status")
    ST, TQ = ("/pure_pursuit/status", "/wheel_pi/cmd_raw") if is_a else ("/torque_mpc/status", "/torque_mpc/cmd_raw")
    stt = [t for t, _ in D.get(ST, [])]
    if arm is None or not stt or len(E) == 0:
        return dict(lab=lab, ok=False)
    # bag 수신 시각 → stamp 로 창 맞추기: 상태 stamp 와 수신 시각 차는 작다 (시뮬, 같은 시계)
    t0, t1 = arm, max(stt)
    R = []
    for _, m in D["/dra_mppi/reference_trajectory"]:
        if m.references:   # PlanarReferenceArray.msg: references[i] 의 목표 시각 = header.stamp + (i+1)·dt (reference.hpp 와 같은 규약)
            r0 = m.references[0]
            R.append((st(m.header) + m.dt, r0.x, r0.y, r0.yaw))
    R = np.array(sorted(R))
    win = (G[:, 0] >= t0) & (G[:, 0] <= t1) & (np.hypot(G[:, 4], G[:, 5]) > 0.05) & (G[:, 0] >= E[0, 0])
    g = G[win]
    rx, ry = np.interp(g[:, 0], R[:, 0], R[:, 1]), np.interp(g[:, 0], R[:, 0], R[:, 2])
    ryaw = np.interp(g[:, 0], R[:, 0], np.unwrap(R[:, 3]))
    exy = np.hypot(g[:, 1] - rx, g[:, 2] - ry)
    eth = np.degrees(np.abs(wrap(g[:, 3] - ryaw)))
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in D["/figure8/path"][-1][1].poses])
    # 꺾은선(선분)까지 최단 거리 — 정사각형 path 는 꼭짓점 5 개뿐이라 점까지 거리로 재면 변 가운데서 크게 나온다
    a_, b_ = path[:-1], path[1:]
    ab = b_ - a_
    q = g[:, None, 1:3] - a_[None]
    tt = np.clip((q * ab[None]).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12)[None], 0.0, 1.0)
    dist = np.linalg.norm(q - tt[..., None] * ab[None], axis=-1).min(1)
    # 추정기 − 참값 (참값 stamp 에 보간)
    ei = lambda c: np.interp(g[:, 0], E[:, 0], E[:, c] if c != 3 else np.unwrap(E[:, 3]))
    pe = np.hypot(ei(1) - g[:, 1], ei(2) - g[:, 2])
    ye = np.degrees(np.abs(wrap(ei(3) - g[:, 3])))
    # 추정 위치 오차를 진행 방향(앞 +)·옆으로 나눔: 참 속도 방향 u = R(ψ)[v_x, v_y]/|v|
    c, s_ = np.cos(g[:, 3]), np.sin(g[:, 3])
    vm = np.stack([c * g[:, 4] - s_ * g[:, 5], s_ * g[:, 4] + c * g[:, 5]], 1)
    u = vm / np.linalg.norm(vm, axis=1, keepdims=True)
    e_m = np.stack([ei(1) - g[:, 1], ei(2) - g[:, 2]], 1)
    along, cross = (e_m * u).sum(1), e_m[:, 0] * (-u[:, 1]) + e_m[:, 1] * u[:, 0]
    # 제어기가 본 오차 (기준 − 추정, /torque_mpc/status 앞 칸): 참값 오차 = 제어기 오차 + 추정 오차
    Sm = np.array([[t, *m.data[1:3]] for t, m in D[ST] if t0 <= t <= t1])
    vSm = np.interp(Sm[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5]))
    ectrl = np.hypot(Sm[vSm > 0.05, 1], Sm[vSm > 0.05, 2])
    ve = np.hypot(ei(4) - g[:, 4], ei(5) - g[:, 5])
    we = np.abs(ei(6) - g[:, 6])
    # v_x 지연: 같은 격자(10 ms)에서 상호상관
    tg = np.arange(g[0, 0], g[-1, 0], 0.01)
    a = np.interp(tg, G[:, 0], G[:, 4]); bvx = np.interp(tg, E[:, 0], E[:, 4])
    a, bvx = a - a.mean(), bvx - bvx.mean()
    lags = np.arange(-30, 61)
    cc = [np.dot(a[max(0, -l):len(a) - max(0, l)], bvx[max(0, l):len(bvx) - max(0, -l)]) for l in lags]
    lag_ms = 10.0 * lags[int(np.argmax(cc))]
    # 토크
    # 토크: MODE_TORQUE(0) 만 (DAMPING 은 torque[] 를 안 쓴다), 주행 구간
    T = np.array([[t, *m.torque] for t, m in D[TQ] if t0 <= t <= t1 and m.mode == 0])
    T = T[np.interp(T[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5])) > 0.05]
    tau = T[:, 1:5]
    k = 20
    hp = (tau - np.array([np.convolve(tau[:, i], np.ones(k) / k, "same") for i in range(4)]).T)[k:-k]
    tb = tau_bar_from_log(lab)
    sat = float(100 * np.mean(np.abs(tau) >= 0.98 * tb[None, :])) if tb is not None else float("nan")
    # 슬립 (참값 λ)
    S = np.array([[t, *m.data[9:13]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 13 and t0 <= t <= t1])
    vS = np.interp(S[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5]))
    lam = S[vS > 0.05, 1:5]
    lam = lam[np.all(np.isfinite(lam), axis=1)]
    al = np.abs(lam); mx = al.max(1); big = mx > 0.5
    sgn = np.sign(lam[np.arange(len(lam)), al.argmax(1)])
    faults = [m.data for _, m in D.get("/fault_monitor/fault", [])]
    # 바퀴 각속도 (/wheel_states, A 판부터 기록): 정격 38.746 rad/s 넘는 시간 % (네 바퀴 중 하나라도), 최대 |ω|
    W = np.array([[t, *m.velocity] for t, m in D.get("/wheel_states", []) if t0 <= t <= t1])
    if len(W):
        W = W[np.interp(W[:, 0], G[:, 0], np.hypot(G[:, 4], G[:, 5])) > 0.05]
    w_over = float(100 * np.mean(np.abs(W[:, 1:5]).max(1) > 38.746)) if len(W) else float("nan")
    w_max = float(np.abs(W[:, 1:5]).max()) if len(W) else float("nan")
    jumps = int(D["/state_estimator/status"][-1][1].data[7]) if D.get("/state_estimator/status") else -1
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return dict(lab=lab, ok=True, ctrl="A" if is_a else "C", faults=faults, jumps=jumps, T=float(g[-1, 0] - g[0, 0]),
                w_over=w_over, w_max=w_max, tau_max=float(np.abs(tau).max()), tau_bar=float(tb.max()) if tb is not None else float("nan"),
                exy_rms=rms(exy), exy_max=float(exy.max()), eth_rms=rms(eth), dist_rms=rms(dist), dist_max=float(dist.max()),
                est_p_rms=rms(pe), est_p_max=float(pe.max()), est_yaw_rms=rms(ye), est_v_rms=rms(ve), est_w_rms=rms(we), lag_ms=lag_ms,
                est_along_mean=float(along.mean()), est_along_rms=rms(along), est_cross_rms=rms(cross), ctrl_rms=rms(ectrl),
                tau_hp=rms(hp), dtau=rms(np.diff(tau, axis=0)), sat=sat,
                big=float(100 * big.mean()), slip01=float(100 * (mx > 0.1).mean()), lmax=float(mx.max()), lrms=rms(lam),
                drive=float(100 * (sgn[big] > 0).mean()) if big.any() else float("nan"),
                series=dict(t=(g[:, 0] - g[0, 0]).tolist(), pe=pe.tolist(), ve=ve.tolist(), exy=exy.tolist(),
                            ts=(S[vS > 0.05, 0][np.all(np.isfinite(S[vS > 0.05, 1:5]), axis=1)] - g[0, 0]).tolist(), lmx=mx.tolist()))


ROWS = [("exy_rms", "추종 |e| RMS cm", 100, "{:.1f}"), ("exy_max", "추종 |e| 최대 cm", 100, "{:.1f}"),
        ("dist_rms", "경로 거리 RMS cm", 100, "{:.1f}"), ("eth_rms", "헤딩 오차 RMS °", 1, "{:.2f}"),
        ("ctrl_rms", "제어기가 본 오차 (기준−추정) RMS cm", 100, "{:.1f}"),
        ("est_along_mean", "추정 오차 진행 방향 평균 cm (+ 앞섬)", 100, "{:+.2f}"), ("est_along_rms", "추정 오차 진행 방향 RMS cm", 100, "{:.2f}"),
        ("est_cross_rms", "추정 오차 옆 방향 RMS cm", 100, "{:.2f}"),
        ("est_p_rms", "추정 위치 오차 RMS cm", 100, "{:.2f}"), ("est_p_max", "추정 위치 오차 최대 cm", 100, "{:.1f}"),
        ("est_yaw_rms", "추정 헤딩 오차 RMS °", 1, "{:.2f}"), ("est_v_rms", "추정 속도 오차 RMS cm/s", 100, "{:.1f}"),
        ("est_w_rms", "추정 각속도 오차 RMS rad/s", 1, "{:.3f}"), ("lag_ms", "추정 v_x 지연 ms", 1, "{:.0f}"),
        ("tau_hp", "토크 떨림 RMS N·m", 1, "{:.4f}"), ("dtau", "틱간 Δτ RMS N·m", 1, "{:.4f}"), ("sat", "상한 닿음 %", 1, "{:.1f}"),
        ("big", "큰 슬립 % (max|λ|>0.5)", 1, "{:.1f}"), ("slip01", "미끄럼 % (>0.1)", 1, "{:.1f}"), ("lmax", "λ 최대", 1, "{:.2f}"),
        ("lrms", "λ RMS", 1, "{:.3f}"), ("drive", "큰 슬립 중 헛돎(λ>0) %", 1, "{:.0f}")]
ROWS_AC = [("exy_rms", "참값 추종 |e| RMS cm", 100, "{:.1f}"), ("exy_max", "참값 추종 |e| 최대 cm", 100, "{:.1f}"),
           ("dist_rms", "contour error RMS cm", 100, "{:.1f}"), ("dist_max", "contour error 최대 cm", 100, "{:.1f}"),
           ("eth_rms", "헤딩 오차 RMS °", 1, "{:.2f}"), ("ctrl_rms", "제어기가 본 오차 (기준−추정) RMS cm", 100, "{:.1f}"),
           ("est_p_rms", "추정 위치 오차 RMS cm", 100, "{:.2f}"), ("est_v_rms", "추정 속도 오차 RMS cm/s", 100, "{:.1f}"),
           ("tau_hp", "토크 떨림 RMS N·m", 1, "{:.4f}"), ("dtau", "틱간 Δτ RMS N·m", 1, "{:.4f}"),
           ("tau_max", "최대 |τ| N·m", 1, "{:.3f}"), ("tau_bar", "토크 상한 τ̄ N·m (최대 바퀴)", 1, "{:.3f}"), ("sat", "상한 닿음 %", 1, "{:.1f}"),
           ("w_over", "|ω| > 정격 38.7 rad/s %", 1, "{:.1f}"), ("w_max", "최대 |ω| rad/s", 1, "{:.1f}"),
           ("big", "큰 슬립 % (max|λ|>0.5)", 1, "{:.1f}"), ("slip01", "미끄럼 % (>0.1)", 1, "{:.1f}"), ("lmax", "λ 최대", 1, "{:.2f}"),
           ("lrms", "λ RMS", 1, "{:.3f}"), ("drive", "큰 슬립 중 헛돎(λ>0) %", 1, "{:.0f}")]


def cell(ms, key, sc, fmt):
    v = [m[key] * sc for m in ms if np.isfinite(m[key])]
    if not v:
        return "—"
    return fmt.format(np.mean(v)) + (f" ({fmt.format(min(v))}~{fmt.format(max(v))})" if len(v) > 1 else "")


if __name__ == "__main__":
    labs = sorted({os.path.basename(p) for p in glob.glob(os.path.join(RUNS, "*")) if os.path.isdir(p) and not os.path.basename(p).startswith("smoke")})
    res = {lab: met(lab) for lab in labs}
    groups = {}
    for lab, m in res.items():
        if not m:
            continue
        parts = lab.split("_")                      # C: <경로>_<모드>_r<n>,  A: A<e|w>_<경로>_<모드>_r<n>
        ctrl, (path, mode) = ("C", parts[:2]) if len(parts) == 3 else (parts[0], parts[1:3])
        groups.setdefault((ctrl, path, mode), []).append(m)
    for lab, m in res.items():
        if m and (not m["ok"] or m["faults"]):
            print(f"  ! {lab}: ok={m['ok']} faults={m.get('faults')}")
    for path, name in (("c8", "circles8 W2.0 tangent 0.7 m/s"), ("sq", "정사각형 1.5 m, 가속 1.5, 최고 1.5 m/s")):
        kf, sl = [m for m in groups.get(("C", path, "kf"), []) if m["ok"]], [m for m in groups.get(("C", path, "slam"), []) if m["ok"]]
        print(f"\n### {name}  (kf {len(kf)} 판 / slam {len(sl)} 판, 값은 판별 '/' 로)")
        print("| 지표 | kf | slam | slam / kf (평균) |")
        print("|---|---|---|---|")
        for key, lab, sc, fmt in ROWS:
            a = [m[key] * sc for m in kf]; b = [m[key] * sc for m in sl]
            ratio = (np.nanmean(b) / np.nanmean(a)) if a and b and np.nanmean(a) not in (0,) else float("nan")
            print(f"| {lab} | {' / '.join(fmt.format(x) for x in a)} | {' / '.join(fmt.format(x) for x in b)} | {ratio:.2f} |")
        print(f"| 튐 수 | {' / '.join(str(m['jumps']) for m in kf)} | {' / '.join(str(m['jumps']) for m in sl)} | |")
    # A vs C — 추정기 모드별. 값 = 평균 (최소~최대), 판 수는 머리줄
    CT = (("C", "C (MPC, friction τ̄)"), ("Ae", "A (PP + PI, 되먹임 추정기)"), ("Aw", "A (PP + PI, 되먹임 엔코더)"))
    for path, name in (("c8", "circles8 W2.0 tangent 0.7 m/s"), ("sq", "정사각형 1.5 m, 가속 1.5, 최고 1.5 m/s")):
        for mode in ("kf", "slam"):
            gs = {c: [m for m in groups.get((c, path, mode), []) if m["ok"]] for c, _ in CT}
            print(f"\n### A vs C — {name}, est_mode {mode}")
            nf = {c: sum(1 for m in gs[c] if m["faults"]) for c, _ in CT}
            print("| 지표 | " + " | ".join(f"{n} ({len(gs[c])} 판" + (f", **정지 {nf[c]}**" if nf[c] else "") + ")" for c, n in CT) + " |")
            print("|---|" + "---|" * len(CT))
            for key, lab, sc, fmt in ROWS_AC:
                print(f"| {lab} | " + " | ".join(cell(gs[c], key, sc, fmt) for c, _ in CT) + " |")
    json.dump({k: {kk: vv for kk, vv in v.items() if kk != "series"} if v else None for k, v in res.items()},
              open(os.path.join(HERE, "summary.json"), "w"), ensure_ascii=False, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Noto Sans CJK JP", "font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(4, 2, figsize=(12, 10), sharex="col")
    for c, path in enumerate(("c8", "sq")):
        for mode, col in (("kf", "C0"), ("slam", "C3")):
            m = res.get(f"{path}_{mode}_r1")
            if not m or not m["ok"]:
                continue
            s = m["series"]
            ax[0, c].plot(s["t"], np.array(s["exy"]) * 100, col, lw=0.8, label=mode)
            ax[1, c].plot(s["t"], np.array(s["pe"]) * 100, col, lw=0.8, label=mode)
            ax[2, c].plot(s["t"], np.array(s["ve"]) * 100, col, lw=0.8, label=mode)
            ax[3, c].plot(s["ts"], s["lmx"], col, lw=0.6, label=mode)
        ax[0, c].set_title({"c8": "circles8 W2.0 0.7 m/s (r1)", "sq": "정사각형 1.5 m/s (r1)"}[path])
        ax[0, c].set_ylabel("참값 추종 |e| [cm]"); ax[1, c].set_ylabel("추정 위치 오차 [cm]")
        ax[2, c].set_ylabel("추정 속도 오차 [cm/s]"); ax[3, c].set_ylabel("참값 max|λ|"); ax[3, c].axhline(0.5, color="k", ls=":", lw=0.6)
        ax[3, c].set_xlabel("주행 시간 [s]")
        for a_ in ax[:, c]:
            a_.legend(fontsize=8, loc="upper right")
    fig.suptitle("추정기 kf vs slam — 시뮬 참값 (C, velocity_source estimator, 바닥 μ 0.45, 잡음 1 배)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_est_kf_vs_slam.png"), dpi=90)
    print("\n그림: fig_est_kf_vs_slam.png")

    # ── A vs C 그림: 지표 4 개 × 경로 2 개. 막대 = 평균, 점 = 판. x 축 = est_mode × 제어기 (색은 제어기, x 라벨에도 이름)
    COL = {"C": "#2a78d6", "Ae": "#eb6834", "Aw": "#1baf7a"}      # dataviz 기본 categorical 1~3 번 (고정 순서)
    NAME = {"C": "C", "Ae": "A·PI 추정기", "Aw": "A·PI 엔코더"}
    MET = (("exy_rms", "참값 추종 |e| RMS [cm]", 100), ("dist_rms", "contour error RMS [cm]", 100),
           ("tau_hp", "토크 떨림 RMS [N·m]", 1), ("slip01", "미끄럼 시간 % (max|λ|>0.1)", 1))
    fig, ax = plt.subplots(len(MET), 2, figsize=(12, 11))
    for c, (path, ttl) in enumerate((("c8", "circles8 W2.0 0.7 m/s"), ("sq", "정사각형 1.5 m/s"))):
        for r, (key, ylab, sc) in enumerate(MET):
            a_ = ax[r, c]; xt, xl = [], []
            for i, mode in enumerate(("kf", "slam")):
                for j, ct in enumerate(("C", "Ae", "Aw")):
                    x = i * 4 + j
                    ms_ = [m for m in groups.get((ct, path, mode), []) if m["ok"]]
                    v = [m[key] * sc for m in ms_ if np.isfinite(m[key])]
                    if ms_ and all(m["faults"] for m in ms_):          # 전부 정지(발산) — 값이 축을 망가뜨리니 막대 대신 글자
                        a_.text(x, 0, "발산\n정지", ha="center", va="bottom", fontsize=7, color=COL[ct])
                    elif v:
                        a_.bar(x, np.mean(v), 0.8, color=COL[ct], alpha=0.35, edgecolor="none", label=NAME[ct] if (i == 0 and r == 0) else None)
                        a_.plot([x] * len(v), v, "o", ms=4, color=COL[ct], mec="white", mew=0.8)
                    xt.append(x); xl.append(f"{NAME[ct]}\n{mode}")
            a_.set_xticks(xt); a_.set_xticklabels(xl, fontsize=7); a_.set_ylabel(ylab)
            a_.grid(axis="y", lw=0.4, alpha=0.4); a_.set_axisbelow(True)
            if r == 0:
                a_.set_title(ttl)
    ax[0, 0].legend(fontsize=8, loc="upper left")
    fig.suptitle("A vs C — 추정기 모드별 (시뮬 참값, 바닥 μ 0.45, 잡음 1 배). 막대 = 평균, 점 = 판")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_A_vs_C.png"), dpi=90)
    print("그림: fig_A_vs_C.png")
