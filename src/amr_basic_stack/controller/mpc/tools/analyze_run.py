#!/usr/bin/python3
"""실차 한 판 bag 분석 — record_all.sh(amrrec, amrall bag:=)로 남긴 bag 을 표준 지표로 요약한다 (2026-11-16).

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 tools/analyze_run.py <bag 폴더> [--plot 그림.png] [--wait 3.0]

보는 것 (판 하나에 무장 구간이 여러 번이면 구간마다)
  0. 실행 정보   run_info/args.txt, run_info/params/torque_mpc_node.yaml (W, w_delta, r_u, 외란 추정치)
  1. 추종         /torque_mpc/status 의 오차 (2026-11-16 부터 hold·stopped·U_s 칸도: hold 구간, stopped 비율, τ − U_s) [ex, ey, eθ](map) 를 추정 헤딩으로 몸체 축에 돌린 값. 앞 wait 초(기준 정지) / 그 뒤(주행).
                  최대·RMS, 1 cm 넘는 부호 바뀜, 토크 상한 닿음 비율, 틱간 Δτ RMS, 바퀴 토크 부호 바뀜/s
  2. 떨림         IMU 자이로 ω (추정기를 거치지 않은 값) RMS 와 주된 주파수
  3. 추정기 vs SLAM  속도: SLAM 포즈 중앙 차분(±0.1 s) 대비 지연(상호상관)·진폭비. 위치: SLAM 포즈 시각의 차이.
                  보정 잔차(/state_estimator/status) RMS·최대, 포즈 튐 수
  4. 정지 판정    바퀴 4 개가 모두 0.3 rad/s 아래로 0.6 s 넘게 멈춘 순간부터 추정기 속도가 hold 문턱(0.04 m/s, ω·k)
                  아래로 들어가 머물 때까지의 시간
  5. CAN          /wheel_states fresh=False 횟수 (바퀴별), /ak40_can/status 누적 누락·늦은 응답
  6. 슬립         바퀴 기구학 속도 − 추정기 속도 RMS (무장 구간)
  7. 로그         /rosout 의 WARN 이상을 노드·내용별로 센다
※ 오차는 추정기 기준이다. 실제 위치와의 차는 3. 의 SLAM 비교로 가늠한다.
"""
import argparse
import math
import os
import re
from collections import Counter, defaultdict

import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "config")
TOPICS = ["/torque_mpc/status", "/state_estimator/odom", "/state_estimator/status", "/tracked_pose", "/imu/base_link",
          "/wheel_states", "/wheel_torque_cmd", "/mpc/armed", "/ak40_can/status", "/rosout"]


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    have = [t for t in TOPICS if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=have))
    cls = {t: get_message(types[t]) for t in have}
    D = defaultdict(list)
    while r.has_next():
        topic, data, trx = r.read_next()
        m = deserialize_message(data, cls[topic]); tr = trx * 1e-9
        if topic == "/torque_mpc/status":
            row = list(m.data)                                              # [t, ex, ey, eθ, τ×4, τ̄, active, iters, fb, ms, ref_age, d̂×6,
            D[topic].append((row + [np.nan] * 6)[:26])                      #  hold, stopped, U_s×4]  (뒤 6 칸은 2026-11-16 부터. 없으면 NaN)
        elif topic == "/state_estimator/odom":
            D[topic].append([stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
                             m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])
        elif topic == "/state_estimator/status":
            D[topic].append(list(m.data))                                   # [t, e_x, e_y, e_ψ, b_ax, b_ay, b_g, 튐, 보정, 나이]
        elif topic == "/tracked_pose":
            D[topic].append([stamp(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)])
        elif topic == "/imu/base_link":
            D[topic].append([stamp(m.header), m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z])
        elif topic == "/wheel_states":
            D[topic].append([stamp(m.header), *m.velocity, *[float(f) for f in m.fresh]])
        elif topic == "/wheel_torque_cmd":
            D[topic].append([stamp(m.header), float(m.mode), *m.torque])
        elif topic == "/mpc/armed":
            D[topic].append([tr, float(m.data)])
        elif topic == "/ak40_can/status":
            D[topic].append(list(m.data))                                   # [t, mode, 지연 ms, fresh 수, 주기 ms, 누적 누락, 누적 늦음]
        elif topic == "/rosout":
            D[topic].append((tr, m.level, m.name, m.msg))
    return {k: (np.array(v, float) if k != "/rosout" else v) for k, v in D.items()}, types


def domfreq(x, dt):
    if len(x) < 16:
        return float("nan")
    x = x - x.mean(); X = np.abs(np.fft.rfft(x * np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), dt); m = f > 0.3
    return float(f[m][np.argmax(X[m])])


def sign_changes(e, thr=0.01):
    n, last = 0, 0
    for v in e:
        if abs(v) > thr:
            s = 1 if v > 0 else -1
            n += last != 0 and s != last
            last = s
    return n


def segments(t, mask, min_len):
    out, i, n = [], 0, len(t)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            if t[j - 1] - t[i] >= min_len:
                out.append((t[i], t[j - 1]))
            i = j
        else:
            i += 1
    return out


def xcorr_lag(a, b, maxlag):
    """a 가 b 보다 몇 표본 늦나 (a(i+L) ~ b(i)). 반환 (L, r)."""
    a = a - a.mean(); b = b - b.mean(); best = (-2.0, 0)
    for L in range(-maxlag, maxlag + 1):
        x, y = (a[L:], b[:len(b) - L]) if L >= 0 else (a[:L], b[-L:])
        d = math.sqrt(np.dot(x, x) * np.dot(y, y))
        if d > 0:
            best = max(best, (float(np.dot(x, y) / d), L))
    return best[1], best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--plot", default="")
    ap.add_argument("--wait", type=float, default=3.0, help="무장 뒤 기준이 서 있는 시간 [s] (figure8 start_delay)")
    a = ap.parse_args()

    plant = yaml.safe_load(open(os.path.join(CFG, "torque_plant.yaml")))
    est_cfg = yaml.safe_load(open(os.path.join(CFG, "torque_estimator.yaml")))["estimator"]
    r = plant["geometry"]["wheel_radius_m"]; k = 0.5 * (plant["geometry"]["wheel_base_m"] + plant["geometry"]["robot_width_m"])
    Tinv = (1 / r) * np.array([[1, -1, -k], [1, 1, k], [1, 1, -k], [1, -1, k]]); Tfwd = np.linalg.pinv(Tinv)
    rimu = np.array([float(est_cfg.get("imu_offset_x", 0)), float(est_cfg.get("imu_offset_y", 0))])
    hold_v = 0.04

    print(f"== {a.bag}")
    ri = os.path.join(a.bag, "run_info")
    if os.path.isdir(ri):
        args_txt = open(os.path.join(ri, "args.txt")).read().strip().splitlines() if os.path.isfile(os.path.join(ri, "args.txt")) else []
        print("  실행:", " | ".join(args_txt[:1] + args_txt[3:4]))
        pf = os.path.join(ri, "params", "torque_mpc_node.yaml")
        if os.path.isfile(pf):
            p = list(yaml.safe_load(open(pf)).values())[0]["ros__parameters"]
            print(f"  MPC: W {p.get('W')}  w_delta {p.get('w_delta')}  r_u {p.get('r_u')}  q_null {p.get('q_null')}  "
                  f"외란 {p.get('disturbance_enable')} α {p.get('disturbance_alpha')}  상한 {p.get('limit_mode')} μ̂ {p.get('mu_hat')}")
    else:
        print("  run_info/ 없음 (record_all.sh 로 녹화한 bag 이 아니다) — MPC 파라미터 모름")
    D, types = read_bag(a.bag)
    print("  토픽 수:", {kk.lstrip('/'): len(v) for kk, v in D.items()})

    O = D.get("/state_estimator/odom"); S = D.get("/torque_mpc/status"); W = D.get("/wheel_states"); I = D.get("/imu/base_link")
    P = D.get("/tracked_pose"); C = D.get("/wheel_torque_cmd")
    if O is None or not len(O):
        print("  추정기 출력 없음 — 끝"); return
    t0 = O[0, 0]

    # ── 무장(추종) 구간: /torque_mpc/status 가 나오는 연속 구간 (0.3 s 넘게 끊기면 나눈다)
    runs = []
    if S is not None and len(S):
        ts = S[:, 0]; brk = np.where(np.diff(ts) > 0.3)[0]
        starts = np.r_[0, brk + 1]; ends = np.r_[brk, len(ts) - 1]
        runs = [(ts[s0], ts[e0], slice(s0, e0 + 1)) for s0, e0 in zip(starts, ends) if ts[e0] - ts[s0] > 1.0]
    print(f"  추종 구간 {len(runs)} 개: " + ", ".join(f"{ra - t0:.1f}~{rb - t0:.1f} s" for ra, rb, _ in runs))
    rosout = D.get("/rosout", [])

    for ra, rb, sl in runs:
        st = S[sl]; t = st[:, 0] - ra
        psi = np.interp(st[:, 0], O[:, 0], np.unwrap(O[:, 3]))
        c, s = np.cos(psi), np.sin(psi)
        ebx = c * st[:, 1] + s * st[:, 2]; eby = -s * st[:, 1] + c * st[:, 2]; eth = np.degrees(st[:, 3])
        tau = st[:, 4:8]; tb = st[:, 8]
        holds = [m for m in rosout if ra - 0.5 <= m[0] <= rb + 1.0 and "hold 진입" in m[3]]
        print(f"\n  ── 추종 {ra - t0:.1f}~{rb - t0:.1f} s ({rb - ra:.1f} s)" + (f"   {holds[0][3]}" if holds else "   hold 진입 로그 없음"))
        print("     구간        | 앞 최대/RMS cm | 옆 최대/RMS cm | 헤딩 최대° | 상한 % | Δτ RMS | τ 부호/s | 자이로 ω RMS °/s (Hz) | 1cm 부호 앞/옆")
        for name, (x0, x1) in [("대기(기준 정지)", (0, a.wait)), ("주행", (a.wait, 1e9)), ("전체", (0, 1e9))]:
            m = (t >= x0) & (t < x1)
            if m.sum() < 20:
                continue
            sat = (np.abs(tau[m]).max(1) > 0.98 * tb[m]).mean() * 100
            dtau = np.sqrt(np.mean(np.diff(tau[m], axis=0) ** 2))
            zc = (np.diff(np.sign(tau[m]), axis=0) != 0).sum(0).mean() / max(t[m][-1] - t[m][0], 1e-6)
            wi = (I[:, 0] >= ra + x0) & (I[:, 0] < min(ra + x1, rb)) if I is not None else None
            wrms = np.degrees(np.std(I[wi, 3])) if wi is not None and wi.sum() > 20 else float("nan")
            wf = domfreq(I[wi, 3], 0.005) if wi is not None and wi.sum() > 20 else float("nan")
            print(f"     {name:12s}| {np.abs(ebx[m]).max()*100:4.1f} / {np.sqrt(np.mean(ebx[m]**2))*100:4.2f}  | "
                  f"{np.abs(eby[m]).max()*100:4.1f} / {np.sqrt(np.mean(eby[m]**2))*100:4.2f}  | {np.abs(eth[m]).max():5.2f}    | "
                  f"{sat:4.0f}   | {dtau:.4f} | {zc:5.1f}    | {wrms:5.1f} ({wf:.1f})          | {sign_changes(ebx[m])}/{sign_changes(eby[m])}")
        print(f"     외란 추정치 d̂ |최대| ẍ {np.abs(st[:, 17]).max():.3f} ÿ {np.abs(st[:, 18]).max():.3f} θ̈ {np.abs(st[:, 19]).max():.3f}   "
              f"QP fallback {int(st[:, 11].sum())}  풀이 최대 {st[:, 12].max():.2f} ms")
        if not np.isnan(st[:, 20]).all():
            hold = st[:, 20] > 0.5; stp = st[:, 21] > 0.5; trk = ~hold
            edges = np.flatnonzero(np.diff(np.r_[0, hold.astype(int), 0]))
            iv = [(t[a], t[b - 1]) for a, b in zip(edges[::2], edges[1::2])]
            dev = np.sqrt(np.mean((st[trk, 4:8] - st[trk, 22:26]) ** 2)) if trk.any() else float("nan")
            print(f"     hold 구간 (추종 시작 기준): " + (", ".join(f"{a:.1f}~{b:.1f} s" for a, b in iv) or "없음")
                  + f" | 추종 중 stopped 판정 {stp[trk].mean()*100:.0f} % | 토크 − 목표 입력 U_s RMS {dev:.3f} N·m")

        # 3. 추정기 vs SLAM (이 구간)
        if P is not None and len(P) > 10:
            _, ix = np.unique(P[:, 0], return_index=True); Pp = P[ix]
            mp = (Pp[:, 0] >= ra) & (Pp[:, 0] <= rb); Pp = Pp[mp]
            if len(Pp) > 10:
                # SLAM 포즈(IMU 점) → 중심으로, 그 시각의 추정 위치와 차
                cx = Pp[:, 1] - (np.cos(Pp[:, 3]) * rimu[0] - np.sin(Pp[:, 3]) * rimu[1])
                cy = Pp[:, 2] - (np.sin(Pp[:, 3]) * rimu[0] + np.cos(Pp[:, 3]) * rimu[1])
                dxy = np.hypot(cx - np.interp(Pp[:, 0], O[:, 0], O[:, 1]), cy - np.interp(Pp[:, 0], O[:, 0], O[:, 2]))
                # SLAM 속도: ±0.1 s 중앙 차분 → 몸체 축
                tp = Pp[:, 0]; vs = []
                for i in range(len(tp)):
                    j0 = np.searchsorted(tp, tp[i] - 0.1); j1 = min(np.searchsorted(tp, tp[i] + 0.1), len(tp) - 1)
                    if j1 <= j0:
                        vs.append([np.nan, np.nan]); continue
                    dt = tp[j1] - tp[j0]; vx = (cx[j1] - cx[j0]) / dt; vy = (cy[j1] - cy[j0]) / dt
                    vs.append([math.cos(Pp[i, 3]) * vx + math.sin(Pp[i, 3]) * vy, -math.sin(Pp[i, 3]) * vx + math.cos(Pp[i, 3]) * vy])
                vs = np.array(vs); ok = ~np.isnan(vs[:, 0])
                g = np.arange(ra + 0.2, rb - 0.2, 0.01)
                out = []
                for j, lab in ((0, "vx"), (1, "vy")):
                    e = np.interp(g, O[:, 0], O[:, 4 + j]); sv = np.interp(g, tp[ok], vs[ok, j])
                    L, rr = xcorr_lag(e, sv, 25)
                    ss = np.roll(sv, L); amp = np.dot(e[30:-30], ss[30:-30]) / max(np.dot(ss[30:-30], ss[30:-30]), 1e-12)
                    out.append(f"{lab} 지연 {L*10:+d} ms 진폭비 {amp:.2f} (r {rr:.2f}, 차 RMS {np.sqrt(np.mean((e-sv)**2)):.3f} m/s)")
                print(f"     추정기 − SLAM: 위치 차 RMS {np.sqrt(np.mean(dxy**2))*100:.1f} 최대 {dxy.max()*100:.1f} cm | " + " | ".join(out))
        E = D.get("/state_estimator/status")
        if E is not None and len(E):
            me = (E[:, 0] >= ra) & (E[:, 0] <= rb)
            if me.sum():
                res = np.hypot(E[me, 1], E[me, 2])
                print(f"     SLAM 보정 잔차 RMS {np.sqrt(np.mean(res**2))*100:.2f} 최대 {res.max()*100:.2f} cm (→ v 계단 최대 ≈ K_v·e), "
                      f"yaw 잔차 최대 {np.degrees(np.abs(E[me, 3]).max()):.2f}°, 포즈 튐 누적 {int(E[me, 7].max())}")
        # 5. CAN·6. 슬립 (이 구간)
        if W is not None and len(W):
            mw = (W[:, 0] >= ra) & (W[:, 0] <= rb)
            miss = (W[mw, 5:9] < 0.5).sum(0)
            Vw = W[mw, 1:5] @ Tfwd.T
            ve = np.c_[[np.interp(W[mw, 0], O[:, 0], O[:, 4 + j]) for j in range(3)]].T
            sl_rms = np.sqrt(np.mean((Vw - ve) ** 2, 0))
            print(f"     CAN 응답 없음(fresh=False) [FL FR RL RR] {miss.tolist()} / {mw.sum()} 주기 | "
                  f"슬립(바퀴 기구학 − 추정기) RMS vx {sl_rms[0]:.3f} vy {sl_rms[1]:.3f} m/s ω {sl_rms[2]:.3f} rad/s")

    # 4. 정지 판정 (판 전체)
    if W is not None and len(W):
        wmax = np.abs(W[:, 1:5]).max(1); tw = W[:, 0]
        ev = []
        for sa, sb in segments(tw, wmax < 0.3, 0.6):
            pm = (tw > sa - 0.5) & (tw < sa)
            if (wmax[pm] > 3.0).any():
                ev.append(sa)
        if ev:
            vabs = np.hypot(O[:, 4], O[:, 5]); okk = (vabs < hold_v) & (np.abs(O[:, 6]) * k < hold_v)
            txt = []
            for e0 in ev:
                m = (O[:, 0] >= e0) & (O[:, 0] <= e0 + 2.0)
                bad = np.where(~okk[m])[0]
                lag = (O[m, 0][bad[-1]] - e0 + 0.01) if len(bad) else 0.0
                txt.append(f"{e0 - t0:.1f} s: {lag:.2f} s (최대 |v| {vabs[m].max():.3f})")
            print(f"\n  정지 판정 지연 (바퀴 정지 → 추정기 |v| < {hold_v}): " + " / ".join(txt))
    K = D.get("/ak40_can/status")
    if K is not None and len(K):
        print(f"  CAN 누적: 누락 {int(K[-1, 5])} 바퀴·주기, 늦은 응답 {int(K[-1, 6])}, 명령→응답 지연 중앙 {np.nanmedian(K[:, 2]):.1f} ms")
    # 7. 로그
    warn = Counter()
    for tr, lvl, name, msg in rosout:
        if lvl >= 30:
            warn[(name, re.sub(r"[-+]?\d+(\.\d+)?", "#", msg)[:90])] += 1
    if warn:
        print("  로그 WARN 이상 (노드: 내용 × 횟수):")
        for (name, msg), n in warn.most_common(12):
            print(f"    {name}: {msg} × {n}")

    if a.plot and runs:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, len(runs), figsize=(7 * len(runs), 11), sharex="col", squeeze=False)
        B = np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1, 1, -1, 1], [1, 1, -1, -1]]) / 2.0
        for ci, (ra, rb, sl) in enumerate(runs):
            st = S[sl]; t = st[:, 0] - ra; psi = np.interp(st[:, 0], O[:, 0], np.unwrap(O[:, 3])); c, s = np.cos(psi), np.sin(psi)
            ax[0, ci].plot(t, (c * st[:, 1] + s * st[:, 2]) * 100, label="body-x err [cm]"); ax[0, ci].plot(t, (-s * st[:, 1] + c * st[:, 2]) * 100, label="body-y err [cm]")
            mo = (O[:, 0] >= ra) & (O[:, 0] <= rb)
            ax[1, ci].plot(O[mo, 0] - ra, O[mo, 4], label="est vx"); ax[1, ci].plot(O[mo, 0] - ra, O[mo, 5], label="est vy")
            if W is not None:
                mw = (W[:, 0] >= ra) & (W[:, 0] <= rb); Vw = W[mw, 1:5] @ Tfwd.T
                ax[1, ci].plot(W[mw, 0] - ra, Vw[:, 0], color="0.6", lw=.6, label="wheel vx")
            Cc = st[:, 4:8] @ B.T
            for j, nm in enumerate(["x force", "y force", "yaw", "null"]):
                ax[2, ci].plot(t, Cc[:, j], lw=.6, label=nm)
            if I is not None:
                mi = (I[:, 0] >= ra) & (I[:, 0] <= rb); ax[3, ci].plot(I[mi, 0] - ra, np.degrees(I[mi, 3]), lw=.5, label="gyro ω [deg/s]")
            for rr in range(4):
                ax[rr, ci].axvspan(0, a.wait, color="0.92"); ax[rr, ci].grid(alpha=.3); ax[rr, ci].legend(fontsize=7, ncol=2)
            ax[0, ci].set_title(f"{os.path.basename(a.bag.rstrip('/'))}  {ra - t0:.1f}~{rb - t0:.1f} s", fontsize=9)
            ax[3, ci].set_xlabel("t from tracking start [s] (grey = reference standing)")
        plt.tight_layout(); plt.savefig(a.plot, dpi=70); print(f"  그림: {a.plot}")


if __name__ == "__main__":
    main()
