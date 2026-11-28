"""Q 튜닝 판 (circles8, laps N) 바퀴별 분석 — bag 을 직접 읽어 판·바퀴마다 추종·슬립·토크 지표를 <출력 이름>_rows.npy 로 저장.
  시각: 기준·추정기·SLAM 은 header, 슬립 (/slip_logger/slip data[0] = wheel_states header), 되먹임 (/torque_mpc/slip_fb data[0] = now),
        무장·모터 명령은 bag 기록 시각 (실차는 같은 시계, ms 차).
  바퀴 경계 = 기준 호 길이 s(t) = ∫ v_ref dt 를 laps 로 등분 (circles8 은 0 ~ 2π·laps 를 한 번에 만든다 — 바퀴 사이 안 멈춤).
  오차 = 기준 − 추정기 (map). 진행 + = 로봇이 뒤처짐, 옆 + = 로봇이 기준 왼쪽. 기준 지평 점 j 의 시각 = header + (j+1)·dt (REF_SHIFT).
  슬립률 10 Hz 1차 Butterworth filtfilt, 창 = 무장 중 · 추정 속력 > 0.05 m/s. 후보 = run_info/args.txt 의 mpc_W (없으면 "기준(yaml)").
  검산 (2026-11-27): 한 바퀴 판 201313·201410 의 위치 RMS 3.41·3.92 cm, 미끄럼 속도 8.43·8.99 cm/s 가 앞선 분석과 같고,
  두 바퀴 판 174149 는 바퀴마다 10.5 s 로 나뉘며 판 전체 3.64 cm.
source install/setup.bash 뒤  /usr/bin/python3 q_laps.py <출력 이름> <bag ...>"""
import math
import os
import re
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT, BAGS = sys.argv[1], [p.rstrip("/") for p in sys.argv[2:]]
REF_SHIFT = +0.05
WANT = ("/dra_mppi/reference_trajectory", "/state_estimator/odom", "/mpc/armed", "/slip_logger/slip", "/torque_mpc/slip_fb", "/wheel_torque_cmd", "/tracked_pose")
r_w = 0.04
Q = np.array([20, 20, 10, 2, 2, 1.0])


def stamp(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else np.nan


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in WANT}
    ref, od, arm, slip, sfb, mot, sl = {}, [], [], [], [], [], []
    while r.has_next():
        name, data, ns = r.read_next()
        if name not in types:
            continue
        m = deserialize_message(data, types[name]); tb = ns * 1e-9
        if name == "/dra_mppi/reference_trajectory":
            t0 = stamp(m.header)
            for j, p in enumerate(m.references):
                if j * m.dt > 0.15:
                    break
                ref[round((t0 + j * m.dt + REF_SHIFT) * 1000)] = (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate)
        elif name == "/state_estimator/odom":
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            od.append([stamp(m.header), p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), tw.linear.x, tw.linear.y, tw.angular.z])
        elif name == "/mpc/armed":
            arm.append([tb, float(m.data)])
        elif name == "/slip_logger/slip":
            slip.append(list(m.data[:9]))                     # [t, λ×4, ω_nom×4]
        elif name == "/torque_mpc/slip_fb":
            d = list(m.data); sfb.append(d[:22] + [math.nan] * (22 - len(d)))   # [t, on, fresh, e×3, Δτ×4, τ_MPC×4, τ×4, e_ω×4]
        elif name == "/wheel_torque_cmd":
            if len(m.torque) == 4:
                mot.append([tb, *m.torque])
        else:
            sl.append([stamp(m.header), m.pose.position.x, m.pose.position.y])
    kt = np.array(sorted(ref))
    return dict(tr=kt / 1000.0, R=np.array([ref[k] for k in kt]), od=np.array(od), arm=np.array(arm), slip=np.array(slip), sfb=np.array(sfb),
                mot=np.array(mot), sl=np.array(sl))


def args_of(path):
    a = open(os.path.join(path, "run_info", "args.txt")).read()
    g = lambda k, d="": (re.search(rf"\b{k}:=(\S+)", a) or [None, d])[1]
    shape = g("shape", "figure8")                                     # polyline·square·zigzag 은 laps 를 안 쓴다 (polyline 은 한 바퀴) → 1
    laps = float(g("laps", "1")) if shape in ("figure8", "circles8", "") else 1.0
    return dict(W=g("mpc_W", "기준(yaml)"), laps=laps, speed=g("speed"), scale=g("slip_fb_scale", "yaml"), accel=g("accel"))


rows = []        # (bag, W, lap(0 = 판 전체, -1 = 2~N 바퀴), 지표 dict)
for path in BAGS:
    name = os.path.basename(path); A = args_of(path); D = read(path)
    tr, R, od = D["tr"], D["R"], D["od"]
    arm = D["arm"]
    if not (arm[:, 1] > 0.5).any():
        print(f"{name}: 무장 안 됨 — 건너뜀"); continue
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
    vr_grid = np.hypot(R[:, 3], R[:, 4]); s_grid = np.concatenate([[0.0], np.cumsum(0.5 * (vr_grid[1:] + vr_grid[:-1]) * np.diff(tr))])
    Ltot = s_grid[-1]; laps = A["laps"]
    lap_of = lambda tt: np.clip(np.floor(laps * np.interp(tt, tr, s_grid) / Ltot), 0, laps - 1).astype(int) + 1
    lap_e = lap_of(t)
    S = D["slip"]; ts = S[:, 0]; L = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0)
    Lf = filtfilt(*butter(1, 10.0 / (0.5 / np.median(np.diff(ts)))), L, axis=0)
    sp = np.hypot(np.interp(ts, od[:, 0], od[:, 4]), np.interp(ts, od[:, 0], od[:, 5]))
    ws = (ts >= t_on) & (ts <= t_off) & (sp > 0.05); lap_s = lap_of(ts)
    F = D["sfb"]; tf = F[:, 0]; wf = (tf >= t_on) & (tf <= t_off) & (np.interp(tf, od[:, 0], np.hypot(od[:, 4], od[:, 5])) > 0.05); lap_f = lap_of(tf)
    t_mv = t[vref > 0.05]                                            # 모터 명령 떨림은 기준이 움직이는 동안만 (해제 안 한 판은 정지 뒤까지 들어가 낮게 나온다)
    M = D["mot"]; tm = M[:, 0]; wm = (tm >= max(t_on, t_mv[0])) & (tm <= min(t_off, t_mv[-1])); lap_m = lap_of(tm)
    SL = D["sl"]; ss = (SL[:, 0] >= t_on) & (SL[:, 0] <= t_off)
    stepv = np.hypot(*np.diff(SL[ss, 1:3], axis=0).T) / np.maximum(np.diff(SL[ss, 0]), 1e-3)
    jumps = SL[ss, 0][1:][stepv > 3.0]
    v_off = float(np.interp(t_off, tr, vr_grid)); done = v_off < 0.05
    for lap in [0, -1] + list(range(1, int(laps) + 1)):
        sel = (lambda lp: np.ones_like(lp, bool)) if lap == 0 else ((lambda lp: lp >= 2) if lap == -1 else (lambda lp, k=lap: lp == k))
        if lap == -1 and laps < 2:
            continue
        e = win & sel(lap_e); s_ = ws & sel(lap_s); f_ = wf & sel(lap_f); m_ = wm & sel(lap_m)
        if e.sum() < 50:
            continue
        X4 = np.abs(Lf[s_]); big = X4.max(1) > 0.5
        Ee = E[e]
        rows.append((name, A["W"], lap, dict(
            pos=100 * rms(np.hypot(Ee[:, 0], Ee[:, 1])), along=100 * float(np.mean(along[e])), along_r=100 * rms(along[e]),
            cross=100 * float(np.mean(cross[e])), cross_r=100 * rms(cross[e]), head=float(np.degrees(rms(Ee[:, 2]))),
            vel=100 * rms(np.hypot(Ee[:, 3], Ee[:, 4])), yaw=rms(Ee[:, 5]), cost=float(np.mean(Ee ** 2 @ Q)),
            m50=100 * float(big.mean()), m25=100 * float(np.mean(X4.max(1) > 0.25)), p50=100 * (X4 > 0.5).mean(0),
            vslip=100 * r_w * rms(F[f_, 18:22]), dtau=1e3 * rms(F[f_, 6:10]), dtick=1e3 * rms(np.diff(M[m_, 1:5], axis=0)),
            dur=float(e.sum() * np.median(np.diff(t))), jumps=int(np.sum(sel(lap_of(jumps)))) if len(jumps) else 0, done=done, t0=t_on)))
    print(f"{name}: W {A['W']} laps {laps:g} speed {A['speed']} accel {A['accel']} 배율 {A['scale']} · 무장 {t_off - t_on:.1f} s · 끝까지 {'예' if done else '아니오 (해제 때 v_ref ' + f'{v_off:.2f})'}"
          f" · SLAM 튐 (> 3 m/s 걸음) {len(jumps)} 번" + (f" @ 무장 뒤 {', '.join(f'{j - t_on:.1f}' for j in jumps)} s" if len(jumps) else ""))

np.save(os.path.join(HERE, OUT + "_rows.npy"), np.array(rows, dtype=object), allow_pickle=True)
print("저장:", os.path.join(HERE, OUT + "_rows.npy"))
