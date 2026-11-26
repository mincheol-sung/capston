#!/usr/bin/python3
"""바닥 시험 분석 (2026-11-24) — tools/floor_tests_w4_t7.sh 로 녹화한 bag 을 읽는다. 식은 amr_mpc/torque/wheel_dynamics.py (식-W1).

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 tools/floor_wheel_tests.py cv   bags/2026-11-24/*_w4_cv_*    [--out 폴더]   # 등속 직진 → 주행 저항·바닥 구름 손실 (W4)
  /usr/bin/python3 tools/floor_wheel_tests.py step bags/2026-11-24/*_t7_step_*  [--out 폴더]   # 단계 토크 → 슬립·바닥 전달 토크·마찰계수 (T7)
  /usr/bin/python3 tools/floor_wheel_tests.py quick <bag>                                      # 한 판 요약 (토크·속도·이동·선 거리)

cv (등속 직진, 지시서 T7 1단계 · W4) — 직선 기준은 기본이 왕복이라 한 판에서 전진·후진 등속 구간이 하나씩 나온다.
  등속 구간 (네 바퀴 평균 속도가 가장 오래 머문 값의 ±5 % 안, 처음 0.3 s · 끝 0.15 s 는 뺌) 에서 몸체 가속이 0 이므로  모터 토크 = 주행 저항 τ_res(ω).
  구간 안에 남은 가속 (평균 속도의 기울기) 은 (m r²/4 + J_w)·ω̇ 를 빼서 고친다 (네 바퀴가 같은 몫을 낸다고 본 값 — 네 바퀴 평균에는 정확).
  구름 손실 τ_roll(ω) = τ_res(ω) − (τ_c + b_v·ω)  (띄운 상태 마찰을 뺀 것). 띄운 상태 마찰은 torque_plant.yaml 값 —
  --lifted-tc · --lifted-bv 로 다른 날 (다른 온도) 의 띄운 시험 직선을 주면 그 값으로 뺀 것도 같이 낸다.
  ★ 바퀴별 값은 믿지 말 것: 주행 중 제어기가 옆·회전 보정 토크를 얹는다 (옆 [−,+,+,−], 회전 [−,+,−,+], 앞뒤 맞섬 [+,+,−,−]).
    네 바퀴 평균 (전진 성분) 에는 그 세 꼴이 0 으로 빠지므로 평균만 주행 저항이다. 보정 토크 크기는 따로 적는다
    (모터 토크 − 띄운 마찰을 세 꼴에 투영한 값, 몸체 기준: 옆 + = 왼쪽으로 민다 — 띄운 마찰의 바퀴별 차이만으로도 옆 꼴에 0.75 mN·m 가 생기므로 먼저 뺀다).
step (단계 토크, 지시서 T7 2·3단계)
  단계 구간 = 토크 모드 명령의 네 바퀴 평균 크기가 최대의 절반 이상인 이어진 구간 — 무장 키를 누를 때마다 하나 (한 bag 에 여러 회).
  몸체 가속 세 가지: IMU 앞 방향 가속 (서 있을 때 평균을 0 점으로 — 기울기·바이어스 제거), 추정기 속도 기울기, 추정기 거리 2d/t².
  바퀴 쪽: 바닥 전달 토크 τ_g = τ − τ_fric(ω) − J_w·ω̇ (실제 ω̇, 식-W1).  몸체 쪽: Σ τ_g = m·a·r  (두 값이 맞는지 = 식-W3 정합성).
  헛돎 판정: 바퀴 r·ω̇ 가 몸체 가속보다 15 % 넘게 크면 그 바퀴는 헛돈다 (IMU 가 없으면 추정기 기울기로).
  마찰계수 μ_i = √2·τ_g,i / (r·N_i).  N_i 는 정지 하중 (하중 몫 × m g) 과, 앞뒤 하중 이동을 넣은 값 N_i ∓ m·a·h/(2L) 두 가지로 낸다
  (h = 무게중심 높이 — 실측 없음, --com-height 기본 0.105 는 URDF 추정). 네 바퀴 합 √2·Στ_g/(r·m g) 는 h 와 무관하다.
  헛돌지 않은 바퀴의 μ 는 "적어도 이만큼" (하한) 이고, 헛돈 바퀴의 μ 가 그 바퀴의 한계값이다.
  토크를 끊은 뒤 선 거리 (추정기 위치, 몸체·바퀴가 다 멈춘 때까지) 도 낸다 — 다음 단계로 올려도 되는지 보려고.
rot (제자리 회전 단계 토크, 2026-11-25) — floor_tests_w4_t7.sh rot · 옛 basic_motion_test.py ccw/cw bag 모두 읽는다
  회전 운동 방정식 (바퀴 관성을 몸체 쪽에 합친 꼴, 미끄럼 없음):  M_w(t) = I_eff·ψ̈ + M_c·sgn(ψ̇) + B·ψ̇
    M_w = Σ_i c_i·(τ_i − τ_fric,i(ω_i))   c_i = T⁻¹ 의 회전 열 (±k/r = ±4.83), τ_fric = 띄운 상태 마찰 (torque_plant.yaml)
    I_eff = I_z + J_w·Σc_i²  (모델 0.06 + 0.083 = 0.143),  M_c·sgn + B·ψ̇ = 띄운 마찰 밖의 바닥 회전 저항 (모델에 없음)
  ψ̇ = IMU 자이로 z (서 있을 때 평균을 0 점으로), ψ̈ = Savitzky–Golay 미분 (0.15 s — 0.04·0.08 s 는 미분 잡음이 관성을 0 쪽으로 깎았다, 11-25 실차). 회전 속도 0.2 rad/s 넘은 표본만 (정지 마찰 구간 뺌),
  토크를 켠·끊은 순간에서 미분 창 반폭 안 표본도 뺀다 (창이 끊은 뒤 DAMPING 감속을 걸치면 가속이 음수로 섞인다).
  두 길로 맞춘다: (1) 표본 전부 최소제곱 (equation-error, Ljung 1999 7장), (2) 회마다 평균 가속 (자이로 직선 맞춤) → 회들끼리 최소제곱.
  검산: 바퀴 각가속도를 따로 빼서 I_z 를 직접 — Σ c_i·(τ_i − τ_fric,i − J_w·ω̇_i) = I_z·ψ̈ + 저항.
lat (옆걸음 단계 토크, 2026-11-25) — floor_tests_w4_t7.sh lat · 옛 basic_motion_test left/right · amrstep dir:=left bag
  Σ c_y,i·(τ_i − τ_fric,i(ω_i)) = m_eff·a_y + F_c·sgn(v_y) [+ B·v_y]    c_y = T⁻¹ 의 옆 열 (±1/r), m_eff = m + J_w·Σc_y² = 6.785 kg
  몸체 중심 옆 가속 = IMU 옆 가속 (서 있을 때 평균을 0 점으로) − ψ̈·r_x + ψ̇²·r_y  (IMU 위치 r = torque_estimator.yaml imu_offset — 옆걸음 중 돌면 IMU 점 가속이 다르다).
  회마다 평균 가속 세 길: IMU (중심으로 옮김), 추정기 옆 속도 기울기, 추정기 거리 2d/T².  옆 저항 F_c 는 (1) m_eff 를 6.785 로 두고 회마다 F − m_eff·a,
  (2) 회들끼리 m·F_c 를 같이 맞춤. 비교: 롤러 구름 저항 가설 (회전 저항 0.40 N·m / l_x) 4.6 N, 지금 모델 0.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from amr_mpc.ak40 import protocol as P                   # noqa: E402
from amr_mpc.core import config                          # noqa: E402
from amr_mpc.torque import wheel_dynamics as wd          # noqa: E402

WHEELS = ("FL", "FR", "RL", "RR")
PAT_LAT = np.array([-1.0, 1.0, 1.0, -1.0])               # 옆 (+y) 힘을 내는 토크 꼴
PAT_YAW = np.array([-1.0, 1.0, -1.0, 1.0])               # 회전 (+z)
PAT_NULL = np.array([1.0, 1.0, -1.0, -1.0])              # 몸체에 힘을 안 내는 꼴 (앞뒤 맞섬)
FRONT = np.array([1.0, 1.0, -1.0, -1.0])                 # 앞바퀴 +1, 뒷바퀴 −1 (하중 이동 부호)


def _arr(rows):
    if not rows:
        return np.zeros((0, 1))
    n = min(len(r) for r in rows)
    return np.array([r[:n] for r in rows], float)


def load_bag(path):
    """bag → dict. 시각은 bag 에 받은 시각 [s] (첫 메시지 기준 0). 없는 토픽은 행 0 개."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    key = {"/wheel_states": "ws", "/wheel_torque_cmd": "cmd", "/state_estimator/odom": "odom", "/tracked_pose": "slam",
           "/imu/base_link": "imu", "/torque_mpc/status": "status", "/torque_mpc/cap": "cap"}
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in key}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    d = {k: [] for k in key.values()}
    t0 = None
    while r.has_next():
        name, data, ns = r.read_next()
        if t0 is None:
            t0 = ns
        t = (ns - t0) * 1e-9
        m = deserialize_message(data, types[name])
        k = key[name]
        if k == "ws":
            d[k].append([t, *m.position, *m.velocity, *m.torque, *m.temperature])
        elif k == "cmd":
            tq = list(m.torque) if len(m.torque) == 4 else [math.nan] * 4
            d[k].append([t, float(m.mode), *tq])
        elif k == "odom":
            q, p, tw = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            d[k].append([t, p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z])
        elif k == "slam":
            d[k].append([t, m.pose.position.x, m.pose.position.y])
        elif k == "imu":
            d[k].append([t, m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z])
        else:
            d[k].append([t, *m.data])
    out = {k: _arr(v) for k, v in d.items()}
    out["path"] = path
    args = os.path.join(path, "run_info", "args.txt")
    out["args"] = open(args).read() if os.path.exists(args) else ""
    return out


def quantize(x):
    """MIT 12 비트로 실제 나가는 토크 (0 은 격자에 없다 — ±1.22 mN·m 가 가장 작은 값)."""
    lim = P.MitLimits()
    return P.uint_to_float(P.float_to_uint(x, -lim.t_max, lim.t_max, 12), -lim.t_max, lim.t_max, 12)


def _runs(mask, gap=1):
    """True 인 인덱스를 이어진 묶음들로 (사이가 gap 표본 이하로 끊긴 것은 잇는다)."""
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    return np.split(idx, np.flatnonzero(np.diff(idx) > gap) + 1)


def torque_window(run):
    """모터에 토크 모드 명령이 나간 전체 구간 [t_start, t_end]. 없으면 None."""
    c = run["cmd"]
    if c.shape[1] < 6:
        return None
    on = (c[:, 1] == 0.0) & np.isfinite(c[:, 2:6]).all(axis=1)
    if not on.any():
        return None
    i = np.flatnonzero(on)
    return float(c[i[0], 0]), float(c[i[-1], 0])


def plant_values():
    y = config.load("torque_plant.yaml")
    return {"r": float(y["geometry"]["wheel_radius_m"]), "L": float(y["geometry"]["wheel_base_m"]),
            "m": float(y["body"]["mass_kg"]), "g": float(y["model"]["gravity"]),
            "J": float(y["wheel"]["inertia_J_w_kgm2"]), "share": np.array(y["body"]["wheel_load_share"], float),
            "tc": np.array(y["friction"]["coulomb_Nm"], float), "bv": np.array(y["friction"]["viscous_Nms_per_rad"], float)}


def _fmt(x, s=1.0, n=1):
    return " ".join(f"{v * s:.{n}f}" for v in np.atleast_1d(x))


# ── cv: 등속 직진 ────────────────────────────────────────────────
def cruise_segments(t, om_mean, settle=0.3, band=0.05, min_dur=0.25, tail=0.15):
    """평균 바퀴 속도의 등속 구간 — 부호별 (전진 +1, 후진 −1) 로 하나씩. 반환 [(부호, t_start, t_end, 머문 속도)].
    머문 속도 = 움직인 표본의 속도 분포에서 가장 많이 모인 값 (출발 직후 넘침에 안 끌려가게 최고값 대신)."""
    if len(t) < 20:
        return []
    dt = float(np.median(np.diff(t)))
    k = max(3, int(round(0.2 / dt)) | 1)
    sm = np.convolve(om_mean, np.ones(k) / k, mode="same")
    out = []
    for sgn in (1.0, -1.0):
        s = sgn * sm
        peak = float(s.max())
        if peak < 1.0:
            continue
        mv = s[s > 0.5 * peak]
        hist, edges = np.histogram(mv, bins=np.arange(0.5 * peak, 1.0001 * peak + 0.03 * peak, 0.03 * peak))
        j = int(np.argmax(hist))
        c0 = 0.5 * (edges[j] + edges[j + 1])
        near = mv[np.abs(mv - c0) <= 0.05 * c0]
        plateau = float(np.median(near)) if len(near) else c0
        tol = max(band * plateau, 0.3)
        segs = _runs(np.abs(s - plateau) <= tol, gap=max(1, int(round(0.05 / dt))))
        if not segs:
            continue
        seg = max(segs, key=len)
        seg = seg[(t[seg] >= t[seg[0]] + settle) & (t[seg] <= t[seg[-1]] - tail)]   # 끝도 자른다: 평활한 속도는 감속 첫 표본까지 대역 안에 남는다
        if len(seg) < 5 or t[seg[-1]] - t[seg[0]] < min_dur:
            continue
        out.append((sgn, float(t[seg[0]]), float(t[seg[-1]]), plateau))
    return out


def analyze_cv(run, pv, settle=0.3, band=0.05, min_dur=0.25, tail=0.15):
    """등속 구간마다 (전진·후진) dict 하나. 토크는 가는 방향을 + 로 맞춘다 (저항이 양수)."""
    w = torque_window(run)
    ws, cmd = run["ws"], run["cmd"]
    if w is None or len(ws) < 50:
        return []
    m = (ws[:, 0] >= w[0]) & (ws[:, 0] <= w[1])
    W = ws[m]
    out = []
    for sgn, ts, te, _ in cruise_segments(W[:, 0], W[:, 5:9].mean(axis=1), settle, band, min_dur, tail):
        s = (W[:, 0] >= ts) & (W[:, 0] <= te)
        cm = (cmd[:, 0] >= ts) & (cmd[:, 0] <= te) & (cmd[:, 1] == 0.0)
        if s.sum() < 5 or cm.sum() < 5:
            continue
        om = W[s, 5:9]
        wdot = float(np.polyfit(W[s, 0], om.mean(axis=1), 1)[0]) * sgn          # 가는 방향 가속 [rad/s²]
        tq = np.vectorize(quantize)(cmd[cm, 2:6]).mean(axis=0) * sgn             # 가는 방향 + 의 모터 토크
        corr = (pv["m"] * pv["r"] ** 2 / 4.0 + pv["J"]) * wdot
        a = {"dir": sgn, "t": (ts, te), "dur": te - ts, "omega": np.abs(om.mean(axis=0)), "omega_std": om.std(axis=0),
             "wdot": wdot, "accel_corr": corr, "tau_raw": tq, "tau_res": tq - corr,
             "tau_fb": W[s, 9:13].mean(axis=0) * sgn, "temp": np.nanmean(W[s, 13:17], axis=0)}
        rest = tq - (pv["tc"] + pv["bv"] * a["omega"])                           # 띄운 마찰의 바퀴별 차이가 세 꼴로 새지 않게 먼저 뺀다
        a["lat"], a["yaw"], a["null"] = (sgn * float(rest @ p) / 4.0 for p in (PAT_LAT, PAT_YAW, PAT_NULL))   # 몸체 기준 (+ 옆 = 왼쪽으로 민다)
        od = run["odom"]
        so = (od[:, 0] >= ts) & (od[:, 0] <= te) if od.shape[1] >= 7 else np.zeros(0, bool)
        a["v_body"] = float(np.hypot(od[so, 4], od[so, 5]).mean()) if so.any() else math.nan
        a["v_wheel"] = float(pv["r"] * a["omega"].mean())
        out.append(a)
    return out


def report_cv(results, pv, lifted=None):
    """results [(이름, [등속 구간 dict …])]. lifted = (τ_c×4, b_v×4) 를 주면 그 띄운 시험 직선으로 뺀 구름 손실도 낸다."""
    L = ["# 등속 직진 — 주행 저항과 바닥 구름 손실\n",
         "단위: 토크 mN·m, 속도 rad/s. 주행 저항 = 등속 구간의 모터 토크 (명령 = 실제 나간 값, 가는 방향 +) 에서 남은 가속 몫을 뺀 것.",
         "구름 손실 = 주행 저항 − 띄운 상태 마찰 (torque_plant.yaml). **네 바퀴 평균이 주행 저항이다** — 바퀴별 값에는 제어기의 옆·회전 보정 토크가 섞여 있다.\n",
         "| 판 | 방향 | 구간 s | 바퀴 속도 | 몸체/바퀴 속도 | 온도 °C | 남은 가속 몫 | 주행 저항 FL FR RL RR | **네 바퀴 평균** | 띄운 마찰 평균 | **구름 손실 평균** | 옆·회전·맞섬 보정 | 피드백 − 명령 평균 |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    pts = []
    for name, segs in results:
        if not segs:
            L.append(f"| {name} | 등속 구간을 못 찾음 | | | | | | | | | | | |")
            continue
        for a in segs:
            w = a["omega"]
            lift = pv["tc"] + pv["bv"] * w
            res = a["tau_res"]
            pts.append((a["dir"], w.mean(), res.mean(), lift.mean(), np.nanmean(a["temp"]), w, res))
            L.append(f"| {name} | {'전진' if a['dir'] > 0 else '후진'} | {a['dur']:.2f} | {w.mean():.1f} | {a['v_body'] / a['v_wheel']:.3f} | "
                     f"{np.nanmean(a['temp']):.0f} | {a['accel_corr'] * 1e3:+.1f} | {_fmt(res, 1e3)} | **{res.mean() * 1e3:.1f}** | {lift.mean() * 1e3:.1f} | "
                     f"**{(res.mean() - lift.mean()) * 1e3:.1f}** | {a['lat'] * 1e3:+.1f} · {a['yaw'] * 1e3:+.1f} · {a['null'] * 1e3:+.1f} | "
                     f"{(a['tau_fb'].mean() - a['tau_raw'].mean()) * 1e3:+.1f} |")
    if len(pts) >= 2:
        D = np.array([p[:5] for p in pts])
        L.append("\n## 네 바퀴 평균을 속도에 대해 (판·방향을 모아)\n")
        L.append("| 묶음 | 점 수 | 주행 저항 = c + b·ω : c mN·m · b mN·m·s/rad | 띄운 마찰 (설정, 네 바퀴 평균) c · b | 구름 손실 평균 (범위) mN·m | 몸체 힘으로 N (무게의 %) |")
        L.append("|---|---|---|---|---|---|")
        mg = pv["m"] * pv["g"]
        for label, sel in (("전진", D[:, 0] > 0), ("후진", D[:, 0] < 0), ("모두", np.ones(len(D), bool))):
            if sel.sum() < 1:
                continue
            w, res, lift = D[sel, 1], D[sel, 2], D[sel, 3]
            fit = "속도가 한 가지뿐"
            if sel.sum() >= 2 and np.ptp(w) > 1.0:
                b, c = np.polyfit(w, res, 1)
                fit = f"{c * 1e3:.1f} · {b * 1e3:.3f}"
            roll = res - lift
            F = 4.0 * roll.mean() / pv["r"]
            L.append(f"| {label} | {int(sel.sum())} | {fit} | {pv['tc'].mean() * 1e3:.1f} · {pv['bv'].mean() * 1e3:.3f} | "
                     f"{roll.mean() * 1e3:.1f} ({roll.min() * 1e3:.1f}~{roll.max() * 1e3:.1f}) | {F:.2f} ({100 * F / mg:.2f} %) |")
        if lifted is not None:
            tc2, bv2 = (np.asarray(x, float) for x in lifted)
            roll2 = np.array([p[2] - (tc2 + bv2 * p[5]).mean() for p in pts])
            L.append(f"\n다른 띄운 시험 직선 (τ_c {_fmt(tc2, 1e3)} · b_v {_fmt(bv2, 1e3, 3)}) 으로 빼면 구름 손실 평균 "
                     f"{roll2.mean() * 1e3:.1f} mN·m (범위 {roll2.min() * 1e3:.1f}~{roll2.max() * 1e3:.1f}) — 띄운 마찰은 온도에 약 1 %/°C 로 바뀌므로 두 값 사이가 참값의 범위다.")
        L.append("\n## 속도 구간별 구름 손실 (네 바퀴 평균, 설정 띄운 마찰 기준)\n")
        L.append("| 바퀴 속도 rad/s | 판 수 | 주행 저항 평균 | 구름 손실 평균 | 판 사이 표준편차 | 온도 °C |")
        L.append("|---|---|---|---|---|---|")
        for wv in sorted({round(p[1] / 2.5) * 2.5 for p in pts}):
            sel = np.array([abs(p[1] - wv) <= 1.25 for p in pts])
            roll = D[sel, 2] - D[sel, 3]
            L.append(f"| {wv:.1f} | {int(sel.sum())} | {D[sel, 2].mean() * 1e3:.1f} | {roll.mean() * 1e3:.1f} | {roll.std() * 1e3:.1f} | {D[sel, 4].mean():.0f} |")
    return "\n".join(L) + "\n"


# ── step: 단계 토크 ──────────────────────────────────────────────
def step_windows(run, frac=0.5, min_len=3):
    """단계 토크가 걸린 구간들 [(t0, t1) …] — 무장 키를 누를 때마다 하나. 토크 모드 명령의 네 바퀴 평균 크기가 (bag 최대의) frac 이상인 이어진 구간."""
    c = run["cmd"]
    if c.shape[1] < 6:
        return []
    ok = (c[:, 1] == 0.0) & np.isfinite(c[:, 2:6]).all(axis=1)
    amp = np.where(ok, np.abs(np.nan_to_num(c[:, 2:6])).mean(axis=1), 0.0)
    if amp.max() <= 1e-4:
        return []
    return [(float(c[s[0], 0]), float(c[s[-1], 0])) for s in _runs(amp >= frac * amp.max()) if len(s) >= min_len]


def step_window(run, frac=0.5):
    """가장 긴 단계 구간 하나 (없으면 None)."""
    w = step_windows(run, frac)
    return max(w, key=lambda x: x[1] - x[0]) if w else None


def imu_rest_baseline(run, w_still=0.2, guard=0.4, col=1):
    """서 있을 때의 IMU 가속 평균 (바닥 기울기·바이어스) — 바퀴가 멈춰 있고 단계 구간에서 guard 초 넘게 떨어진 표본들. col 1 = 앞, 2 = 옆."""
    imu, ws = run["imu"], run["ws"]
    if imu.shape[1] < 3 or len(imu) < 50 or len(ws) < 10:
        return math.nan
    still = np.interp(imu[:, 0], ws[:, 0], np.abs(ws[:, 5:9]).max(axis=1)) < w_still
    for t0, t1 in step_windows(run):
        still &= (imu[:, 0] < t0 - 0.02) | (imu[:, 0] > t1 + guard)
    return float(imu[still, col].mean()) if still.sum() > 50 else math.nan


def analyze_step(run, pv, window=None, lead=0.05, com_height=0.105, slip_ratio=1.15, roll=0.0, w_cap=35.0, t_next=None, imu0=None):
    """단계 구간 하나의 가속·바닥 전달 토크·μ·헛돎, 끊은 뒤 선 거리. window 를 안 주면 가장 긴 구간. 반환 dict 또는 None.
    roll  바퀴당 구름 손실 [N·m] (등속 시험 값) — 바닥 전달 토크에서 더 뺀다 (0 이면 안 뺌)
    w_cap 바퀴 각가속도는 그 바퀴 속도가 이 값 [rad/s] 아래인 표본으로만 맞춘다 (전압 한계 약 42 rad/s 근처에서는 토크가 덜 나온다)
    t_next 다음 단계 시작 시각 (끊은 뒤 구간을 거기서 자른다),  imu0 서 있을 때 IMU 값 (안 주면 이 판에서 구한다)"""
    w = window or step_window(run)
    ws, od, sl, imu, cmd = run["ws"], run["odom"], run["slam"], run["imu"], run["cmd"]
    if w is None or len(ws) < 20:
        return None
    t0, t1 = w
    T = t1 - t0
    r, J, mass = pv["r"], pv["J"], pv["m"]
    cm = (cmd[:, 0] >= t0) & (cmd[:, 0] <= t1) & (cmd[:, 1] == 0.0)
    tau = np.median(np.vectorize(quantize)(cmd[cm, 2:6]), axis=0)
    m = (ws[:, 0] >= t0) & (ws[:, 0] <= t1)
    t, pos, om, fb = ws[m, 0], ws[m, 1:5], ws[m, 5:9], ws[m, 9:13]
    sel = t > t0 + lead
    if sel.sum() < 6:
        return None
    sg = float(np.sign(om[-1].mean()) or 1.0)                                    # 가는 방향
    wdot, om_mid = np.full(4, np.nan), np.full(4, np.nan)
    for i in range(4):
        s_i = sel & (np.abs(om[:, i]) < w_cap)
        if s_i.sum() >= 6:
            wdot[i], om_mid[i] = np.polyfit(t[s_i], om[s_i, i], 1)[0], om[s_i, i].mean()
    fr = wd.CoulombViscous(pv["tc"], pv["bv"], smooth=0.5)
    tg = wd.traction_torque(tau, om_mid, wdot, J, fr) - sg * roll                 # 바퀴 쪽 (식-W1, 실제 ω̇)
    out = {"t": (t0, t1), "dur": T, "tau": tau, "tau_fb": fb[sel].mean(axis=0), "temp": np.nanmean(ws[m, 13:17], axis=0),
           "wdot": wdot, "a_wheel": sg * r * wdot, "tau_g": tg, "w_max": np.abs(om).max(axis=0), "w_start": float(np.abs(om[0]).mean()),
           "wheel_dist": sg * r * (pos[-1] - pos[0]), "v_end_wheel": sg * r * om[-1]}
    # 몸체 가속: IMU (서 있을 때 값을 0 점으로), 추정기 속도 기울기, 추정기 거리
    a_imu = math.nan
    if imu0 is None:
        imu0 = imu_rest_baseline(run)
    if imu.shape[1] >= 3 and len(imu) > 20 and math.isfinite(imu0):
        dur = (imu[:, 0] > t0 + lead) & (imu[:, 0] <= t1)
        if dur.sum() > 10:
            a_imu = sg * float(imu[dur, 1].mean() - imu0)
    a_est, d_est, v_end, dyaw = math.nan, math.nan, math.nan, math.nan
    if od.shape[1] >= 7 and len(od) > 10:
        mo = (od[:, 0] > t0 + lead) & (od[:, 0] <= t1)
        v = np.hypot(od[:, 4], od[:, 5])
        if mo.sum() > 5:
            a_est = float(np.polyfit(od[mo, 0], v[mo], 1)[0])
            yw = np.unwrap(od[mo, 3])
            dyaw = float(np.degrees(yw[-1] - yw[0]))
        p0 = np.array([np.interp(t0, od[:, 0], od[:, c]) for c in (1, 2)])
        p1 = np.array([np.interp(t1, od[:, 0], od[:, c]) for c in (1, 2)])
        d_est, v_end = float(np.hypot(*(p1 - p0))), float(np.interp(t1, od[:, 0], v))
        # 끊은 뒤: 바퀴가 다 멈추고 0.3 s 지난 때 (늦어도 1.5 s 뒤, 다음 단계 시작 전) 의 위치
        t_lim = min(t1 + 1.5, od[-1, 0], t_next if t_next is not None else math.inf)
        aw = (ws[:, 0] >= t1) & (ws[:, 0] <= t_lim)
        stopped = np.abs(ws[aw, 5:9]).max(axis=1) < 0.5
        t_w = float(ws[aw, 0][np.argmax(stopped)]) if stopped.any() else math.nan   # 바퀴가 멈춘 때
        t_ref = min(t_w + 0.3, t_lim) if math.isfinite(t_w) else t_lim
        pe = np.array([np.interp(t_ref, od[:, 0], od[:, c]) for c in (1, 2)])
        out["t_wheel_stop"] = t_w - t1 if math.isfinite(t_w) else math.nan
        out["stop_dist"], out["total_dist"] = float(np.hypot(*(pe - p1))), float(np.hypot(*(pe - p0)))
        if sl.shape[1] >= 3 and len(sl) > 5:                                      # SLAM 으로 선 자리끼리 (지연 없는 값)
            s0 = sl[(sl[:, 0] >= t0 - 0.5) & (sl[:, 0] <= t0), 1:3]
            s1 = sl[(sl[:, 0] >= t_ref) & (sl[:, 0] <= t_ref + 0.5) & (sl[:, 0] <= (t_next if t_next is not None else math.inf)), 1:3]
            if len(s0) and len(s1):
                out["total_dist_slam"] = float(np.hypot(*(s1.mean(axis=0) - s0.mean(axis=0))))
    out.update(a_imu=a_imu, a_est=a_est, a_dist=2.0 * d_est / T ** 2 if T > 0 else math.nan, body_dist=d_est, v_end_body=v_end, dyaw=dyaw)
    a_body = a_imu if math.isfinite(a_imu) else a_est
    out["a_body"], out["a_source"] = a_body, "IMU" if math.isfinite(a_imu) else "추정기"
    good = math.isfinite(a_body) and a_body > 0.05
    out["spin_excess"] = out["a_wheel"] / a_body - 1.0 if good else np.full(4, np.nan)
    out["spin"] = np.nan_to_num(out["spin_excess"], nan=0.0) > slip_ratio - 1.0
    out["dist_ratio"] = float(np.mean(out["wheel_dist"]) / d_est) if d_est > 1e-3 else math.nan
    out["speed_ratio"] = out["v_end_wheel"] / v_end if v_end > 0.05 else np.full(4, np.nan)   # 끝 속도: 바퀴 / 몸체
    # 무슬립이면 나올 가속 (모델): 네 바퀴가 같이 돌 때 Σ(τ − 띄운 마찰 − 구름 손실) = (m r + 4 J/r)·a
    w_ref = np.nanmean(om_mid)
    out["a_noslip"] = float(np.sum(sg * tau - fr(np.full(4, sg * abs(w_ref))) * sg - roll)) / (mass * r + 4.0 * J / r)
    # μ: 정지 하중, 하중 이동 넣은 하중
    N0 = pv["share"] * mass * pv["g"]
    dN = mass * (a_body if math.isfinite(a_body) else 0.0) * com_height / (2.0 * pv["L"]) * sg
    N1 = N0 - FRONT * dN
    out["N_static"], out["N_shift"] = N0, N1
    out["mu_static"] = math.sqrt(2.0) * sg * tg / (r * N0)
    out["mu_shift"] = math.sqrt(2.0) * sg * tg / (r * np.maximum(N1, 1e-6))
    out["mu_total"] = math.sqrt(2.0) * sg * float(np.nansum(tg)) / (r * mass * pv["g"])
    out["mu_body"] = math.sqrt(2.0) * a_body / pv["g"] if math.isfinite(a_body) else math.nan   # 몸체 쪽: √2·m·a/(m g) — 모델과 무관
    out["F_wheel"] = sg * float(np.nansum(tg)) / r                                # 바퀴 쪽에서 본 몸체 힘
    out["F_body"] = mass * a_body if math.isfinite(a_body) else math.nan          # 몸체 쪽 (m·a)
    return out


def analyze_steps(run, pv, **kw):
    """한 bag 의 단계 구간 모두 (무장 키를 여러 번 누른 판). 반환 [dict …] — 출발 때 바퀴가 돌고 있던 구간은 뺀다."""
    wins = step_windows(run)
    imu0 = imu_rest_baseline(run)
    out = []
    for k, w in enumerate(wins):
        a = analyze_step(run, pv, window=w, t_next=wins[k + 1][0] if k + 1 < len(wins) else None, imu0=imu0, **kw)
        if a is not None and a["w_start"] < 1.0:
            a["k"] = k + 1
            out.append(a)
    return out


def _med(rows, key, idx=None):
    v = np.array([np.atleast_1d(a[key]) if idx is None else np.atleast_1d(a[key])[idx] for a in rows], float)
    return np.nanmedian(v, axis=0)


def report_step(results, pv, com_height=0.105, roll=0.0):
    """results [(이름, dict | [dict …] | None)]. 토크 단계별 중앙값 표 + 회별 표."""
    N = pv["share"] * pv["m"] * pv["g"]
    flat = []
    for name, a in results:
        for x in ([] if a is None else a if isinstance(a, list) else [a]):
            flat.append((name, x))
    L = ["# 단계 토크 — 가속·바닥 전달 토크·마찰계수\n",
         f"수직 하중 (정지) [FL FR RL RR] = {_fmt(N, 1.0, 2)} N. 바닥 전달 토크 = 모터 토크 − 띄운 상태 마찰 − 구름 손실 {roll * 1e3:.1f} mN·m − 관성 × 바퀴 각가속도.",
         f"하중 이동 계산의 무게중심 높이 {com_height:g} m 는 **실측이 아니다** (URDF 추정). 몸체 쪽 μ (= √2 × 몸체 가속 / 중력가속도) 는 모델·하중 몫과 무관하다.",
         "헛돎 = 바퀴 가속 (반지름 × 각가속도) 이 몸체 가속 (IMU) 보다 15 % 넘게 큰 것. 바퀴 각가속도는 바퀴 속도 35 rad/s 아래 표본으로만 맞췄다.\n"]
    for name, a in results:
        if a is None or (isinstance(a, list) and not a):
            L.append(f"- {name}: 단계 구간을 못 찾음")
    levels = sorted({round(float(np.mean(a["tau"])), 3) for _, a in flat})
    if flat:
        L += ["\n## 토크 단계별 (회들의 중앙값)\n",
              "| 모터 토크 mN·m | 회 수 | 온도 °C | 몸체 가속 m/s² IMU (최소~최대) · 추정기 | 미끄럼 없을 때 예상 | 바퀴 가속 FL FR RL RR | 헛돈 회 수 FL FR RL RR | 끝 속도 바퀴/몸체 FL FR RL RR | 도는 각 ° (중앙 · 최대) | 바퀴 최고 속도 rad/s |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for lv in levels:
            rows = [a for _, a in flat if abs(float(np.mean(a["tau"])) - lv) < 2e-3]
            ai = np.array([a["a_imu"] for a in rows])
            spin = np.sum([a["spin"] for a in rows], axis=0)
            dy = np.abs([a["dyaw"] for a in rows])
            L.append(f"| {lv * 1e3:.0f} | {len(rows)} | {_med(rows, 'temp').mean():.0f} | {np.nanmedian(ai):.2f} ({np.nanmin(ai):.2f}~{np.nanmax(ai):.2f}) · {_med(rows, 'a_est')[0]:.2f} | "
                     f"{_med(rows, 'a_noslip')[0]:.2f} | {_fmt(_med(rows, 'a_wheel'), 1.0, 2)} | {' '.join(str(int(x)) for x in spin)} | {_fmt(_med(rows, 'speed_ratio'), 1.0, 2)} | "
                     f"{np.nanmedian(dy):.1f} · {np.nanmax(dy):.1f} | {_fmt(np.max([a['w_max'] for a in rows], axis=0), 1.0, 0)} |")
        L += ["\n## 토크 단계별 바닥 전달 토크와 마찰계수 (회들의 중앙값)\n",
              "| 모터 토크 mN·m | 피드백 − 명령 mN·m FL FR RL RR | 바닥 전달 토크 mN·m FL FR RL RR | 몸체 힘 N: 바퀴 쪽 · 몸체 쪽 (비) | μ 정지 하중 FL FR RL RR | μ 하중 이동 넣음 FL FR RL RR | 네 바퀴 합 μ: 바퀴 쪽 · 몸체 쪽 |",
              "|---|---|---|---|---|---|---|"]
        for lv in levels:
            rows = [a for _, a in flat if abs(float(np.mean(a["tau"])) - lv) < 2e-3]
            Fw, Fb = _med(rows, "F_wheel")[0], _med(rows, "F_body")[0]
            L.append(f"| {lv * 1e3:.0f} | {_fmt(_med(rows, 'tau_fb') - _med(rows, 'tau'), 1e3)} | {_fmt(_med(rows, 'tau_g'), 1e3)} | {Fw:.2f} · {Fb:.2f} ({Fw / Fb:.2f}) | "
                     f"{_fmt(_med(rows, 'mu_static'), 1.0, 3)} | {_fmt(_med(rows, 'mu_shift'), 1.0, 3)} | {_med(rows, 'mu_total')[0]:.3f} · {_med(rows, 'mu_body')[0]:.3f} |")
        L.append("\n헛돌지 않은 바퀴의 μ 는 하한 (그만큼은 버텼다), 헛돈 바퀴의 μ 가 그 바퀴의 한계값이다. "
                 "몸체 힘의 비가 1 보다 크면 바퀴 쪽 계산이 못 뺀 손실이 남았다는 뜻이다.")
        L += ["\n## 회별\n",
              "| 판 · 회 | 모터 토크 | 구간 s | 몸체 가속 IMU · 추정기 · 거리 | 바퀴 가속 FL FR RL RR | 헛돈 바퀴 | 도는 각 ° | 굴린/몸체 거리 | 끝 속도 m/s | 끊은 뒤 간 거리 m | 네 바퀴 합 μ 바퀴 쪽 · 몸체 쪽 |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for name, a in flat:
            spin = " ".join(WHEELS[i] for i in range(4) if a["spin"][i]) or "없음"
            L.append(f"| {name} · {a.get('k', 1)} | {np.mean(a['tau']) * 1e3:.0f} | {a['dur']:.2f} | {a['a_imu']:.2f} · {a['a_est']:.2f} · {a['a_dist']:.2f} | "
                     f"{_fmt(a['a_wheel'], 1.0, 2)} | {spin} | {a['dyaw']:+.1f} | {a['dist_ratio']:.3f} | {a['v_end_body']:.2f} | {a.get('stop_dist', math.nan):.2f} | "
                     f"{a['mu_total']:.3f} · {a['mu_body']:.3f} |")
    return "\n".join(L) + "\n"


def rot_geometry():
    """회전 열 c (4,), 바퀴→몸체 pinv(T⁻¹), 모델 I_z·바퀴 관성 켬 여부 (torque_plant.yaml)."""
    from amr_mpc.torque.model import TorquePlant
    p = TorquePlant.from_yaml()
    Tinv = np.asarray(p.Tinv, float)
    c = Tinv[:, 2]
    I_w = float(p.J_w) * float(c @ c)
    return {"c": c, "fk": np.linalg.pinv(Tinv), "Iz": float(p.inertia_z), "I_w": I_w,
            "I_model": float(p.inertia_z) + (I_w if p.use_wheel_inertia else 0.0)}


def gyro_rest_bias(run, w_still=0.2, guard=0.4):
    """서 있을 때 자이로 z 평균 (바이어스) — 바퀴가 멈춰 있고 토크 구간에서 guard 초 넘게 떨어진 표본."""
    imu, ws = run["imu"], run["ws"]
    if imu.shape[1] < 4 or len(imu) < 50 or len(ws) < 10:
        return 0.0
    still = np.interp(imu[:, 0], ws[:, 0], np.abs(ws[:, 5:9]).max(axis=1)) < w_still
    for t0, t1 in step_windows(run):
        still &= (imu[:, 0] < t0 - 0.02) | (imu[:, 0] > t1 + guard)
    return float(imu[still, 3].mean()) if still.sum() > 50 else 0.0


def analyze_rot(run, pv, rg, window, bias=0.0, w_move=0.2, sg_s=0.15, t_next=None):
    """제자리 회전 단계 구간 하나. 반환 dict (표본 배열 포함) 또는 None."""
    from scipy.signal import savgol_filter
    t0, t1 = window
    ws, imu, cmd, sl = run["ws"], run["imu"], run["cmd"], run["slam"]
    if imu.shape[1] < 4 or len(ws) < 20:
        return None
    cm = (cmd[:, 0] >= t0) & (cmd[:, 0] <= t1) & (cmd[:, 1] == 0.0)
    tau = np.median(np.vectorize(quantize)(cmd[cm, 2:6]), axis=0)
    yaw_amp = float(rg["c"] @ tau) / float(np.abs(rg["c"]).sum())                 # 회전 꼴 성분 (바퀴당, 부호 = 반시계 +)
    if abs(yaw_amp) < 0.5 * np.abs(tau).mean():                                    # 회전 판이 아니다
        return None
    sg = float(np.sign(yaw_amp))
    pad = 0.3
    mi = (imu[:, 0] >= t0 - pad) & (imu[:, 0] <= t1 + pad)
    ti, wz = imu[mi, 0], imu[mi, 3] - bias
    if len(ti) < 30:
        return None
    fs = 1.0 / np.median(np.diff(ti))
    n = max(5, int(round(sg_s * fs)) | 1)
    wz_s = savgol_filter(wz, n, 2)
    al = savgol_filter(wz, n, 2, deriv=1, delta=1.0 / fs)
    om = np.column_stack([np.interp(ti, ws[:, 0], ws[:, 5 + i]) for i in range(4)])
    fb = np.column_stack([np.interp(ti, ws[:, 0], ws[:, 9 + i]) for i in range(4)])
    wdot = np.gradient(savgol_filter(om, n, 2, axis=0), ti, axis=0)
    fr = wd.CoulombViscous(pv["tc"], pv["bv"], smooth=0.5)
    f_om = np.array([fr(o) for o in om])
    M_cmd = (tau - f_om) @ rg["c"]
    M_fb = (fb - f_om) @ rg["c"]
    M_wh = (tau - f_om - pv["J"] * wdot) @ rg["c"]                                  # 바퀴 관성 뺀 몸체 몫 (검산)
    inwin = (ti >= t0) & (ti <= t1)
    h = 0.5 * n / fs                                                              # 미분 창 반폭: 토크 켠·끊은 순간을 걸친 표본은 뺀다
    moved = inwin & (sg * wz_s > w_move) & (ti >= t0 + h) & (ti <= t1 - h)
    started = inwin & (sg * wz_s > w_move)
    out = {"t": (t0, t1), "dur": t1 - t0, "tau": tau, "yaw_amp": yaw_amp, "sg": sg, "temp": np.nanmean(ws[(ws[:, 0] >= t0) & (ws[:, 0] <= t1), 13:17], axis=0),
           "moved": bool(started.any()), "w_peak": float(np.max(sg * wz_s[inwin])) if inwin.any() else math.nan,
           "w_start": float(np.interp(t0, ti, wz_s))}                              # 토크 켤 때 이미 돌고 있었나 (앞 회가 덜 멈춤)
    out["breakaway"] = float(ti[started][0] - t0) if started.any() else math.nan
    # 회전각: 토크 구간 · 끊은 뒤 멈출 때까지 (자이로 적분)
    t_end = min(t1 + 1.5, t_next if t_next is not None else math.inf)
    me = (imu[:, 0] >= t0) & (imu[:, 0] <= t_end)
    te, we = imu[me, 0], imu[me, 3] - bias
    ang = np.concatenate([[0.0], np.cumsum(0.5 * (we[1:] + we[:-1]) * np.diff(te))])
    out["ang_torque"] = float(np.degrees(np.interp(t1, te, ang)))
    out["ang_total"] = float(np.degrees(ang[-1]))
    # 바퀴로 계산한 회전 속도 대 자이로 (미끄럼) — 토크 구간 끝 0.1 s
    tail = inwin & (ti >= t1 - 0.1) & moved
    if tail.any():
        w_wh = (om[tail] @ rg["fk"].T)[:, 2]
        out["eff"] = float(np.mean(wz_s[tail]) / np.mean(w_wh)) if abs(np.mean(w_wh)) > 0.1 else math.nan
    else:
        out["eff"] = math.nan
    # 회마다 평균 가속: 자이로 직선 맞춤 (움직인 표본)
    if moved.sum() >= 10:
        out["alpha_fit"] = float(np.polyfit(ti[moved], wz[moved], 1)[0])
        out["w_mean"] = float(np.mean(wz[moved]))
        out["M_mean"] = float(np.mean(M_cmd[moved]))
        out["M_fb_mean"] = float(np.mean(M_fb[moved]))
    else:
        out["alpha_fit"] = out["w_mean"] = out["M_mean"] = out["M_fb_mean"] = math.nan
    out["alpha_model"] = out["M_mean"] / rg["I_model"] if math.isfinite(out["M_mean"]) else math.nan
    out["samples"] = {"al": al[moved], "w": wz_s[moved], "M": M_cmd[moved], "M_fb": M_fb[moved], "M_wh": M_wh[moved]}
    # SLAM 제자리 오차 (회전 중심이 얼마나 움직였나 — SLAM 추적점은 IMU 점이라 회전하면 원을 그린다: 참고용 끝점끼리)
    if sl.shape[1] >= 3 and len(sl) > 5:
        s0 = sl[(sl[:, 0] >= t0 - 0.5) & (sl[:, 0] <= t0), 1:3]
        s1 = sl[(sl[:, 0] >= t_end - 0.3) & (sl[:, 0] <= t_end), 1:3]
        out["slam_move"] = float(np.hypot(*(s1.mean(0) - s0.mean(0)))) if len(s0) and len(s1) else math.nan
    return out


def fit_rot(rows, key="M", alpha="al", with_b=True):
    """M = I·α + M_c·sgn(ψ̇) [+ B·ψ̇] 최소제곱. 반환 (계수, 표준오차, 잔차 RMS, 표본 수)."""
    A, y = [], []
    for a in rows:
        S = a["samples"]
        if len(S["M"]) == 0:
            continue
        cols = [S[alpha], np.sign(S["w"])] + ([S["w"]] if with_b else [])
        A.append(np.column_stack(cols))
        y.append(S[key])
    if not A:
        return None
    A, y = np.vstack(A), np.concatenate(y)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ x
    dof = max(1, len(y) - A.shape[1])
    s2 = float(res @ res) / dof
    se = np.sqrt(np.diag(s2 * np.linalg.inv(A.T @ A)))
    return x, se, math.sqrt(s2), len(y)


def fit_rot_runs(rows, key="M_mean", with_b=True):
    """회마다 평균 (자이로 직선 기울기·평균 회전 속도·평균 모멘트) 로 같은 식을 맞춘다 — 회 수가 계수 수보다 많아야."""
    R = [a for a in rows if math.isfinite(a["alpha_fit"])]
    k = 3 if with_b else 2
    if len(R) <= k:
        return None
    A = np.array([[a["alpha_fit"], np.sign(a["w_mean"])] + ([a["w_mean"]] if with_b else []) for a in R])
    y = np.array([a[key] for a in R])
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ x
    s2 = float(res @ res) / max(1, len(y) - k)
    return x, np.sqrt(np.diag(s2 * np.linalg.inv(A.T @ A))), math.sqrt(s2), len(y)


def analyze_rots(run, pv, rg):
    wins = step_windows(run)
    bias = gyro_rest_bias(run)
    out = []
    for k, w in enumerate(wins):
        a = analyze_rot(run, pv, rg, w, bias=bias, t_next=wins[k + 1][0] if k + 1 < len(wins) else None)
        if a is not None:
            a["k"], a["bias"] = k + 1, bias
            out.append(a)
    return out


def report_rot(results, pv, rg):
    flat = [(n, x) for n, a in results for x in (a or [])]
    csum = float(np.abs(rg["c"]).sum())                                           # Σ|c_i| = 4k/r — 바퀴당 토크 ↔ 몸체 모멘트
    L = ["# 제자리 회전 단계 토크 — 회전 관성·바닥 회전 저항\n",
         f"식: Σ c_i·(모터 토크 − 띄운 마찰) = I_eff·ψ̈ + M_c·sgn(ψ̇) + B·ψ̇.  c = ±{rg['c'][1]:.2f} (k/r), 바퀴당 토크 1 mN·m = 몸체 모멘트 {csum:.2f} mN·m.",
         f"모델: I_z {rg['Iz']:.3f} + 바퀴 몫 J_w·Σc² {rg['I_w']:.3f} = {rg['Iz'] + rg['I_w']:.3f} kg·m² (지금 모델이 쓰는 값 {rg['I_model']:.3f}). 띄운 마찰 밖 저항은 모델에 없다 (0).\n"]
    for name, a in results:
        if not a:
            L.append(f"- {name}: 회전 단계 구간을 못 찾음")
    if not flat:
        return "\n".join(L) + "\n"
    L += ["## 회별\n",
          "| 판 · 회 | 바퀴당 토크 mN·m (회전 꼴) | 온도 °C | 토크 구간 s | 시작 회전 속도 rad/s | 출발까지 s | 최고 회전 속도 rad/s | 회전각 ° 토크 중 · 끝 | 평균 가속 rad/s² 측정 · 모델 (비) | 자이로/바퀴 회전 속도 | SLAM 끝점 이동 cm |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, a in flat:
        ratio = a["alpha_fit"] / a["alpha_model"] if math.isfinite(a["alpha_model"]) and a["alpha_model"] != 0 else math.nan
        L.append(f"| {name} · {a['k']} | {a['yaw_amp'] * 1e3:+.0f} | {np.nanmean(a['temp']):.0f} | {a['dur']:.2f} | {a['w_start']:+.2f} | {a['breakaway']:.2f} | {a['w_peak']:.2f} | "
                 f"{a['ang_torque']:+.1f} · {a['ang_total']:+.1f} | {a['alpha_fit']:+.1f} · {a['alpha_model']:+.1f} ({ratio:.2f}) | {a['eff']:.2f} | {100 * a.get('slam_move', math.nan):.1f} |")
    L.append("\n(출발까지 = 토크를 준 뒤 자이로가 0.2 rad/s 를 넘은 때. 안 넘었으면 nan — 정지 마찰을 못 넘은 판. 모델 가속 = 평균 모멘트 / 지금 모델 관성 — 바닥 저항 0 이라고 본 값.)")
    names = {0: "I_eff kg·m²", 1: "M_c N·m", 2: "B N·m·s/rad"}
    L += ["\n## 맞춘 값\n", "| 방법 | I_eff kg·m² | M_c N·m (바퀴당 mN·m) | B N·m·s/rad | 잔차 RMS N·m | 표본 |", "|---|---|---|---|---|---|"]
    for lab, f in (("표본 전부, 모멘트 = 명령", fit_rot([a for _, a in flat], "M")),
                   ("표본 전부, B 없이", fit_rot([a for _, a in flat], "M", with_b=False)),
                   ("표본 전부, 모멘트 = 피드백 토크", fit_rot([a for _, a in flat], "M_fb")),
                   ("회마다 평균 → 회들끼리", fit_rot_runs([a for _, a in flat])),
                   ("회마다 평균, B 없이", fit_rot_runs([a for _, a in flat], with_b=False))):
        if f is None:
            L.append(f"| {lab} | 표본 모자람 | | | | |")
            continue
        x, se, rms, n = f
        b = f"{x[2]:.4f} ± {se[2]:.4f}" if len(x) > 2 else "(0 으로 둠)"
        L.append(f"| {lab} | {x[0]:.4f} ± {se[0]:.4f} | {x[1]:.3f} ± {se[1]:.3f} ({x[1] / csum * 1e3:.1f}) | {b} | {rms:.3f} | {n} |")
    g = fit_rot([a for _, a in flat], "M_wh")
    f0 = fit_rot([a for _, a in flat], "M")
    if g is not None and f0 is not None and g[2] > 5.0 * f0[2]:
        L.append(f"\n검산 (바퀴 쪽 I_z) 은 못 했다: 바퀴 각가속도 (/wheel_states 받은 시각 미분) 잡음으로 잔차 RMS {g[2]:.2f} N·m — 위 맞춤 {f0[2]:.3f} 의 {g[2] / f0[2]:.0f} 배.")
    elif g is not None:
        x, se, rms, n = g
        L.append(f"\n검산 (바퀴 각가속도 몫을 바퀴 쪽에서 뺌 → I_z 직접): I_z {x[0]:.4f} ± {se[0]:.4f} kg·m², M_c {x[1]:.3f}, 잔차 RMS {rms:.3f}. "
                 f"위 I_eff − 바퀴 몫 {rg['I_w']:.3f} 와 같아야 한다 (미끄럼이 없으면).")
    L.append("\n± 는 최소제곱 표준오차 (표본끼리 독립이라 본 값 — 실제 불확도는 회들끼리 차이가 더 정직하다).")
    return "\n".join(L) + "\n"


def lat_geometry():
    """옆 열 c_y (4,), 실제 m_eff (바퀴 관성 포함), 모델 m, IMU 위치 (torque_estimator.yaml)."""
    from amr_mpc.torque.model import TorquePlant
    p = TorquePlant.from_yaml()
    Tinv = np.asarray(p.Tinv, float)
    c = Tinv[:, 1]
    est = config.load("torque_estimator.yaml")

    def find(d, key):
        if isinstance(d, dict):
            if key in d:
                return d[key]
            for v in d.values():
                x = find(v, key)
                if x is not None:
                    return x
        return None
    m_w = float(p.J_w) * float(c @ c)
    return {"c": c, "fk": np.linalg.pinv(Tinv), "m_eff": float(p.mass) + m_w, "m_model": float(p.mass) + (m_w if p.use_wheel_inertia else 0.0),
            "r_imu": np.array([float(find(est, "imu_offset_x") or 0.0), float(find(est, "imu_offset_y") or 0.0)]),
            "lx": 0.5 * float(p.wheel_base)}


def analyze_lat(run, pv, lg, window, base=(0.0, 0.0, 0.0), lead=0.05, tail=0.03, sg_s=0.15, v_move=0.05, t_next=None):
    """옆걸음 단계 구간 하나. base = (IMU 앞 0 점, 옆 0 점, 자이로 0 점). 반환 dict 또는 None."""
    from scipy.signal import savgol_filter
    t0, t1 = window
    ws, imu, cmd, od = run["ws"], run["imu"], run["cmd"], run["odom"]
    if imu.shape[1] < 4 or len(ws) < 20:
        return None
    cm = (cmd[:, 0] >= t0) & (cmd[:, 0] <= t1) & (cmd[:, 1] == 0.0)
    tau = np.median(np.vectorize(quantize)(cmd[cm, 2:6]), axis=0)
    amp = float(lg["c"] @ tau) / float(np.abs(lg["c"]).sum())                     # 옆 꼴 성분 (바퀴당, + = 왼쪽)
    if abs(amp) < 0.5 * np.abs(tau).mean():
        return None
    sg = float(np.sign(amp))
    pad = 0.3
    mi = (imu[:, 0] >= t0 - pad) & (imu[:, 0] <= t1 + pad)
    ti = imu[mi, 0]
    if len(ti) < 30:
        return None
    fs = 1.0 / np.median(np.diff(ti))
    n = max(5, int(round(sg_s * fs)) | 1)
    wz = imu[mi, 3] - base[2]
    al = savgol_filter(wz, n, 2, deriv=1, delta=1.0 / fs)
    rx, ry = lg["r_imu"]
    ay = imu[mi, 2] - base[1] - al * rx + wz ** 2 * ry                           # 몸체 중심 옆 가속
    ax = imu[mi, 1] - base[0] + al * ry + wz ** 2 * rx
    om = np.column_stack([np.interp(ti, ws[:, 0], ws[:, 5 + i]) for i in range(4)])
    fr = wd.CoulombViscous(pv["tc"], pv["bv"], smooth=0.5)
    F = (tau - np.array([fr(o) for o in om])) @ lg["c"]                           # 바퀴 쪽 옆 힘 [N] (띄운 마찰 뺌)
    v_wh = (om @ lg["fk"].T)[:, 1]
    inwin = (ti >= t0) & (ti <= t1)
    h = 0.5 * n / fs                                                              # ψ̈ 미분 창 반폭 — 켠·끊은 순간을 걸친 표본은 뺀다 (rot 과 같음)
    mv = inwin & (ti >= t0 + max(lead, h)) & (ti <= t1 - max(tail, h)) & (sg * v_wh > v_move)
    out = {"t": (t0, t1), "dur": t1 - t0, "tau": tau, "amp": amp, "sg": sg,
           "temp": np.nanmean(ws[(ws[:, 0] >= t0) & (ws[:, 0] <= t1), 13:17], axis=0),
           "v_start": float(np.interp(t0, ti, v_wh)), "moved": bool(mv.any())}
    started = inwin & (sg * v_wh > v_move)
    out["breakaway"] = float(ti[started][0] - t0) if started.any() else math.nan
    ang = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ti))])
    out["dyaw"] = float(np.degrees(np.interp(t1, ti, ang) - np.interp(t0, ti, ang)))
    nan = math.nan
    out.update(a_imu=nan, a_fwd=nan, F=nan, a_est=nan, a_dist=nan, v_end_est=nan, v_end_wh=nan, w_end=np.full(4, nan))
    if mv.sum() >= 10:
        out["a_imu"], out["a_fwd"] = float(ay[mv].mean()), float(ax[mv].mean())
        out["F"] = float(F[mv].mean())
        e = ti[inwin][-1]
        out["v_end_wh"] = float(np.interp(e, ti, v_wh))
        out["w_end"] = np.abs(om[inwin][-1])
    if od.shape[1] >= 7 and len(od) > 10:
        mo = (od[:, 0] >= t0 + lead) & (od[:, 0] <= t1)
        if mo.sum() > 5:
            out["a_est"] = float(np.polyfit(od[mo, 0], od[mo, 5], 1)[0])
            out["v_end_est"] = float(np.interp(t1, od[:, 0], od[:, 5]))
        p0 = np.array([np.interp(t0, od[:, 0], od[:, c]) for c in (1, 2, 3)])
        p1 = np.array([np.interp(t1, od[:, 0], od[:, c]) for c in (1, 2)])
        d = p1 - p0[:2]
        d_lat = -math.sin(p0[2]) * d[0] + math.cos(p0[2]) * d[1]                  # 출발 때 몸체 옆 방향 성분
        out["d_lat"] = float(d_lat)
        out["a_dist"] = 2.0 * d_lat / (t1 - t0) ** 2 if t1 > t0 else nan
    m = lg["m_eff"]
    out["F_res"] = sg * (out["F"] - m * out["a_imu"])                           # 옆 저항 (m_eff 고정, + = 움직임 반대)
    out["F_res_est"] = sg * (out["F"] - m * out["a_est"])
    out["a_model"] = out["F"] / lg["m_model"] if math.isfinite(out["F"]) else nan
    out["slip"] = out["v_end_wh"] / out["v_end_est"] if math.isfinite(out["v_end_est"]) and abs(out["v_end_est"]) > 0.05 else nan
    return out


def analyze_lats(run, pv, lg):
    wins = step_windows(run)
    base = (imu_rest_baseline(run, col=1), imu_rest_baseline(run, col=2), gyro_rest_bias(run))
    base = tuple(0.0 if not math.isfinite(b) else b for b in base)
    out = []
    for k, w in enumerate(wins):
        a = analyze_lat(run, pv, lg, w, base=base, t_next=wins[k + 1][0] if k + 1 < len(wins) else None)
        if a is not None:
            a["k"] = k + 1
            out.append(a)
    return out


def fit_lat_runs(rows, key="a_imu", with_b=False):
    """회마다 평균으로 F = m·a + F_c·sgn(v) [+ B·v] — m 도 같이 맞춘다. 서 있다가 시작한 회만."""
    R = [a for a in rows if math.isfinite(a[key]) and math.isfinite(a["F"]) and abs(a["v_start"]) < 0.05]
    k = 3 if with_b else 2
    if len(R) <= k:
        return None
    A = np.array([[a[key], a["sg"]] + ([a["v_end_wh"] / 2.0] if with_b else []) for a in R])
    y = np.array([a["F"] for a in R])
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ x
    s2 = float(res @ res) / max(1, len(y) - k)
    return x, np.sqrt(np.diag(s2 * np.linalg.inv(A.T @ A))), math.sqrt(s2), len(y)


def report_lat(results, pv, lg, M_rot=0.40):
    flat = [(n, x) for n, a in results for x in (a or [])]
    pred = M_rot / lg["lx"]
    L = ["# 옆걸음 단계 토크 — 옆 저항 (롤러 구름 저항 가설 확인)\n",
         f"식: Σ c_y,i·(모터 토크 − 띄운 마찰) = m_eff·a_y + F_c·sgn(v_y).  c_y = ±{abs(lg['c'][0]):.0f} 1/m (1/r), 바퀴당 1 mN·m = 옆 힘 {abs(lg['c'][0]) * 4e-3:.2f} N.",
         f"m_eff = {lg['m_eff']:.3f} kg (바퀴 관성 포함, 지금 모델 {lg['m_model']:.3f}). IMU 위치 ({lg['r_imu'][0]:.4f}, {lg['r_imu'][1]:.4f}) m 로 중심 가속을 옮김.",
         f"예측: 롤러 구름 저항 가설 (회전 저항 {M_rot:.2f} N·m / l_x {lg['lx']:.4f} m) → **F_c {pred:.1f} N**, 지금 모델 → 0.\n"]
    for name, a in results:
        if not a:
            L.append(f"- {name}: 옆걸음 단계 구간을 못 찾음")
    if not flat:
        return "\n".join(L) + "\n"
    L += ["## 회별\n",
          "| 판 · 회 | 바퀴당 토크 mN·m (옆 꼴, + 왼쪽) | 온도 °C | 구간 s | 시작 옆 속도 m/s | 출발까지 s | 옆 가속 m/s² IMU · 추정기 · 거리 · 모델 | 앞 가속 IMU | 바퀴 쪽 옆 힘 N | 옆 저항 N (IMU · 추정기) | 회전 ° | 끝 바퀴 속도 FL FR RL RR | 바퀴/추정기 옆 속도 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, a in flat:
        L.append(f"| {name} · {a['k']} | {a['amp'] * 1e3:+.0f} | {np.nanmean(a['temp']):.0f} | {a['dur']:.2f} | {a['v_start']:+.2f} | {a['breakaway']:.2f} | "
                 f"{a['a_imu']:+.2f} · {a['a_est']:+.2f} · {a['a_dist']:+.2f} · {a['a_model']:+.2f} | {a['a_fwd']:+.2f} | {a['F']:+.2f} | {a['F_res']:.2f} · {a['F_res_est']:.2f} | "
                 f"{a['dyaw']:+.1f} | {_fmt(a['w_end'], 1.0, 1)} | {a['slip']:.2f} |")
    levels = sorted({round(abs(a["amp"]), 3) for _, a in flat})
    L += ["\n## 토크별 (서 있다가 시작한 회의 중앙값)\n",
          "| 바퀴당 토크 mN·m | 회 (왼쪽 · 오른쪽) | 옆 가속 IMU · 모델 m/s² (비) | 옆 저항 N: IMU · 추정기 | 회전 ° (중앙 · 최대) | 바퀴/추정기 옆 속도 |", "|---|---|---|---|---|---|"]
    for lv in levels:
        G = [a for _, a in flat if abs(abs(a["amp"]) - lv) < 2e-3 and abs(a["v_start"]) < 0.05]
        if not G:
            continue
        md = lambda k: float(np.nanmedian([a[k] for a in G]))
        ai = np.nanmedian([abs(a["a_imu"]) for a in G]); am = np.nanmedian([abs(a["a_model"]) for a in G])
        L.append(f"| {lv * 1e3:.0f} | {sum(a['sg'] > 0 for a in G)} · {sum(a['sg'] < 0 for a in G)} | {ai:.2f} · {am:.2f} ({ai / am:.2f}) | {md('F_res'):.2f} · {md('F_res_est'):.2f} | "
                 f"{np.nanmedian([abs(a['dyaw']) for a in G]):.1f} · {np.nanmax([abs(a['dyaw']) for a in G]):.1f} | {md('slip'):.2f} |")
    L += ["\n## 회들끼리 맞춤 (질량도 같이)\n", "| 가속 출처 | m kg | F_c N | 잔차 N | 회 |", "|---|---|---|---|---|"]
    for key, lab in (("a_imu", "IMU (중심)"), ("a_est", "추정기 기울기"), ("a_dist", "추정기 거리")):
        f = fit_lat_runs([a for _, a in flat], key)
        if f is None:
            L.append(f"| {lab} | 회 모자람 | | | |")
            continue
        x, se, rms, n = f
        L.append(f"| {lab} | {x[0]:.2f} ± {se[0]:.2f} | {x[1]:.2f} ± {se[1]:.2f} | {rms:.2f} | {n} |")
    L.append("\n옆 저항 = 바퀴 쪽 옆 힘 − m_eff × 옆 가속 (움직임 반대가 +). 헛도는 바퀴가 있으면 그 바퀴 토크가 바닥에 다 안 가서 저항이 크게 나온다 — 끝 바퀴 속도로 본다.")
    return "\n".join(L) + "\n"


def quick(run, pv):
    w = torque_window(run)
    if w is None:
        return "토크 명령이 나간 구간이 없다"
    ws, c = run["ws"], run["cmd"]
    m = (ws[:, 0] >= w[0]) & (ws[:, 0] <= w[1])
    cm = (c[:, 0] >= w[0]) & (c[:, 0] <= w[1]) & (c[:, 1] == 0.0)
    L = [f"토크 모드 구간 {w[1] - w[0]:.2f} s.  모터 명령 |τ| 최대 {np.nanmax(np.abs(c[cm, 2:6])):.3f}, 피드백 |τ| 최대 {np.abs(ws[m, 9:13]).max():.3f} N·m,  "
         f"바퀴 |ω| 최대 {np.abs(ws[m, 5:9]).max():.1f} rad/s,  온도 {np.nanmean(ws[m, 13:17]):.0f} °C,  "
         f"바퀴 굴린 거리 (끝 − 처음) {_fmt(pv['r'] * (ws[m, 1:5][-1] - ws[m, 1:5][0]), 1.0, 3)} m"]
    for a in analyze_cv(run, pv):
        L.append(f"등속 {'전진' if a['dir'] > 0 else '후진'} {a['dur']:.2f} s @ {a['omega'].mean():.1f} rad/s: 주행 저항 네 바퀴 평균 {a['tau_res'].mean() * 1e3:.1f} mN·m "
                 f"(띄운 마찰 {(pv['tc'] + pv['bv'] * a['omega']).mean() * 1e3:.1f}), 옆 보정 {a['lat'] * 1e3:+.1f}")
    st = run["status"]
    if st.shape[1] >= 14:
        D = st[:, 1:]
        L.append(f"MPC: 위치 오차 RMS {1e3 * np.sqrt(np.mean(D[:, 1] ** 2 + D[:, 2] ** 2)):.1f} mm 최대 {1e3 * np.hypot(D[:, 1], D[:, 2]).max():.1f} mm, "
                 f"헤딩 최대 {np.degrees(np.abs(D[:, 3]).max()):.2f}°, 제약 걸린 스텝 {100 * np.mean(D[:, 9] > 0):.1f} %, 계산 실패 {int((D[:, 11] > 0).sum())} 번, "
                 f"계산 시간 평균 {D[:, 12].mean():.2f} 최대 {D[:, 12].max():.2f} ms")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["cv", "step", "quick", "rot", "lat"])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--out", default="", help="결과 폴더 (floor_cv.md / floor_step.md)")
    ap.add_argument("--lifted-tc", default="", help="cv: 다른 띄운 시험의 τ_c [N·m] 네 개 (쉼표)")
    ap.add_argument("--lifted-bv", default="", help="cv: 다른 띄운 시험의 b_v [N·m·s/rad] 네 개 (쉼표)")
    ap.add_argument("--com-height", type=float, default=0.105, help="step: 하중 이동 계산의 무게중심 높이 [m] (실측 없음, URDF 추정)")
    ap.add_argument("--settle", type=float, default=0.3, help="cv: 등속 구간 처음에서 버리는 시간 [s]")
    ap.add_argument("--tail", type=float, default=0.15, help="cv: 등속 구간 끝에서 버리는 시간 [s]")
    ap.add_argument("--min-dur", type=float, default=0.25, help="cv: 등속 구간으로 치는 최소 길이 [s] (짧은 직선·빠른 속도에서는 줄여서 참고값으로)")
    ap.add_argument("--roll", type=float, default=0.0, help="step: 바퀴당 구름 손실 [N·m] (등속 시험 값) — 바닥 전달 토크에서 더 뺀다")
    a = ap.parse_args()
    pv = plant_values()
    bags = sorted({b.rstrip("/") for p in a.bags for b in (glob.glob(p) or [p]) if os.path.isdir(b) and glob.glob(os.path.join(b, "*.mcap"))})
    if not bags:
        raise SystemExit("bag 폴더가 없다")
    if a.what == "quick":
        for b in bags:
            print(f"── {os.path.basename(b)}\n{quick(load_bag(b), pv)}")
        return
    if a.what == "lat":
        lg = lat_geometry()
        results = [(os.path.basename(b), analyze_lats(load_bag(b), pv, lg)) for b in bags]
        txt = report_lat(results, pv, lg)
        print(txt)
        if a.out:
            os.makedirs(a.out, exist_ok=True)
            with open(os.path.join(a.out, "floor_lat.md"), "w") as f:
                f.write(txt)
        return
    if a.what == "rot":
        rg = rot_geometry()
        results = [(os.path.basename(b), analyze_rots(load_bag(b), pv, rg)) for b in bags]
        txt = report_rot(results, pv, rg)
        print(txt)
        if a.out:
            os.makedirs(a.out, exist_ok=True)
            with open(os.path.join(a.out, "floor_rot.md"), "w") as f:
                f.write(txt)
        return
    lifted = None
    if a.lifted_tc and a.lifted_bv:
        lifted = ([float(x) for x in a.lifted_tc.split(",")], [float(x) for x in a.lifted_bv.split(",")])
    results = []
    for b in bags:
        run = load_bag(b)
        results.append((os.path.basename(b), analyze_cv(run, pv, settle=a.settle, min_dur=a.min_dur, tail=a.tail) if a.what == "cv" else analyze_steps(run, pv, com_height=a.com_height, roll=a.roll)))
    txt = report_cv(results, pv, lifted) if a.what == "cv" else report_step(results, pv, a.com_height, a.roll)
    print(txt)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(os.path.join(a.out, "floor_cv.md" if a.what == "cv" else "floor_step.md"), "w") as f:
            f.write(txt)


if __name__ == "__main__":
    main()
