#!/usr/bin/python3
"""(2026-11-25 복사: 꺾은선 accel 2.0 (직진 가감속 상한 밖), MPC 바퀴 속도 · PP · 어댑티브 PP. 원본 ../2026-11-22_speedmod_pp_vs_mpc/analyze.py — 안쪽·교차점 지표는 원 기준이라 여기서는 안 본다)
원본 설명: circles8 + 속도 sin 변조, PP(A) vs 토크 MPC(C) 한 판씩 — 추종·속도·가속도·v_y·슬립·토크 (2026-11-22).

  cd ~/capston/capston && source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-22_speedmod_pp_vs_mpc/analyze.py bags/2026-11-22/212837 bags/2026-11-22/212703

잣대 (두 판 같은 계산):
  기준       /dra_mppi/reference_trajectory 를 core/reference.py ReferenceHorizon 으로 시각 t 에 보간 (제어기가 받은 것과 같은 기준열).
             t 마다 header.stamp ≤ t 인 가장 최근 메시지를 쓴다.
  창         무장 구간 ∩ "주행 중" (기준 속력 > 0.05 m/s 가 처음 된 때 ~ 마지막 때).
  위치 오차   e = p − p_ref (월드). along = e·t̂ (+ 앞섬), cross = e·n̂ (n̂ = t̂ 를 +90°), 안쪽 = cross·sgn(ω_ref) (+ 원 중심 쪽).
             추정기 포즈(제어기가 보는 값)와 SLAM /tracked_pose (추적점 → 중심 lever arm 보정) 둘 다. 경로 거리 = /figure8/path 까지 최단 거리 (타이밍 무관).
  헤딩 오차   e_θ = wrap(ψ − ψ_ref).
  속력       v_ref = |u_ref[0:2]|, v = |v̂| (추정기 몸체 속도). 지연 = 교차상관 최대 (+ 실제가 늦음), 이득 = 지연 맞춘 뒤 회귀 기울기.
  가속도      100 Hz 격자에서 0.2 s 가운데 이동평균 뒤 미분 (위상 지연 0). 실제 = d|v̂|/dt, 독립 확인 = IMU a_x (몸체, 정지 구간 평균 뺌, 추적점 lever arm 보정 + ω v_y).
             yaw_mode tangent 라 몸체 x = 접선 → a_x ≈ dv/dt (a_x = v̇_x − ω v_y).
  v_y        추정기 몸체 v_y (기준은 tangent 라 0).
  슬립       /slip_logger/slip λ (바퀴 ω vs ω_nom = T⁻¹v̂). 큰 슬립 % = max|λ| > 0.5 인 시간 비율, 0.5 s 평균 뒤 같은 것,
             헛돎(λ>0)/끌림(λ<0) 몫 = |λ|>0.5 인 바퀴 표본 중 부호 비율.
  토크       순토크 (마찰 보상 전): C /torque_mpc/cmd_raw, A /wheel_pi/cmd_raw. 포화 = C 는 status τ̄ 의 98 % 이상, A 는 wheel_pi/status 포화 바퀴 수.
"""
import json
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from amr_mpc.core.reference import ReferenceHorizon

HERE = os.path.dirname(os.path.abspath(__file__))
TOPICS = ("/dra_mppi/reference_trajectory", "/state_estimator/odom", "/tracked_pose", "/figure8/path", "/mpc/armed",
          "/slip_logger/slip", "/wheel_states", "/imu/base_link", "/torque_mpc/status", "/pure_pursuit/status",
          "/wheel_pi/status", "/torque_mpc/cmd_raw", "/wheel_pi/cmd_raw", "/torque_output/cmd_raw", "/fault_monitor/fault")
FS = 100.0          # 격자 [Hz]
R_IMU = np.array([0.1093, 0.017])   # /tracked_pose·IMU 기준점 (base_footprint 기준 [m]) — config/torque_estimator.yaml imu_offset_x/y
SM = 0.2            # 가속도용 이동평균 [s]


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in TOPICS}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    out = {k: [] for k in TOPICS}
    while r.has_next():
        n, d, t = r.read_next()
        out[n].append((t * 1e-9, deserialize_message(d, types[n])))
    return out


def ma(x, n):
    if n <= 1:
        return x
    k = np.ones(n) / n
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xp, k, "valid")


def lag_gain(ref, act, fs, max_lag=0.6):
    """교차상관 최대 지연 [s] (+ act 가 늦음), 그 지연에서 회귀 기울기·상관계수."""
    a, b = ref - ref.mean(), act - act.mean()
    L = int(max_lag * fs)
    best = (-np.inf, 0)
    for k in range(-L, L + 1):
        if k >= 0:
            c = np.dot(a[:len(a) - k], b[k:]) / (len(a) - k)
        else:
            c = np.dot(a[-k:], b[:len(b) + k]) / (len(a) + k)
        best = max(best, (c, k))
    k = best[1]
    ra, rb = (a[:len(a) - k], b[k:]) if k >= 0 else (a[-k:], b[:len(b) + k])
    gain = np.dot(ra, rb) / np.dot(ra, ra)
    rho = np.corrcoef(ra, rb)[0, 1]
    return k / fs, gain, rho


def rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def analyze(bag):
    raw = read(bag)
    name = os.path.basename(bag.rstrip("/"))
    if raw["/pure_pursuit/status"]:          # pp·adpp 는 status 가 같다 → 추종 중 L_d (status 7) 가 변하면 adpp (2026-11-23)
        Ld = np.array([m.data[7] for t, m in raw["/pure_pursuit/status"] if len(m.data) >= 8 and m.data[6] == 0.0])
        ctrl = "A (adPP)" if len(Ld) and np.ptp(Ld) > 0.01 else "A (PP)"
    else:
        ctrl = "C (MPC)"
    # --- 무장 구간
    arm = [(t, bool(m.data)) for t, m in raw["/mpc/armed"]]
    s0 = next(t for t, on in arm if on)
    s1 = next((t for t, on in arm if not on and t > s0), raw["/state_estimator/odom"][-1][0])
    # --- 기준열
    refs = []
    for t, m in raw["/dra_mppi/reference_trajectory"]:
        if m.references:
            rows = [(r.x, r.y, r.yaw, r.vx, r.vy, r.yaw_rate) for r in m.references]
            refs.append((stamp(m.header), ReferenceHorizon.from_rows(stamp(m.header), m.dt, rows)))
    refs.sort(key=lambda x: x[0])
    rst = np.array([s for s, _ in refs])

    def ref_at(t):
        i = int(np.searchsorted(rst, t, side="right")) - 1
        if i < 0:
            return None
        Xr, ur = refs[i][1].at(t)
        return [*Xr, *ur]

    # --- 격자 (무장 구간)
    tg = np.arange(s0, s1, 1 / FS)
    R = np.array([ref_at(t) or [np.nan] * 6 for t in tg])            # x y ψ vx vy ω (몸체)
    vref = np.hypot(R[:, 3], R[:, 4])
    mv = np.where(vref > 0.05)[0]
    m0, m1 = tg[mv[0]], tg[mv[-1]]
    sel = (tg >= m0) & (tg <= m1)
    tg, R, vref = tg[sel], R[sel], vref[sel]
    # 추정기 → 격자
    O = np.array([[stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
                   m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z]
                  for t, m in raw["/state_estimator/odom"]])
    O = O[np.argsort(O[:, 0])]
    E = np.column_stack([np.interp(tg, O[:, 0], O[:, i]) for i in (1, 2, 4, 5, 6)])
    psi = np.interp(tg, O[:, 0], np.unwrap(O[:, 3]))
    x, y, vx, vy, wz = E.T
    v = np.hypot(vx, vy)
    # 위치 오차 (추정기)
    th = R[:, 2] + np.arctan2(R[:, 4], R[:, 3])                        # 기준 진행 방향 (월드)
    tx, ty = np.cos(th), np.sin(th)
    ex, ey = x - R[:, 0], y - R[:, 1]
    along, cross = ex * tx + ey * ty, -ex * ty + ey * tx
    inside = cross * np.sign(R[:, 5])
    turning = np.abs(R[:, 5]) > 0.2
    exy = np.hypot(ex, ey)
    eth = np.degrees(wrap(psi - R[:, 2]))
    # 경로 거리
    P = raw["/figure8/path"][-1][1]
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in P.poses])
    dpath = np.sqrt(((np.column_stack([x, y])[::5, None, :] - path[None]) ** 2).sum(-1)).min(1)
    # SLAM 포즈로 같은 오차
    T = np.array([[stamp(m.header), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)]
                  for t, m in raw["/tracked_pose"]])
    c, s_ = np.cos(T[:, 3]), np.sin(T[:, 3])                          # 추적점 → 로봇 중심: p_c = p_t − R(ψ) r
    T[:, 1] -= c * R_IMU[0] - s_ * R_IMU[1]
    T[:, 2] -= s_ * R_IMU[0] + c * R_IMU[1]
    T = T[(T[:, 0] >= m0) & (T[:, 0] <= m1)]
    RT = np.array([ref_at(t) for t in T[:, 0]])
    exy_slam = np.hypot(T[:, 1] - RT[:, 0], T[:, 2] - RT[:, 1])
    eth_slam = np.degrees(wrap(T[:, 3] - RT[:, 2]))
    # status 오차 (검산용)
    stn = "/pure_pursuit/status" if raw["/pure_pursuit/status"] else "/torque_mpc/status"
    St = np.array([m.data for t, m in raw[stn] if m0 <= m.data[0] <= m1])
    exy_status = np.hypot(St[:, 1], St[:, 2])
    # --- 속력·가속도
    n = int(SM * FS)
    vref_s, v_s = ma(vref, n), ma(v, n)
    aref = np.gradient(vref_s, 1 / FS)
    aest = np.gradient(v_s, 1 / FS)
    I = np.array([[stamp(m.header), m.linear_acceleration.x] for t, m in raw["/imu/base_link"]])
    I = I[np.argsort(I[:, 0])]
    pre = (I[:, 0] > s0 - 3.0) & (I[:, 0] < s0 - 0.2)
    ax_bias = I[pre, 1].mean() if pre.any() else 0.0
    ax_raw = np.interp(tg, I[:, 0], ma(I[:, 1], 8) - ax_bias)           # 400 Hz 에서 20 ms 먼저 평균 뒤 격자
    # IMU 는 추적점 r 에서 잰다: a_imu = a_c + ω̇ × r − ω² r  →  a_c,x = a_imu,x + ω̇ r_y + ω² r_x.  몸체 v̇_x = a_c,x + ω v_y
    wz_s = ma(wz, n)
    aimu = ma(ax_raw + np.gradient(wz_s, 1 / FS) * R_IMU[1] + wz_s ** 2 * R_IMU[0] + wz * vy, n)
    lag_v, gain_v, rho_v = lag_gain(vref, v, FS)
    lag_a, gain_a, rho_a = lag_gain(aref, aest, FS)
    lag_ai, gain_ai, rho_ai = lag_gain(aref, aimu, FS)
    cr = (tg >= m0 + 1.0) & (tg <= m1 - 1.5)                           # 변조 구간 (시작·끝 가감속 램프 뺌)
    nh = int(0.5 * FS)
    hp = lambda q: (q - ma(q, nh))[nh:-nh]                             # 0.5 s 이동평균 밖 (2 Hz 위) — 11-21 jitter.py 와 같은 정의
    # --- 슬립
    S = np.array([[t, *m.data[1:9]] for t, m in raw["/slip_logger/slip"]])
    S = S[(S[:, 0] >= m0) & (S[:, 0] <= m1)]
    lam = S[:, 1:5]
    W = np.array([[t, *m.velocity] for t, m in raw["/wheel_states"]])
    W = W[(W[:, 0] >= m0) & (W[:, 0] <= m1)]
    lam_abs_max = np.nanmax(np.abs(lam), axis=1)
    big = lam_abs_max > 0.5
    ns = max(1, int(round(0.5 / np.median(np.diff(S[:, 0])))))
    lam_avg = np.column_stack([ma(np.nan_to_num(lam[:, i]), ns) for i in range(4)])
    big_avg = np.max(np.abs(lam_avg), axis=1) > 0.5
    lb = lam[np.abs(lam) > 0.5]
    spin = float(np.mean(lb > 0)) if len(lb) else float("nan")
    a_at_s = np.interp(S[:, 0], tg, aref)
    phase = {"가속": a_at_s > 0.05, "감속": a_at_s < -0.05, "순항": np.abs(a_at_s) <= 0.05}
    big_by_phase = {k: float(100 * np.mean(big[msk])) if msk.any() else float("nan") for k, msk in phase.items()}
    est_jump = np.nanmax(np.abs(S[:, 5:9])) / np.max(np.abs(W[:, 1:5]))
    late = S[:, 0] > m0 + 2.0                                          # 출발 램프 (0 → 0.3 m/s, 1 m/s²) 뒤
    big_late = float(100 * np.mean(big[late]))
    big_wheel = [float(100 * np.mean(np.abs(lam[:, i]) > 0.5)) for i in range(4)]
    # 교차점 (경로 시작점 0.15 m 안) 근처 헤딩·v_y
    near = np.hypot(x - path[0, 0], y - path[0, 1]) < 0.15
    # --- 토크
    cn = "/wheel_pi/cmd_raw" if raw["/wheel_pi/cmd_raw"] else "/torque_mpc/cmd_raw"
    C = np.array([[t, *m.torque] for t, m in raw[cn] if m0 <= t <= m1])
    tau = C[:, 1:5]
    dtau = np.diff(tau, axis=0)
    rate = np.median(np.diff(C[:, 0]))
    Co = np.array([[t, *m.torque] for t, m in raw["/torque_output/cmd_raw"] if m0 <= t <= m1])
    if stn == "/torque_mpc/status":
        sat = np.mean(np.any(np.abs(St[:, 4:8]) >= 0.98 * St[:, 8:9], axis=1))
        tau_bar = float(np.median(St[:, 8]))
    else:
        Wp = np.array([m.data for t, m in raw["/wheel_pi/status"] if m0 <= m.data[0] <= m1])
        sat = np.mean(Wp[:, 13] > 0)
        tau_bar = float("nan")
    fault = [m.data for t, m in raw["/fault_monitor/fault"] if m.data]

    res = {
        "bag": name, "ctrl": ctrl, "arm_s": s1 - s0, "move_s": m1 - m0, "fault": fault,
        "path_len": float(np.sum(np.hypot(*np.diff(path, axis=0).T))),
        "exy_rms": rms(exy), "exy_max": float(exy.max()), "exy_slam_rms": rms(exy_slam), "exy_status_rms": rms(exy_status),
        "along_mean": float(along.mean()), "along_rms": rms(along), "cross_rms": rms(cross),
        "inside_mean_turn": float(inside[turning].mean()),
        "dpath_rms": rms(dpath), "dpath_max": float(dpath.max()),
        "eth_rms": rms(eth), "eth_max": float(np.abs(eth).max()), "eth_slam_rms": rms(eth_slam), "eth_mean": float(eth.mean()),
        "v_err_rms": rms(v - vref), "v_err_mean": float(np.mean(v - vref)), "v_lag": lag_v, "v_gain": gain_v, "v_rho": rho_v,
        "vref_rng": [float(vref[(tg > m0 + 3)].min()), float(vref.max())], "v_max": float(v.max()),
        "a_err_rms": rms(aest - aref), "a_lag": lag_a, "a_gain": gain_a, "a_rho": rho_a,
        "a_imu_err_rms": rms(aimu - aref), "a_imu_lag": lag_ai, "a_imu_gain": gain_ai, "a_imu_rho": rho_ai,
        "aref_max": float(np.abs(aref).max()),
        "v_err_rms_mod": rms((v - vref)[cr]), "a_err_rms_mod": rms((aest - aref)[cr]), "a_imu_err_rms_mod": rms((aimu - aref)[cr]),
        "jit_v": rms(hp(v)), "jit_vy": rms(hp(vy)), "jit_w": rms(hp(wz)),
        "vy_rms": rms(vy), "vy_max": float(np.abs(vy).max()), "w_err_rms": rms(wz - R[:, 5]),
        "vy_out_mean": float(np.mean((-vy * np.sign(R[:, 5]))[turning])),      # + = 원 바깥쪽 미끄럼
        "vy_rms_late": rms(vy[tg > m0 + 2.0]),
        "eth_rms_near": rms(eth[near]), "eth_rms_far": rms(eth[~near]), "near_pct": float(100 * near.mean()),
        "big_late": big_late, "big_wheel": big_wheel,
        "big_pct": float(100 * big.mean()), "big_avg_pct": float(100 * big_avg.mean()), "lam_rms": rms(lam),
        "lam_max": float(np.nanmax(np.abs(lam))), "spin_share": spin, "big_by_phase": big_by_phase, "est_jump": float(est_jump),
        "tau_p50": float(np.median(np.abs(tau))), "tau_p95": float(np.percentile(np.abs(tau), 95)),
        "tau_max": float(np.abs(tau).max()), "dtau_rms": rms(dtau), "dtau_rate_hz": float(1 / rate),
        "tau_out_p95": float(np.percentile(np.abs(Co[:, 1:5]), 95)), "sat_pct": float(100 * sat), "tau_bar": tau_bar,
    }
    series = dict(t=tg - m0, x=x, y=y, R=R, path=path, vref=vref, v=v, v_s=v_s, aref=aref, aest=aest, aimu=aimu,
                  exy=exy, eth=eth, along=along, inside=inside, vy=vy, wz=wz,
                  ts=S[:, 0] - m0, lam_abs_max=lam_abs_max, big=big,
                  xs=np.interp(S[:, 0], tg, x), ys=np.interp(S[:, 0], tg, y),
                  tt=C[:, 0] - m0, tau=tau)
    return res, series


def fmt_table(rs):
    rows = [
        ("주행 시간 [s]", "move_s", "{:.1f}"),
        ("위치 오차 |e| RMS / 최대 [cm] (추정기)", ("exy_rms", "exy_max"), "{:.1f} / {:.1f}", 100),
        ("  같은 것, SLAM 포즈 기준 RMS [cm]", "exy_slam_rms", "{:.1f}", 100),
        ("  검산: status e_x,e_y RMS [cm]", "exy_status_rms", "{:.1f}", 100),
        ("  along 평균 (+ 앞섬) / RMS [cm]", ("along_mean", "along_rms"), "{:+.1f} / {:.1f}", 100),
        ("  cross RMS [cm]", "cross_rms", "{:.1f}", 100),
        ("  원 위 안쪽 치우침 평균 [cm]", "inside_mean_turn", "{:+.1f}", 100),
        ("경로까지 거리 RMS / 최대 [cm]", ("dpath_rms", "dpath_max"), "{:.1f} / {:.1f}", 100),
        ("헤딩 오차 RMS / 최대 [°]", ("eth_rms", "eth_max"), "{:.1f} / {:.1f}"),
        ("  같은 것, SLAM 기준 RMS [°]", "eth_slam_rms", "{:.1f}"),
        ("속력 오차 RMS / 평균 [m/s]", ("v_err_rms", "v_err_mean"), "{:.3f} / {:+.3f}"),
        ("속력 지연 [ms] / 이득 / 상관", ("v_lag", "v_gain", "v_rho"), "{:.0f} / {:.2f} / {:.3f}", (1000, 1, 1)),
        ("가속도 오차 RMS [m/s²] (d|v̂|/dt)", "a_err_rms", "{:.3f}"),
        ("  지연 [ms] / 이득 / 상관", ("a_lag", "a_gain", "a_rho"), "{:.0f} / {:.2f} / {:.3f}", (1000, 1, 1)),
        ("가속도 오차 RMS [m/s²] (IMU a_x)", "a_imu_err_rms", "{:.3f}"),
        ("  지연 [ms] / 이득 / 상관", ("a_imu_lag", "a_imu_gain", "a_imu_rho"), "{:.0f} / {:.2f} / {:.3f}", (1000, 1, 1)),
        ("변조 구간만: 속력 / 가속도 / IMU 가속도 오차 RMS", ("v_err_rms_mod", "a_err_rms_mod", "a_imu_err_rms_mod"), "{:.3f} / {:.3f} / {:.3f}"),
        ("떨림 (2 Hz 위) RMS: |v̂| [m/s] / v_y [m/s] / ω_z [rad/s]", ("jit_v", "jit_vy", "jit_w"), "{:.3f} / {:.3f} / {:.3f}"),
        ("몸체 v_y RMS / 최대 [m/s]", ("vy_rms", "vy_max"), "{:.3f} / {:.3f}"),
        ("  출발 2 s 뒤 v_y RMS / 원 위 바깥쪽 평균 [m/s]", ("vy_rms_late", "vy_out_mean"), "{:.3f} / {:+.3f}"),
        ("헤딩 RMS: 교차점 0.15 m 안 / 그 밖 [°]", ("eth_rms_near", "eth_rms_far"), "{:.1f} / {:.1f}"),
        ("ω 오차 RMS [rad/s]", "w_err_rms", "{:.3f}"),
        ("큰 슬립 % (max|λ|>0.5)", "big_pct", "{:.1f}"),
        ("  0.5 s 평균 뒤 큰 슬립 %", "big_avg_pct", "{:.1f}"),
        ("  출발 2 s 뒤만 큰 슬립 %", "big_late", "{:.1f}"),
        ("λ RMS / |λ| 최대", ("lam_rms", "lam_max"), "{:.3f} / {:.2f}"),
        ("헛돎 몫 (|λ|>0.5 중 λ>0)", "spin_share", "{:.0%}"),
        ("순토크 |τ| 중앙 / 95 % / 최대 [N·m]", ("tau_p50", "tau_p95", "tau_max"), "{:.3f} / {:.3f} / {:.3f}"),
        ("Δτ RMS [N·m/주기]", "dtau_rms", "{:.4f}"),
        ("포화 시간 % (한 바퀴라도)", "sat_pct", "{:.1f}"),
    ]
    head = "| 지표 | " + " | ".join(f"{r['ctrl']} {r['bag']}" for r in rs) + " |"
    out = [head, "|---|" + "---|" * len(rs)]
    for row in rows:
        label, key, f = row[:3]
        scale = row[3] if len(row) > 3 else 1
        cells = []
        for r in rs:
            keys = key if isinstance(key, tuple) else (key,)
            sc = scale if isinstance(scale, tuple) else (scale,) * len(keys)
            vals = [r[k] * s if not isinstance(r[k], str) else r[k] for k, s in zip(keys, sc)]
            cells.append(f.format(*vals))
        out.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(out)


def plot(rs, ss, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [n for n in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic") if n in have] + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    col = {"A (PP)": "#d9822b", "A (adPP)": "#2a9d4b", "C (MPC)": "#2b6cd9"}
    fig = plt.figure(figsize=(16, 21), layout="constrained")
    gs = fig.add_gridspec(6, 2, height_ratios=[2.0, 1, 1, 1, 1, 1])
    # 1행: 궤적
    for j, (r, s) in enumerate(zip(rs, ss)):
        ax = fig.add_subplot(gs[0, j])
        ax.plot(s["path"][:, 0], s["path"][:, 1], color="0.6", lw=3, alpha=.5, label="/figure8/path")
        sc = ax.scatter(s["x"][::5], s["y"][::5], c=s["exy"][::5] * 100, s=4, cmap="viridis", vmin=0, vmax=20)
        b = s["big"]
        ax.scatter(s["xs"][b], s["ys"][b], s=10, facecolors="none", edgecolors="red", lw=.6, label="max|λ|>0.5")
        ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"{r['ctrl']} {r['bag']}  —  |e| RMS {100 * r['exy_rms']:.1f} cm, e_θ RMS {r['eth_rms']:.1f}°, 큰 슬립 {r['big_pct']:.1f} %")
        fig.colorbar(sc, ax=ax, fraction=.03, label="|e| [cm]")

    def ts(pos, key, ylabel, title, ref=None, scale=1.0, extra=None):
        ax = fig.add_subplot(gs[pos])
        if ref is not None:
            ax.plot(ss[0]["t"], ss[0][ref] * scale, "k", lw=1.2, label="기준")
        for r, s in zip(rs, ss):
            ax.plot(s["t"], s[key] * scale, color=col[r["ctrl"]], lw=.9, label=r["ctrl"])
            if extra:
                ax.plot(s["t"], s[extra] * scale, color=col[r["ctrl"]], lw=.6, ls=":", label=f"{r['ctrl']} IMU")
        ax.set_ylabel(ylabel); ax.set_title(title, fontsize=10); ax.grid(alpha=.3); ax.legend(fontsize=7, loc="upper right", ncol=3)
        return ax

    ts((1, 0), "v", "m/s", "속력: 기준 |u_ref| vs 추정기 |v̂|", ref="vref")
    ax = fig.add_subplot(gs[1, 1])
    for r, s in zip(rs, ss):
        ax.plot(s["t"], s["v"] - s["vref"], color=col[r["ctrl"]], lw=.8,
                label=f"{r['ctrl']}  RMS {r['v_err_rms']:.3f}, 지연 {1000 * r['v_lag']:.0f} ms")
    ax.axhline(0, color="k", lw=.5); ax.set_ylabel("m/s"); ax.set_title("속력 오차 |v̂| − v_ref", fontsize=10)
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    ts((2, 0), "aest", "m/s²", "접선 가속도: 기준 d v_ref/dt vs d|v̂|/dt (0.2 s 평균), 점선 IMU (lever arm 보정)", ref="aref", extra="aimu")
    ax = fig.add_subplot(gs[2, 1])
    for r, s in zip(rs, ss):
        ax.plot(s["t"], s["aest"] - s["aref"], color=col[r["ctrl"]], lw=.8,
                label=f"{r['ctrl']}  RMS {r['a_err_rms']:.3f} (IMU {r['a_imu_err_rms']:.3f})")
    ax.axhline(0, color="k", lw=.5); ax.set_ylabel("m/s²"); ax.set_title("가속도 오차 d|v̂|/dt − a_ref", fontsize=10)
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    ts((3, 0), "exy", "cm", "위치 오차 |p̂ − p_ref| (시간 기준)", scale=100)
    ts((3, 1), "eth", "°", "헤딩 오차 ψ̂ − ψ_ref")
    ts((4, 0), "along", "cm", "along 오차 (+ 기준보다 앞섬, − 늦음)", scale=100)
    ts((4, 1), "vy", "m/s", "몸체 v_y (기준 0)")
    ax = fig.add_subplot(gs[5, 0])
    for r, s in zip(rs, ss):
        ax.plot(s["ts"], s["lam_abs_max"], color=col[r["ctrl"]], lw=.5, alpha=.8, label=f"{r['ctrl']}  큰 슬립 {r['big_pct']:.1f} %")
    ax.axhline(0.5, color="r", ls="--", lw=.7); ax.set_ylim(0, 1.05); ax.set_ylabel("max|λ|"); ax.set_xlabel("주행 시작 뒤 [s]")
    ax.set_title("네 바퀴 중 최대 |λ|", fontsize=10); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax = fig.add_subplot(gs[5, 1])
    for r, s in zip(rs, ss):
        ax.plot(s["tt"], np.abs(s["tau"]).max(1), color=col[r["ctrl"]], lw=.5, alpha=.8,
                label=f"{r['ctrl']}  95 % {r['tau_p95']:.3f}, Δτ RMS {r['dtau_rms']:.4f}")
    ax.set_ylabel("N·m"); ax.set_xlabel("주행 시작 뒤 [s]"); ax.set_title("순토크 max|τ_i| (마찰 보상 전)", fontsize=10)
    ax.grid(alpha=.3); ax.legend(fontsize=8)
    fig.suptitle("circles8 width 2.0, speed 0.6 ± 50 % (sin, 랩당 2주기, φ −90°), laps 2 — 2026-11-22 한 판씩", fontsize=13)
    fig.savefig(out, dpi=110, bbox_inches="tight")


def main():
    rs, ss = [], []
    for b in sys.argv[1:]:
        r, s = analyze(b)
        rs.append(r); ss.append(s)
    print(fmt_table(rs))
    for r in rs:
        print(r["bag"], r["ctrl"], "무장", f"{r['arm_s']:.1f}s", "경로", f"{r['path_len']:.2f}m", "fault", r["fault"],
              "v_ref 범위", [round(v, 2) for v in r["vref_rng"]], "v 최대", round(r["v_max"], 2),
              "|a_ref| 최대", round(r["aref_max"], 3), "큰 슬립 국면별 %", {k: round(v, 1) for k, v in r["big_by_phase"].items()},
              "추정튐비", round(r["est_jump"], 2), "τ̄", r["tau_bar"], "τ_out p95", round(r["tau_out_p95"], 3),
              "cmd Hz", round(r["dtau_rate_hz"]), "e_θ 평균", round(r["eth_mean"], 2),
              "교차점 시간 %", round(r["near_pct"], 1), "바퀴별 |λ|>0.5 % FL FR RL RR", [round(v, 1) for v in r["big_wheel"]])
    out = os.path.join(HERE, "a20_three.png")
    plot(rs, ss, out)
    with open(os.path.join(HERE, "metrics.json"), "w") as f:
        json.dump(rs, f, ensure_ascii=False, indent=1)
    print("그림", out)


if __name__ == "__main__":
    main()
