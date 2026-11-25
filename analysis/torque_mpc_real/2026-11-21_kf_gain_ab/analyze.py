#!/usr/bin/python3
"""실차 A/B: kf 게인 지금 (yaml) vs 후보 (est_gains:=candidate_1121) — C, circles8 W2.0 0.7 m/s, origin_mode robot (2026-11-21).

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/torque_mpc_real/2026-11-21_kf_gain_ab/analyze.py

게인이 바뀌면 추정기 출력으로 낸 지표의 기준도 같이 바뀐다 → 게인과 무관한 SLAM 포즈(/tracked_pose)를 기준으로 다시 계산한다.
  창: /torque_mpc/status 의 hold = 0 (추종 중) 첫 틱 ~ 마지막 틱, 기준 속도 > 0.05 m/s
  추종 (SLAM 기준) : SLAM 포즈(IMU 위치)를 로봇 중심으로 옮김 c = p − R(ψ) r (r = torque_estimator.yaml imu_offset) → 그 stamp 의 기준
                     (references[0] 목표 시각 = stamp + dt) 과의 |e_xy|·헤딩. 경로 거리 = /figure8/path 꺾은선까지
  제어기가 본 오차 : /torque_mpc/status e_x, e_y (기준 − 추정)
  추정 속도        : /state_estimator/odom 몸체 v 와 SLAM 중앙 차분 속도 (앞뒤 포즈, 0.2 s) 의 차 RMS. 떨림 = v − 0.2 s 이동평균 의 RMS
  innovation       : /state_estimator/status 잔차 (포즈마다) RMS·lag-1 상관 ρ₁ — 각 판 자기 게인의 것
  토크             : /torque_mpc/cmd_raw (MODE_TORQUE) 떨림 = τ − 0.2 s 이동평균 RMS, 틱간 Δτ RMS, 상한 닿음 = |τ_i| ≥ 0.98 τ̄_i
  슬립             : slip_logger λ (추정 속도 기준, 게인 따라 기준이 바뀜) 와 SLAM 기준 λ (ω_nom = T⁻¹ [v_SLAM, ω_gyro])
                     큰 슬립 = max_i|λ_i| > 0.5 시간 %, > 0.1 %, λ 최대·RMS, 큰 슬립 중 헛돎(λ > 0) 몫 (메모리 analysis-include-slip)
"""
import math
import os
import re
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../bags/2026-11-21")
RUNS = {"지금": ["225012", "225104", "225157"], "후보": ["225302", "225349", "225435"]}
R_IMU = np.array([0.1093, 0.017])          # torque_estimator.yaml imu_offset_x/y
WR, KG, EPS = 0.04, 0.1932, 2.5            # 바퀴 반지름, k = (wheel_base + width)/2, slip_logger omega_eps
TOP = ["/tracked_pose", "/dra_mppi/reference_trajectory", "/torque_mpc/status", "/torque_mpc/cmd_raw", "/state_estimator/odom",
       "/state_estimator/status", "/wheel_states", "/slip_logger/slip", "/figure8/path"]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=TOP))
    ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in TOP}
    while r.has_next():
        n, d, t = r.read_next()
        D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    return D


def tau_bar(run):
    d = os.path.join(ROOT, run, "run_info", "ros_log")
    for f in sorted(os.listdir(d)):
        if f.startswith("torque_mpc_node"):
            g = re.findall(r"쓰는 τ̄ \[([0-9.\s]+)\]", open(os.path.join(d, f), errors="replace").read())
            if g:
                return np.array([float(x) for x in g[-1].split()])
    return None


def lam(w, wn):
    return (w - wn) * np.sign(wn) / np.maximum(np.maximum(np.abs(w), np.abs(wn)), EPS)


def slip_stats(L):
    L = L[np.all(np.isfinite(L), axis=1)]
    a = np.abs(L); mx = a.max(1); big = mx > 0.5
    sg = np.sign(L[np.arange(len(L)), a.argmax(1)])
    return dict(big=100 * big.mean(), s01=100 * (mx > 0.1).mean(), lmax=float(mx.max()), lrms=float(np.sqrt(np.mean(L ** 2))),
                spin=100 * (sg[big] > 0).mean() if big.any() else float("nan"))


def met(run):
    D = read(os.path.join(ROOT, run))
    S = np.array([[t, *m.data[1:3], m.data[20] if len(m.data) > 20 else 0.0] for t, m in D["/torque_mpc/status"]])
    act = S[S[:, 3] == 0]
    t0, t1 = act[0, 0], act[-1, 0]
    R = np.array(sorted((st(m.header) + m.dt, r.x, r.y, r.yaw, r.vx, r.vy) for _, m in D["/dra_mppi/reference_trajectory"]
                        if m.references for r in m.references[:1]))
    P = np.array(sorted({(st(m.header), m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation)) for _, m in D["/tracked_pose"]}))
    c, s = np.cos(P[:, 3]), np.sin(P[:, 3])
    C = np.stack([P[:, 0], P[:, 1] - (c * R_IMU[0] - s * R_IMU[1]), P[:, 2] - (s * R_IMU[0] + c * R_IMU[1]), P[:, 3]], 1)
    # SLAM 중앙 차분 속도 (월드 → 몸체), 앞뒤 포즈
    vw = (C[2:, 1:3] - C[:-2, 1:3]) / (C[2:, 0] - C[:-2, 0])[:, None]
    Cm = C[1:-1]; cm, sm = np.cos(Cm[:, 3]), np.sin(Cm[:, 3])
    vb = np.stack([cm * vw[:, 0] + sm * vw[:, 1], -sm * vw[:, 0] + cm * vw[:, 1]], 1)
    ref_sp = np.hypot(np.interp(Cm[:, 0], R[:, 0], R[:, 4]), np.interp(Cm[:, 0], R[:, 0], R[:, 5]))
    win = (Cm[:, 0] >= t0) & (Cm[:, 0] <= t1) & (ref_sp > 0.05) & (Cm[:, 0] <= R[-1, 0])
    q = Cm[win]
    rx, ry = np.interp(q[:, 0], R[:, 0], R[:, 1]), np.interp(q[:, 0], R[:, 0], R[:, 2])
    ryaw = np.interp(q[:, 0], R[:, 0], np.unwrap(R[:, 3]))
    exy = np.hypot(q[:, 1] - rx, q[:, 2] - ry)
    eth = np.degrees(np.abs(np.arctan2(np.sin(q[:, 3] - ryaw), np.cos(q[:, 3] - ryaw))))
    path = np.array([[p.pose.position.x, p.pose.position.y] for p in D["/figure8/path"][-1][1].poses]) if D["/figure8/path"] else None
    dist = None
    if path is not None and len(path) > 1:
        a_, ab = path[:-1], path[1:] - path[:-1]
        qq = q[:, None, 1:3] - a_[None]
        tt = np.clip((qq * ab[None]).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12)[None], 0, 1)
        dist = np.linalg.norm(qq - tt[..., None] * ab[None], axis=-1).min(1)
    # 추정기
    E = np.array([[st(m.header), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z] for _, m in D["/state_estimator/odom"]])
    ve = np.stack([np.interp(q[:, 0], E[:, 0], E[:, 1]), np.interp(q[:, 0], E[:, 0], E[:, 2])], 1)
    dv = np.hypot(*(ve - vb[win]).T)
    Ew = E[(E[:, 0] >= t0) & (E[:, 0] <= t1)]
    k = 20
    ma = np.array([np.convolve(Ew[:, i], np.ones(k) / k, "same") for i in (1, 2)]).T
    vjit = np.hypot(*(Ew[:, 1:3] - ma)[k:-k].T)
    I = np.array([m.data[:4] for _, m in D["/state_estimator/status"]])
    spI = np.hypot(np.interp(I[:, 0], E[:, 0], E[:, 1]), np.interp(I[:, 0], E[:, 0], E[:, 2]))
    I = I[(I[:, 0] >= t0) & (I[:, 0] <= t1) & (spI > 0.05)]
    rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
    # 제어기 오차
    ectl = np.hypot(act[:, 1], act[:, 2])
    # 토크
    T = np.array([[st(m.header), *m.torque] for _, m in D["/torque_mpc/cmd_raw"] if m.mode == 0])
    T = T[(T[:, 0] >= t0) & (T[:, 0] <= t1)]
    tau = T[:, 1:5]
    hp = (tau - np.array([np.convolve(tau[:, i], np.ones(k) / k, "same") for i in range(4)]).T)[k:-k]
    tb = tau_bar(run)
    # 슬립: logger (추정 속도 기준) / SLAM 기준
    SL = np.array([[t, *m.data[1:5]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 5])
    SL = SL[(SL[:, 0] >= t0) & (SL[:, 0] <= t1)]
    spS = np.hypot(np.interp(SL[:, 0], E[:, 0], E[:, 1]), np.interp(SL[:, 0], E[:, 0], E[:, 2]))
    W = np.array([[st(m.header), *m.velocity] for _, m in D["/wheel_states"]])
    wq = np.stack([np.interp(q[:, 0], W[:, 0], W[:, 1 + i]) for i in range(4)], 1)
    wz = np.interp(q[:, 0], E[:, 0], E[:, 3])
    vx, vy = vb[win, 0], vb[win, 1]
    wn = np.stack([(vx - vy - KG * wz), (vx + vy + KG * wz), (vx + vy - KG * wz), (vx - vy + KG * wz)], 1) / WR
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return dict(run=run, T=float(t1 - t0), n=int(win.sum()),
                exy=rms(exy), exy_max=float(exy.max()), eth=rms(eth), dist=rms(dist) if dist is not None else float("nan"),
                ectl=rms(ectl), dv=rms(dv), vjit=rms(vjit),
                inu=rms(np.hypot(I[:, 1], I[:, 2])), rho=0.5 * (rho(I[:, 1]) + rho(I[:, 2])),
                inu_y=math.degrees(rms(I[:, 3])), rho_y=rho(I[:, 3]),
                thp=rms(hp), dtau=rms(np.diff(tau, axis=0)), tmax=float(np.abs(tau).max()),
                sat=float(100 * np.mean(np.abs(tau) >= 0.98 * tb[None, :])) if tb is not None else float("nan"),
                **{f"lg_{k_}": v for k_, v in slip_stats(SL[spS > 0.05, 1:5]).items()},
                **{f"sl_{k_}": v for k_, v in slip_stats(lam(wq, wn)).items()})


ROWS = [("T", "추종 시간 s", 1, "{:.1f}"), ("exy", "SLAM 기준 추종 |e| RMS cm", 100, "{:.1f}"), ("exy_max", "  최대 cm", 100, "{:.1f}"),
        ("dist", "SLAM 기준 경로 거리 RMS cm", 100, "{:.1f}"), ("eth", "SLAM 기준 헤딩 오차 RMS °", 1, "{:.2f}"),
        ("ectl", "제어기가 본 오차 RMS cm", 100, "{:.1f}"),
        ("dv", "추정 속도 − SLAM 차분 속도 RMS cm/s", 100, "{:.1f}"), ("vjit", "추정 속도 떨림 RMS cm/s", 100, "{:.2f}"),
        ("inu", "innovation xy RMS mm", 1000, "{:.2f}"), ("rho", "innovation ρ₁ xy", 1, "{:+.2f}"),
        ("inu_y", "innovation yaw RMS °", 1, "{:.3f}"), ("rho_y", "innovation ρ₁ yaw", 1, "{:+.2f}"),
        ("thp", "토크 떨림 RMS N·m", 1, "{:.4f}"), ("dtau", "틱간 Δτ RMS N·m", 1, "{:.4f}"), ("tmax", "최대 |τ| N·m", 1, "{:.3f}"),
        ("sat", "상한 닿음 %", 1, "{:.1f}"),
        ("sl_big", "큰 슬립 % (SLAM 기준 λ)", 1, "{:.1f}"), ("sl_s01", "미끄럼 > 0.1 % (SLAM 기준)", 1, "{:.1f}"),
        ("sl_lmax", "λ 최대 (SLAM 기준)", 1, "{:.2f}"), ("sl_lrms", "λ RMS (SLAM 기준)", 1, "{:.3f}"), ("sl_spin", "큰 슬립 중 헛돎 % (SLAM 기준)", 1, "{:.0f}"),
        ("lg_big", "큰 슬립 % (logger λ, 추정 속도 기준)", 1, "{:.1f}"), ("lg_s01", "미끄럼 > 0.1 % (logger)", 1, "{:.1f}"),
        ("lg_lrms", "λ RMS (logger)", 1, "{:.3f}")]

if __name__ == "__main__":
    res = {g: [met(r) for r in runs] for g, runs in RUNS.items()}
    print("| 지표 | 지금 (" + " / ".join(RUNS["지금"]) + ") | 후보 (" + " / ".join(RUNS["후보"]) + ") | 후보/지금 (평균) |")
    print("|---|---|---|---|")
    for key, lab, sc, fmt in ROWS:
        a = [m[key] * sc for m in res["지금"]]; b = [m[key] * sc for m in res["후보"]]
        ra = np.nanmean(b) / np.nanmean(a) if np.nanmean(a) not in (0.0,) and np.isfinite(np.nanmean(a)) else float("nan")
        print(f"| {lab} | {' / '.join(fmt.format(x) for x in a)} | {' / '.join(fmt.format(x) for x in b)} | {ra:.2f} |")
    print(f"\nSLAM 포즈 수 (창 안): 지금 {[m['n'] for m in res['지금']]}, 후보 {[m['n'] for m in res['후보']]}")
