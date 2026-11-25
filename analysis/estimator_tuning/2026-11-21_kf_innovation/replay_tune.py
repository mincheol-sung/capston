#!/usr/bin/python3
"""실차 bag 으로 kf 잡음 값(=게인) 고르기 — innovation 기반 (2026-11-21).

  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 analysis/estimator_tuning/2026-11-21_kf_innovation/replay_tune.py bags/2026-11-21/203133

bag 의 /imu/base_link (400 Hz) 와 /tracked_pose (10 Hz) 를 bag 수신 순서대로 PlanarStateEstimator(kf) 에 넣는다 (노드와 같은 호출).
포즈마다 innovation ν = z − (그 stamp 시각의 예측) = estimator.last_residual.
  1) 지금 yaml 값으로 재생 → bag 에 기록된 /state_estimator/status 잔차와 같아야 한다 (재생이 노드와 같다는 검산)
  2) 잡음 값 격자마다: 주행 구간 innovation RMS (prediction error method 의 기준), lag-1 자기상관 (whiteness), NIS = ν²/S_ss
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../src/amr_basic_stack/controller/mpc"))
from amr_mpc.core import config  # noqa: E402
from amr_mpc.torque.estimator import PlanarStateEstimator, design_gains  # noqa: E402

TOP = ["/imu/base_link", "/tracked_pose", "/state_estimator/status"]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(b):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=TOP))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    ev, status = [], []
    while r.has_next():
        n, d, trx = r.read_next()
        m = deserialize_message(d, types[n])
        if n == "/imu/base_link":
            ev.append((trx, 0, st(m.header), m.linear_acceleration.x, m.linear_acceleration.y, m.angular_velocity.z))
        elif n == "/tracked_pose":
            p = m.pose
            ev.append((trx, 1, st(m.header), p.position.x, p.position.y, yaw(p.orientation)))
        else:
            status.append(list(m.data))
    ev.sort(key=lambda e: (e[0], e[1]))
    return ev, np.array(status)


def replay(ev, cfg, T=0.1):
    est = PlanarStateEstimator(cfg, pose_period=T)
    last = -1e9; out = []
    for _, kind, t, a, b, c in ev:
        if kind == 0:
            est.imu(t, a, b, c)
        else:
            if t <= last:
                continue
            last = t
            r = est.pose(t, a, b, c)
            if r in ("used", "jump", "reset") and est.t is not None:
                s = est.state()
                out.append((est.t, *est.last_residual, math.hypot(s[4], s[5]), r == "used"))
    return np.array(out), est


def stats(R, mov, Sxy, Syaw):
    e = R[mov]
    rho = lambda x: float(np.corrcoef(x[:-1], x[1:])[0, 1])
    exy = e[:, 1:3]
    return dict(n=len(e), rms_xy=float(np.sqrt(np.mean(exy ** 2))), rms_yaw=float(np.sqrt(np.mean(e[:, 3] ** 2))),
                rho_xy=0.5 * (rho(exy[:, 0]) + rho(exy[:, 1])), rho_yaw=rho(e[:, 3]),
                nis_xy=float(np.mean(exy ** 2) / Sxy), nis_yaw=float(np.mean(e[:, 3] ** 2) / Syaw))


def S_ss(T, na, nb, sp, ng, nbg, syaw):
    """정상상태 innovation 분산 S = H P⁻ Hᵀ + R (축 하나, yaw). P⁻ = DARE 해 (scipy)."""
    from scipy.linalg import solve_discrete_are
    F = np.array([[1, T, -.5 * T * T], [0, 1, -T], [0, 0, 1.]]); qa, qb = na ** 2, nb ** 2
    Q = np.array([[qa * T ** 3 / 3, qa * T * T / 2, 0], [qa * T * T / 2, qa * T, 0], [0, 0, qb * T]])
    Pm = solve_discrete_are(F.T, np.array([[1.], [0], [0]]), Q, np.array([[sp ** 2]]))
    Fy = np.array([[1, -T], [0, 1.]]); Qy = np.diag([ng ** 2 * T, nbg ** 2 * T])
    Pym = solve_discrete_are(Fy.T, np.array([[1.], [0]]), Qy, np.array([[syaw ** 2]]))
    return Pm[0, 0] + sp ** 2, Pym[0, 0] + syaw ** 2


if __name__ == "__main__":
    bag = sys.argv[1]
    ev, status = load(bag)
    base = dict(config.section("estimator", "torque_estimator.yaml")); base["mode"] = "kf"
    R0, est0 = replay(ev, base)
    # 1) 검산: 기록된 /state_estimator/status [t, e_x, e_y, e_ψ, ...] 와 재생 잔차 (t 최근접)
    if len(status):
        j = np.searchsorted(R0[:, 0], status[:, 0]).clip(1, len(R0) - 1)
        j = np.where(np.abs(R0[j - 1, 0] - status[:, 0]) < np.abs(R0[j, 0] - status[:, 0]), j - 1, j)
        ok = np.abs(R0[j, 0] - status[:, 0]) < 0.005
        d = np.abs(R0[j[ok], 1:4] - status[ok, 1:4])
        print(f"검산: 기록 잔차 {ok.sum()}/{len(status)} 개와 맞춤 — 차 최대 xy {d[:, :2].max()*1e3:.3f} mm, yaw {np.degrees(d[:, 2].max()):.4f}°")
    mov = (R0[:, 4] > 0.05) & (R0[:, 5] > 0)
    print(f"포즈 {len(R0)} 개 중 주행 (추정 속도 > 0.05 m/s) {mov.sum()} 개, 튐 {int((R0[:, 5] == 0).sum())} 개")
    na0, nb0, sp0 = base["accel_noise_density"], base["accel_bias_walk"], base["pose_noise_xy"]
    ng0, nbg0, sy0 = base["gyro_noise_density"], base["gyro_bias_walk"], math.radians(base["pose_noise_yaw_deg"])
    print("\n| σ_a | σ_b | xy 빠른 τ ms | 바이어스 τ s | ν RMS mm | ρ₁ | NIS (지금 σ_p 3 mm) |")
    print("|---|---|---|---|---|---|---|")
    res = []
    for ma in (0.25, 0.5, 1.0, 2.0, 4.0, 8.0):
        for mb in (0.25, 1.0, 4.0):
            cfg = dict(base); cfg["accel_noise_density"] = na0 * ma; cfg["accel_bias_walk"] = nb0 * mb
            R, _ = replay(ev, cfg)
            m = (R[:, 4] > 0.05) & (R[:, 5] > 0)
            Sx, Sy = S_ss(0.1, na0 * ma, nb0 * mb, sp0, ng0, nbg0, sy0)
            s = stats(R, m, Sx, Sy)
            wn = math.sqrt(na0 * ma / (sp0 * math.sqrt(0.1)))
            res.append((ma, mb, s))
            print(f"| {na0*ma:.3f} | {nb0*mb:.3f} | {1/(0.7071*wn)*1e3:.0f} | {na0*ma/(nb0*mb):.2f} | {s['rms_xy']*1e3:.2f} | {s['rho_xy']:+.2f} | {s['nis_xy']:.1f} |")
    print("\n| σ_g | σ_bg | yaw 빠른 τ ms | 바이어스 τ s | ν_ψ RMS ° | ρ₁ | NIS (지금 σ_ψ 0.2°) |")
    print("|---|---|---|---|---|---|---|")
    for mg in (0.25, 0.5, 1.0, 2.0, 4.0):
        for mbg in (0.25, 1.0, 4.0):
            cfg = dict(base); cfg["gyro_noise_density"] = ng0 * mg; cfg["gyro_bias_walk"] = nbg0 * mbg
            R, _ = replay(ev, cfg)
            m = (R[:, 4] > 0.05) & (R[:, 5] > 0)
            Sx, Sy = S_ss(0.1, na0, nb0, sp0, ng0 * mg, nbg0 * mbg, sy0)
            s = stats(R, m, Sx, Sy)
            print(f"| {ng0*mg:.4f} | {nbg0*mbg:.5f} | {sy0*math.sqrt(0.1)/(ng0*mg)*1e3:.0f} | {ng0*mg/(nbg0*mbg):.1f} | "
                  f"{math.degrees(s['rms_yaw']):.3f} | {s['rho_yaw']:+.2f} | {s['nis_yaw']:.1f} |")
