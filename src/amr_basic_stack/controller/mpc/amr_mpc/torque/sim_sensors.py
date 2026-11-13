#!/usr/bin/env python3
"""시뮬 센서 — 참값 상태에서 실차와 같은 꼴의 IMU 와 SLAM 포즈를 만든다. ROS 에 의존하지 않는다.

실차를 흉내 내는 점 (수치는 bags/mpc_chain_20260923_* 2026-11-11 분석)
  · IMU 와 SLAM 포즈는 로봇 중심이 아니라 tracking_frame(카메라 IMU 위치, 중심에서 r)의 것이다.
  · IMU 200 Hz. 가속도계에 일정한 바이어스(기울어진 장착의 중력 성분, 약 0.15 m/s²)와,
    움직일 때 커지는 진동 잡음(정지 0.015 → 주행 0.95 m/s², 자이로 0.003 → 0.09 rad/s)이 있다.
  · 스캔 매칭은 10 Hz, 결과는 스캔 시각보다 slam_latency(실측 0.13 s) 늦게 나온다.
  · slam_mode
      scan_time    : 스캔 시각 stamp 의 포즈를 늦게 한 번 낸다 (Cartographer use_pose_extrapolator = false 꼴)
      extrapolated : 100 Hz 로 stamp = 지금 인 포즈를 낸다. 마지막 두 스캔 포즈의 차로 구한 일정 속도로 위치를 외삽하고
                     (use_odometry = false 의 Cartographer 외삽기), 방향은 자이로 적분으로 잇는다. 새 스캔 매칭이 반영되는
                     틱에는 scan_matched 사건을 먼저 낸다 (실차의 /scan_matched_points2).
"""
from collections import deque
import math

import numpy as np

DEFAULTS = dict(
    imu_offset_x=0.098, imu_offset_y=0.017,
    accel_bias_x=-0.15, accel_bias_y=0.13, gyro_bias=0.0002,
    accel_noise_still=0.015, accel_noise_moving=0.95,
    gyro_noise_still=0.003, gyro_noise_moving=0.09,
    noise_full_speed=0.10,              # 이 속도[m/s]부터 주행 수준 잡음
    slam_mode="scan_time", slam_period=0.1, slam_latency=0.13,
    slam_noise_xy=0.002, slam_noise_yaw_deg=0.1,
    pose_pub_period=0.01,               # extrapolated 모드의 발행 주기
    gravity=9.81,
)


class SimSensors:
    def __init__(self, cfg=None, seed=0):
        c = dict(DEFAULTS)
        c.update(cfg or {})
        if c["slam_mode"] not in ("scan_time", "extrapolated"):
            raise ValueError(f"slam_mode={c['slam_mode']!r} — scan_time | extrapolated")
        self.c = c
        self.r = np.array([c["imu_offset_x"], c["imu_offset_y"]], float)
        self.rng = np.random.default_rng(seed)
        self.prev = None                    # (t, v_imu_map[2], theta)
        self.next_scan = None
        self.next_pose_pub = None
        self.pending = deque()              # (release_t, t_scan, x, y, yaw, theta_true)
        self.scans = deque(maxlen=2)        # 외삽기가 가진 스캔 포즈 (t_scan, p[2], yaw, theta_true)

    def _point(self, X):
        c, s = math.cos(X[2]), math.sin(X[2])
        return np.array([X[0] + c * self.r[0] - s * self.r[1], X[1] + s * self.r[0] + c * self.r[1]])

    def _point_vel(self, X, Xd):
        c, s = math.cos(X[2]), math.sin(X[2])
        return np.array([Xd[0] + Xd[2] * (-s * self.r[0] - c * self.r[1]), Xd[1] + Xd[2] * (c * self.r[0] - s * self.r[1])])

    def step(self, t, X, Xd):
        """참값 X = [x, y, θ], Ẋ (관성 좌표) → {'imu': (a_x, a_y, a_z, ω_z), 'scan_matched': [t_scan…], 'poses': [(stamp, x, y, yaw)…]}"""
        c = self.c
        v_i = self._point_vel(X, Xd)
        out = {"imu": None, "scan_matched": [], "poses": []}
        # ── IMU: 지난 표본 이후의 평균 가속도·각속도 (적분하면 참 속도가 그대로 나온다)
        if self.prev is not None and t > self.prev[0]:
            t0, v0, th0 = self.prev
            dt = t - t0
            dth = math.atan2(math.sin(X[2] - th0), math.cos(X[2] - th0))
            a_map = (v_i - v0) / dt
            thm = th0 + 0.5 * dth
            cm, sm = math.cos(thm), math.sin(thm)
            speed = math.hypot(Xd[0], Xd[1]) + 0.2 * abs(Xd[2])
            k = min(1.0, speed / c["noise_full_speed"])
            sa = c["accel_noise_still"] + k * (c["accel_noise_moving"] - c["accel_noise_still"])
            sg = c["gyro_noise_still"] + k * (c["gyro_noise_moving"] - c["gyro_noise_still"])
            out["imu"] = (cm * a_map[0] + sm * a_map[1] + c["accel_bias_x"] + self.rng.normal(0, sa),
                          -sm * a_map[0] + cm * a_map[1] + c["accel_bias_y"] + self.rng.normal(0, sa),
                          c["gravity"] + self.rng.normal(0, sa),
                          dth / dt + c["gyro_bias"] + self.rng.normal(0, sg))
        self.prev = (t, v_i, float(X[2]))

        # ── 스캔: 지금의 tracking_frame 포즈를 재고, slam_latency 뒤에 나오게 넣어 둔다
        if self.next_scan is None:
            self.next_scan = t
        if t >= self.next_scan:
            p = self._point(X) + self.rng.normal(0, c["slam_noise_xy"], 2)
            yaw = X[2] + self.rng.normal(0, math.radians(c["slam_noise_yaw_deg"]))
            self.pending.append((t + c["slam_latency"], t, p[0], p[1], yaw, float(X[2])))
            self.next_scan += c["slam_period"]
        while self.pending and self.pending[0][0] <= t:
            _, ts, x, y, yaw, th = self.pending.popleft()
            if c["slam_mode"] == "scan_time":
                out["poses"].append((ts, x, y, math.atan2(math.sin(yaw), math.cos(yaw))))
            else:
                self.scans.append((ts, np.array([x, y]), yaw, th))
                out["scan_matched"].append(ts)

        # ── 외삽 스트림 (100 Hz, stamp = 지금)
        if c["slam_mode"] == "extrapolated" and self.scans:
            if self.next_pose_pub is None:
                self.next_pose_pub = t
            if t >= self.next_pose_pub or out["scan_matched"]:
                ts, p, yaw, th = self.scans[-1]
                v = np.zeros(2)
                if len(self.scans) == 2 and ts > self.scans[0][0]:
                    v = (p - self.scans[0][1]) / (ts - self.scans[0][0])
                pe = p + v * (t - ts)
                ye = yaw + math.atan2(math.sin(X[2] - th), math.cos(X[2] - th))
                out["poses"].append((t, pe[0], pe[1], math.atan2(math.sin(ye), math.cos(ye))))
                if t >= self.next_pose_pub:
                    self.next_pose_pub += c["pose_pub_period"]
        return out
