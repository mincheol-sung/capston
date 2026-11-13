#!/usr/bin/env python3
"""바퀴 각속도 → 휠 오도메트리 (계산부, ROS 무관).

쓰임새 (2026-11-11 결정): Cartographer 로컬 스캔 매칭의 **초기 짐작에만** 쓰는 선택 입력이다.
  · 슬립 측정·제어에는 쓰지 않는다 (그건 state_estimator: 라이다 + IMU).
  · 기본은 꺼 둔다 (config/torque_estimator.yaml slam.use_wheel_odom). 기준물을 둔 트랙에서 휠 오돔 없이
    측위가 버티면 끈 채로 간다. bag replay 에서 통로꼴 구간은 휠 오돔 없이 30 s 에 약 30 cm 미끄러졌다.

식 — 드라이버 twist_to_wheels 의 역 (메카넘, 바퀴 순서 FL, FR, RL, RR)
    ω = T⁻¹ v,   T⁻¹ = (1/r)[[1,−1,−k],[1,1,k],[1,1,−k],[1,−1,k]]
    v = T ω,     T = (T⁻¹)⁺ = (r/4)[[1,1,1,1],[−1,1,1,−1],[−1/k,1/k,−1/k,1/k]]   (최소제곱 — 네 바퀴가 서로 안 맞으면 평균)
  포즈 적분은 dt 동안 몸체 속도가 일정하다고 보고 SE(2) 를 정확히 적분한다 (회전 중에도 원호를 따라간다).
"""
import math

import numpy as np


def forward_matrix(r, k):
    """T (3×4): 바퀴 각속도 → 몸체 [v_x, v_y, ω_z]."""
    return np.array([[1.0, 1.0, 1.0, 1.0],
                     [-1.0, 1.0, 1.0, -1.0],
                     [-1.0 / k, 1.0 / k, -1.0 / k, 1.0 / k]]) * (r / 4.0)


def integrate_se2(x, y, th, vx, vy, w, dt):
    """몸체 속도 (vx, vy, w) 를 dt 동안 일정하게 두고 정확 적분."""
    a = w * dt
    if abs(a) < 1e-9:
        dxb, dyb = vx * dt, vy * dt
    else:
        s, c = math.sin(a) / a, (1.0 - math.cos(a)) / a
        dxb, dyb = (s * vx - c * vy) * dt, (c * vx + s * vy) * dt
    ct, st = math.cos(th), math.sin(th)
    return x + ct * dxb - st * dyb, y + st * dxb + ct * dyb, math.atan2(math.sin(th + a), math.cos(th + a))


class WheelOdometry:
    def __init__(self, r, k, max_dt=0.1):
        self.T = forward_matrix(r, k)
        self.max_dt = max_dt
        self.reset()

    def reset(self, x=0.0, y=0.0, th=0.0):
        self.x, self.y, self.th = x, y, th
        self.v = np.zeros(3)
        self.t = None

    def update(self, t, wheel_omega):
        """바퀴 각속도 [FL, FR, RL, RR] (rad/s) 한 표본. 반환 (x, y, θ, v_x, v_y, ω_z)."""
        v = self.T @ np.asarray(wheel_omega, float)
        if self.t is not None:
            dt = t - self.t
            if 0.0 < dt <= self.max_dt:
                m = 0.5 * (self.v + v)                 # 사다리꼴: 두 표본 사이 평균 속도
                self.x, self.y, self.th = integrate_se2(self.x, self.y, self.th, m[0], m[1], m[2], dt)
        self.t = t
        self.v = v
        return self.x, self.y, self.th, float(v[0]), float(v[1]), float(v[2])
