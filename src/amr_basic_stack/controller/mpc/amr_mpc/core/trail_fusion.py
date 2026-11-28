#!/usr/bin/env python3
"""RViz 지나간 경로용 포즈 두 가지 (표시 전용, 제어에 쓰지 않는다, 2026-11-26).

  slam_to_base(...)  /tracked_pose (tracking_frame = 카메라 IMU 점) → base_footprint 포즈
                     p_base = p_track − R(ψ)·r,   r = (imu_offset_x, imu_offset_y)  (torque_estimator.yaml)
                     2026-11-26 bag 191922·191432: estimator − /tracked_pose 의 몸체 좌표 차이 평균 (−10.9, −1.7) cm,
                     표준편차 0.6~1.5 cm → r = (0.1093, 0.017) 과 맞다.

  SmoothTrail        바퀴 기구학 + IMU 자이로 dead reckoning 에 SLAM 을 2차 complementary filter 로 섞는다.
                     map 포즈 (x, y, ψ) 를 v = R(ψ)(T_fwd ω + b) , ψ̇ = ω_z(자이로) 로 100 Hz 적분.
                     SLAM 표본 (stamp t_s) 이 오면 잔차 Δ = SLAM(base_footprint) − [이력(t_s) + t_s 뒤에 이미 적용한 보정].
                       · 위치·헤딩: Δ 를 미적용 보정(pending)으로 두고, 바퀴 표본마다 (1 − e^{−dt/τ}) 만큼씩 적용 → 계단 없음
                       · 몸체 속도 편향 b (바퀴가 SLAM 보다 빠르게/느리게 나오는 몫, 몸체 좌표):
                         b ← b + R(ψ)ᵀ Δ_xy · Δt_slam / τ²   (위치 이득 1/τ 와 짝: 연속시간 k1 = 1/τ, k2 = 1/τ², 감쇠비 0.5)
                     |b| ≤ b_max 로 자른다.

왜: state_estimator 는 SLAM 포즈가 올 때마다 위치를 고쳐서, 100 Hz 표본 사이에 바퀴 속도로 설명 안 되는 이동이
    최대 46 mm 생긴다 (bag 191922). 그 경로를 두 번 미분한 몸체 횡가속(1~5 Hz)은 IMU 와 상관 −0.22·+0.08 로
    실제 운동이 아니다 (analysis/torque_mpc_real/2026-11-26_trail/RESULTS_KO.md).
한계: 바퀴가 미끄러진 몫은 dead reckoning 이 못 보고 τ 동안 SLAM 쪽으로 끌려간다 — 표시용.
bag 재생 (τ 0.5 s, 같은 코드, analysis/torque_mpc_real/2026-11-26_trail/validate.py):
    191922 꺾은선 1.5 m/s — 표본 간 속도 밖 이동 최대 estimator 46 → 1.5 mm, 0.5 s 횡흔들림 RMS 1.17 → 0.50 cm,
      SLAM(base_footprint) 과 거리 RMS 2.9 cm (estimator 2.3 cm, 평균 편향 0, 시간 밀림 없음 — 미끄러짐 구간의 차)
    191432·190718 지그재그 — 속도 밖 이동 최대 16·18 → 0.6·0.4 mm, SLAM 과 거리 RMS 1.1·0.9 cm
ROS 에 의존하지 않는다 (bag 재생 검증·pytest 에서 같은 코드).
"""
import math
from collections import deque

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def slam_to_base(x, y, psi, rx, ry):
    c, s = math.cos(psi), math.sin(psi)
    return x - (c * rx - s * ry), y - (s * rx + c * ry), psi


def mecanum_fwd(wheel_radius, wheel_base, robot_width):
    """바퀴 각속도 [FL, FR, RL, RR] → 몸체 (v_x, v_y, ω_z).  kinematics.mecanum_matrices 와 같은 T_inv 의 pinv."""
    k = (wheel_base + robot_width) / 2.0
    T_inv = (1.0 / wheel_radius) * np.array([[1.0, -1.0, -k], [1.0, 1.0, k], [1.0, 1.0, -k], [1.0, -1.0, k]])
    return np.linalg.pinv(T_inv)


class SmoothTrail:
    def __init__(self, T_fwd, r_offset, tau=0.5, use_bias=True, b_max=0.2, history_s=1.0, max_dt=0.1):
        self.T_fwd = np.asarray(T_fwd, float)
        self.rx, self.ry = float(r_offset[0]), float(r_offset[1])
        self.tau = float(tau)
        self.use_bias = bool(use_bias)
        self.b_max = float(b_max)
        self.history_s = float(history_s)
        self.max_dt = float(max_dt)          # 바퀴 표본 사이가 이보다 길면 적분하지 않는다 (끊김)
        self.hist = deque()                  # (t, x, y, ψ, Cx, Cy, Cψ)  C = 그때까지 적용한 보정 누적
        self.reset()

    def reset(self):
        self.t = None
        self.x = self.y = self.psi = 0.0
        self.pend = np.zeros(3)              # 아직 적용 안 한 위치·헤딩 보정
        self.C = np.zeros(3)                 # 적용한 보정 누적 (잔차 계산에서 t_s 뒤 적용분을 빼려고)
        self.b = np.zeros(2)                 # 몸체 속도 편향 [m/s]
        self.init = False
        self.last_slam_t = None
        self.gyro = None
        self.hist.clear()

    def on_gyro(self, wz):
        self.gyro = float(wz)

    def on_wheels(self, t, w):
        """바퀴 각속도 [rad/s] 4 개. 반환: 지금 map 포즈 (x, y, ψ) 또는 SLAM 을 아직 못 받았으면 None."""
        if self.t is not None and 0.0 < t - self.t <= self.max_dt:
            dt = t - self.t
            vx, vy, wz_w = self.T_fwd @ np.asarray(w, float)
            vx, vy = vx + self.b[0], vy + self.b[1]
            wz = self.gyro if self.gyro is not None else wz_w
            pm = self.psi + 0.5 * wz * dt                    # 중점 헤딩으로 회전
            c, s = math.cos(pm), math.sin(pm)
            self.x += (c * vx - s * vy) * dt
            self.y += (s * vx + c * vy) * dt
            self.psi += wz * dt
            if self.init and self.tau > 0:
                step = (1.0 - math.exp(-dt / self.tau)) * self.pend
                self.pend -= step
                self.x, self.y, self.psi = self.x + step[0], self.y + step[1], self.psi + step[2]
                self.C += step
        if self.t is None or t > self.t:
            self.t = t
            self.hist.append((t, self.x, self.y, self.psi, *self.C))
            while self.hist and self.hist[0][0] < t - self.history_s:
                self.hist.popleft()
        return self.pose() if self.init else None

    def _hist_at(self, t):
        h = self.hist
        if t <= h[0][0]:
            return np.array(h[0][1:])
        if t >= h[-1][0]:
            return np.array(h[-1][1:])
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if h[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = np.array(h[lo]), np.array(h[hi])
        u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return (a + u * (b - a))[1:]

    def on_slam(self, t, x, y, psi):
        """/tracked_pose (tracking_frame 점) 한 표본. t = header stamp (측정 시각)."""
        if not self.hist:
            return
        bx, by, bp = slam_to_base(x, y, psi, self.rx, self.ry)
        if not self.init:                    # 첫 표본: stamp 시각 SLAM 포즈 + 그 뒤 바퀴로 간 만큼 (SLAM 은 ~110 ms 늦게 온다)
            hs = self._hist_at(t)
            dpsi = wrap(bp - hs[2])
            c, s = math.cos(dpsi), math.sin(dpsi)
            ex, ey = self.x - hs[0], self.y - hs[1]
            self.x, self.y, self.psi = bx + c * ex - s * ey, by + s * ex + c * ey, self.psi + dpsi
            self.hist.clear()
            self.hist.append((self.t, self.x, self.y, self.psi, *self.C))
            self.init, self.last_slam_t = True, t
            return
        hs = self._hist_at(t)
        est = hs[:3] + (self.C - hs[3:])     # stamp 시각 추정 + 그 뒤 적용한 보정
        d = np.array([bx - est[0], by - est[1], wrap(bp - est[2])])
        self.pend = d
        if self.use_bias and self.tau > 0:
            dts = min(max(t - self.last_slam_t, 0.0), 0.5)
            c, s = math.cos(est[2]), math.sin(est[2])
            self.b += np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]]) * dts / self.tau ** 2
            n = np.hypot(*self.b)
            if n > self.b_max:
                self.b *= self.b_max / n
        self.last_slam_t = t

    def pose(self):
        return self.x, self.y, wrap(self.psi)
