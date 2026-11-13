#!/usr/bin/env python3
"""평면 상태 추정기 (구조도 v3 state_estimator 의 계산부). ROS 에 의존하지 않는다.

  입력   IMU (몸체 좌표 가속도 a_x, a_y, 자이로 ω_z, 약 200 Hz)  +  SLAM 포즈 (map 좌표 x, y, yaw, 약 10 Hz, 늦게 도착)
  출력   x̂ = [x, y, ψ] (map),  몸체 속도 [v_x, v_y, ω_z]

결정 (amr/notes_amr/torque_mpc_todo.md "상태 추정", 2026-11-06)
  · 스캔 매칭 포즈가 들어올 때만 보정하고, 그 사이는 IMU 가속도를 직접 적분한다.
  · 속도는 포즈 미분으로 덮어쓰지 않는다. 위치 잔차 e = p_slam − p̂ 로  p += K_p e,  v += K_v e,  b += K_b e.
  · yaw 는 자이로 적분 + SLAM yaw 잔차 보정(상보 필터), 자이로 바이어스 추정. ω_z 출력은 자이로 − 바이어스.
  · 게인은 고정값 = 정상상태 칼만 게인 (잡음 설정에서 계산, steady_gain).

어느 점을 추정하나 — **IMU 가 놓인 점** (2026-11-11 bag 분석으로 정함)
  · Cartographer 의 /tracked_pose 는 로봇 중심이 아니라 tracking_frame(camera_gyro_frame = IMU 위치)의 포즈다.
    제자리 회전(0.4 rad/s) 구간에서 중심이라고 보고 추정하면 옆 속도가 ω·r = 45 mm/s 만큼 일정하게 틀렸다.
  · 가속도계가 재는 것도 그 점의 가속도다. 그래서 안에서는 IMU 점의 [p, v] 를 추정하고(회전에 따른 접선·원심
    보상이 필요 없다), 내보낼 때 로봇 중심(base_footprint)으로 옮긴다:
        p_base = p − R(ψ)·r,    v_base(몸체) = R(ψ)ᵀ v − ω·[−r_y, r_x]          (r = IMU 위치, 몸체 좌표)
  · 포즈의 기준점(pose_offset)이 IMU 와 다르면 측정값을 IMU 점으로 옮겨서 쓴다.

예측 (IMU 한 표본마다, dt = stamp 차)
    ω = ω_gyro − b_g
    a_map = R(ψ + ω dt/2)·(a_imu − b_a)
    p += v dt + ½ a_map dt²,   v += a_map dt,   ψ += ω dt
  b_a 는 몸체 좌표 바이어스다 (기울어진 장착으로 새어 드는 중력 성분 포함 — 실측 약 0.15 m/s²).

보정 (포즈 한 개마다) — 포즈는 stamp 가 과거다 (스캔 시각. 실차 Cartographer 처리 지연 약 0.13 s)
    그 시각의 추정값을 이력에서 찾아 잔차를 구하고, 보정량을 지금까지 전파해서 더한다:
      δp(D) = δp + D δv − ½D² δb,   δv(D) = δv − D δb          (D = 지금 − 포즈 시각)
    이력에도 같은 보정을 넣어 다음 포즈의 잔차가 이미 반영된 보정을 다시 세지 않게 한다.

포즈 튐: 잔차가 jump_xy / jump_yaw 를 넘으면 보정에 쓰지 않고 센다. 연속 jump_reset_count 번이면 SLAM 을 따라 다시 초기화.

포즈 끊김 (R3 리뷰 2026-11-13): SLAM 이 멈춰도 IMU 는 계속 오므로 적분만으로 출력이 이어진다. 그러면 위치가 t² 로
  흘러가는데 MPC 는 그 추정값을 따라가므로 추종 오차 감시(fault_monitor)에도 안 걸린다. 마지막 보정 뒤 IMU 시간이
  max_dead_reckoning 초를 넘으면 ready_to_publish() 가 False → 노드가 출력을 멈추고, MPC state_timeout·fault_monitor 가 세운다.
"""
from collections import deque
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def steady_gain(F, H, Q, R, iters=2000):
    """정상상태 칼만 게인 K (예측 → 보정 순서의 보정 게인). x ← x + K (z − H x)."""
    n = F.shape[0]
    P = np.eye(n)
    K = np.zeros((n, H.shape[0]))
    for _ in range(iters):
        P = F @ P @ F.T + Q
        K_new = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
        P = (np.eye(n) - K_new @ H) @ P
        if np.max(np.abs(K_new - K)) < 1e-12:
            K = K_new
            break
        K = K_new
    return K


def design_gains(T, accel_noise_density, accel_bias_walk, pose_noise_xy,
                 gyro_noise_density, gyro_bias_walk, pose_noise_yaw):
    """포즈 주기 T 에서의 고정 게인. 반환 (K_xy[3] = [K_p, K_v, K_b],  K_yaw[2] = [K_ψ, K_bg]).

    축 하나 모델  x = [p, v, b]:  p' = p + T v + ½T²(a_m − b),  v' = v + T(a_m − b),  b' = b
    yaw 모델      x = [ψ, b_g]:   ψ' = ψ + T(ω_m − b_g),  b_g' = b_g
    K_b, K_bg 는 음수로 나온다 (추정이 뒤처지면 바이어스를 너무 크게 본 것).
    """
    qa, qb = accel_noise_density ** 2, accel_bias_walk ** 2
    F = np.array([[1.0, T, -0.5 * T * T], [0.0, 1.0, -T], [0.0, 0.0, 1.0]])
    Q = np.array([[qa * T ** 3 / 3.0, qa * T * T / 2.0, 0.0],
                  [qa * T * T / 2.0, qa * T, 0.0],
                  [0.0, 0.0, qb * T]])
    K_xy = steady_gain(F, np.array([[1.0, 0.0, 0.0]]), Q, np.array([[pose_noise_xy ** 2]]))[:, 0]
    qg, qbg = gyro_noise_density ** 2, gyro_bias_walk ** 2
    Fy = np.array([[1.0, -T], [0.0, 1.0]])
    Qy = np.array([[qg * T, 0.0], [0.0, qbg * T]])
    K_yaw = steady_gain(Fy, np.array([[1.0, 0.0]]), Qy, np.array([[pose_noise_yaw ** 2]]))[:, 0]
    return K_xy, K_yaw


class PlanarStateEstimator:
    def __init__(self, cfg, pose_period=0.1):
        g = cfg.get
        self.K_xy, self.K_yaw = design_gains(
            pose_period, float(g("accel_noise_density", 0.07)), float(g("accel_bias_walk", 0.02)),
            float(g("pose_noise_xy", 0.003)), float(g("gyro_noise_density", 0.0064)),
            float(g("gyro_bias_walk", 0.0005)), math.radians(float(g("pose_noise_yaw_deg", 0.2))))
        if g("gains_xy"):
            self.K_xy = np.array(g("gains_xy"), float)
        if g("gains_yaw"):
            self.K_yaw = np.array(g("gains_yaw"), float)
        if self.K_xy.shape != (3,) or self.K_yaw.shape != (2,):
            raise ValueError("gains_xy 는 3개 [K_p, K_v, K_b], gains_yaw 는 2개 [K_psi, K_bg]")
        self.r = np.array([float(g("imu_offset_x", 0.0)), float(g("imu_offset_y", 0.0))])     # IMU 위치 (몸체)
        px, py = g("pose_offset_x"), g("pose_offset_y")                                         # 포즈 기준점 (없으면 IMU 와 같다)
        self.r_pose = np.array([self.r[0] if px is None else float(px), self.r[1] if py is None else float(py)])
        self.omega_fc = float(g("omega_lpf_hz", 0.0))
        self.history_s = float(g("history_s", 0.6))
        self.max_pose_age = float(g("max_pose_age", 0.4))
        self.init_s = float(g("init_s", 1.0))
        self.init_max_accel_std = float(g("init_max_accel_std", 0.10))
        self.init_max_gyro_std = float(g("init_max_gyro_std", 0.02))
        self.jump_xy = float(g("jump_xy", 0.15))
        self.jump_yaw = math.radians(float(g("jump_yaw_deg", 15.0)))
        self.jump_reset_count = int(g("jump_reset_count", 5))
        self.max_dead_reckoning = float(g("max_dead_reckoning", 0.5))
        self.reset()

    def reset(self):
        self.p = np.zeros(2)            # IMU 점의 위치·속도 (map)
        self.v = np.zeros(2)
        self.psi = 0.0
        self.b_a = np.zeros(2)          # 몸체 좌표 가속도계 바이어스
        self.b_g = 0.0
        self.t = None                   # 마지막 IMU 시각
        self.omega = 0.0                # ω_gyro − b_g (마지막 표본)
        self.omega_out = 0.0            # 출력용 저역통과
        self.hist = deque()             # (t, px, py, vx, vy, psi)
        self.bias_ready = False
        self.pose_ready = False
        self._static = []               # 정지 초기화용 (t, ax, ay, wz)
        self._first_pose = None
        self.jump_count = 0             # 누적 포즈 튐 수
        self._jump_streak = 0
        self.last_residual = np.zeros(3)
        self.n_corrections = 0
        self.last_fix_t = None          # 마지막으로 포즈를 반영한 IMU 시각 (보정·초기화·재초기화)

    @property
    def ready(self):
        return self.bias_ready and self.pose_ready

    # ── 정지 초기화 ─────────────────────────────────────────────
    def _static_init(self, t, ax, ay, wz):
        self._static.append((t, ax, ay, wz))
        if t - self._static[0][0] < self.init_s:
            return
        s = np.array(self._static)
        if max(s[:, 1].std(), s[:, 2].std()) > self.init_max_accel_std or s[:, 3].std() > self.init_max_gyro_std:
            self._static = []           # 움직였다 — 다시 모은다
            return
        self.b_a = np.array([s[:, 1].mean(), s[:, 2].mean()])
        self.b_g = float(s[:, 3].mean())
        self.bias_ready = True
        self._static = []
        if self._first_pose is not None:
            self._init_pose(*self._first_pose)

    def _to_imu_point(self, x, y, yaw):
        """포즈 기준점의 위치 → IMU 점의 위치."""
        d = self.r - self.r_pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x + c * d[0] - s * d[1], y + s * d[0] + c * d[1]])

    def _init_pose(self, x, y, yaw):
        self.p = self._to_imu_point(x, y, yaw)
        self.v = np.zeros(2)
        self.psi = float(yaw)
        self.hist.clear()
        self.pose_ready = True
        self.last_fix_t = self.t

    # ── 예측 ────────────────────────────────────────────────────
    def imu(self, t, ax, ay, wz):
        """IMU 표본 하나. 준비가 끝났으면 True."""
        if not self.bias_ready:
            self.t = t
            self._static_init(t, ax, ay, wz)
            return False
        if not self.pose_ready or self.t is None:
            self.t = t
            return False
        dt = min(max(t - self.t, 0.0), 0.05)
        self.t = t
        if dt <= 0.0:
            return True
        w = wz - self.b_g
        self.omega = w
        if self.omega_fc > 0.0:
            self.omega_out += (1.0 - math.exp(-2.0 * math.pi * self.omega_fc * dt)) * (w - self.omega_out)
        else:
            self.omega_out = w
        acx = ax - self.b_a[0]
        acy = ay - self.b_a[1]
        pm = self.psi + 0.5 * w * dt
        c, s = math.cos(pm), math.sin(pm)
        a_map = np.array([c * acx - s * acy, s * acx + c * acy])
        self.p = self.p + self.v * dt + 0.5 * a_map * dt * dt
        self.v = self.v + a_map * dt
        self.psi += w * dt
        self.hist.append((t, self.p[0], self.p[1], self.v[0], self.v[1], self.psi))
        while self.hist and t - self.hist[0][0] > self.history_s:
            self.hist.popleft()
        return True

    # ── 보정 ────────────────────────────────────────────────────
    def _state_at(self, tm):
        """이력에서 tm 시각의 (p, psi). 이력보다 새 시각이면 최신 상태."""
        h = self.hist
        if not h or tm >= h[-1][0]:
            return self.p.copy(), self.psi
        if tm <= h[0][0]:
            return np.array(h[0][1:3]), h[0][5]
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= tm:
                lo = mid
            else:
                hi = mid
        a, b = h[lo], h[hi]
        u = (tm - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return np.array([a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])]), a[5] + u * (b[5] - a[5])

    def pose(self, tm, x, y, yaw):
        """SLAM 포즈 하나 (tm = 그 포즈의 시각). 반환: 'init' | 'used' | 'old' | 'jump' | 'reset'."""
        if not self.pose_ready:
            self._first_pose = (x, y, yaw)
            if self.bias_ready:
                self._init_pose(x, y, yaw)
            return "init"
        if self.t is not None and self.t - tm > self.max_pose_age:
            return "old"
        p_m, psi_m = self._state_at(tm)
        e = self._to_imu_point(x, y, yaw) - p_m
        e_psi = wrap(yaw - psi_m)
        self.last_residual = np.array([e[0], e[1], e_psi])
        if math.hypot(*e) > self.jump_xy or abs(e_psi) > self.jump_yaw:
            self.jump_count += 1
            self._jump_streak += 1
            if self._jump_streak >= self.jump_reset_count:
                v = self.v.copy()
                self._init_pose(x, y, yaw)      # SLAM 이 계속 다른 곳을 가리킨다 → 따라간다
                self.v = v
                self._jump_streak = 0
                return "reset"
            return "jump"
        self._jump_streak = 0
        Kp, Kv, Kb = self.K_xy
        dp, dv, db = Kp * e, Kv * e, Kb * e                       # map 좌표
        dpsi, dbg = self.K_yaw[0] * e_psi, self.K_yaw[1] * e_psi
        # 이력에 반영 (tm 이후 항목)
        if self.hist:
            h = self.hist
            for i in range(len(h) - 1, -1, -1):
                ti = h[i][0]
                if ti < tm:
                    break
                D = ti - tm
                h[i] = (ti,
                        h[i][1] + dp[0] + D * dv[0] - 0.5 * D * D * db[0],
                        h[i][2] + dp[1] + D * dv[1] - 0.5 * D * D * db[1],
                        h[i][3] + dv[0] - D * db[0],
                        h[i][4] + dv[1] - D * db[1],
                        h[i][5] + dpsi - D * dbg)
        D = max(0.0, (self.t if self.t is not None else tm) - tm)
        self.p = self.p + dp + D * dv - 0.5 * D * D * db
        self.v = self.v + dv - D * db
        self.psi += dpsi - D * dbg
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.b_a = self.b_a + np.array([c * db[0] + s * db[1], -s * db[0] + c * db[1]])   # map → 몸체
        self.b_g += dbg
        self.n_corrections += 1
        self.last_fix_t = self.t
        return "used"

    # ── 출력 ────────────────────────────────────────────────────
    def dead_reckoning_age(self):
        """마지막으로 포즈를 반영한 뒤 IMU 적분만으로 흐른 시간 [s]. 아직 반영한 적이 없으면 0."""
        if self.t is None or self.last_fix_t is None:
            return 0.0
        return max(0.0, self.t - self.last_fix_t)

    def ready_to_publish(self):
        """준비가 끝났고 SLAM 포즈가 끊긴 지 max_dead_reckoning 초 안이다 (≤ 0 이면 검사 안 함)."""
        return self.ready and (self.max_dead_reckoning <= 0.0 or self.dead_reckoning_age() <= self.max_dead_reckoning)

    def state(self):
        """로봇 중심(base_footprint)의 (t, x, y, ψ, v_x, v_y, ω_z) — 포즈는 map, 속도는 몸체 좌표."""
        c, s = math.cos(self.psi), math.sin(self.psi)
        rx, ry = self.r
        w = self.omega_out
        return (self.t, float(self.p[0] - (c * rx - s * ry)), float(self.p[1] - (s * rx + c * ry)), wrap(self.psi),
                float(c * self.v[0] + s * self.v[1] + w * ry), float(-s * self.v[0] + c * self.v[1] - w * rx), float(w))

    def imu_point_state(self):
        """IMU 점의 (x, y, v_x_map, v_y_map) — 디버그·시험용."""
        return float(self.p[0]), float(self.p[1]), float(self.v[0]), float(self.v[1])


if __name__ == "__main__":
    from amr_mpc.core import config
    cfg = config.section("estimator", "torque_estimator.yaml")
    est = PlanarStateEstimator(cfg)
    print(f"게인 (포즈 주기 0.1 s):  K_p {est.K_xy[0]:.3f}   K_v {est.K_xy[1]:.3f} 1/s   K_b {est.K_xy[2]:.3f} 1/s²   "
          f"K_psi {est.K_yaw[0]:.3f}   K_bg {est.K_yaw[1]:.4f} 1/s")
