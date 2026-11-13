#!/usr/bin/env python3
"""바퀴별 PI 속도 루프 (비교군 A 의 속도 → 토크 변환).  ROS 에 의존하지 않는다.

    ω_ref = T⁻¹·[v_x, v_y, ω_z]          드라이버 twist_to_wheels 와 같은 역기구학 (torque/model.py TorquePlant.Tinv)
    e_i   = ω_ref,i − ω_i                 ω_i = /wheel_states.velocity
    τ_i   = K_p·e_i + K_i·∫e_i dt          바퀴 4개가 서로 독립 (RoboClaw 가 내부에서 하던 속도 PID 를 우리 쪽으로 옮긴 것)

제약 — 비교군 B(torque_mpc_node, limit_mode motor_only)와 같은 것만 둔다 (공정성)
    |τ_i| ≤ τ̄              τ̄ = τ_lim − τ_fric,max  (B 의 상한, torque_plant.yaml)
    |τ_i(k) − τ_i(k−1)| ≤ Δτ̄·(dt/Ts)    Δτ̄ = τ̄ × rate_fraction  (B 의 변화율 제약, torque_mpc.yaml)

적분 와인드업 방지 — 조건부 적분 (clamping)
    출력이 상한(크기 또는 변화율)에 잘렸고 오차가 더 잘리는 쪽으로 밀 때는 그 바퀴의 적분을 멈춘다.
    덧붙여 적분항 K_i·∫e 자체도 ±τ̄ 안에 둔다. 리셋(무장 해제·끊김)하면 적분과 직전 출력 둘 다 0.
"""
import math

import numpy as np


class WheelPI:
    def __init__(self, kp, ki, tau_max, dtau_rate=math.inf, dt_max=0.03):
        """kp [N·m/(rad/s)], ki [N·m/rad], tau_max [N·m], dtau_rate [N·m/s] (= Δτ̄/Ts), dt_max [s]."""
        if kp < 0 or ki < 0 or tau_max <= 0 or dtau_rate <= 0:
            raise ValueError(f"kp, ki ≥ 0, tau_max·dtau_rate > 0 이어야 한다 (받은 값 {kp}, {ki}, {tau_max}, {dtau_rate})")
        self.kp, self.ki = float(kp), float(ki)
        self.tau_max, self.dtau_rate, self.dt_max = float(tau_max), float(dtau_rate), float(dt_max)
        self.reset()

    def reset(self):
        self.integ = np.zeros(4)        # ∫e dt [rad]
        self.u_prev = np.zeros(4)       # 직전 출력 [N·m] (변화율 제약의 기준. DAMPING 뒤에는 0 에서 출발 — MPC reset 과 같다)
        self.saturated = np.zeros(4, bool)

    def step(self, omega_ref, omega, dt):
        """한 주기. 반환 τ (4,).  dt 는 [0, dt_max] 로 자른다 (메시지가 몰리거나 늦어도 적분이 튀지 않게)."""
        dt = float(np.clip(dt, 0.0, self.dt_max))
        e = np.asarray(omega_ref, float) - np.asarray(omega, float)
        integ_new = self.integ + e * dt
        u_raw = self.kp * e + self.ki * integ_new
        du = self.dtau_rate * dt
        lo = np.maximum(-self.tau_max, self.u_prev - du)
        hi = np.minimum(self.tau_max, self.u_prev + du)
        u = np.minimum(np.maximum(u_raw, lo), hi)
        sat_hi, sat_lo = u_raw > hi, u_raw < lo
        freeze = (sat_hi & (e > 0.0)) | (sat_lo & (e < 0.0))
        self.integ = np.where(freeze, self.integ, integ_new)
        if self.ki > 0.0:
            lim = self.tau_max / self.ki
            self.integ = np.clip(self.integ, -lim, lim)
        self.saturated = sat_hi | sat_lo
        self.u_prev = u
        return u.copy()


def wheel_targets(Tinv, v_body):
    """몸체 [v_x, v_y, ω_z] → 바퀴 목표 각속도 (4,), 행 순서 FL, FR, RL, RR."""
    return np.asarray(Tinv, float) @ np.asarray(v_body, float)
