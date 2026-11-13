#!/usr/bin/env python3
"""토크 입력 메카넘 차체 모델 — 『메카넘 토크 MPC 1판 설계』 식-1~6.

시뮬 플랜트(sim_plant_torque_node)와 이후 torque_mpc_node 가 같은 식을 쓴다
(1판 통과시험은 "플랜트 = 모델"에서 시작한다). ROS 에 의존하지 않는다.

상태(관성 좌표, map):  X = [x, y, θ],  Ẋ = [ẋ, ẏ, θ̇]
입력:                  τ = [τ_FL, τ_FR, τ_RL, τ_RR]  바퀴(모터 출력축) 순토크 [N·m]

    Ẍ = M⁻¹ ( R(θ)·(T⁻¹)ᵀ·τ  +  [μ·ω_z·S·Ẋ_xy ; 0] )                  (식-6)

    T⁻¹ = (1/r)·[[1,−1,−k],[1,1,k],[1,1,−k],[1,−1,k]]   바퀴속도 = T⁻¹·v_body  (드라이버 twist_to_wheels)
    (T⁻¹)ᵀ                                             바퀴 토크 → 몸체 [F_x, F_y, M_z]  (가상일, 식-2)
    S = [[0,−1],[1,0]],  μ = 4J_w/r²
    M = diag(m+μ, m+μ, I_z+μk²)   (use_wheel_inertia)
      = diag(m,   m,   I_z)        (끔: 10/29 방향 정리의 "휠 관성 무시", μ = 0 → w = 0)

바퀴 관성 항의 부호는 몸체 좌표식 m_eff·v̇ = F − m·ω_z·S·v 를 관성 좌표로 옮겨 확인했다:
Ẍ_xy = R v̇ + ω_z S Ẋ_xy 이므로 m_eff·Ẍ_xy = R F + (m_eff − m)·ω_z·S·Ẋ_xy.
"""
from dataclasses import dataclass
import math

import numpy as np

from amr_mpc.core import config

PLANT_FILE = "torque_plant.yaml"
S2 = np.array([[0.0, -1.0], [1.0, 0.0]])
NULL_DIR = np.array([1.0, 1.0, -1.0, -1.0]) / 2.0   # (T⁻¹)ᵀ·n = 0 — 앞뒤가 서로 미는 조합 (1판 식-2)


@dataclass
class TorquePlant:
    r: float
    k: float
    mass: float
    inertia_z: float
    J_w: float
    use_wheel_inertia: bool
    tau_rated: float
    tau_peak: float
    damping_kd: float
    fric_coulomb: float
    fric_viscous: float
    fric_max: float
    mu_hat: float
    c_s: float
    g: float

    @classmethod
    def from_yaml(cls, filename=PLANT_FILE, **override):
        d = config.load(filename)
        geo, body, wheel, motor = d["geometry"], d["body"], d["wheel"], d["motor"]
        fric, surf, model = d["friction"], d["surface"], d["model"]
        p = dict(
            r=float(geo["wheel_radius_m"]),
            k=(float(geo["wheel_base_m"]) + float(geo["robot_width_m"])) / 2.0,
            mass=float(body["mass_kg"]) + float(body["payload_kg"]),
            inertia_z=float(body["inertia_z_kgm2"]),
            J_w=float(wheel["inertia_J_w_kgm2"]),
            use_wheel_inertia=bool(model["use_wheel_inertia"]),
            tau_rated=float(motor["tau_rated_Nm"]),
            tau_peak=float(motor["tau_peak_Nm"]),
            damping_kd=float(motor["damping_kd"]),
            fric_coulomb=float(fric["coulomb_Nm"]),
            fric_viscous=float(fric["viscous_Nms_per_rad"]),
            fric_max=float(fric["tau_fric_max_Nm"]),
            mu_hat=float(surf["mu_hat"]),
            c_s=float(surf["c_s"]),
            g=float(model["gravity"]),
        )
        p.update(override)
        return cls(**p)

    # ── 기구학 ──────────────────────────────────────────────
    @property
    def Tinv(self):
        """바퀴속도 = Tinv @ v_body,  (4×3). 행 순서 FL, FR, RL, RR."""
        k = self.k
        return np.array([[1.0, -1.0, -k],
                         [1.0, 1.0, k],
                         [1.0, 1.0, -k],
                         [1.0, -1.0, k]]) / self.r

    @property
    def mu(self):
        """바퀴 반영 질량 μ = 4J_w/r² (스위치가 꺼져 있으면 0)."""
        return 4.0 * self.J_w / self.r ** 2 if self.use_wheel_inertia else 0.0

    @property
    def M_eff(self):
        mu = self.mu
        return np.diag([self.mass + mu, self.mass + mu, self.inertia_z + mu * self.k ** 2])

    @staticmethod
    def R(theta):
        c, s = math.cos(theta), math.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def G(self, theta):
        """Ẍ = G(θ)·τ + w  의 입력 행렬 (3×4), 식-6."""
        return np.linalg.solve(self.M_eff, self.R(theta) @ self.Tinv.T)

    def w(self, Xdot):
        """바퀴 반영 관성 항 (3,). 스위치가 꺼지면 0."""
        mu = self.mu
        if mu == 0.0:
            return np.zeros(3)
        f = np.zeros(3)
        f[:2] = mu * Xdot[2] * (S2 @ Xdot[:2])
        return np.linalg.solve(self.M_eff, f)

    def accel(self, X, Xdot, tau):
        return self.G(X[2]) @ np.asarray(tau, float) + self.w(Xdot)

    def body_velocity(self, X, Xdot):
        """관성 좌표 속도 → 몸체 [v_x, v_y, ω_z]."""
        return self.R(X[2]).T @ Xdot

    def wheel_speeds(self, X, Xdot):
        """무슬립 바퀴 각속도 [rad/s] (4,)."""
        return self.Tinv @ self.body_velocity(X, Xdot)

    # ── 토크 상한 (1판 식-21) ───────────────────────────────
    def tau_friction_limit(self):
        """τ_μ = (r/√2)·μ̂·(m g/4)·c_s  — 비교군 C 의 마찰 한계."""
        return self.r / math.sqrt(2.0) * self.mu_hat * self.mass * self.g / 4.0 * self.c_s

    def tau_motor_limit(self):
        """τ_lim − τ_fric,max  — 비교군 B 는 이것만 둔다."""
        return self.tau_rated - self.fric_max

    # ── 적분 ────────────────────────────────────────────────
    def step(self, X, Xdot, tau, dt, fric=None):
        """τ 를 dt 동안 일정하게 두고 RK4 로 한 스텝. 반환 (X, Xdot).

        fric: 구동계 마찰 (DrivetrainFriction, 시뮬 플랜트 전용). 주면 바퀴에 전달되는 토크가 τ − τ_fric(ω) 가 된다.
              MPC 예측 모델은 마찰을 모른다 (torque_output 이 미리 더해 지운다고 본다).
        """
        tau = np.asarray(tau, float)

        def f(s):
            t = tau if fric is None else tau - fric(self.wheel_speeds(s[:3], s[3:]))
            return np.concatenate([s[3:], self.accel(s[:3], s[3:], t)])

        s = np.concatenate([X, Xdot])
        k1 = f(s)
        k2 = f(s + 0.5 * dt * k1)
        k3 = f(s + 0.5 * dt * k2)
        k4 = f(s + dt * k3)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        s[2] = math.atan2(math.sin(s[2]), math.cos(s[2]))
        return s[:3], s[3:]


if __name__ == "__main__":
    for uwi in (False, True):
        p = TorquePlant.from_yaml(use_wheel_inertia=uwi)
        print(f"use_wheel_inertia={uwi}:  m={p.mass} kg  μ={p.mu:.3f} kg  M_eff diag={np.round(np.diag(p.M_eff), 4)}")
    p = TorquePlant.from_yaml()
    print(f"τ_μ (C 상한) = {p.tau_friction_limit():.4f} N·m,  τ_lim − τ_fric,max (B 상한) = {p.tau_motor_limit():.3f} N·m")


# ── 슬립 모델 (5단계, 2026-11-11) ──────────────────────────────────────────────────
# 위 TorquePlant 는 "바퀴가 절대 안 미끄러진다" 고 보는 모델이다 (MPC 예측 모델과 같다).
# SlipPlant 는 시뮬 플랜트 전용으로, 바퀴 각속도를 따로 상태로 두고 바닥 마찰력을 마찰 한계로 자른다.
#
# 메카넘 바퀴는 롤러가 45° 로 자유롭게 돌아서, 바닥에 힘을 전할 수 있는 방향이 바퀴마다 하나로 정해져 있다
# (롤러 축에 수직인 방향 u_i). 1판 식-2 의 (T⁻¹)ᵀ 가 바로 그 방향이다:
#   u_i = (1, s_i)/√2 (몸체 좌표),  s = [−1, +1, +1, −1],   그 힘의 회전 팔 = c_i k/√2,  c = [−1, +1, −1, +1]
#   미끄러지지 않으면 F_i = √2 τ_i / r  → 몸체 힘 Σ F_i u_i = (T⁻¹)ᵀ τ  (기존 모델과 같아진다)
# 상태:  X, Ẋ (map),  ω_i (바퀴 4개)
#   미끄럼 속도 (u_i 방향):   Δ_i = (r/√2) (ω_i − (T⁻¹ v_body)_i)
#   바닥 마찰력:              F_i = μ N tanh(Δ_i / v_ε),   N = m g / 4   (하중 이동 없음 — 1판 식-21 과 같은 가정)
#   바퀴:                     J_w ω̇_i = τ_i − (r/√2) F_i
#   몸체 (관성 좌표):          m Ẍ_xy = R(θ) Σ F_i u_i,    I_z θ̈ = Σ (c_i k/√2) F_i
# 단순화: 롤러 축 방향 마찰(롤러 베어링), 하중 이동, 바퀴별 μ 차이는 넣지 않았다. tanh 는 Coulomb 마찰을
# 부드럽게 이은 것이라 v_ε 보다 작은 미끄럼은 "거의 안 미끄러짐" 이다 (v_ε 0.02 m/s 기본).
# 바퀴 관성은 J_w 그대로 쓴다 (torque_plant.yaml, 2026-11-12 팀원 벤치 8.9e-4). 이 스위치와 무관하게 몸체 질량은 m 이다.
# ── 구동계 마찰 (R2 리뷰, 2026-11-12) ────────────────────────────────────────────────
# 감속기·베어링이 먹는 토크. 모터 출력축 토크 τ 중 바퀴에 전달되는 것은 τ − τ_fric(ω).
#   τ_fric(ω) = coulomb·tanh(ω/ω_s) + viscous·ω     (팀원 벤치: amr/notes_amr/CAN/BENCH_RESULTS_KO.md 의 τc·bv)
# torque_output 의 보상식과 모양은 같지만 ω_s 가 훨씬 작다 (플랜트 0.05 rad/s ≈ 바퀴 2 mm/s, 보상 0.5 rad/s).
# 진짜 쿨롱 마찰에 가깝게 해서, 정지 근처에서는 보상과 실제가 어긋나는 실차 상황을 흉내 낸다.
# 정지 마찰(stiction)은 따로 두지 않았다 — 힘이 τc 보다 작으면 ω_s 수준으로 아주 천천히 기어간다.
class DrivetrainFriction:
    def __init__(self, coulomb, viscous, smooth=0.05):
        self.coulomb, self.viscous, self.smooth = float(coulomb), float(viscous), max(1e-6, float(smooth))

    def __call__(self, omega):
        omega = np.asarray(omega, float)
        return self.coulomb * np.tanh(omega / self.smooth) + self.viscous * omega

    @property
    def stiffness(self):
        """dτ_fric/dω 의 최댓값 [N·m·s/rad] (ω = 0). 적분 간격을 정할 때 쓴다."""
        return self.coulomb / self.smooth + self.viscous

    def active(self):
        return self.coulomb != 0.0 or self.viscous != 0.0

    def stable_dt(self, plant, slip=False):
        """RK4 가 안정한 적분 간격 대략값. 슬립 모델은 바퀴 관성(J_w) 하나가, 무슬립은 몸체 전체가 마찰에 붙는다."""
        d = self.stiffness
        if d <= 0.0:
            return math.inf
        if slip:
            lam = d / plant.J_w
        else:
            M = np.diag(plant.M_eff)
            lam = d / plant.r ** 2 * max(4.0 / M[0], 4.0 * plant.k ** 2 / M[2])
        return 2.0 / lam                      # RK4 실축 안정 한계 2.78/λ 에 여유


S_SIGN = np.array([-1.0, 1.0, 1.0, -1.0])
C_SIGN = np.array([-1.0, 1.0, -1.0, 1.0])


class SlipPlant:
    def __init__(self, plant, mu, v_eps=0.02, fric=None):
        self.p = plant
        self.fric = fric                     # 구동계 마찰 (DrivetrainFriction) 또는 None
        self.mu = float(mu)
        self.v_eps = float(v_eps)
        self.N = plant.mass * plant.g / 4.0
        self.Fmax = self.mu * self.N

    def contact_speed(self, X, Xd):
        """무슬립이라면 바퀴가 돌아야 할 각속도 (T⁻¹ v_body)."""
        return self.p.Tinv @ self.p.body_velocity(X, Xd)

    def forces(self, X, Xd, w):
        q = self.p.r / math.sqrt(2.0)
        slip = q * (np.asarray(w, float) - self.contact_speed(X, Xd))
        return self.Fmax * np.tanh(slip / self.v_eps)

    def deriv(self, s, tau):
        """s = [X(3), Ẋ(3), ω(4)]."""
        p = self.p
        X, Xd, w = s[:3], s[3:6], s[6:]
        F = self.forces(X, Xd, w)
        Fb = np.array([np.sum(F) / math.sqrt(2.0), np.sum(F * S_SIGN) / math.sqrt(2.0)])
        Mz = np.sum(F * C_SIGN) * p.k / math.sqrt(2.0)
        c, sn = math.cos(X[2]), math.sin(X[2])
        acc = np.array([(c * Fb[0] - sn * Fb[1]) / p.mass, (sn * Fb[0] + c * Fb[1]) / p.mass, Mz / p.inertia_z])
        tau = np.asarray(tau, float)
        if self.fric is not None:
            tau = tau - self.fric(w)
        wd = (tau - p.r / math.sqrt(2.0) * F) / p.J_w
        return np.concatenate([Xd, acc, wd])

    def step(self, X, Xd, w, tau, dt):
        s = np.concatenate([X, Xd, w])
        k1 = self.deriv(s, tau)
        k2 = self.deriv(s + 0.5 * dt * k1, tau)
        k3 = self.deriv(s + 0.5 * dt * k2, tau)
        k4 = self.deriv(s + dt * k3, tau)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        s[2] = math.atan2(math.sin(s[2]), math.cos(s[2]))
        return s[:3], s[3:6], s[6:]

    def stable_dt(self):
        """RK4 가 안정한 적분 간격의 대략값 (바퀴–바닥 마찰 시정수의 절반)."""
        tc = self.p.J_w * 2.0 * self.v_eps / (self.p.r ** 2 * max(self.Fmax, 1e-9))
        dt = 0.5 * tc
        if self.fric is not None:                # 두 마찰이 같은 바퀴에 붙으므로 강성이 더해진다 → 간격의 조화합
            dt = 1.0 / (1.0 / dt + 1.0 / self.fric.stable_dt(self.p, slip=True))
        return dt
