#!/usr/bin/env python3
# 플래너 기준열(PlanarReferenceArray, dt=0.2 s × T=20) → MPC 시간축(Ts=0.04 s × Np=20) 보간.
# ---------------------------------------------------------------
# 메시지 규약 (DRA_MPPI/README):  references[i] 의 목표 시각은 header.stamp + (i+1)·dt.
#   → 샘플 시각 t_i = t0 + (i+1)·dt,  i = 0..T−1.   header.stamp 자체에는 샘플이 없다.
#
# 보간 규칙
#   t_0 ≤ t ≤ t_{T−1}  : 이웃 두 샘플의 선형 보간.  yaw 는 np.unwrap 한 뒤 보간 (±π 경계에서 튀지 않게).
#   t <  t_0            : 첫 샘플에서 그 샘플의 몸체속도로 뒤로 외삽 (상수 twist 의 정확 적분 = SE(2) 지수사상,
#                         원호는 정확히 되짚는다. 최대 dt 만큼).
#                         플래너가 10 Hz 로 내고 첫 샘플이 0.2 s 뒤이므로 "지금" 은 거의 항상 이 구간이다.
#   t >  t_{T−1}        : 끝 포즈 유지, 속도 0 (플래너가 멈춘 경우 안전 정지).
#
# ROS 타입에 의존하지 않는다 — pytest 로 단독 검사 (test/test_reference_horizon.py).
# ---------------------------------------------------------------
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ReferenceHorizon:
    def __init__(self, t0, dt, X, U):
        """t0: header.stamp [s], dt: 샘플 간격 [s], X: (T,3) [x,y,yaw], U: (T,3) 몸체속도 [vx,vy,wz]."""
        X = np.asarray(X, float).reshape(-1, 3); U = np.asarray(U, float).reshape(-1, 3)
        if len(X) == 0 or dt <= 0:
            raise ValueError("빈 기준열 또는 dt<=0")
        self.t0, self.dt, self.T = float(t0), float(dt), len(X)
        self.times = self.t0 + self.dt * np.arange(1, self.T + 1)
        self.X = X.copy(); self.X[:, 2] = np.unwrap(self.X[:, 2])
        self.U = U.copy()

    @property
    def t_start(self): return self.times[0]
    @property
    def t_end(self):   return self.times[-1]

    @staticmethod
    def from_rows(t0, dt, rows):
        """rows: [(x, y, yaw, vx, vy, yaw_rate), ...]  (README 의 X_star 매핑과 같은 순서)."""
        R = np.asarray(rows, float).reshape(-1, 6)
        return ReferenceHorizon(t0, dt, R[:, :3], R[:, 3:])

    def at(self, t):
        """시각 t 의 (Xr[3], ur[3]).  yaw 는 (−π, π] 로 랩."""
        if t <= self.times[0]:                                   # 뒤로 외삽: 상수 twist 의 정확 적분 (SE(2) 지수사상)
            X0, u0 = self.X[0], self.U[0]
            dtb = self.times[0] - t
            phi = -u0[2] * dtb                                   # 뒤로 가는 동안 도는 각
            if abs(phi) < 1e-9:
                V = np.eye(2)
            else:                                                # ∫_0^1 R(sφ) ds  — 원호를 정확히 되짚는다
                V = np.array([[math.sin(phi) / phi, -(1 - math.cos(phi)) / phi],
                              [(1 - math.cos(phi)) / phi, math.sin(phi) / phi]])
            c, s = math.cos(X0[2]), math.sin(X0[2])              # X0 의 몸체 프레임에서 본 뒤쪽 점의 변위
            d_body = V @ (-u0[:2] * dtb)                         # 상수 twist (−u) 를 dtb 동안 적분
            Xr = np.array([X0[0] + c * d_body[0] - s * d_body[1],
                           X0[1] + s * d_body[0] + c * d_body[1], X0[2] + phi])
            ur = u0.copy()
        elif t >= self.times[-1]:                                # 끝점 정지 유지
            Xr, ur = self.X[-1].copy(), np.zeros(3)
        else:
            i = int(np.searchsorted(self.times, t, side="right")) - 1
            w = (t - self.times[i]) / self.dt
            Xr = (1 - w) * self.X[i] + w * self.X[i + 1]
            ur = (1 - w) * self.U[i] + w * self.U[i + 1]
        Xr[2] = wrap(Xr[2])
        return Xr, ur

    def horizon(self, t, Ts, Np):
        """t, t+Ts, …, t+(Np−1)Ts 의 [(Xr, ur), …].  mpc.step 의 ref/Xr/ur 재료."""
        return [self.at(t + i * Ts) for i in range(Np)]

    @property
    def is_hold(self):
        """플래너의 정지 기준열인가 — 속도 전부 0 이고 포즈가 한 점.  dra_mppi_node._publish_stop 이
        stop_sequence[:, :3] = reference_state 로 만드는 꼴 그대로다 (goal completed, 경로 대기, odom stale 전부).
        실제 궤적은 20 행이 한 점에 겹치는 일이 없으므로 플래그 없이 판별할 수 있다."""
        return bool(np.abs(self.U).max() < 1e-9 and np.ptp(self.X, axis=0).max() < 1e-6)


class HoldLatch:
    """정지 기준에서의 hysteresis — Nav2 의 stateful goal checker 와 같은 규칙.

        idle(명령 0, MPC 리셋)  ←  hold 기준이고 |Xe_xy| < enter_xy 이고 |Xe_yaw| < enter_yaw
        추종 재개               ←  |Xe_xy| > exit_xy 또는 |Xe_yaw| > exit_yaw  (밀렸을 때)
        기준이 hold 가 아니면 항상 추종.

    왜 필요한가 (2026-09-10, bags/mpc_chain_20260910_172254): 도착 후 MPC 가 hold 포즈를 계속 능동 추종하면
    포즈 잡음 1 mm·0.3° 가 몸체명령 ~10 mm/s 로 나가고, 바퀴 0.5~0.9 rad/s 는 stiction 아래라 드라이버
    속도 PI 가 wind-up 하며 stick-slip 한다 (자이로 0.5 Hz, 1σ 24 mrad/s). 적분 작용 + Coulomb 마찰의
    limit cycle (Franklin–Powell–Emami-Naeini, nonlinear elements). 정지 상태에서는 적분기를 돌리지 않는 것이 해법.
    enter 는 잡음 3σ 보다 훨씬 크게(떨림 없음), exit 는 플래너 goal_tolerance 보다 작게(밀림은 잡음) 둔다.
    idle 진입에는 '멈춰 있음'(stopped) 도 필요하다 — Nav2 StoppedGoalChecker 와 같다. 플래너가 주행 중에 정지
    기준(현재 포즈, v=0)을 내면 그 순간 오차는 0 이지만 로봇은 아직 움직이고 있다. 그때 바로 0 을 내면 rate 제약을
    건너뛴 급정지가 되므로, MPC 가 제약 안에서 감속을 끝낸 뒤에 idle 로 넘긴다."""

    def __init__(self, enter_xy, enter_yaw, exit_xy, exit_yaw):
        if not (0 < enter_xy < exit_xy and 0 < enter_yaw < exit_yaw):
            raise ValueError(f"hold 문턱은 0 < enter < exit 여야 한다: xy {enter_xy}/{exit_xy}, yaw {enter_yaw}/{exit_yaw}")
        self.enter_xy, self.enter_yaw, self.exit_xy, self.exit_yaw = map(float, (enter_xy, enter_yaw, exit_xy, exit_yaw))
        self.idle = False

    def update(self, is_hold, Xe, stopped=True):
        """이번 틱의 판정. True 면 명령 0 을 내고 MPC 를 쉬게 한다. stopped: 측정 속도가 정지 문턱 아래인가."""
        if not is_hold:
            self.idle = False
            return False
        d, a = math.hypot(Xe[0], Xe[1]), abs(wrap(Xe[2]))
        if self.idle:
            if d > self.exit_xy or a > self.exit_yaw:
                self.idle = False
        elif stopped and d < self.enter_xy and a < self.enter_yaw:
            self.idle = True
        return self.idle


def circle_reference(t, origin, radius, speed, ramp_s=1.0, duration=0.0):
    """기동 포즈 origin=[x0,y0,yaw0] 를 원점으로 한 반시계 원궤적 (fake_reference_node 가 쓴다).
    속도는 ramp_s 동안 0→speed, duration>0 이면 끝 ramp_s 동안 speed→0.  반환 (Xr[3], ur[3])."""
    r, T, V = ramp_s, duration, speed
    if r <= 0:
        v, arc = V, V * t
    elif t < r:
        v, arc = V * t / r, 0.5 * V * t * t / r
    elif T <= 0 or t < T - r:
        v, arc = V, V * (t - 0.5 * r)
    elif t < T:
        td = T - t; v = V * td / r; arc = V * (T - r) - 0.5 * V * td * td / r
    else:
        v, arc = 0.0, V * (T - r)
    th = arc / radius
    x0, y0, a0 = origin
    lx, ly = radius * math.sin(th), radius * (1.0 - math.cos(th))
    c, s = math.cos(a0), math.sin(a0)
    Xr = np.array([x0 + c * lx - s * ly, y0 + s * lx + c * ly, wrap(a0 + th)])
    return Xr, np.array([v, 0.0, v / radius])
