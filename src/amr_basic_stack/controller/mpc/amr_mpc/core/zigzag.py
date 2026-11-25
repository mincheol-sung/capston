"""지그재그 기준 (2026-11-24) — 길이 L 직선 n 개, 이웃 직선 사이 각 α, 직선마다 전진·후진을 번갈아.

Figure8·Square 와 같은 꼴(at, peak_load, max_curvature, path_xy, T_total, T_path, T_align, V, A)이라 figure8_reference_node 가
shape:=zigzag 로 고른다. 속도 ur 는 몸체 좌표 [vx, vy, ω] — Figure8.at 과 같다.

모양 (로컬 좌표: 출발 포즈가 원점, 출발 헤딩이 +x)
  α = 이웃 두 직선이 꼭짓점에서 이루는 사이각 (angle_deg, 기본 10°). 진행 방향은 꼭짓점마다 180° − α 꺾이고, 꺾는 쪽이 번갈아
  바뀐다 (지그재그). angle_deg > 0 이면 왼쪽(+y)으로, < 0 이면 오른쪽으로 나아간다.
    직선 k 의 헤딩  θ_k = 0 (k 짝수),  −s·α (k 홀수)        s = sign(angle_deg)
    직선 k 의 진행  d_k = +1 전진 (k 짝수), −1 후진 (k 홀수)   진행 방향 = θ_k + (0 | π)
  헤딩은 늘 직선 방향이고, 진행 방향이 꺾이면 전진·후진이 바뀐다 → 꼭짓점에서 몸체는 α 만 돌면 된다 (180° − α 가 아니라).
  n = 5, L = 1, α = 10° 이면 꼭짓점 (0,0) → (1,0) → (0.015,0.174) → (1.015,0.174) → (0.030,0.347) → (1.030,0.347).
  몸체 v_y = 0, 직선 위 ω = 0 → 퓨어 퍼슛 (v_y = 0) 도 원리상 정확히 낼 수 있다 (후진은 v_x < 0).

헤딩 yaw_mode (2026-11-24 (9), 다른 경로와 같은 세 가지)
  tangent : 위 설명 (헤딩 = 직선 방향, 전진·후진 번갈아). align_s > 0 이고 끝 헤딩이 출발과 다르면 (n 짝수) 선 뒤 align_s 동안
            출발 헤딩으로 제자리 회전 (Square 와 같은 스무스스텝).
  fixed   : 출발 헤딩 고정 — 메카넘 옆걸음. 진행 방향 그대로 몸체 [v_x, v_y] 로 낸다. 꼭짓점 회전 없음 (쉼 2·dwell). 퓨어 퍼슛은 못 간다.
  spin    : Figure8 spin 과 같은 식 — 헤딩 = κ_s·S, κ_s = 2π·n_s/L, n_s = round(spin_rate·L/V / 2π) ≥ 1 (V = 직선 최고 속력).
            ω = κ_s·v. 끝에서 출발 헤딩. 꼭짓점 회전 없음 (쉼 2·dwell). 퓨어 퍼슛은 못 간다.

속도 (직선마다 양 끝 0, 몸체 v_x 만)
  profile "square" : 사다리꼴 — 가속 +A 로 올라가 v_top 에서 등속, −A 로 내려온다. v_top = min(√(A·L), v_max).
                     자르지 않으면 삼각형 (bang-bang, 최소 시간 2√(L/A)).
  profile "sine"   : 반 사인  v(τ) = V_p·sin(πτ/T_leg). 최대 가속도 |a| = π·V_p/T_leg (양 끝) 를 A 로 둔다
                     → V_p = √(A·L/2), T_leg = π·√(L/(2A)). V_p > v_max 면 V_p = v_max, T_leg = π·L/(2·V_p) (최대 가속도 < A).
  A 는 launch accel:= (지그재그에서 비우면 max = C 마찰 상한 안 직선 가속 한계, core/fig8_limits.line_accel_max), v_max 는 launch speed:=
  (비우면 max = 부하 예산에서만 자름) 로 정해 노드 accel·speed 로 넘긴다 (2026-11-24 (8)).

꼭짓점 (Square corner_mode stop 과 같은 꼴)
  선 자리에서 dwell_s 쉼 → rot_s 동안 θ_k → θ_{k+1} 제자리 회전 (스무스스텝 3x² − 2x³, 양 끝 ω = 0) → dwell_s 쉼 → 다음 직선.
  dwell 은 torque_mpc 의 기준 가속도 차분 창 (acc_dt 0.2 s) 이 감속 끝과 회전 시작을 한 창에 섞지 않게 하는 틈.
  (tangent 만. fixed·spin 은 쉼 2·dwell 뿐.) 끝 정렬 회전은 위 "헤딩" 의 align_s.

꼭짓점 corner_mode pass (2026-11-25, 사용자: PP·MPC 를 같은 경로·속도·가속도 부하에서 비교 — 서지 않고 이어 달리기)
  turn_m > 0 (기본): 꼭짓점 앞뒤 turn_m [m] 를 cubic spiral (Kanayama & Hartman 1989, heading 이 호길이의 3차식) 로 깎아 이어 달린다.
    turn 의 heading  θ(s) = θ_k + φ_k·S(s/L_t),  S = 스무스스텝 3x² − 2x³,  φ_k = θ_{k+1} − θ_k (늘 전진이면 ±(π − α), 번갈아)
    곡률 κ(s) = φ_k·6x(1 − x)/L_t → turn 양 끝 κ = 0 이라 직선과 곡률이 이어진다 (ω 계단 없음), 최대 1.5·|φ|/L_t (가운데).
    L_t 는 turn 이 꼭짓점에서 양쪽 직선 위 turn_m 지점에서 시작·끝나게:  turn_m = L_t·d₁(φ),  d₁ = X₁ − Y₁/tan|φ|
    ((X₁, Y₁) = 길이 1 turn 의 끝점, 수치 적분). 대칭이라 끝점도 다음 직선 위 turn_m. 0 < turn_m ≤ L/2. 꼭짓점 자체는 지나지 않는다.
    turn 위는 corner_speed v_c 등속 (몸체 [v_c, 0, v_c·κ(s)]) → 퓨어 퍼슛 (v_y = 0, ω = vκ) 도 원리상 정확히 낼 수 있다.
    turn 앞뒤 직선에 dwell_s 동안 v_c 등속 (거리 v_c·dwell_s, 직선이 짧으면 줄임) — stop 의 쉼과 같은 이유:
    torque_mpc 기준 가속도 차분 창 (acc_dt 0.2 s) 에 직선 감속 끝과 turn 의 횡가속이 한꺼번에 들어가지 않게.
    tangent·늘 전진만 (reverse 는 False 로 — 전진·후진이 바뀌는 cusp 는 v = 0 을 지나야 한다).
    path following 창 segment_window: 직선 k = (turn k−1 가운데, turn k 가운데) — 빈틈 없이 이어진다.
  turn_m = 0: 모양은 꺾인 선 그대로, 꼭짓점을 v_c 로 지난다. 속도 방향·heading 이 꼭짓점에서 계단 — 기준 자체가 낼 수 없는 값
    (2026-11-25 실차: PP·MPC 둘 다 못 따라감).
  직선 남은 부분은 v_c →(가속 A)→ 최고 →(감속 A)→ v_c 사다리꼴 (Square._Edge 와 같은 꼴, v_max 에서 자름), 첫 직선은 0 에서 출발,
  마지막은 0 으로 선다. profile square 만.
"""
import bisect
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _smooth(x):
    """스무스스텝 S(x) = 3x² − 2x³ 와 S'(x). x 는 [0, 1] 로 자른다."""
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x), 6.0 * x * (1.0 - x)


class _Leg:
    """한 직선의 속도 프로파일 (양 끝 0). at(τ) → (이동 거리, 속력 ≥ 0, 가속도)."""

    def __init__(self, L, A, v_max, profile):
        self.L, self.A, self.profile = float(L), float(A), profile
        if profile == "square":
            vp = math.sqrt(A * L)
            if v_max > 0.0:
                vp = min(vp, v_max)
            self.vp = vp
            self.ta = vp / A
            self.da = vp * vp / (2.0 * A)
            self.tc = (L - 2.0 * self.da) / vp
            self.T = 2.0 * self.ta + self.tc
            self.a_peak = A
        else:                                                           # sine
            vp = math.sqrt(0.5 * A * L)
            if v_max > 0.0 and vp > v_max:
                vp = v_max
            self.vp = vp
            self.T = math.pi * L / (2.0 * vp)
            self.a_peak = math.pi * vp / self.T

    def at(self, tau):
        T, vp = self.T, self.vp
        if tau <= 0.0:
            return 0.0, 0.0, 0.0
        if tau >= T:
            return self.L, 0.0, 0.0
        if self.profile == "sine":
            w = math.pi / T
            return vp / w * (1.0 - math.cos(w * tau)), vp * math.sin(w * tau), vp * w * math.cos(w * tau)
        A, ta = self.A, self.ta
        if tau < ta:
            return 0.5 * A * tau * tau, A * tau, A
        if tau < ta + self.tc:
            return self.da + vp * (tau - ta), vp, 0.0
        u = T - tau
        return self.L - 0.5 * A * u * u, A * u, -A


class _PassEdge:
    """pass 의 직선 (남은 부분): v_s 등속 b_s → v_s →(가속 A)→ v_p →(감속 A)→ v_e → v_e 등속 b_e. at(τ) → (이동 거리, 속력, 가속도)."""

    def __init__(self, L, vs, ve, A, v_max, bs=0.0, be=0.0):
        if bs + be > L:                                                 # 직선이 짧으면 등속 구간을 비례로 줄임
            k = L / (bs + be)
            bs, be = bs * k, be * k
        Lm = L - bs - be
        vp = math.sqrt(0.5 * (vs * vs + ve * ve + 2.0 * A * Lm))
        if v_max > 0.0:
            vp = min(vp, v_max)
        if vp < max(vs, ve) - 1e-9:
            raise ValueError(f"zigzag pass: 직선 {L:.3f} m 에서 accel {A:.2f} 로 corner_speed {max(vs, ve):.3f} m/s 를 못 낸다 "
                             f"(최고 {vp:.3f}) — zigzag_corner_speed 를 낮춰라")
        vp = max(vp, vs, ve)
        self.L, self.vs, self.ve, self.vp, self.A, self.a_peak = L, vs, ve, vp, A, A
        self.bs, self.be = bs, be
        self.ts = bs / vs if bs > 0.0 else 0.0
        self.te = be / ve if be > 0.0 else 0.0
        self.d1 = (vp * vp - vs * vs) / (2.0 * A)
        self.d3 = (vp * vp - ve * ve) / (2.0 * A)
        self.d2 = max(0.0, Lm - self.d1 - self.d3)
        self.t1, self.t3 = (vp - vs) / A, (vp - ve) / A
        self.t2 = self.d2 / vp if vp > 0.0 else 0.0
        self.T = self.ts + self.t1 + self.t2 + self.t3 + self.te

    def at(self, tau):
        A, vs, ve, vp = self.A, self.vs, self.ve, self.vp
        if tau <= 0.0:
            return 0.0, vs, 0.0
        if tau >= self.T:
            return self.L, ve, 0.0
        if tau < self.ts:
            return vs * tau, vs, 0.0
        u, x0 = tau - self.ts, self.bs
        if u < self.t1:
            return x0 + vs * u + 0.5 * A * u * u, vs + A * u, A
        u -= self.t1; x0 += self.d1
        if u < self.t2:
            return x0 + vp * u, vp, 0.0
        u -= self.t2; x0 += self.d2
        if u < self.t3:
            return x0 + vp * u - 0.5 * A * u * u, vp - A * u, -A
        u -= self.t3; x0 += self.d3
        return x0 + ve * u, ve, 0.0


class _Turn:
    """길이 1, 들어갈 때 heading 0 인 cubic spiral θ(x) = φ·S(x) (φ > 0, 왼쪽) 의 위치 표와 꼭짓점 거리 d₁.
    pos(x) 는 Hermite 보간 — 접선 (cos θ, sin θ) 을 그대로 써서 위치 미분이 heading 과 맞는다."""

    N = 4000

    def __init__(self, phi):
        self.phi = float(phi)
        x = np.linspace(0.0, 1.0, 8 * self.N + 1)                       # 사다리꼴 적분은 8 배 촘촘히, 표는 N 칸
        th = self.phi * x * x * (3.0 - 2.0 * x)
        c, s = np.cos(th), np.sin(th)
        h = x[1] - x[0]
        X = np.concatenate([[0.0], np.cumsum(0.5 * h * (c[1:] + c[:-1]))])
        Y = np.concatenate([[0.0], np.cumsum(0.5 * h * (s[1:] + s[:-1]))])
        self.P = np.stack([X[::8], Y[::8]], 1)
        self.Tg = np.stack([c[::8], s[::8]], 1)
        X1, Y1 = self.P[-1]
        self.d1 = float(X1 - Y1 / math.tan(self.phi))                   # 시작점에서 들어가는 직선을 따라 꼭짓점까지 (L_t = 1)

    def pos(self, x):
        x = min(max(x, 0.0), 1.0)
        i = min(int(x * self.N), self.N - 1)
        h = 1.0 / self.N
        u = x * self.N - i
        h00, h10, h01, h11 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u, -2 * u**3 + 3 * u**2, u**3 - u**2
        return h00 * self.P[i] + h10 * h * self.Tg[i] + h01 * self.P[i + 1] + h11 * h * self.Tg[i + 1]


class Zigzag:
    def __init__(self, n=5, length=1.0, angle_deg=10.0, accel=1.0, v_max=0.0, profile="sine",
                 rot_s=0.5, dwell_s=0.1, yaw_mode="tangent", spin_rate=1.0, align_s=0.0, reverse=True,
                 corner_mode="stop", corner_speed=0.3, turn_m=0.5):
        if yaw_mode not in ("tangent", "fixed", "spin"):
            raise ValueError(f"zigzag 의 yaw_mode 는 tangent | fixed | spin — 받은 값 {yaw_mode!r}")
        if yaw_mode == "spin" and not spin_rate > 0.0:
            raise ValueError(f"spin_rate 는 양수 [rad/s] (받은 값 {spin_rate!r})")
        if profile not in ("sine", "square"):
            raise ValueError(f"zigzag_profile 은 sine | square — 받은 값 {profile!r}")
        n_f = float(n)
        if n_f != int(n_f) or int(n_f) < 1:
            raise ValueError(f"zigzag_n 은 1 이상 정수 — 받은 값 {n}")
        if not (length > 0.0 and accel > 0.0 and v_max >= 0.0):
            raise ValueError(f"zigzag: length·accel > 0, v_max ≥ 0 — 받은 값 {length}, {accel}, {v_max}")
        if not 0.0 < abs(angle_deg) < 180.0:
            raise ValueError(f"zigzag_angle_deg 는 0 < |값| < 180 (직선 사이각) — 받은 값 {angle_deg}")
        if corner_mode not in ("stop", "pass"):
            raise ValueError(f"zigzag_corner 는 stop | pass — 받은 값 {corner_mode!r}")
        self.corner_mode = corner_mode
        self.pass_ = corner_mode == "pass"
        if self.pass_ and profile != "square":
            raise ValueError(f"zigzag_corner pass 는 zigzag_profile square 만 — 받은 값 {profile!r}")
        if self.pass_ and not corner_speed > 0.0:
            raise ValueError(f"zigzag_corner_speed 는 양수 [m/s] — 받은 값 {corner_speed}")
        self.round_ = self.pass_ and turn_m > 0.0                      # cubic spiral 로 깎음 (머리말 pass)
        if self.round_:
            if yaw_mode != "tangent":
                raise ValueError(f"zigzag_turn_m > 0 (꼭짓점 깎기) 는 yaw_mode tangent 만 — 받은 값 {yaw_mode!r}")
            if turn_m > 0.5 * length + 1e-12:
                raise ValueError(f"zigzag_turn_m 은 0 ≤ 값 ≤ zigzag_length/2 ({0.5 * length:g}) — 받은 값 {turn_m}")
            reverse = False
        elif self.pass_ and turn_m < 0.0:
            raise ValueError(f"zigzag_turn_m 은 0 이상 — 받은 값 {turn_m}")
        if (yaw_mode == "tangent" and rot_s <= 0.0 and not self.pass_) or dwell_s < 0.0:
            raise ValueError(f"zigzag: rot_s > 0 (tangent), dwell_s ≥ 0 — 받은 값 {rot_s}, {dwell_s}")
        self.n, self.length, self.profile = int(n_f), float(length), profile
        self.angle_deg = float(angle_deg)
        self.alpha = math.radians(abs(angle_deg))
        sgn = 1.0 if angle_deg > 0.0 else -1.0
        self.yaw_mode = yaw_mode
        self.tangent = yaw_mode == "tangent"
        self.rot = float(rot_s) if self.tangent and not self.pass_ else 0.0
        self.dwell = 0.0 if (self.pass_ and not self.round_) else float(dwell_s)   # 깎은 pass: turn 앞뒤 v_c 등속 시간
        self.T_corner = self.rot + 2.0 * self.dwell                      # fixed·spin 은 꼭짓점에서 돌 일이 없어 쉼만
        self.A = float(accel)
        self.vc = float(corner_speed) if self.pass_ else 0.0
        if self.pass_:                                                  # 꼭짓점 v_c, 첫 직선 0 에서 출발·마지막 0 으로 선다
            self.legs = [_PassEdge(self.length, 0.0 if k == 0 else self.vc, 0.0 if k == self.n - 1 else self.vc, self.A, float(v_max))
                         for k in range(self.n)]
        else:
            self.legs = [_Leg(self.length, self.A, float(v_max), profile) for _ in range(self.n)]
        # tangent 헤딩 θ_k 와 진행 부호 (2026-11-24 (10) reverse):
        #   reverse True  — θ_k = 0 | −s·α, 홀수 직선 후진 → 꼭짓점 회전 ∓α
        #   reverse False — θ_k = 0 | s·(π − α) (= 진행 방향), 늘 전진 → 꼭짓점 회전 ±(π − α), 짧은 쪽으로 번갈아 (헤딩이 한쪽으로 쌓이지 않음)
        #   진행 방향 d_k·(cos θ_k, sin θ_k) 는 둘이 같다 → 꼭짓점·모양 같음
        self.reverse = bool(reverse)
        if self.reverse:
            self.heads = [0.0 if k % 2 == 0 else -sgn * self.alpha for k in range(self.n)]
            self.dirs_sign = [1.0 if k % 2 == 0 else -1.0 for k in range(self.n)]
        else:
            self.heads = [0.0 if k % 2 == 0 else sgn * (math.pi - self.alpha) for k in range(self.n)]
            self.dirs_sign = [1.0] * self.n
        self.dirs = [d * np.array([math.cos(h), math.sin(h)]) for d, h in zip(self.dirs_sign, self.heads)]   # 진행 방향 (로컬)
        self.corners = [np.zeros(2)]
        for u in self.dirs:
            self.corners.append(self.corners[-1] + self.length * u)
        if self.round_:
            self._build_round(float(turn_m), float(v_max))
        else:
            self.t_start = list(np.cumsum([0.0] + [g.T + self.T_corner for g in self.legs[:-1]]))
            self.t_stop = [ts + g.T for ts, g in zip(self.t_start, self.legs)]   # 직선 k 가 끝나 서는 (pass: 꼭짓점을 지나는) 시각
            self.T_path = float(sum(g.T for g in self.legs)) + (self.n - 1) * self.T_corner
        self.V = max(g.vp for g in self.legs)
        self.L = self.n * self.length
        # spin (Figure8 과 같은 식): 헤딩 = κ_s·S (S = 누적 호길이), κ_s = 2π·n_s/L, n_s = round(spin_rate·L/V / 2π) ≥ 1 → 끝에서 출발 헤딩.
        #   ω = κ_s·v 라 직선 양 끝 (v = 0) 에서 0, 최고 ω = κ_s·V ≈ spin_rate (V = 직선 최고 속력).
        self.spin_turns, self.kappa_spin = 0, 0.0
        if yaw_mode == "spin":
            self.spin_turns = max(1, round(float(spin_rate) * self.L / self.V / (2.0 * math.pi)))
            self.kappa_spin = 2.0 * math.pi * self.spin_turns / self.L
        self.yaw_end = (self.heads[-1] if self.tangent else 0.0) + self.kappa_spin * self.L   # 주행 끝 헤딩 (펼친 값)
        self.dtheta = wrap(-self.yaw_end) if self.tangent else 0.0       # tangent: 출발 헤딩으로 돌아가는 회전 (n 짝수면 s·α)
        self.T_align = float(align_s) if (self.tangent and align_s > 0.0 and abs(self.dtheta) > 1e-9) else 0.0
        self.T_total = self.T_path + self.T_align
        self.drive_window = (0.0, self.T_path)                           # path following 구간 (정렬 회전은 끝에)

    def _build_round(self, d, v_max):
        """pass + turn_m > 0: 직선 남은 부분 (_PassEdge) 과 turn 을 번갈아 이은 시간표. self.legs 를 직선 남은 부분으로 바꾼다."""
        self.turn_m = d
        self.turn = _Turn(math.pi - self.alpha)                         # 늘 전진 → |φ| = π − α
        self.Lt = d / self.turn.d1
        self.T_corner = self.Lt / self.vc                               # turn 시간 (등속)
        self.kappa_max = 1.5 * self.turn.phi / self.Lt
        n, vc, b = self.n, self.vc, self.vc * self.dwell
        self.legs, self.seg = [], []                                    # seg: (시작 시각, 시간, 종류, k)
        t = 0.0
        for k in range(n):
            o_s, o_e = (d if k > 0 else 0.0), (d if k < n - 1 else 0.0)
            e = _PassEdge(self.length - o_s - o_e, vc if k > 0 else 0.0, vc if k < n - 1 else 0.0, self.A, v_max,
                          b if k > 0 else 0.0, b if k < n - 1 else 0.0)
            e.p0 = self.corners[k] + o_s * self.dirs[k]
            self.legs.append(e)
            self.seg.append((t, e.T, "edge", k)); t += e.T
            if k < n - 1:
                self.seg.append((t, self.T_corner, "turn", k)); t += self.T_corner
        self.seg_t = [x[0] for x in self.seg]
        self.T_path = t
        mids = [s0 + 0.5 * T for s0, T, kind, _ in self.seg if kind == "turn"]
        self.t_start, self.t_stop = [0.0] + mids, mids + [self.T_path]   # path following 창

    def _at_round(self, t):
        """깎은 pass 의 (로컬 위치, 로컬 월드 속도, 로컬 heading, ω)."""
        i = max(0, bisect.bisect_right(self.seg_t, t) - 1)
        t0, _, kind, k = self.seg[i]
        tau = t - t0
        if kind == "edge":
            e = self.legs[k]
            dist, v, _ = e.at(tau)
            return e.p0 + dist * self.dirs[k], v * self.dirs[k], self.heads[k], 0.0
        sg = 1.0 if wrap(self.heads[k + 1] - self.heads[k]) > 0.0 else -1.0
        x = min(max(tau / self.T_corner, 0.0), 1.0)
        P = self.turn.pos(x)
        h0 = self.heads[k]
        c0, s0 = math.cos(h0), math.sin(h0)
        px, py = self.Lt * P[0], self.Lt * sg * P[1]
        start = self.corners[k + 1] - self.turn_m * self.dirs[k]
        S, dS = _smooth(x)
        yl = h0 + sg * self.turn.phi * S
        w = sg * self.turn.phi * dS / self.Lt * self.vc
        return (start + np.array([c0 * px - s0 * py, s0 * px + c0 * py]), self.vc * np.array([math.cos(yl), math.sin(yl)]), yl, w)

    def _yaw(self, k, S):
        """직선 k, 누적 호길이 S 에서 (로컬 헤딩, dψ/dS)."""
        if self.tangent:
            return self.heads[k], 0.0
        return self.kappa_spin * S, self.kappa_spin                     # fixed 는 κ_s = 0

    def segment_window(self, t):
        """기준 시각 t 가 직선 j 위 (t_start[j] ≤ t ≤ t_stop[j]) 면 (t_start[j], t_stop[j]), 꼭짓점 (쉼·회전) 이나 주행 밖이면 None.
        path following (figure8_reference govern) 은 None 이면 시계대로 두고, 아니면 사영 창을 그 직선 안에 가둔다.
        이웃 직선이 10° 옆에 겹쳐 같은 자리가 두 직선 위에 있다 — 창이 직선 끝을 넘으면 (시뮬 2026-11-24)
          · 후진 중인 로봇이 지난 직선에 사영돼 t_proj 가 거꾸로 내려가고 기준을 지난 직선으로 끌어당김 (끝없이 되풀이)
          · 꼭짓점에서 조금 덜 간 로봇이 다음 직선 중간에 사영돼 하한 t_ref ≥ t_proj 가 쉼·회전·직선 앞부분을 건너뜀 (주행 10.5 → 6.4 s)."""
        for ts, te in zip(self.t_start, self.t_stop):
            if ts - 1e-9 <= t <= te + 1e-9:
                return ts, te
        return None

    def at(self, t, origin=(0.0, 0.0, 0.0)):
        """시각 t 의 (Xr=[x,y,yaw] 세계 좌표, ur=[vx,vy,ω] 몸체 좌표). origin = 출발 포즈."""
        t = float(t)
        p, vw, yl, wl = self.corners[0], np.zeros(2), 0.0, 0.0
        if t >= self.T_path:
            p, yl = self.corners[self.n], self.yaw_end
            if self.T_align > 0.0:
                s, ds = _smooth((t - self.T_path) / self.T_align)
                yl += self.dtheta * s
                wl = self.dtheta * ds / self.T_align if t < self.T_total else 0.0
        elif t > 0.0 and self.round_:
            p, vw, yl, wl = self._at_round(t)
        elif t > 0.0:
            k = bisect.bisect_right(self.t_start, t) - 1
            tau = t - self.t_start[k]
            g = self.legs[k]
            if tau >= g.T and not self.pass_:                           # 꼭짓점 k+1 에 서 있다 (pass 는 T_corner 0 이라 안 옴)
                p = self.corners[k + 1]
                yl, _ = self._yaw(k, (k + 1) * self.length)
                if self.tangent:                                        # θ_k → θ_{k+1} 제자리 회전
                    dth = self.heads[k + 1] - self.heads[k]
                    s, ds = _smooth((tau - g.T - self.dwell) / self.rot)
                    yl += dth * s
                    wl = dth * ds / self.rot
            else:
                d, v, _ = g.at(tau)
                p = self.corners[k] + d * self.dirs[k]
                vw = v * self.dirs[k]
                yl, dyds = self._yaw(k, k * self.length + d)
                wl = dyds * v
        c, s_ = math.cos(yl), math.sin(yl)                               # 로컬 속도 → 몸체 (−ψ 회전)
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        Xr = np.array([x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1], wrap(a0 + yl)])
        return Xr, np.array([c * vw[0] + s_ * vw[1], -s_ * vw[0] + c * vw[1], wl])

    def peak_load(self, k_geom, n=6000):
        """(주행, 정렬 회전) L1 부하 |vx|+|vy|+k|ω| 최대 — Figure8.peak_load 와 같은 뜻."""
        run = spin = 0.0
        for t in np.linspace(0.0, self.T_total, n):
            _, u = self.at(t)
            load = abs(u[0]) + abs(u[1]) + k_geom * abs(u[2])
            if t < self.T_path:
                run = max(run, load)
            else:
                spin = max(spin, load)
        return run, spin

    def max_curvature(self):
        return self.kappa_max if self.round_ else math.inf             # 꺾인 꼭짓점은 ∞

    def leg_summary(self):
        """[(직선 번호, 진행 부호, 직선 시작 헤딩 [deg], 최고 속력, 최대 가속도, 시간)] — 기동 로그용. 진행 부호는 tangent 의 전진 +1 / 후진 −1."""
        return [(k, self.dirs_sign[k], math.degrees(wrap(self._yaw(k, k * self.length)[0])), g.vp, g.a_peak, g.T)
                for k, g in enumerate(self.legs)]

    def path_xy(self, n=0, origin=(0.0, 0.0, 0.0)):
        """RViz 표시용 꼭짓점 (깎은 pass 는 곡선을 촘촘히)."""
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        pts = self.corners
        if self.round_:
            pts = [self._at_round(t)[0] for t in np.linspace(0.0, self.T_path, max(int(n), 40 * self.n + 1))]
        return [(x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1]) for p in pts]


if __name__ == "__main__":
    # 검산: 속도를 위치의 유한차분과 비교 (몸체 좌표로), 직선 양 끝 속도 0, 최대 가속도 = A, 사이각, 끝 헤딩
    for kw in (dict(), dict(profile="square"), dict(profile="sine", v_max=0.6), dict(n=4, align_s=1.5),
               dict(angle_deg=-10.0, n=4), dict(accel=1.7, profile="square"), dict(yaw_mode="fixed"), dict(yaw_mode="spin"),
               dict(yaw_mode="spin", spin_rate=3.0, accel=1.7, profile="square"),
               dict(corner_mode="pass", n=10, angle_deg=30.0, accel=1.15, corner_speed=0.5, align_s=1.5, reverse=False, profile="square", turn_m=0.0),
               dict(corner_mode="pass", n=4, angle_deg=-60.0, accel=1.15, corner_speed=0.4, v_max=0.8, profile="square", turn_m=0.0),
               dict(corner_mode="pass", n=10, angle_deg=30.0, accel=1.15, corner_speed=0.3, align_s=1.5, profile="square", turn_m=0.5),
               dict(corner_mode="pass", n=4, angle_deg=-60.0, accel=1.15, corner_speed=0.4, v_max=0.8, profile="square", turn_m=0.3)):
        z = Zigzag(**kw)
        o = (0.3, -0.2, 0.7)
        h, err = 1e-5, 0.0
        for t in np.linspace(0.003, z.T_total - 0.003, 8000):
            X1, u = z.at(t, o)
            X2, _ = z.at(t + h, o)
            Xm, _ = z.at(t - h, o)
            d = (X2 - Xm) / (2 * h)
            d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
            c, s = math.cos(X1[2]), math.sin(X1[2])
            body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
            err = max(err, float(np.abs(body - u).max()))
        if z.pass_:                                                     # pass: 출발·끝만 0, 사이 최저 속력 = corner_speed
            ends = np.hypot(*z.at(0.0)[1][:2]) + np.hypot(*z.at(z.T_path)[1][:2])
        else:
            ends = max(np.hypot(*z.at(z.t_start[k] + g.T)[1][:2]) + np.hypot(*z.at(z.t_start[k])[1][:2]) for k, g in enumerate(z.legs))
        Xe, _ = z.at(z.T_total + 1.0, o)
        g = z.legs[0]
        print(f"{kw}: 직선 {z.n} × {z.length} m, 최고 {g.vp:.3f} m/s, 최대 가속 {g.a_peak:.3f}, 주행 {z.T_path:.2f} s + 정렬 {z.T_align:.2f} s, "
              f"spin {z.spin_turns} 바퀴 (κ {z.kappa_spin:.3f}), 끝 헤딩 {math.degrees(wrap(Xe[2] - o[2])):+.2f}°, 끝 속도 합 {ends:.1e}, 속도 vs 유한차분 {err:.1e}")
