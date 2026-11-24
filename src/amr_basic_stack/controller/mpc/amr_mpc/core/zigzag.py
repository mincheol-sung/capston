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


class Zigzag:
    def __init__(self, n=5, length=1.0, angle_deg=10.0, accel=1.0, v_max=0.0, profile="sine",
                 rot_s=0.5, dwell_s=0.1, yaw_mode="tangent", spin_rate=1.0, align_s=0.0):
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
        if (yaw_mode == "tangent" and rot_s <= 0.0) or dwell_s < 0.0:
            raise ValueError(f"zigzag: rot_s > 0 (tangent), dwell_s ≥ 0 — 받은 값 {rot_s}, {dwell_s}")
        self.n, self.length, self.profile = int(n_f), float(length), profile
        self.angle_deg = float(angle_deg)
        self.alpha = math.radians(abs(angle_deg))
        sgn = 1.0 if angle_deg > 0.0 else -1.0
        self.yaw_mode = yaw_mode
        self.tangent = yaw_mode == "tangent"
        self.rot, self.dwell = (float(rot_s) if self.tangent else 0.0), float(dwell_s)
        self.T_corner = self.rot + 2.0 * self.dwell                      # fixed·spin 은 꼭짓점에서 돌 일이 없어 쉼만
        self.A = float(accel)
        self.legs = [_Leg(self.length, self.A, float(v_max), profile) for _ in range(self.n)]
        self.heads = [0.0 if k % 2 == 0 else -sgn * self.alpha for k in range(self.n)]   # 직선 k 의 축 방향 (tangent 헤딩) θ_k
        self.dirs_sign = [1.0 if k % 2 == 0 else -1.0 for k in range(self.n)]           # tangent 에서 전진 +1 / 후진 −1
        self.dirs = [d * np.array([math.cos(h), math.sin(h)]) for d, h in zip(self.dirs_sign, self.heads)]   # 진행 방향 (로컬)
        self.corners = [np.zeros(2)]
        for u in self.dirs:
            self.corners.append(self.corners[-1] + self.length * u)
        self.t_start = list(np.cumsum([0.0] + [g.T + self.T_corner for g in self.legs[:-1]]))
        self.t_stop = [ts + g.T for ts, g in zip(self.t_start, self.legs)]   # 직선 k 가 끝나 서는 시각
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
        elif t > 0.0:
            k = bisect.bisect_right(self.t_start, t) - 1
            tau = t - self.t_start[k]
            g = self.legs[k]
            if tau >= g.T:                                              # 꼭짓점 k+1 에 서 있다
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
        return math.inf                                                 # 꼭짓점

    def leg_summary(self):
        """[(직선 번호, 진행 부호, 직선 시작 헤딩 [deg], 최고 속력, 최대 가속도, 시간)] — 기동 로그용. 진행 부호는 tangent 의 전진 +1 / 후진 −1."""
        return [(k, self.dirs_sign[k], math.degrees(wrap(self._yaw(k, k * self.length)[0])), g.vp, g.a_peak, g.T)
                for k, g in enumerate(self.legs)]

    def path_xy(self, n=0, origin=(0.0, 0.0, 0.0)):
        """RViz 표시용 꼭짓점."""
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        return [(x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1]) for p in self.corners]


if __name__ == "__main__":
    # 검산: 속도를 위치의 유한차분과 비교 (몸체 좌표로), 직선 양 끝 속도 0, 최대 가속도 = A, 사이각, 끝 헤딩
    for kw in (dict(), dict(profile="square"), dict(profile="sine", v_max=0.6), dict(n=4, align_s=1.5),
               dict(angle_deg=-10.0, n=4), dict(accel=1.7, profile="square"), dict(yaw_mode="fixed"), dict(yaw_mode="spin"),
               dict(yaw_mode="spin", spin_rate=3.0, accel=1.7, profile="square")):
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
        ends = max(np.hypot(*z.at(z.t_start[k] + g.T)[1][:2]) + np.hypot(*z.at(z.t_start[k])[1][:2]) for k, g in enumerate(z.legs))
        Xe, _ = z.at(z.T_total + 1.0, o)
        g = z.legs[0]
        print(f"{kw}: 직선 {z.n} × {z.length} m, 최고 {g.vp:.3f} m/s, 최대 가속 {g.a_peak:.3f}, 주행 {z.T_path:.2f} s + 정렬 {z.T_align:.2f} s, "
              f"spin {z.spin_turns} 바퀴 (κ {z.kappa_spin:.3f}), 끝 헤딩 {math.degrees(wrap(Xe[2] - o[2])):+.2f}°, 끝 속도 합 {ends:.1e}, 속도 vs 유한차분 {err:.1e}")
