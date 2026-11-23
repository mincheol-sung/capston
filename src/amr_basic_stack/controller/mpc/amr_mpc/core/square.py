"""정사각형 기준 (2026-11-20) — 변 가운데에서 가속 한계로 낼 수 있는 최고속도, 꼭짓점에서 corner_speed.

Figure8·Segment 와 같은 꼴(at, peak_load, max_curvature, path_xy, T_total, V, A)이라 figure8_reference_node 가
shape:=square 로 고른다. 속도 ur 는 몸체 좌표 [vx, vy, ω] — Figure8.at 과 같다.

모양
  출발 포즈(origin)가 한 꼭짓점이고 첫 변이 출발 헤딩 방향, 왼쪽으로 돈다 (로컬 좌표 앞 → 왼 → 뒤 → 오른).
  laps 바퀴 = 4·laps 변 (반올림). 한 바퀴마다 출발 꼭짓점으로 돌아온다.

속도 (호길이 축, 변마다 가속 a 로 올라가 감속 a 로 내려온다)
  변 끝 속도 v_s, v_e 사이 최고  v_p² = (v_s² + v_e² + 2aL)/2   (삼각형. 대칭 v_s = v_e = v_c 면 v_c² + aL, 변 가운데)
  v_max > 0 이면 그 값에서 자른다 (사다리꼴). 첫 변은 0 에서 출발, 마지막 변은 0 으로 선다.
  꼭짓점에서 속도 크기는 v_c 로 이어지지만 방향이 90° 꺾인다 → 속도 계단 √2·v_c (위치는 정확히 정사각형).
    토크 MPC 노드는 기준 가속도를 기준 속도의 중앙 차분(acc_dt 0.2 s)으로 내므로 꼭짓점에서 √2·v_c/acc_dt 를 0.2 s 동안 본다
    (v_c 0.1 → 0.71 m/s²). 가속 한계 a 인 점질량이 이 계단을 대칭 등가속으로 지나면 기준에서 v_c²/(4a) 벗어난다
    (a 0.92 → 0.27 cm). 정확한 정사각형 추종이 필요하면 corner_speed 0 (꼭짓점 정지).

헤딩
  fixed   : 출발 헤딩 고정 — 메카넘 옆걸음. 퓨어 퍼슛(비교군 A)은 v_y = 0 이라 옆 변을 못 간다.
  tangent : 변 방향. 꼭짓점에서 헤딩이 90° 계단이 되지 않게 꼭짓점 ±yaw_blend/2 호길이 안에서 스무스스텝으로 돌린다
            (위치는 그대로 정사각형, 그 구간에서만 옆걸음이 섞인다). 정지하는 마지막 꼭짓점은 완화하지 않고,
            선 뒤 align_s 동안 출발 헤딩으로 제자리 회전한다 (Figure8 역정렬과 같은 스무스스텝).

꼭짓점 처리 corner_mode (2026-11-21)
  pass : 위 설명대로 — 꼭짓점을 corner_speed 로 지나고, tangent 면 헤딩을 꼭짓점 앞뒤에서 돌린다.
         accel 1.5 · v_max 1.5 에서 헤딩 회전 토크가 직선 가감속 토크 (가장 약한 RR 상한의 96 %) 위에 겹쳐
         앞먹임 U_s 가 C 마찰 상한의 2.55 배 (analysis/torque_mpc_sim/2026-11-20_square_profile).
  stop : 꼭짓점마다 선다 (corner_speed 무시, 0). 선 자리에서 dwell_s 쉬고 → (tangent 면) rot_s 동안 +90° 제자리 회전
         (스무스스텝, 정렬 회전과 같은 꼴) → dwell_s 쉬고 → 다음 변. 병진 가감속과 헤딩 회전이 시간상 겹치지 않는다.
         몸체 v_y 가 어디서나 0 이라 퓨어 퍼슛 (v_y = 0) 도 원리상 정확히 낼 수 있는 기준이다.
         dwell 은 torque_mpc 의 기준 가속도 차분 창 (acc_dt 0.2 s) 이 감속 끝과 회전 시작을 한 창에 섞지 않게 하는 틈.
         fixed 면 회전 없이 dwell 만 (2·dwell_s 정지).
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


class _Edge:
    """한 변의 속도 프로파일: v_s → (가속 a) → v_p → (등속) → (감속 a) → v_e."""

    def __init__(self, L, vs, ve, a, v_max):
        vp = math.sqrt(0.5 * (vs * vs + ve * ve + 2.0 * a * L))
        if v_max > 0.0:
            vp = min(vp, v_max)
        if vp < max(vs, ve) - 1e-12:
            raise ValueError(f"변 끝 속도 {max(vs, ve):.3f} m/s 가 최고속도 {vp:.3f} 보다 크다 (corner_speed·v_max 확인)")
        self.L, self.vs, self.ve, self.vp, self.a = L, vs, ve, vp, a
        self.d1 = (vp * vp - vs * vs) / (2.0 * a)
        self.d3 = (vp * vp - ve * ve) / (2.0 * a)
        self.d2 = max(0.0, L - self.d1 - self.d3)
        self.t1, self.t3 = (vp - vs) / a, (vp - ve) / a
        self.t2 = self.d2 / vp if vp > 0.0 else 0.0
        self.T = self.t1 + self.t2 + self.t3

    def at(self, tau):
        """변 시작부터 τ 초 → (이동 거리, 속도)."""
        a = self.a
        if tau <= 0.0:
            return 0.0, self.vs
        if tau < self.t1:
            return self.vs * tau + 0.5 * a * tau * tau, self.vs + a * tau
        if tau < self.t1 + self.t2:
            return self.d1 + self.vp * (tau - self.t1), self.vp
        if tau < self.T:
            u = tau - self.t1 - self.t2
            return self.d1 + self.d2 + self.vp * u - 0.5 * a * u * u, self.vp - a * u
        return self.L, self.ve


class Square:
    def __init__(self, side=1.5, corner_speed=0.1, accel=0.4, v_max=0.0, laps=1.0,
                 yaw_mode="tangent", yaw_blend=0.3, align_s=1.5, corner_mode="pass", rot_s=1.0, dwell_s=0.2):
        if yaw_mode not in ("tangent", "fixed"):
            raise ValueError(f"square 의 yaw_mode 는 tangent | fixed (받은 값 {yaw_mode!r})")
        if corner_mode not in ("pass", "stop"):
            raise ValueError(f"square 의 corner_mode 는 pass | stop (받은 값 {corner_mode!r})")
        if side <= 0.0 or accel <= 0.0 or corner_speed < 0.0:
            raise ValueError("square: side·accel > 0, corner_speed ≥ 0")
        self.n = int(round(4.0 * float(laps)))
        if self.n < 1:
            raise ValueError(f"square: laps {laps} → 변 {self.n} 개 (1 개 이상이어야)")
        self.tangent = yaw_mode == "tangent"
        self.yaw_mode = yaw_mode
        self.corner_mode = corner_mode
        self.stop = corner_mode == "stop"
        self.side, self.A = float(side), float(accel)
        self.vc = 0.0 if self.stop else float(corner_speed)
        self.b = float(yaw_blend) if (self.tangent and not self.stop) else 0.0
        if self.tangent and not self.stop and not (0.0 < self.b <= self.side):
            raise ValueError(f"square tangent: 0 < yaw_blend ≤ side ({self.side} m) — 받은 값 {yaw_blend}")
        self.rot = float(rot_s) if (self.stop and self.tangent) else 0.0     # 꼭짓점 제자리 회전 시간
        self.dwell = float(dwell_s) if self.stop else 0.0                   # 회전 앞뒤 정지
        if self.stop and (self.dwell < 0.0 or (self.tangent and self.rot <= 0.0)):
            raise ValueError(f"square stop: rot_s > 0 (tangent), dwell_s ≥ 0 — 받은 값 {rot_s}, {dwell_s}")
        self.T_corner = self.rot + 2.0 * self.dwell if self.stop else 0.0  # 사이 꼭짓점 하나에서 서 있는 시간
        self.edges = [_Edge(self.side, 0.0 if i == 0 else self.vc, 0.0 if i == self.n - 1 else self.vc,
                            self.A, float(v_max)) for i in range(self.n)]
        self.t_start = list(np.cumsum([0.0] + [e.T + self.T_corner for e in self.edges[:-1]]))   # 변 i 의 출발 시각
        self.T_path = float(sum(e.T for e in self.edges)) + (self.n - 1) * self.T_corner
        self.yaw_end = (self.n - 1) * 0.5 * math.pi                     # 마지막 변 방향 (로컬, 펼친 값)
        self.dtheta = wrap(-self.yaw_end) if self.tangent else 0.0       # 출발 헤딩으로 돌아가는 회전
        self.T_align = float(align_s) if (self.tangent and align_s > 0.0 and abs(self.dtheta) > 1e-9) else 0.0
        self.T_total = self.T_path + self.T_align
        self.V = max(e.vp for e in self.edges)                          # 노드의 재개 배속 램프 V/A 에 쓴다
        self.L = self.n * self.side
        d = [i * 0.5 * math.pi for i in range(self.n)]
        self.dirs = [np.array([math.cos(x), math.sin(x)]) for x in d]
        self.corners = [np.zeros(2)]
        for u in self.dirs:
            self.corners.append(self.corners[-1] + self.side * u)

    # ---------- 헤딩 ----------
    def _yaw(self, i, S):
        """변 i, 누적 호길이 S 에서 (로컬 헤딩, dψ/dS)."""
        if not self.tangent:
            return 0.0, 0.0
        if self.stop:                                                   # 변 위에서는 늘 변 방향 (회전은 꼭짓점에서 서서)
            return i * 0.5 * math.pi, 0.0
        j = int(round(S / self.side))                                   # 가장 가까운 꼭짓점 (1..n−1 만 완화)
        if 1 <= j <= self.n - 1 and abs(S - j * self.side) <= 0.5 * self.b:
            s, ds = _smooth((S - j * self.side + 0.5 * self.b) / self.b)
            return (j - 1) * 0.5 * math.pi + 0.5 * math.pi * s, 0.5 * math.pi * ds / self.b
        return i * 0.5 * math.pi, 0.0

    # ---------- 기준 ----------
    def at(self, t, origin=(0.0, 0.0, 0.0)):
        """시각 t 의 (Xr=[x,y,yaw] 세계 좌표, ur=[vx,vy,ω] 몸체 좌표). origin = 출발 꼭짓점 포즈."""
        t = float(t)
        p, v_loc, yl, wl = np.zeros(2), np.zeros(2), 0.0, 0.0
        if t <= 0.0:
            pass
        elif t < self.T_path:
            i = bisect.bisect_right(self.t_start, t) - 1
            tau = t - self.t_start[i]
            if self.stop and tau >= self.edges[i].T:                    # stop: 변 i 끝 꼭짓점 i+1 에 서 있다
                p = self.corners[i + 1]
                yl = i * 0.5 * math.pi if self.tangent else 0.0
                if self.rot > 0.0:
                    s, ds = _smooth((tau - self.edges[i].T - self.dwell) / self.rot)
                    yl += 0.5 * math.pi * s
                    wl = 0.5 * math.pi * ds / self.rot
            else:
                s, v = self.edges[i].at(tau)
                p = self.corners[i] + s * self.dirs[i]
                v_loc = v * self.dirs[i]
                yl, dyds = self._yaw(i, i * self.side + s)
                wl = dyds * v
        else:
            p = self.corners[self.n]
            yl = self.yaw_end if self.tangent else 0.0
            if self.T_align > 0.0:
                s, ds = _smooth((t - self.T_path) / self.T_align)
                yl += self.dtheta * s
                wl = self.dtheta * ds / self.T_align if t < self.T_total else 0.0
        c, s_ = math.cos(yl), math.sin(yl)                               # 로컬 속도 → 몸체 (−ψ 회전)
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        Xr = np.array([x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1], wrap(a0 + yl)])
        return Xr, np.array([c * v_loc[0] + s_ * v_loc[1], -s_ * v_loc[0] + c * v_loc[1], wl])

    # ---------- 검토 ----------
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

    def edge_summary(self):
        """[(변 번호, v_s, v_p, v_e, 시간)] — 기동 로그용."""
        return [(i, e.vs, e.vp, e.ve, e.T) for i, e in enumerate(self.edges)]

    def path_xy(self, n=0, origin=(0.0, 0.0, 0.0)):
        """RViz 표시용 꼭짓점 (한 바퀴까지)."""
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        return [(x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1]) for p in self.corners[:min(self.n, 4) + 1]]


if __name__ == "__main__":
    # 검산: 속도를 위치의 유한차분과 비교 (몸체 좌표로), 변 가운데 최고속도 = √(v_c² + aL), 끝 포즈
    for kw in (dict(yaw_mode="tangent"), dict(yaw_mode="fixed"), dict(yaw_mode="tangent", accel=1.0),
               dict(yaw_mode="fixed", v_max=0.5), dict(yaw_mode="tangent", laps=2.0),
               dict(yaw_mode="tangent", corner_mode="stop", accel=1.5, v_max=1.5), dict(yaw_mode="fixed", corner_mode="stop")):
        sq = Square(**kw)
        o = (0.3, -0.2, 0.7)
        h, err, wmax = 1e-5, 0.0, 0.0
        for t in np.linspace(0.005, sq.T_total - 0.005, 6000):
            X1, u = sq.at(t, o)
            X2, _ = sq.at(t + h, o)
            Xm, _ = sq.at(t - h, o)
            d = (X2 - Xm) / (2 * h)
            d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
            c, s = math.cos(X1[2]), math.sin(X1[2])
            body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
            if np.abs(body - u).max() < 1e-2:                            # 꼭짓점 속도 계단을 걸친 표본은 뺀다
                err = max(err, float(np.abs(body - u).max()))
            wmax = max(wmax, abs(u[2]))
        Xe, ue = sq.at(sq.T_total + 1.0, o)
        mid = sq.edges[min(1, sq.n - 1)]
        print(f"{kw}: 변 {sq.n}, 주행 {sq.T_path:.2f} s + 정렬 {sq.T_align:.2f} s, 가운데 변 최고 {mid.vp:.3f} "
              f"(√(v_c²+aL) {math.sqrt(sq.vc**2 + sq.A*sq.side):.3f}), |ω| 최대 {wmax:.2f} rad/s, "
              f"끝 포즈 오차 {math.hypot(Xe[0]-o[0], Xe[1]-o[1])*1000:.2f} mm·{math.degrees(wrap(Xe[2]-o[2])):.2f}°, "
              f"속도 vs 유한차분 {err:.1e}")
