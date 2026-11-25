"""꺾은선 기준 (2026-11-25) — "돌고 간다" 명령 줄로 만든 경로. 기본은 정사각형 두 개를 꼭짓점 하나로 이은 8 자 꼴.

shape:=polyline, poly_path:="R1,L1,L2,R1,R1,R2" (torque_track.yaml). Square 와 같은 꼴 (at, peak_load, max_curvature, path_xy,
T_total, T_path, T_align, V, A) 이라 figure8_reference_node 가 고른다. 속도 ur 는 몸체 좌표 [vx, vy, ω].

명령 (쉼표로 구분, 공백 무시)
  R<길이> = 오른쪽으로 90° 돌고 <길이> m 간다, L<길이> = 왼쪽으로 90° 돌고 간다, F<길이> = 돌지 않고 간다 (우·좌·앞 도 된다).
  첫 명령의 회전은 출발 헤딩 기준이다. 중간의 F 는 앞 직선에 이어 붙인다 (꼭짓점이 아니다).
  기본 "R1,L1,L2,R1,R1,R2" (출발 헤딩 = 로컬 +x):  (0,0) → (0,−1) → (1,−1) → (1,1) → (2,1) → (2,0) → (0,0)
    정사각형 [0,1]×[−1,0] 을 왼쪽으로, [1,2]×[0,1] 을 오른쪽으로 돈다. 2 m 직선 두 개가 두 정사각형이 맞닿은 점 (1,0) 을 지난다.
    "F1,L1,L2,R1,R1,R2" 는 같은 모양을 90° 돌린 것 — 첫 직선이 출발 헤딩 방향이라 tangent 에서 출발 제자리 회전이 없다.

속도·꼭짓점·헤딩은 core/square.py 와 같은 규칙이고 square_* 값을 같이 쓴다:
  직선마다 accel 로 가속·감속 (_Edge), 최고 속도는 v_max 에서 자름. 출발·도착 0.
  corner_mode pass : 꼭짓점을 corner_speed 로 지난다. tangent 면 꼭짓점 앞뒤 yaw_blend/2 [m] 에서 헤딩을 스무스스텝으로 ±90° 돌린다.
  corner_mode stop : 꼭짓점마다 서서 dwell_s 쉬고 → (tangent 면) rot_s 동안 그 꼭짓점의 회전각만큼 제자리 회전 → dwell_s 쉬고 다음 직선.
  tangent 이고 첫 명령에 회전이 있으면 출발 때 같은 방식 (dwell_s · rot_s · dwell_s) 으로 먼저 돈다 (pass 여도 — 서 있으므로).
  끝나면 tangent 는 align_s 동안 출발 헤딩으로 돌아간다. fixed 는 헤딩을 출발 헤딩에 두고 옆걸음으로 간다 (회전 없음).
  rot_s 는 90° 기준 — 다른 각도는 각도에 비례.
"""
import bisect
import math

import numpy as np

from amr_mpc.core.square import _Edge, _smooth, wrap

_TURN = {"R": -0.5 * math.pi, "우": -0.5 * math.pi, "L": 0.5 * math.pi, "좌": 0.5 * math.pi, "F": 0.0, "앞": 0.0}


def parse_path(text):
    """"R1,L1,L2" → [(회전 rad, 길이 m), …]. 중간의 F 는 앞 직선에 붙인다."""
    cmds = []
    for raw in str(text).replace(" ", "").split(","):
        if not raw:
            continue
        head, num = raw[0].upper() if raw[0].isascii() else raw[0], raw[1:]
        if head not in _TURN:
            raise ValueError(f"poly_path 명령 {raw!r} — R<길이> | L<길이> | F<길이> (예: R1,L1,L2,R1,R1,R2)")
        try:
            L = float(num)
        except ValueError:
            raise ValueError(f"poly_path 명령 {raw!r} — 길이가 숫자가 아니다") from None
        if not L > 0.0:
            raise ValueError(f"poly_path 명령 {raw!r} — 길이는 0 보다 커야 한다")
        if cmds and _TURN[head] == 0.0:
            cmds[-1] = (cmds[-1][0], cmds[-1][1] + L)
        else:
            cmds.append((_TURN[head], L))
    if not cmds:
        raise ValueError("poly_path 가 비었다")
    return cmds


class Polyline:
    def __init__(self, path="R1,L1,L2,R1,R1,R2", corner_speed=0.1, accel=0.4, v_max=0.0,
                 yaw_mode="tangent", yaw_blend=0.3, align_s=1.5, corner_mode="pass", rot_s=1.0, dwell_s=0.2):
        if yaw_mode not in ("tangent", "fixed"):
            raise ValueError(f"polyline 의 yaw_mode 는 tangent | fixed (받은 값 {yaw_mode!r})")
        if corner_mode not in ("pass", "stop"):
            raise ValueError(f"polyline 의 corner_mode 는 pass | stop (받은 값 {corner_mode!r})")
        if accel <= 0.0 or corner_speed < 0.0 or dwell_s < 0.0:
            raise ValueError("polyline: accel > 0, corner_speed ≥ 0, dwell_s ≥ 0")
        self.text = str(path)
        cmds = parse_path(path)
        self.n = len(cmds)
        self.lengths = [c[1] for c in cmds]
        self.turns = [c[0] for c in cmds]                                # turns[0] = 출발 헤딩 기준, turns[i≥1] = 꼭짓점 i 의 회전
        self.tangent, self.yaw_mode = yaw_mode == "tangent", yaw_mode
        self.corner_mode, self.stop = corner_mode, corner_mode == "stop"
        self.A = float(accel)
        self.vc = 0.0 if self.stop else float(corner_speed)
        self.b = float(yaw_blend) if (self.tangent and not self.stop) else 0.0
        if self.tangent and not self.stop and not (0.0 < self.b <= min(self.lengths)):
            raise ValueError(f"polyline tangent pass: 0 < yaw_blend ≤ 가장 짧은 직선 ({min(self.lengths)} m) — 받은 값 {yaw_blend}")
        self.rot90 = float(rot_s)
        if self.tangent and not self.rot90 > 0.0:
            raise ValueError(f"polyline tangent: rot_s > 0 — 받은 값 {rot_s}")
        self.dwell = float(dwell_s)
        self.h = list(np.cumsum(self.turns))                             # 직선 i 의 방향 (로컬, 펼친 값)
        self.dirs = [np.array([math.cos(x), math.sin(x)]) for x in self.h]
        self.corners = [np.zeros(2)]
        for u, L in zip(self.dirs, self.lengths):
            self.corners.append(self.corners[-1] + L * u)
        rot_of = lambda a: self.rot90 * abs(a) / (0.5 * math.pi) if self.tangent else 0.0
        # 출발 제자리 회전 (tangent 이고 첫 명령에 회전이 있을 때)
        self.rot0 = rot_of(self.turns[0]) if abs(self.turns[0]) > 1e-12 else 0.0
        self.T_start = self.rot0 + 2.0 * self.dwell if self.rot0 > 0.0 else 0.0
        # 꼭짓점 i (1..n−1) 에서 서 있는 시간 (stop)
        self.rots = [0.0] + [rot_of(self.turns[i]) if self.stop else 0.0 for i in range(1, self.n)]
        self.T_corner = [0.0] + [(self.rots[i] + 2.0 * self.dwell) if self.stop else 0.0 for i in range(1, self.n)]
        self.edges = [_Edge(L, 0.0 if i == 0 else self.vc, 0.0 if i == self.n - 1 else self.vc, self.A, float(v_max))
                      for i, L in enumerate(self.lengths)]
        t0, self.t_start = self.T_start, []
        for i, e in enumerate(self.edges):                                # 직선 i 의 출발 시각 (출발 회전 포함 시계)
            t0 += self.T_corner[i]
            self.t_start.append(t0)
            t0 += e.T
        self.T_path = t0                                                  # 출발 회전 + 직선 + 꼭짓점 (정렬 회전 전)
        self.yaw_end = self.h[-1] if self.tangent else 0.0
        self.dtheta = wrap(-self.yaw_end) if self.tangent else 0.0
        self.T_align = float(align_s) if (self.tangent and align_s > 0.0 and abs(self.dtheta) > 1e-9) else 0.0
        self.T_total = self.T_path + self.T_align
        self.V = max(e.vp for e in self.edges)
        self.L = float(sum(self.lengths))
        self.S0 = list(np.cumsum([0.0] + self.lengths[:-1]))              # 직선 i 시작의 누적 호길이
        self.close_err = float(np.hypot(*self.corners[-1]))               # 끝점과 출발점 거리 (닫힌 경로면 0)

    # ---------- 헤딩 ----------
    def _yaw(self, i, s):
        """직선 i 위, 시작에서 s [m] 인 점의 (로컬 헤딩, dψ/ds)."""
        if not self.tangent:
            return 0.0, 0.0
        if self.stop:
            return self.h[i], 0.0
        hb = 0.5 * self.b
        if i >= 1 and s < hb:                                             # 꼭짓점 i 뒤쪽 절반
            x, dx = _smooth((s + hb) / self.b)
            return self.h[i - 1] + self.turns[i] * x, self.turns[i] * dx / self.b
        if i <= self.n - 2 and self.lengths[i] - s < hb:                  # 꼭짓점 i+1 앞쪽 절반
            x, dx = _smooth((s - self.lengths[i] + hb) / self.b)
            return self.h[i] + self.turns[i + 1] * x, self.turns[i + 1] * dx / self.b
        return self.h[i], 0.0

    # ---------- 기준 ----------
    def at(self, t, origin=(0.0, 0.0, 0.0)):
        """시각 t 의 (Xr=[x,y,yaw] 세계 좌표, ur=[vx,vy,ω] 몸체 좌표). origin = 출발 포즈."""
        t = float(t)
        p, v_loc, yl, wl = np.zeros(2), np.zeros(2), 0.0, 0.0
        if t <= 0.0:
            pass
        elif t < self.T_start:                                           # 출발 제자리 회전
            x, dx = _smooth((t - self.dwell) / self.rot0)
            yl, wl = self.turns[0] * x, self.turns[0] * dx / self.rot0
        elif t < self.T_path:
            i = max(0, bisect.bisect_right(self.t_start, t) - 1)
            tau = t - self.t_start[i]
            if self.stop and i < self.n - 1 and tau >= self.edges[i].T:   # 직선 i 끝 꼭짓점 i+1 에 서 있다 (stop)
                j = i + 1
                p = self.corners[j]
                yl = self.h[i] if self.tangent else 0.0
                if self.rots[j] > 0.0:
                    x, dx = _smooth((tau - self.edges[i].T - self.dwell) / self.rots[j])
                    yl += self.turns[j] * x
                    wl = self.turns[j] * dx / self.rots[j]
            else:
                s, v = self.edges[i].at(tau)
                p = self.corners[i] + s * self.dirs[i]
                v_loc = v * self.dirs[i]
                yl, dyds = self._yaw(i, s)
                wl = dyds * v
        else:
            p = self.corners[self.n]
            yl = self.yaw_end
            if self.T_align > 0.0:
                x, dx = _smooth((t - self.T_path) / self.T_align)
                yl += self.dtheta * x
                wl = self.dtheta * dx / self.T_align if t < self.T_total else 0.0
        c, s_ = math.cos(yl), math.sin(yl)
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        Xr = np.array([x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1], wrap(a0 + yl)])
        return Xr, np.array([c * v_loc[0] + s_ * v_loc[1], -s_ * v_loc[0] + c * v_loc[1], wl])

    # ---------- 검토 ----------
    def peak_load(self, k_geom, n=8000):
        """(주행, 정렬 회전) L1 부하 |vx|+|vy|+k|ω| 최대 — Square.peak_load 와 같은 뜻."""
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
        return math.inf

    def edge_summary(self):
        """[(직선 번호, 회전 °, 길이, v_s, v_p, v_e, 시간)] — 기동 로그용."""
        return [(i, math.degrees(self.turns[i]), L, e.vs, e.vp, e.ve, e.T) for i, (L, e) in enumerate(zip(self.lengths, self.edges))]

    def path_xy(self, n=0, origin=(0.0, 0.0, 0.0), step=0.02):
        """RViz 표시·contouring 용 점 — 꼭짓점을 지나고 직선 위는 step [m] 간격 (꼭짓점만 주면 기록기의 10 배 보간이 2 m 직선에서 0.2 m 간격이라
        가장 가까운 점까지 거리가 0.1 m 까지 부풀었다)."""
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        pts = [self.corners[0]]
        for i, L in enumerate(self.lengths):
            k = max(1, int(math.ceil(L / step)))
            pts += [self.corners[i] + (j / k) * L * self.dirs[i] for j in range(1, k + 1)]
        return [(x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1]) for p in pts]


if __name__ == "__main__":
    # 검산: 꼭짓점, 닫힘, 속도 vs 위치 유한차분 (몸체 좌표), 끝 포즈
    for kw in (dict(), dict(corner_mode="stop"), dict(yaw_mode="fixed"), dict(path="F1,L1,L2,R1,R1,R2"),
               dict(corner_mode="stop", accel=1.5, v_max=1.0), dict(yaw_mode="fixed", corner_mode="stop")):
        pl = Polyline(**kw)
        o = (0.3, -0.2, 0.7)
        h, err = 1e-5, 0.0
        for t in np.linspace(0.005, pl.T_total - 0.005, 8000):
            X1, u = pl.at(t, o)
            X2, _ = pl.at(t + h, o)
            Xm, _ = pl.at(t - h, o)
            d = (X2 - Xm) / (2 * h)
            d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
            c, s = math.cos(X1[2]), math.sin(X1[2])
            body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
            if np.abs(body - u).max() < 1e-2:
                err = max(err, float(np.abs(body - u).max()))
        Xe, _ = pl.at(pl.T_total + 1.0, o)
        print(f"{kw}: 꼭짓점 {[tuple(np.round(c, 3)) for c in pl.corners]}, 출발 회전 {pl.T_start:.2f} s, 주행 {pl.T_path:.2f} s + 정렬 {pl.T_align:.2f} s, "
              f"끝 포즈 오차 {math.hypot(Xe[0]-o[0], Xe[1]-o[1])*1000:.2f} mm·{math.degrees(wrap(Xe[2]-o[2])):.2f}°, 속도 vs 유한차분 {err:.1e}")
