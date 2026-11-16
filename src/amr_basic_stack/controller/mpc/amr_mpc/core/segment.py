"""5-3 단일 축 기준 — 직선(앞·옆·대각) 이동과 제자리 회전 (2026-11-16).

Figure8 과 같은 꼴(at, peak_load, max_curvature, path_xy, T_total, V, A)이라 figure8_reference_node 가 shape 인자로 고른다.
  line   : 출발 포즈의 몸체 방향 dir_deg(0 = 앞, 90 = 왼쪽, 45 = 앞왼쪽 대각)로 distance [m]. 헤딩은 그대로.
           속도는 사다리꼴(최고 speed, 가감속 accel). 거리가 짧으면 삼각형.
  rotate : 제자리에서 angle_deg 만큼. 각속도 사다리꼴(최고 yaw_rate, 각가속 yaw_accel).
  out_and_back = True 면 갔다가 pause_s 쉬고 같은 길로 돌아온다 (공간을 덜 쓰고 끝 위치 = 출발 위치).
속도 ur 는 몸체 좌표 [vx, vy, ω] — Figure8.at 과 같다.
"""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def trapezoid(t, L, V, A):
    """거리 L 을 최고 V, 가속 A 사다리꼴로 갈 때 시각 t 의 (이동량, 속도). 반환 T 는 전체 시간."""
    if L <= 0.0:
        return 0.0, 0.0
    ta = V / A
    if A * ta * ta > L:                       # 삼각형 — 최고 속도에 못 닿는다
        ta = math.sqrt(L / A)
        V = A * ta
    tc = (L - A * ta * ta) / V
    if t <= 0.0:
        return 0.0, 0.0
    if t < ta:
        return 0.5 * A * t * t, A * t
    if t < ta + tc:
        return 0.5 * A * ta * ta + V * (t - ta), V
    td = t - ta - tc
    if td < ta:
        return L - 0.5 * A * (ta - td) ** 2, A * (ta - td)
    return L, 0.0


def trapezoid_time(L, V, A):
    if L <= 0.0:
        return 0.0
    ta = V / A
    if A * ta * ta > L:
        return 2.0 * math.sqrt(L / A)
    return 2.0 * ta + (L - A * ta * ta) / V


class Segment:
    def __init__(self, shape="line", distance=0.5, dir_deg=0.0, speed=0.2, accel=0.4,
                 angle_deg=90.0, yaw_rate=0.8, yaw_accel=1.6, out_and_back=True, pause_s=1.0):
        if shape not in ("line", "rotate"):
            raise ValueError(f"shape={shape!r} — line 또는 rotate")
        self.shape = shape
        if shape == "line":
            if distance <= 0 or speed <= 0 or accel <= 0:
                raise ValueError("line: distance·speed·accel > 0")
            self.L, self.V, self.A = float(distance), float(speed), float(accel)
            d = math.radians(float(dir_deg))
            self.u = np.array([math.cos(d), math.sin(d)])     # 출발 몸체 좌표의 이동 방향
        else:
            if angle_deg == 0 or yaw_rate <= 0 or yaw_accel <= 0:
                raise ValueError("rotate: angle_deg ≠ 0, yaw_rate·yaw_accel > 0")
            self.L, self.V, self.A = abs(math.radians(float(angle_deg))), float(yaw_rate), float(yaw_accel)
            self.sign = 1.0 if angle_deg > 0 else -1.0
        self.back = bool(out_and_back)
        self.pause = max(0.0, float(pause_s))
        self.T1 = trapezoid_time(self.L, self.V, self.A)
        self.T_total = self.T1 + ((self.pause + self.T1) if self.back else 0.0)

    def _s(self, t):
        """출발에서의 이동량(거리 또는 각)과 그 변화율 (갔다 오면 돌아올 때 음수)."""
        if t < self.T1 or not self.back:
            return trapezoid(t, self.L, self.V, self.A)
        if t < self.T1 + self.pause:
            return self.L, 0.0
        s, v = trapezoid(t - self.T1 - self.pause, self.L, self.V, self.A)
        return self.L - s, -v

    def at(self, t, origin=(0.0, 0.0, 0.0)):
        s, v = self._s(float(t))
        x0, y0, a0 = origin
        if self.shape == "line":
            c0, s0 = math.cos(a0), math.sin(a0)
            px, py = s * self.u
            Xr = np.array([x0 + c0 * px - s0 * py, y0 + s0 * px + c0 * py, wrap(a0)])
            return Xr, np.array([v * self.u[0], v * self.u[1], 0.0])     # 헤딩 고정 → 몸체 = 출발 몸체 좌표
        return np.array([x0, y0, wrap(a0 + self.sign * s)]), np.array([0.0, 0.0, self.sign * v])

    def peak_load(self, k_geom, n=0):
        """(주행, 회전) L1 부하 |vx|+|vy|+k|ω| 최대 — Figure8.peak_load 와 같은 뜻."""
        if self.shape == "line":
            return self.V_peak() * (abs(self.u[0]) + abs(self.u[1])), 0.0
        return 0.0, k_geom * self.V_peak()

    def V_peak(self):
        ta = self.V / self.A
        return self.V if self.A * ta * ta <= self.L else math.sqrt(self.L * self.A)

    def max_curvature(self):
        return 0.0

    def path_xy(self, n=50, origin=(0.0, 0.0, 0.0)):
        """RViz 표시용. 회전은 출발 위치 한 점."""
        if self.shape == "rotate":
            return [(origin[0], origin[1])]
        return [tuple(self.at(t, origin)[0][:2]) for t in np.linspace(0.0, self.T1, n)]


if __name__ == "__main__":
    # 검산: at() 의 속도를 위치의 유한차분과 비교 (몸체 좌표로 돌려서), 끝 위치·전체 시간
    for kw in (dict(shape="line", distance=0.5, dir_deg=0), dict(shape="line", distance=0.3, dir_deg=90),
               dict(shape="line", distance=0.4, dir_deg=45, speed=0.3), dict(shape="line", distance=0.05),
               dict(shape="rotate", angle_deg=90), dict(shape="rotate", angle_deg=-180, yaw_rate=1.0)):
        sg = Segment(**kw)
        o = (0.3, -0.2, 0.7)
        h, err = 1e-4, 0.0
        for t in np.linspace(0.01, sg.T_total - 0.01, 400):
            X1, u = sg.at(t, o)
            X2, _ = sg.at(t + h, o)
            Xm, _ = sg.at(t - h, o)
            d = (X2 - Xm) / (2 * h)
            d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
            c, s = math.cos(X1[2]), math.sin(X1[2])
            body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
            err = max(err, float(np.abs(body - u).max()))
        Xe, ue = sg.at(sg.T_total, o)
        Xh, _ = sg.at(sg.T1, o)
        far = math.hypot(Xh[0] - o[0], Xh[1] - o[1]) if sg.shape == "line" else math.degrees(wrap(Xh[2] - o[2]))
        print(f"{kw}: T1 {sg.T1:.2f} s, 전체 {sg.T_total:.2f} s, 끝까지 {far:.3f}, 끝 위치 오차 "
              f"{math.hypot(Xe[0]-o[0], Xe[1]-o[1])*1000:.2f} mm·{math.degrees(wrap(Xe[2]-o[2])):.2f}°, "
              f"속도 vs 유한차분 최대 차 {err:.2e}, 최고 {sg.V_peak():.3f}")
