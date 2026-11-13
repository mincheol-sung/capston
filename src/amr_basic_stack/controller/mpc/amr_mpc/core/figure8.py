#!/usr/bin/env python3
# 8자 기준궤적 — Gerono 렘니스케이트(lemniscate of Gerono, "figure-eight curve")
# ---------------------------------------------------------------
#   p(s) = [ (W/2)·sin s ,  (H/2)·sin 2s ]        s ∈ [0, 2π)
#
#   · s=0 과 s=π 에서 원점을 지난다 (교차점).  기동 포즈가 그 교차점이다.
#   · 경계상자는 정확히 W × H, 기동 포즈를 중심으로 ±W/2, ±H/2.  → 방 크기 검토가 쉽다.
#   · 회전지수(rotation index) 0 — 한 바퀴 돌면 접선각이 제자리로 온다.  두 잎의 회전이 상쇄되기
#     때문이고, 원(회전지수 1, 접선각 +2π)과 다른 점이다.  그래서 "제자리 복귀"가 위치·헤딩
#     모두 성립한다 (__main__ 자기검사가 수치로 확인한다).
#
# 왜 Bernoulli 렘니스케이트(r²=a²cos2θ)가 아닌가: 그쪽은 경계상자가 2a × a 로 가로세로가
# 2:1 에 묶인다.  Gerono 는 W, H 를 따로 준다 — 정사각 공간에 8자를 꽉 채울 수 있다.
#
# 시간 재매개화 (등속 호길이)
#   s 는 호길이가 아니다 (|dp/ds| 가 s 에 따라 변한다).  누적 사다리꼴 적분으로 σ(s) 표를 만들고
#   역보간해 s(σ) 를 얻는다.  σ(t) 는 사다리꼴 속도 프로파일 (0 → V 가속 a, 순항, V → 0 감속).
#   호길이가 짧아 순항 구간이 안 나오면 삼각 프로파일 (V ← √(a·L)) 로 자동 강등.
#   lap_speeds=[V₁, V₂, …] (2026-11-11, 토크 MPC 실험): 랩마다 순항 속도를 바꾼다. 랩 경계(교차점)를 지나면서
#   가감속 a 로 다음 랩 속도로 바꾸고, 끝에서는 정확히 멈추게 감속한다. 1 ms 격자로 미리 적분한 표를 보간한다.
#   주지 않으면 위 사다리꼴 그대로 (기존 사용처 동작 불변).
#
# 헤딩 (yaw_mode)
#   "tangent"  접선 추종.  경로 시작 접선각 θ(0)=atan2(H, W/2) 만큼 기동 헤딩과 어긋나므로,
#              앞뒤에 align_s 초짜리 제자리 회전을 붙인다 (3τ²−2τ³ 스무스스텝: 양 끝 ω=0).
#              [정렬 회전] → [8자 주행] → [역정렬 회전] → [정지 유지].
#   "fixed"    헤딩 고정.  메카넘이므로 몸체를 안 돌리고 게걸음으로 8자를 그린다.  정렬 구간이
#              필요 없고 ω≡0 이라 부하가 |vx|+|vy| 뿐이다.  헤딩 오차 없이 위치 추종만 보고 싶을 때.
#
# 출력 규약은 amr_interfaces/PlanarReference 와 같다: 포즈는 세계(odom)프레임, 속도는 몸체프레임.
#
#   python3 figure8.py       자기검사 (닫힘오차·유한차분 검산·부하·곡률)
# ---------------------------------------------------------------
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ramp_elapsed(tau, rate0, ramp_s):
    """기준 시간 배속 r(τ) = min(1, rate0 + τ/ramp_s) 를 0..τ 로 적분한 값 (기준 시간이 얼마나 흘렀나).
    정지 뒤 다시 무장할 때 기준 시간을 0 배속에서 1 배속까지 ramp_s 초에 걸쳐 올린다 — 순항 구간에서 멈췄다가
    재개해도 기준 속도가 0 → V 로 한 번에 뛰지 않는다 (가속도 ≤ V/ramp_s)."""
    if ramp_s <= 0.0 or rate0 >= 1.0:
        return float(tau)
    t1 = (1.0 - rate0) * ramp_s                  # 1 배속에 닿는 시각
    if tau <= t1:
        return rate0 * tau + tau * tau / (2.0 * ramp_s)
    return rate0 * t1 + t1 * t1 / (2.0 * ramp_s) + (tau - t1)


def lead_in_duration(start, goal, lead_speed, lead_yaw_rate, min_s=0.0):
    """진입 구간 길이 [s]: 평균 속도·회전속도가 상한을 넘지 않게 (스무스스텝 최고값은 평균의 1.5 배)."""
    d = math.hypot(goal[0] - start[0], goal[1] - start[1])
    a = abs(wrap(goal[2] - start[2]))
    return max(float(min_s), d / max(lead_speed, 1e-6), a / max(lead_yaw_rate, 1e-6))


def lead_in(t, T, start, goal):
    """진입 구간 기준: t ∈ [0, T] 동안 start → goal 포즈를 스무스스텝(3τ²−2τ³)으로 잇는다. 양 끝 속도 0.
    반환 (Xr=[x,y,yaw] 세계프레임, ur=[vx,vy,yaw_rate] 몸체프레임) — Figure8.at() 과 같은 규약."""
    start = np.asarray(start, float); goal = np.asarray(goal, float)
    dyaw = wrap(goal[2] - start[2])
    if T <= 0.0:
        return np.array([goal[0], goal[1], wrap(goal[2])]), np.zeros(3)
    tau = min(max(t / T, 0.0), 1.0)
    s = tau * tau * (3.0 - 2.0 * tau)
    sd = 6.0 * (tau - tau * tau) / T
    yaw = start[2] + dyaw * s
    vx_w, vy_w = (goal[0] - start[0]) * sd, (goal[1] - start[1]) * sd
    c, sn = math.cos(yaw), math.sin(yaw)
    Xr = np.array([start[0] + (goal[0] - start[0]) * s, start[1] + (goal[1] - start[1]) * s, wrap(yaw)])
    return Xr, np.array([c * vx_w + sn * vy_w, -sn * vx_w + c * vy_w, dyaw * sd])


def ramp_rate(tau, rate0, ramp_s):
    """τ 초 뒤의 배속."""
    if ramp_s <= 0.0:
        return 1.0
    return min(1.0, rate0 + tau / ramp_s)


class Figure8:
    def __init__(self, width=1.4, height=0.8, speed=0.25, accel=0.4,
                 laps=1.0, yaw_mode="tangent", align_s=1.5, samples=8000, lap_speeds=None):
        if width <= 0.0 or height <= 0.0:
            raise ValueError("width, height 는 양수")
        if speed <= 0.0 or accel <= 0.0:
            raise ValueError("speed, accel 은 양수")
        self.lap_speeds = [float(v) for v in lap_speeds] if lap_speeds else None
        if self.lap_speeds is not None:
            if min(self.lap_speeds) <= 0.0:
                raise ValueError("lap_speeds 는 모두 양수")
            laps = float(len(self.lap_speeds))       # 랩 수 = 목록 길이
            speed = max(self.lap_speeds)
        if laps <= 0.0:
            raise ValueError("laps 는 양수")
        if yaw_mode not in ("tangent", "fixed"):
            raise ValueError(f"yaw_mode 는 tangent | fixed (받은 값 {yaw_mode!r})")
        self.W, self.H, self.laps = float(width), float(height), float(laps)
        self.yaw_mode, self.tangent = yaw_mode, (yaw_mode == "tangent")

        # ---- 호길이 표 σ(s).  |dp/ds| = √((W/2)²cos²s + H²cos²2s) 는 0 이 되는 s 가 없다
        #      (cos s=0 → cos 2s=−1) — 첨점 없는 정칙곡선이라 등속 재매개화가 성립한다.
        n = max(400, int(samples * self.laps)) + 1
        s = np.linspace(0.0, 2.0 * math.pi * self.laps, n)
        sp = np.hypot(0.5 * self.W * np.cos(s), self.H * np.cos(2.0 * s))
        self.s_tab = s
        self.arc_tab = np.concatenate(([0.0], np.cumsum(0.5 * (sp[1:] + sp[:-1]) * np.diff(s))))
        self.L = float(self.arc_tab[-1])
        self.th_tab = np.unwrap(np.arctan2(self.H * np.cos(2.0 * s), 0.5 * self.W * np.cos(s)))

        # ---- 사다리꼴 속도 프로파일 (호길이 축)
        V, A = float(speed), float(accel)
        if V * V / A > self.L:                       # 순항 구간 없음 → 삼각 프로파일
            V = math.sqrt(A * self.L)
        self.V, self.A = V, A
        self.t_acc = V / A
        self.t_cru = (self.L - V * V / A) / V
        self.T_path = 2.0 * self.t_acc + self.t_cru
        if self.lap_speeds is not None:
            self._build_lap_profile()

        # ---- 제자리 정렬 회전 (tangent 모드만)
        self.dtheta = float(self.th_tab[0]) if self.tangent else 0.0
        self.T_align = float(align_s) if (self.tangent and abs(self.dtheta) > 1e-6) else 0.0
        self.T_total = 2.0 * self.T_align + self.T_path

    # ---------- 곡선 ----------
    def _point(self, s):
        return np.array([0.5 * self.W * math.sin(s), 0.5 * self.H * math.sin(2.0 * s)])

    def _kappa(self, s):
        """부호 있는 곡률 κ = (x'y'' − y'x'') / |p'|³.   ω = κ·v."""
        xp, yp = 0.5 * self.W * math.cos(s), self.H * math.cos(2.0 * s)
        xpp, ypp = -0.5 * self.W * math.sin(s), -2.0 * self.H * math.sin(2.0 * s)
        return (xp * ypp - yp * xpp) / (xp * xp + yp * yp) ** 1.5

    def _build_lap_profile(self, h=1e-3):
        """랩별 목표 속도를 가감속 A 로 따라가고 끝에서 정확히 서는 σ(t), v(t) 표."""
        L_lap, A = self.L / len(self.lap_speeds), self.A
        ts, sig, vel = [0.0], [0.0], [0.0]
        t = s = v = 0.0
        while s < self.L - 1e-9:
            k = min(int(s / L_lap), len(self.lap_speeds) - 1)
            target = self.lap_speeds[k]
            v_brake = math.sqrt(max(0.0, 2.0 * A * (self.L - s)))   # 이 속도를 넘으면 끝에서 못 선다
            v_new = min(v + A * h, target, v_brake) if v <= target else max(v - A * h, target)
            v_new = min(v_new, v_brake)
            if v_new <= 1e-9 and s > 0.5 * self.L:
                break
            s = min(self.L, s + 0.5 * (v + v_new) * h)
            v, t = max(v_new, 0.0), t + h
            ts.append(t); sig.append(s); vel.append(v)
        sig[-1], vel[-1] = self.L, 0.0
        self._lap_t, self._lap_s, self._lap_v = np.array(ts), np.array(sig), np.array(vel)
        self.T_path = float(ts[-1])

    def lap_profile_summary(self):
        """[(랩 번호, 목표 속도, 실제 최고 속도, 랩 시작 시각)]"""
        if self.lap_speeds is None:
            return []
        L_lap, out = self.L / len(self.lap_speeds), []
        for k, V in enumerate(self.lap_speeds):
            m = (self._lap_s >= k * L_lap) & (self._lap_s < (k + 1) * L_lap)
            out.append((k + 1, V, float(self._lap_v[m].max()), float(self._lap_t[m][0])))
        return out

    def _profile(self, t):
        """주행 구간 경과시간 t → (호길이 σ, 접선속도 v)."""
        if self.lap_speeds is not None:
            if t <= 0.0:
                return 0.0, 0.0
            if t >= self.T_path:
                return self.L, 0.0
            return float(np.interp(t, self._lap_t, self._lap_s)), float(np.interp(t, self._lap_t, self._lap_v))
        V, A, ta = self.V, self.A, self.t_acc
        if t <= 0.0:
            return 0.0, 0.0
        if t < ta:
            return 0.5 * A * t * t, A * t
        if t < ta + self.t_cru:
            return 0.5 * V * ta + V * (t - ta), V
        if t < self.T_path:
            td = self.T_path - t
            return self.L - 0.5 * A * td * td, A * td
        return self.L, 0.0

    def _spin(self, tau):
        """정렬 회전 스무스스텝. τ∈[0,1] → (각 α, 각속도 dα/dt).  양 끝에서 α'=0."""
        tau = min(max(tau, 0.0), 1.0)
        return (self.dtheta * tau * tau * (3.0 - 2.0 * tau),
                self.dtheta * 6.0 * (tau - tau * tau) / self.T_align)

    # ---------- 기준 ----------
    def at(self, t, origin=(0.0, 0.0, 0.0)):
        """시각 t 의 (Xr=[x,y,yaw] 세계프레임, ur=[vx,vy,yaw_rate] 몸체프레임).
        origin=[x0,y0,yaw0] 는 기동 포즈 — 8자의 교차점이자 로컬프레임 원점."""
        t = float(t)
        v = th = 0.0
        p = np.zeros(2)
        if self.T_align > 0.0 and t < self.T_align:                       # 1) 정렬 회전
            yl, wl = self._spin(t / self.T_align)
        elif t < self.T_align + self.T_path:                              # 2) 8자 주행
            sig, v = self._profile(t - self.T_align)
            s = float(np.interp(sig, self.arc_tab, self.s_tab))
            p = self._point(s)
            th = float(np.interp(s, self.s_tab, self.th_tab))
            yl, wl = (th, self._kappa(s) * v) if self.tangent else (0.0, 0.0)
        elif self.T_align > 0.0 and t < self.T_total:                     # 3) 역정렬 회전
            a, w = self._spin(1.0 - (t - self.T_align - self.T_path) / self.T_align)
            yl, wl = a, -w
        else:                                                             # 4) 기동 포즈 유지
            yl = wl = 0.0

        cy, sy = math.cos(yl), math.sin(yl)                # 로컬프레임 속도 → 몸체프레임 (−yl 회전)
        vx, vy = v * math.cos(th), v * math.sin(th)
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        Xr = np.array([x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1], wrap(a0 + yl)])
        return Xr, np.array([cy * vx + sy * vy, -sy * vx + cy * vy, wl])

    # ---------- 검토 ----------
    def peak_load(self, k_geom, n=2000):
        """L1 부하 |vx|+|vy|+k|ω| 의 최대값을 (주행, 정렬회전) 으로 나눠 돌려준다.
        주행 쪽은 speed 에 선형, 정렬 쪽은 align_s 에 반비례한다 — 초과 시 고칠 손잡이가 다르다."""
        run = spin = 0.0
        for t in np.linspace(0.0, self.T_total, n):
            _, u = self.at(t)
            load = abs(u[0]) + abs(u[1]) + k_geom * abs(u[2])
            if self.T_align <= t < self.T_align + self.T_path:
                run = max(run, load)
            else:
                spin = max(spin, load)
        return run, spin

    def max_curvature(self):
        return max(abs(self._kappa(s)) for s in np.linspace(0.0, 2.0 * math.pi, 2001))

    def path_xy(self, n=400, origin=(0.0, 0.0, 0.0)):
        """RViz 표시용 한 바퀴 경로점 (세계프레임)."""
        x0, y0, a0 = origin
        c0, s0 = math.cos(a0), math.sin(a0)
        out = []
        for s in np.linspace(0.0, 2.0 * math.pi, n):
            p = self._point(s)
            out.append((x0 + c0 * p[0] - s0 * p[1], y0 + s0 * p[0] + c0 * p[1]))
        return out


# ---------------------------------------------------------------
if __name__ == "__main__":
    try:
        from .mpc import K_GEOM, V_LIMIT
    except ImportError:
        from mpc import K_GEOM, V_LIMIT

    for mode in ("tangent", "fixed"):
        f = Figure8(width=1.4, height=0.8, speed=0.25, accel=0.40, yaw_mode=mode, align_s=1.5)
        print(f"\n=== yaw_mode={mode}  W×H {f.W}×{f.H} m  V {f.V:.2f} m/s  a {f.A} m/s² ===")
        print(f"호길이 L = {f.L:.4f} m    정렬 {f.T_align:.2f}s ×2 + 주행 {f.T_path:.2f}s "
              f"= 총 {f.T_total:.2f} s")
        print(f"시작 접선각 θ(0) = {math.degrees(f.th_tab[0]):.2f}°   "
              f"한 바퀴 접선각 변화 = {math.degrees(f.th_tab[-1] - f.th_tab[0]):+.3e}°  (회전지수 0 확인)")

        # 닫힘: 종료 시각의 기준이 기동 포즈로 돌아오는가
        X0, _ = f.at(0.0)
        Xe, ue = f.at(f.T_total + 0.5)
        print(f"닫힘오차  위치 {np.hypot(*(Xe[:2] - X0[:2]))*1000:.3e} mm   "
              f"헤딩 {math.degrees(wrap(Xe[2]-X0[2])):.3e}°   종료속도 {np.abs(ue).max():.1e}")

        # 유한차분 검산: 발행하는 몸체속도가 발행하는 포즈의 도함수와 맞는가 (독립 경로)
        h, err_v, err_w = 1e-4, 0.0, 0.0
        for t in np.linspace(h, f.T_total - h, 4000):
            Xm, _ = f.at(t - h); Xp, _ = f.at(t + h); X, u = f.at(t)
            dxy = (Xp[:2] - Xm[:2]) / (2 * h)
            c, s = math.cos(X[2]), math.sin(X[2])
            err_v = max(err_v, np.abs(np.array([c*u[0] - s*u[1], s*u[0] + c*u[1]]) - dxy).max())
            err_w = max(err_w, abs(u[2] - wrap(Xp[2] - Xm[2]) / (2 * h)))
        print(f"유한차분  |R(yaw)·u_xy − dp/dt| max {err_v:.2e} m/s    |ω − dyaw/dt| max {err_w:.2e} rad/s")

        run, spin = f.peak_load(K_GEOM)
        print(f"최대곡률 κ = {f.max_curvature():.3f} 1/m (R_min {1/f.max_curvature():.3f} m)")
        print(f"부하 최대  주행 {run:.3f} = V_LIMIT 의 {run/V_LIMIT*100:.1f} %   "
              f"정렬회전 {spin:.3f} = {spin/V_LIMIT*100:.1f} %   (예산 65 % = {0.65*V_LIMIT:.3f})")
