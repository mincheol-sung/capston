#!/usr/bin/env python3
"""바퀴축 둘레 모멘트 평형 — 분석용 함수 모음 (2026-11-24, 바퀴 회전자 식별 지시서 W2·W3). ROS 에 의존하지 않는다.

  식-W1 (바퀴축, 감속 뒤 기준. 전진 회전 +, τ 는 전진 방향 구동이 +):
      J_w·ω̇ = τ − τ_c·sgn(ω) − b_v·ω − τ_g          τ_g = (r/√2)·f   (견인 토크 = 바닥에 전달되는 토크)
  바퀴를 띄우면 τ_g = 0  →  식-W2:   τ = J_w·ω̇ + τ_c·sgn(ω) + b_v·ω      (J_w, τ_c, b_v 에 선형 → 최소제곱)
  견인 토크 (W3, 힘 센서가 없어 식-W1 을 거꾸로 쓴다):   τ_g = τ − τ_fric(ω) − J_w·ω̇

  쓰는 곳: tools/wheel_param_id.py (W2 식별), 회의 후속 지시서 T6 (접지력 한계 초과 비율 — ω̇ 자리에 무슬립 ω̇_nom) ·
           T7 (μ 추출 — 실제 ω̇). 제어 노드는 이 파일을 쓰지 않는다 (torque_mpc·torque_output 의 파라미터도 여기서 바꾸지 않는다).

  전처리 (W2 1항): ω 와 τ 에 같은 영위상 저역통과 (2차 Butterworth, filtfilt — 4차 감쇠, 위상 지연 0) 를 걸고 ω 를 중앙 차분.
    차단 주파수 10 Hz 에서 시작 [구현 선택]. 표본 간격이 고르지 않으면 (벤치 CSV 중앙 10.2 ms) 먼저 고른 격자로 옮긴다 (resample_uniform).
  한 과도 구간만으로는 J_w 와 b_v 가 갈리지 않는다 (ω̇ = −(b_v/J_w)(ω − ω_ss) 라 둘의 비만 보인다). 가르는 것은
    토크 계단 (Δτ = J_w·Δω̇), 서로 다른 일정 속도 (b_v = Δτ/Δω_ss), 자유 감속 (τ ≈ 0 에서 J_w·ω̇ = −마찰) 이다 — 그래서
    회귀 (fit_wheel_params) 와 따로 일정 속도 직선 (steady_state_line)·자유 감속 (coast_inertia) 으로 교차 확인한다.
  속도 잡음: ω̇ 는 ω 를 미분해 잡음이 크다. 지시서 식 그대로의 회귀 (ω̇ 가 설명 변수) 는 그 잡음만큼 J_w 를 작게 낸다 →
    ω̇ 를 왼쪽에 둔 회귀 (fit_wheel_params_accel) 를 같이 낸다. 어느 쪽을 대표값으로 쓸지는 사용자가 정한다.
"""
import math

import numpy as np


# ── 전처리 ───────────────────────────────────────────────────────
def resample_uniform(t, *cols, dt=None, max_gap=2.5):
    """고르지 않은 시각 t 의 열들을 간격 dt (기본 = 중앙 간격) 격자로 선형 보간한다.
    반환 (t_u, [열…], ok). ok[i] 는 그 격자점이 원래 표본 간격 ≤ max_gap·dt 인 구간 안에 있나 (응답 누락 구간이면 False)."""
    t = np.asarray(t, float)
    if len(t) < 3 or np.any(np.diff(t) <= 0.0):
        raise ValueError("시각은 3 개 이상, 늘어나는 순서여야 한다")
    if dt is None:
        dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    tu = tu[tu <= t[-1] + 1e-12]
    out = [np.interp(tu, t, np.asarray(c, float)) for c in cols]
    k = np.clip(np.searchsorted(t, tu, side="right") - 1, 0, len(t) - 2)
    ok = (t[k + 1] - t[k]) <= max_gap * dt
    return tu, out, ok


def zero_phase_lowpass(x, fs, fc, order=2):
    """영위상 저역통과 (Butterworth order 차를 앞뒤로 — filtfilt). fc ≤ 0 이거나 fc ≥ fs/2 면 그대로 돌려준다."""
    x = np.asarray(x, float)
    if not (0.0 < fc < 0.5 * fs):
        return x.copy()
    from scipy.signal import butter, filtfilt        # 분석 때만 쓴다 — 노드 실행에는 scipy 가 필요 없다
    b, a = butter(order, fc / (0.5 * fs))
    return filtfilt(b, a, x)


def central_difference(x, dt):
    """고른 간격 dt 의 중앙 차분 (양 끝은 한쪽 차분)."""
    return np.gradient(np.asarray(x, float), dt)


# ── 마찰 모델 (식-W1 의 τ_fric(ω) — ω 와 같은 부호) ───────────────
class CoulombViscous:
    """τ_fric = τ_c·sgn(ω) + b_v·ω.  smooth > 0 이면 sgn 자리에 tanh(ω/smooth) (torque_output·설계 식-45 의 근사)."""

    def __init__(self, tau_c, b_v, smooth=0.0):
        self.tau_c, self.b_v, self.smooth = np.asarray(tau_c, float), np.asarray(b_v, float), float(smooth)

    def __call__(self, omega):
        omega = np.asarray(omega, float)
        s = np.tanh(omega / self.smooth) if self.smooth > 0.0 else np.sign(omega)
        return self.tau_c * s + self.b_v * omega


class ResistanceTable:
    """바닥 등속 직진에서 잰 주행 저항 τ_res(|ω|) 표 (회의 후속 지시서 T7 1단계). τ_fric = sgn(ω)·interp(|ω|).
    표 밖은 끝 값 유지. 이 모델을 쓰면 τ_c·b_v 를 따로 더하지 않는다 — 표가 이미 구동계 마찰 + 바닥 구름 손실이다 (이중 차감 금지)."""

    def __init__(self, omega_abs, tau_res):
        w, tr = np.asarray(omega_abs, float), np.asarray(tau_res, float)
        if w.ndim != 1 or w.shape != tr.shape or len(w) < 1 or np.any(np.diff(w) <= 0.0) or w[0] < 0.0:
            raise ValueError("ResistanceTable: |ω| 는 0 이상의 늘어나는 1 차원 격자, τ_res 는 같은 길이")
        self.w, self.tr = w, tr

    def __call__(self, omega):
        omega = np.asarray(omega, float)
        return np.sign(omega) * np.interp(np.abs(omega), self.w, self.tr)


# ── W3: 견인 토크 ────────────────────────────────────────────────
def traction_torque(tau, omega, omega_dot, J_w, friction):
    """τ_g = τ − τ_fric(ω) − J_w·ω̇   (식-W1 을 τ_g 에 대해 푼 것).
    tau       모터 토크 [N·m] — 명령 (실제 나간 값) 또는 피드백. 부르는 쪽이 고른다
    omega     바퀴 각속도 [rad/s] (마찰의 부호·크기를 정한다)
    omega_dot 바퀴 각가속도 [rad/s²] — 실제 값 (T7 μ 추출) 또는 무슬립 값 ω̇_nom (T6 요구 견인 토크). 부르는 쪽이 고른다
    friction  CoulombViscous 또는 ResistanceTable (또는 ω → τ_fric 인 함수). 둘을 같이 빼지 않는다
    모양은 서로 broadcast 되면 된다 (예: (N, 4) 와 바퀴별 계수 (4,))."""
    return np.asarray(tau, float) - friction(omega) - np.asarray(J_w, float) * np.asarray(omega_dot, float)


def body_force_consistency(tau_g, r, mass, a_x):
    """직진 구간 정합성 (W3): F_x = Σ_i τ_g,i / r 와 m·a_x 를 견준다.
    tau_g (N, 4) 또는 (4,), a_x (N,) 또는 스칼라 [m/s²] (몸체 가속도 — 같은 필터를 건 추정기 속도의 미분).
    반환 dict(F_x, m_a, diff, mean_diff, slope) — slope 는 diff 를 a_x 에 대해 맞춘 기울기 [kg]
    (가속도에 비례해 어긋나면 J_w·m 오차, 일정하게 어긋나면 모델 밖 힘·추정기 바이어스)."""
    tg = np.atleast_2d(np.asarray(tau_g, float))
    ax = np.atleast_1d(np.asarray(a_x, float))
    Fx = tg.sum(axis=1) / r
    ma = mass * ax
    diff = Fx - ma
    slope = float(np.polyfit(ax, diff, 1)[0]) if len(ax) >= 2 and np.ptp(ax) > 1e-9 else math.nan
    return {"F_x": Fx, "m_a": ma, "diff": diff, "mean_diff": float(np.mean(diff)), "slope": slope}


# ── W2: 식별 ─────────────────────────────────────────────────────
def fit_wheel_params(tau, omega, omega_dot, mask=None):
    """식-W2 회귀:  [ω̇  sgn(ω)  ω]·[J_w  τ_c  b_v]ᵀ = τ  (선형 최소제곱).
    mask (bool) 인 표본만 쓴다. 반환 dict(J_w, tau_c, b_v, r2, rms, n, cond) — cond 는 열을 정규화한 회귀 행렬의 조건수
    (크면 J_w·b_v 가 잘 안 갈린다: 일정 속도 수준이 하나뿐이거나 계단·자유 감속이 없을 때)."""
    tau, w, wd = (np.asarray(x, float) for x in (tau, omega, omega_dot))
    m = np.ones(len(tau), bool) if mask is None else np.asarray(mask, bool)
    m = m & np.isfinite(tau) & np.isfinite(w) & np.isfinite(wd)
    n = int(m.sum())
    if n < 6:
        return {"J_w": math.nan, "tau_c": math.nan, "b_v": math.nan, "r2": math.nan, "rms": math.nan, "n": n, "cond": math.nan}
    X = np.c_[wd[m], np.sign(w[m]), w[m]]
    y = tau[m]
    scale = np.linalg.norm(X, axis=0)
    scale[scale == 0.0] = 1.0
    c, *_ = np.linalg.lstsq(X / scale, y, rcond=None)
    c = c / scale
    res = y - X @ c
    ss = float(np.sum((y - y.mean()) ** 2))
    return {"J_w": float(c[0]), "tau_c": float(c[1]), "b_v": float(c[2]),
            "r2": 1.0 - float(np.sum(res ** 2)) / ss if ss > 0.0 else math.nan,
            "rms": float(np.sqrt(np.mean(res ** 2))), "n": n, "cond": float(np.linalg.cond(X / scale))}


def fit_wheel_params_accel(tau, omega, omega_dot, mask=None):
    """식-W2 를 ω̇ 에 대해 푼 회귀:  ω̇ = (1/J_w)·τ − (τ_c/J_w)·sgn(ω) − (b_v/J_w)·ω.
    세 신호 가운데 잡음이 가장 큰 ω̇ (ω 를 미분한 값) 를 왼쪽 (종속 변수) 에 둔다. 최소제곱은 종속 변수의 잡음에는 치우치지 않지만
    설명 변수의 잡음에는 그 계수가 0 쪽으로 줄어든다 (attenuation bias · regression dilution — errors-in-variables,
    Fuller 1987 "Measurement Error Models"). fit_wheel_params (지시서 식 그대로: τ 가 왼쪽, ω̇ 가 설명 변수) 는 그래서 속도 잡음이
    크면 J_w 가 작게 나온다 (tools/wheel_param_id.py --noise-study, 시험 I2).
    반환은 fit_wheel_params 와 같은 꼴 — r2 는 ω̇ 맞춤의 값, rms 는 토크로 바꾼 잔차 J_w·RMS(ω̇ 잔차)."""
    tau, w, wd = (np.asarray(x, float) for x in (tau, omega, omega_dot))
    m = np.ones(len(tau), bool) if mask is None else np.asarray(mask, bool)
    m = m & np.isfinite(tau) & np.isfinite(w) & np.isfinite(wd)
    n = int(m.sum())
    bad = {"J_w": math.nan, "tau_c": math.nan, "b_v": math.nan, "r2": math.nan, "rms": math.nan, "n": n, "cond": math.nan}
    if n < 6:
        return bad
    X = np.c_[tau[m], np.sign(w[m]), w[m]]
    y = wd[m]
    scale = np.linalg.norm(X, axis=0)
    scale[scale == 0.0] = 1.0
    c, *_ = np.linalg.lstsq(X / scale, y, rcond=None)
    c = c / scale
    if not (c[0] > 0.0):
        return bad
    Jw = 1.0 / c[0]
    res = y - X @ c
    ss = float(np.sum((y - y.mean()) ** 2))
    return {"J_w": float(Jw), "tau_c": float(-c[1] * Jw), "b_v": float(-c[2] * Jw),
            "r2": 1.0 - float(np.sum(res ** 2)) / ss if ss > 0.0 else math.nan,
            "rms": float(Jw * np.sqrt(np.mean(res ** 2))), "n": n, "cond": float(np.linalg.cond(X / scale))}


def steady_state_line(omega_ss, tau_ss):
    """일정 속도 구간만: τ = τ_c + b_v·|ω| 직선 (한 방향 자료, 점 2 개 이상). 반환 dict(tau_c, b_v, rms, n)."""
    w, tq = np.abs(np.asarray(omega_ss, float)), np.abs(np.asarray(tau_ss, float))
    if len(w) < 2 or np.ptp(w) < 1e-9:
        return {"tau_c": math.nan, "b_v": math.nan, "rms": math.nan, "n": len(w)}
    b, c = np.polyfit(w, tq, 1)
    return {"tau_c": float(c), "b_v": float(b), "rms": float(np.sqrt(np.mean((c + b * w - tq) ** 2))), "n": len(w)}


def coast_inertia(tau, omega, omega_dot, tau_c, b_v, mask=None):
    """자유 감속 구간만: J_w = (τ − τ_c·sgn ω − b_v·ω)/ω̇  (τ 는 그 구간에 실제 나간 값 — "0" 명령도 ±1.22 mN·m 다).
    τ_c·b_v 는 밖에서 준다 (회귀 값 또는 일정 속도 직선 값). 표본별 값의 중앙값·사분위. 반환 dict(J_w, q25, q75, n)."""
    tau, w, wd = (np.asarray(x, float) for x in (tau, omega, omega_dot))
    m = np.ones(len(tau), bool) if mask is None else np.asarray(mask, bool)
    m = m & (np.abs(wd) > 1e-6)
    if m.sum() < 3:
        return {"J_w": math.nan, "q25": math.nan, "q75": math.nan, "n": int(m.sum())}
    J = (tau[m] - tau_c * np.sign(w[m]) - b_v * w[m]) / wd[m]
    return {"J_w": float(np.median(J)), "q25": float(np.percentile(J, 25)), "q75": float(np.percentile(J, 75)), "n": int(m.sum())}


def simulate_lifted(taus, holds, J_w, tau_c, b_v, dt=0.01, w0=0.0, stiction=None):
    """식-W2 의 해석해로 만든 합성 자료 (시험 I1~I3 용): 단계마다 τ 일정, ω(t) = ω_ss + (ω0 − ω_ss)·exp(−t·b_v/J_w).
    속도가 0 을 지나면 선다 (마찰 부호가 바뀌는 곳 — |τ| ≤ stiction (기본 τ_c) 이면 그대로 정지).
    반환 (t, tau, omega) — 간격 dt, 단계 경계는 dt 의 배수로 맞춘다."""
    stiction = tau_c if stiction is None else stiction
    ts, taus_out, ws = [], [], []
    w, t0 = float(w0), 0.0
    for tau, hold in zip(taus, holds):
        n = int(round(hold / dt))
        tt = np.arange(n) * dt
        if abs(w) < 1e-12 and abs(tau) <= stiction:
            seg = np.zeros(n)
            w_end = 0.0
        else:
            s = math.copysign(1.0, w if abs(w) >= 1e-12 else tau)
            w_ss = (tau - tau_c * s) / b_v
            seg = w_ss + (w - w_ss) * np.exp(-tt * b_v / J_w)
            w_end = w_ss + (w - w_ss) * math.exp(-hold * b_v / J_w)
            cross = np.flatnonzero(seg * s < 0.0)                 # 마찰만으로 0 을 지난다 → 그 자리에서 선다
            if len(cross) and abs(tau) <= stiction:
                seg[cross[0]:] = 0.0
                w_end = 0.0
        ts.append(t0 + tt); taus_out.append(np.full(n, float(tau))); ws.append(seg)
        w, t0 = w_end, t0 + n * dt
    return np.concatenate(ts), np.concatenate(taus_out), np.concatenate(ws)
