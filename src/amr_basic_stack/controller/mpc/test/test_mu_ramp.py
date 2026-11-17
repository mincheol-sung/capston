"""tools/mu_ramp_test.py 의 계산부 (2026-11-16) — μ̂ 규약이 1판 식-21·시뮬 SlipPlant 의 45° 롤러 모델과 같은가."""
import importlib.util
import math
import os

import numpy as np
import pytest

from amr_mpc.torque.model import TorquePlant

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("mu_ramp_test", os.path.join(HERE, "..", "tools", "mu_ramp_test.py"))
mrt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mrt)


def _slip_limited_accel(p, mu, u):
    """SlipPlant 기하로 방향 u 의 최대 몸체 가속 — 바퀴 롤러 힘 F_i ∈ [−μN, μN] 을 훑어 u 성분 최대, 옆 성분·모멘트 0."""
    best = 0.0
    N = p.mass * p.g / 4.0
    for s in np.array(np.meshgrid(*[[-1, 0, 1]] * 4)).T.reshape(-1, 4):       # 꼭짓점·변 중점
        F = mu * N * s
        Fb = np.array([np.sum(F) / math.sqrt(2.0), np.sum(F * np.array([-1, 1, 1, -1])) / math.sqrt(2.0)])
        Mz = np.sum(F * np.array([-1, 1, -1, 1]))
        perp = Fb @ np.array([-u[1], u[0]])
        if abs(Mz) < 1e-9 and abs(perp) < 1e-9:
            best = max(best, Fb @ u / p.mass)
    return best


@pytest.mark.parametrize("key", ["fwd", "left", "fl", "br"])
def test_mu_convention_matches_slip_plant(key):
    """SlipPlant 가 μ 0.4 에서 낼 수 있는 최대 가속을 넣으면 μ̂ = 0.4 가 나온다 (앞·옆 √2 a/g, 대각 2 a/g)."""
    p = TorquePlant.from_yaml()
    u = np.array(mrt.DIRS[key])
    a = _slip_limited_accel(p, 0.4, u)
    mu, mu_iso, _ = mrt.mu_from_accel(a * u, p)
    assert mu == pytest.approx(0.4, rel=1e-6)
    expect = math.sqrt(2.0) * a / p.g if key in ("fwd", "left") else 2.0 * a / p.g
    assert mu == pytest.approx(expect, rel=1e-6)
    assert mu_iso == pytest.approx(a / p.g)


def _rec(gap_rate_hold):
    """램프 0~0.5 s (몸체 가속이 0 → 2.0 으로 오름), 0.5 s 판정 → 유지 0.2 s (몸체 2.0 m/s² 일정, 바퀴는 gap_rate 로 앞섬) → 굴림·DAMPING."""
    rec = []
    for i in range(150):
        t = i * mrt.DT
        if t < 0.5:
            ph, vi, vw = mrt.PH_RAMP, 2.0 * t * t, 2.0 * t * t + max(0.0, t - 0.45)
        elif t < 0.7:
            ph, vi = mrt.PH_HOLD, 0.5 + 2.0 * (t - 0.5)
            vw = vi + 0.05 + gap_rate_hold * (t - 0.5)
        else:
            ph, vi, vw = (mrt.PH_COAST if t < 1.0 else mrt.PH_DAMP), 0.9, 0.9
        rec.append((100.0 + t, ph, 0.5 * t, 0, 0, vi, 0, vw, 0))
    return rec


def test_analyze_confirmed_slip_uses_hold_window():
    """유지 구간 몸체 가속 2.0 m/s², 바퀴–몸체 차가 0.2 s 에 0.3 m/s 더 벌어짐 → slip, μ̂ = √2·2.0/g."""
    p = TorquePlant.from_yaml()
    o = mrt.analyze(_rec(1.5), np.array([1.0, 0.0]), p)
    assert o["result"] == "slip" and o["t_slip"] == pytest.approx(0.5, abs=0.011)
    assert o["a"] == pytest.approx(2.0, abs=0.01)
    assert o["mu"] == pytest.approx(math.sqrt(2.0) * 2.0 / p.g, rel=0.01)
    assert o["mu_onset"] < o["mu"]                                     # 판정 직전은 가속이 아직 오르는 중


def test_analyze_unconfirmed_when_gap_does_not_grow():
    """유지 중 차가 안 벌어지면 (IMU 적분 오차로 본 거짓 판정) μ̂ 를 내지 않는다."""
    o = mrt.analyze(_rec(0.0), np.array([1.0, 0.0]), TorquePlant.from_yaml())
    assert o["result"] == "unconfirmed" and not np.isfinite(o["mu"])


def test_analyze_no_slip_gives_lower_bound():
    p = TorquePlant.from_yaml()
    rec = [(i * mrt.DT, mrt.PH_RAMP, 0.35, 0, 0, 1.5 * i * mrt.DT, 0, 1.5 * i * mrt.DT, 0) for i in range(60)]
    o = mrt.analyze(rec, np.array([1.0, 0.0]), p)
    assert o["result"] == "no_slip" and o["mu"] == pytest.approx(math.sqrt(2.0) * 1.5 / p.g, rel=0.02)
