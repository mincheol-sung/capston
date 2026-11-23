"""바퀴별 슬립 가드 (amr_mpc/torque/slip_guard.py, 2026-11-21)."""
import numpy as np
import pytest

from amr_mpc.torque.model import TorquePlant
from amr_mpc.torque.slip_guard import SlipGuard, loo_residual

TINV = TorquePlant.from_yaml().Tinv


def wheels(v):
    return TINV @ np.asarray(v, float)


@pytest.mark.parametrize("v", [[0.5, 0.0, 0.0], [0.3, -0.2, 1.0], [0.0, 0.0, 2.0], [-0.4, 0.3, -1.5]])
def test_loo_residual_is_zero_without_slip(v):
    e, pred = loo_residual(wheels(v), v[2], TINV)
    assert np.allclose(e, 0.0, atol=1e-9)
    assert np.allclose(pred, wheels(v), atol=1e-9)


def test_single_wheel_spin_is_largest_residual():
    v = [0.5, 0.0, 0.0]
    om = wheels(v)
    om[0] += 8.0                                   # FL 이 8 rad/s 더 돈다 (0.32 m/s)
    e, _ = loo_residual(om, 0.0, TINV)
    # ω_z 를 알면 대각 짝 (FL·RR, FR·RL) 은 같은 조합 (v_x ∓ v_y) 을 잰다 → FL 의 +δ 는 RR 에 −δ 로 똑같이 나온다.
    # 짝 중 어느 쪽인지는 토크 방향으로 가른다 (σ = e·sign(τ)): 구동 중이면 더 빨리 도는 FL 이 양수.
    assert e[0] == pytest.approx(8.0, rel=1e-9) and e[3] == pytest.approx(-8.0, rel=1e-9)
    assert np.allclose(e[1:3], 0.0, atol=1e-9)
    sig = e * np.sign(np.full(4, 0.1))
    assert int(np.argmax(sig)) == 0


def run(guard, om, wz, tau, n, dt=0.01):
    for _ in range(n):
        g = guard.update(om, wz, tau, dt)
    return g


def test_spin_cuts_that_wheel_only_after_min_on():
    guard = SlipGuard(TINV, threshold=0.5, floor=5.0, min_on=0.05, win=3, g_min=0.4, down_rate=8.0, up_rate=2.0)
    om = wheels([0.3, 0.0, 0.0])
    om[0] *= 2.0                                    # FL 헛돎 σ ≈ 1
    tau = np.full(4, 0.12)
    g = run(guard, om, 0.0, tau, 4)                 # 40 ms — 아직 판정 전
    assert guard.flagged == -1 and np.allclose(g, 1.0)
    g = run(guard, om, 0.0, tau, 10)
    assert guard.flagged == 0
    assert g[0] < 1.0 and np.allclose(g[1:], 1.0)
    g = run(guard, om, 0.0, tau, 100)
    assert g[0] == pytest.approx(0.4)               # 하한에서 멈춘다


def test_braking_lock_is_detected():
    guard = SlipGuard(TINV)
    om = wheels([0.6, 0.0, 0.0])
    om[3] = 0.1 * om[3]                             # RR 이 거의 안 돈다 (잠김)
    tau = np.full(4, -0.12)                         # 제동 토크
    run(guard, om, 0.0, tau, 20)
    assert guard.flagged == 3 and guard.g[3] < 1.0


def test_wheel_faster_against_its_torque_is_not_slip():
    guard = SlipGuard(TINV)
    om = wheels([0.6, 0.0, 0.0])
    om[0] *= 1.5                                    # FL 이 빠른데 토크는 반대 (끌려서 빠른 게 아니라 제동 중) → σ 음수
    run(guard, om, 0.0, np.full(4, -0.1), 20)
    assert guard.flagged == -1 and np.allclose(guard.g, 1.0)


def test_small_mismatch_below_threshold_is_ignored_and_gain_recovers():
    guard = SlipGuard(TINV)
    om = wheels([0.5, 0.0, 0.0])
    om[1] *= 1.2                                    # σ ≈ 0.2 < 0.5
    run(guard, om, 0.0, np.full(4, 0.1), 50)
    assert guard.flagged == -1 and np.allclose(guard.g, 1.0)
    guard.g[:] = 0.4
    run(guard, wheels([0.5, 0.0, 0.0]), 0.0, np.full(4, 0.1), 30)   # 0.3 s × 2/s = +0.6
    assert np.allclose(guard.g, 1.0)


def test_bad_settings_raise():
    with pytest.raises(ValueError):
        SlipGuard(TINV, g_min=0.0)
    with pytest.raises(ValueError):
        SlipGuard(TINV, threshold=-1.0)
