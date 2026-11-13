"""core.kalman 단위시험. ROS 없이 돈다."""
import numpy as np
import pytest

from object_tracker.core.kalman import (H, KalmanFilter, chi2_2dof,
                                        mahalanobis, process_noise,
                                        transition, velocity_chi2)


def test_transition_is_constant_velocity():
    dt = 0.1
    F = transition(dt)
    x = np.array([1.0, 2.0, 0.5, -0.3])
    assert np.allclose(F @ x, [1.0 + 0.05, 2.0 - 0.03, 0.5, -0.3])


def test_process_noise_matches_numerical_integration():
    """Q 를 수치적분으로 독립 검산한다.

    Q = int_0^dt F(tau) G sigma_a^2 G^T F(tau)^T d tau,  F(tau)G = [tau, 1]^T
    """
    dt, sigma_a = 0.083, 1.3
    Q = process_noise(dt, sigma_a)
    tau = np.linspace(0.0, dt, 200001)
    q = sigma_a ** 2
    assert Q[0, 0] == pytest.approx(q * np.trapz(tau ** 2, tau), rel=1e-9)
    assert Q[0, 2] == pytest.approx(q * np.trapz(tau, tau), rel=1e-9)
    assert Q[2, 2] == pytest.approx(q * dt, rel=1e-12)
    # x, y 축은 독립이므로 축 사이 교차항은 0
    assert Q[0, 1] == 0.0 and Q[0, 3] == 0.0 and Q[1, 2] == 0.0


def test_process_noise_is_psd():
    Q = process_noise(0.083, 1.0)
    assert np.all(np.linalg.eigvalsh(Q) >= -1e-15)


@pytest.mark.parametrize('p,expected', [
    (0.90, 4.605170), (0.95, 5.991465), (0.99, 9.210340)])
def test_chi2_2dof_matches_table(p, expected):
    """자유도 2 는 -2 ln(1-p) 로 닫힌 형태. 교재 표와 대조한다."""
    assert chi2_2dof(p) == pytest.approx(expected, abs=1e-5)


def test_chi2_2dof_rejects_bad_p():
    for p in (0.0, 1.0, -0.1, 1.5):
        with pytest.raises(ValueError):
            chi2_2dof(p)


def test_update_keeps_covariance_symmetric_psd():
    """Joseph form 을 쓰는 이유. P_vv 가 velocity_chi2 의 역행렬 대상이다."""
    R = np.eye(2) * 0.004 ** 2
    kf = KalmanFilter([1.0, 0.0], R, sigma_v0=1.5)
    rng = np.random.default_rng(0)
    for _ in range(500):
        kf.predict(1.0 / 12.0, 1.0)
        kf.update(np.array([1.0, 0.0]) + rng.normal(0, 0.004, 2), R)
        assert np.allclose(kf.P, kf.P.T, atol=1e-18)
        assert np.all(np.linalg.eigvalsh(kf.P) > 0.0)


def test_update_reduces_position_uncertainty():
    R = np.eye(2) * 0.01 ** 2
    kf = KalmanFilter([0.0, 0.0], R, sigma_v0=1.5)
    before = np.trace(kf.P_pp)
    kf.predict(0.1, 1.0)
    kf.update([0.0, 0.0], R)
    assert np.trace(kf.P_pp) < before


def test_innovation_matches_manual_computation():
    R = np.eye(2) * 0.01 ** 2
    kf = KalmanFilter([0.0, 0.0], R, sigma_v0=1.5)
    kf.predict(0.1, 1.0)
    z = np.array([0.5, -0.2])
    y, S = kf.innovation(z, R)
    assert np.allclose(y, z - H @ kf.x)
    assert np.allclose(S, H @ kf.P @ H.T + R)


def test_velocity_chi2_zero_for_zero_velocity():
    kf = KalmanFilter([0.0, 0.0], np.eye(2) * 1e-4, sigma_v0=1.5)
    assert velocity_chi2(kf) == pytest.approx(0.0)


def test_velocity_chi2_scales_with_uncertainty():
    """같은 속도라도 P_vv 가 크면 chi^2 가 작다 — 원거리에서 보수적이 되는 성질."""
    kf_tight = KalmanFilter([0.0, 0.0], np.eye(2) * 1e-6, sigma_v0=0.1)
    kf_loose = KalmanFilter([0.0, 0.0], np.eye(2) * 1e-6, sigma_v0=2.0)
    for kf in (kf_tight, kf_loose):
        kf.x[2:4] = [1.0, 0.0]
    assert velocity_chi2(kf_tight) > velocity_chi2(kf_loose)


def test_mahalanobis_singular_returns_inf():
    assert mahalanobis(np.array([1.0, 0.0]), np.zeros((2, 2))) == float('inf')


def test_constant_velocity_target_converges():
    """등속 1.2 m/s 물체를 5초 추종하면 속도 추정이 참값에 붙는다."""
    dt, sigma_z = 1.0 / 12.0, 0.004
    R = np.eye(2) * sigma_z ** 2
    rng = np.random.default_rng(3)
    p, v = np.array([3.0, 0.0]), np.array([1.2, 0.0])
    kf = KalmanFilter(p, R, sigma_v0=1.5)
    for _ in range(60):
        p = p + v * dt
        kf.predict(dt, 1.0)
        kf.update(p + rng.normal(0, sigma_z, 2), R)
    assert kf.speed == pytest.approx(1.2, abs=0.25)
    assert velocity_chi2(kf) > chi2_2dof(0.99)


def test_extrapolated_covariance_needs_cross_block():
    """TrackedObject.position_velocity_covariance 를 싣는 근거.

    등속 외삽 [F(tau) P F(tau)^T]_pp = P_pp + tau (P_pv + P_pv^T) + tau^2 P_vv.
    coast 중에는 P_pv 가 양수라 이 항을 빼면 위치 분산을 과소평가한다.
    """
    dt, R = 1.0 / 12.0, np.eye(2) * 0.004 ** 2
    kf = KalmanFilter([4.0, 0.0], R, sigma_v0=1.5)
    for _ in range(120):
        kf.predict(dt, 1.0)
        kf.update([4.0, 0.0], R)
    for _ in range(12):                     # 1.0 s coast = max_coast
        kf.predict(dt, 1.0)
    tau = 1.0
    full = (transition(tau) @ kf.P @ transition(tau).T)[0:2, 0:2]
    expanded = kf.P_pp + tau * (kf.P_pv + kf.P_pv.T) + tau ** 2 * kf.P_vv
    assert np.allclose(full, expanded, rtol=1e-12, atol=1e-15)
    dropped = kf.P_pp + tau ** 2 * kf.P_vv
    assert np.all(np.diag(dropped) < 0.6 * np.diag(full))
