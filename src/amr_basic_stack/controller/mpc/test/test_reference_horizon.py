# pytest -q controller/mpc/test   (ROS 불필요)
import math
import numpy as np
import pytest

from amr_mpc.core.reference import ReferenceHorizon, HoldLatch, circle_reference, wrap

DT, T = 0.2, 20


def circle_rows(t0, origin=(0.0, 0.0, 0.0), R=1.0, V=0.5):
    return [(*circle_reference(t0 + (i + 1) * DT, origin, R, V, ramp_s=0.0)[0],
             *circle_reference(t0 + (i + 1) * DT, origin, R, V, ramp_s=0.0)[1]) for i in range(T)]


def test_sample_times_follow_readme():
    h = ReferenceHorizon.from_rows(10.0, DT, circle_rows(10.0))
    assert h.t_start == pytest.approx(10.0 + DT)          # references[0] → stamp + dt
    assert h.t_end == pytest.approx(10.0 + T * DT)
    Xr, ur = h.at(h.times[3])
    exp = circle_reference(10.0 + 4 * DT, (0, 0, 0), 1.0, 0.5, 0.0)
    assert np.allclose(Xr, exp[0]) and np.allclose(ur, exp[1])


def test_interpolation_error_small_on_circle():
    """0.2 s 샘플의 선형 보간이 0.5 m/s, R=1 원(곡률 1)에서 얼마나 벗어나는지 — 현 오차 (1−cos(Δθ/2))·R."""
    h = ReferenceHorizon.from_rows(0.0, DT, circle_rows(0.0))
    worst = 0.0
    for t in np.arange(h.t_start, h.t_end, 0.04):
        Xr, _ = h.at(t)
        true = circle_reference(t, (0, 0, 0), 1.0, 0.5, 0.0)[0]
        worst = max(worst, np.hypot(*(Xr[:2] - true[:2])))
    bound = (1 - math.cos(0.5 * DT / 2)) * 1.0                # 현(chord) 중점 오차, 0.5 * DT = 호 각도 [rad]
    assert worst <= bound * 1.01 and worst < 2e-3               # 1.25 mm


def test_backward_extrapolation_matches_kinematics():
    h = ReferenceHorizon.from_rows(0.0, DT, circle_rows(0.0))
    Xr, ur = h.at(0.0)                                         # stamp 시각: 첫 샘플에서 0.2 s 뒤로
    true = circle_reference(0.0, (0, 0, 0), 1.0, 0.5, 0.0)[0]
    assert np.hypot(*(Xr[:2] - true[:2])) < 1e-9               # 상수 twist 정확 적분이라 원호는 오차 0 (직선 외삽이면 0.5·v·ω·dt² = 5 mm)
    assert abs(wrap(Xr[2] - true[2])) < 1e-9
    assert np.allclose(ur, h.U[0])


def test_hold_after_end_is_stop():
    h = ReferenceHorizon.from_rows(0.0, DT, circle_rows(0.0))
    Xr, ur = h.at(h.t_end + 5.0)
    assert np.allclose(Xr[:2], h.X[-1][:2]) and np.allclose(ur, 0)


def test_yaw_wrap_across_pi():
    rows = [(0, 0, math.pi - 0.1, 0, 0, 1.0), (0, 0, -math.pi + 0.1, 0, 0, 1.0)]   # π 를 넘는 두 샘플
    h = ReferenceHorizon.from_rows(0.0, DT, rows)
    Xr, _ = h.at(h.t_start + 0.5 * DT)
    assert abs(wrap(Xr[2] - math.pi)) < 1e-9                   # 중간은 ±π (0 이 아니다)


def test_horizon_shape():
    h = ReferenceHorizon.from_rows(0.0, DT, circle_rows(0.0))
    hz = h.horizon(0.05, 0.04, 20)
    assert len(hz) == 20 and all(x.shape == (3,) and u.shape == (3,) for x, u in hz)
    assert hz[-1][0][0] != hz[0][0][0]                         # 지평이 시간에 따라 움직인다


def test_rejects_empty():
    with pytest.raises(ValueError):
        ReferenceHorizon.from_rows(0.0, DT, [])


def test_is_hold_only_for_planner_stop_sequence():
    """dra_mppi _publish_stop 의 꼴(속도 0, 포즈 한 점)만 hold. 실제 궤적·끝점 정지는 hold 가 아니다."""
    stop = [(1.0, 2.0, 0.3, 0.0, 0.0, 0.0)] * T
    assert ReferenceHorizon.from_rows(0.0, DT, stop).is_hold
    assert not ReferenceHorizon.from_rows(0.0, DT, circle_rows(0.0)).is_hold
    rows = circle_rows(0.0); rows[-1] = (*rows[-1][:3], 0.0, 0.0, 0.0)      # 끝에서만 속도 0
    assert not ReferenceHorizon.from_rows(0.0, DT, rows).is_hold
    drift = [(1.0 + 1e-3 * i, 2.0, 0.3, 0.0, 0.0, 0.0) for i in range(T)]  # 속도 0 인데 포즈가 움직임
    assert not ReferenceHorizon.from_rows(0.0, DT, drift).is_hold


def test_hold_latch_hysteresis():
    L = HoldLatch(0.01, math.radians(2), 0.03, math.radians(5))
    assert not L.update(False, [0.0, 0.0, 0.0])                 # hold 기준이 아니면 항상 추종
    assert not L.update(True, [0.02, 0.0, 0.0])                 # enter 밖 → 추종 (오차를 줄인다)
    assert L.update(True, [0.005, 0.0, math.radians(1)])        # enter 안 → idle
    assert L.update(True, [0.02, 0.0, 0.0])                     # idle 중 enter~exit 사이 → 유지 (hysteresis)
    assert not L.update(True, [0.035, 0.0, 0.0])                # exit 초과 → 재개
    assert not L.update(True, [0.02, 0.0, 0.0])                 # 재개 뒤 enter 밖 → 계속 추종
    assert L.update(True, [0.0, 0.0, 0.0])
    assert not L.update(True, [0.0, 0.0, math.radians(6)])      # yaw 만 넘어도 재개
    assert not L.update(True, [0.0, 0.0, 0.0], stopped=False)   # 아직 움직이면 진입 안 함 (StoppedGoalChecker)
    assert L.update(True, [0.0, 0.0, 0.0], stopped=True)
    assert L.update(True, [0.0, 0.0, 0.0], stopped=False)       # idle 유지에는 stopped 불필요
    assert L.update(True, [0.0, 0.0, math.radians(-1)])
    assert not L.update(False, [0.0, 0.0, 0.0])                 # 새 궤적 → 즉시 추종
    with pytest.raises(ValueError):
        HoldLatch(0.03, 0.1, 0.01, 0.2)
