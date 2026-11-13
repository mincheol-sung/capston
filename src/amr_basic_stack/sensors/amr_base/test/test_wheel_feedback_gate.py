"""엔코더 속도 샘플 게이트 — gate_wheel_sample 과 노드 경로.

계기: 2026-09-23 bag 135606 t=314.24 s, rear_right 한 샘플이 -16,779,142 qpps
(≈ -2^24) 로 읽혀 /odom 이 22.6 m 순간이동했다.
"""
import math

import pytest
import rclpy

from amr_base.roboclaw_mecanum_node import RoboClawMecanumNode, gate_wheel_sample

MAX_ABS, MAX_ACCEL, MAX_STREAK, DT = 60.0, 1250.0, 2, 0.02
GLITCH_QPPS = -16_779_142
QPPS_PER_RAD = 210.08


def gate(value, last, dt=DT, streak=0):
    return gate_wheel_sample(value, last, dt, MAX_ABS, MAX_ACCEL, streak, MAX_STREAK)


def test_glitch_rejected_by_magnitude():
    ok, reason, streak = gate(GLITCH_QPPS / QPPS_PER_RAD, 9.0)
    assert (ok, reason, streak) == (False, 'magnitude', 0)


def test_magnitude_has_no_streak_escape():
    streak = 0
    for _ in range(10):
        ok, reason, streak = gate(100.0, 9.0, streak=streak)
        assert not ok and reason == 'magnitude'


def test_nan_rejected():
    assert gate(math.nan, 1.0)[0] is False


def test_first_sample_skips_step_gate():
    assert gate(38.0, None, None) == (True, None, 0)


def test_normal_change_accepted():
    # 로그 실측 p99 0.88 rad/s/샘플, 문턱은 1250 x 0.02 = 25
    assert gate(10.9, 10.0) == (True, None, 0)
    assert gate(34.9, 10.0)[0] is True


def test_step_rejected_twice_then_accepted():
    streak = 0
    results = []
    for _ in range(3):
        ok, reason, streak = gate(40.0, 10.0, streak=streak)
        results.append((ok, reason))
    assert results == [(False, 'step'), (False, 'step'), (True, None)]
    assert streak == 0


def test_step_threshold_scales_with_elapsed_time():
    # 1 s 스톨 뒤에는 1250 rad/s 까지 허용 -> 정상 변화가 걸리지 않는다
    assert gate(30.0, -30.0, dt=1.0)[0] is True
    assert gate(30.0, -30.0, dt=DT)[0] is False


@pytest.fixture
def node():
    rclpy.init()
    n = RoboClawMecanumNode()     # mock_mode 기본 True
    yield n
    n.destroy_node()
    rclpy.shutdown()


def feed(n, rr_qpps, other_qpps=2000):
    """rear_right 기본 출력은 right_m2. 나머지 세 바퀴는 other_qpps."""
    n.left_driver.mock = False
    n.right_driver.mock = False
    n.left_driver.read_motor_speeds = lambda addr: (other_qpps, other_qpps)
    n.right_driver.read_motor_speeds = lambda addr: (other_qpps, rr_qpps)
    n.read_encoder_feedback()


def test_node_holds_last_value_on_glitch(node):
    node.qpps_per_rad_per_sec = QPPS_PER_RAD
    feed(node, 2000)
    held = node.measured_wheels.rear_right
    assert held == pytest.approx(2000 * node.invert['rear_right'] / QPPS_PER_RAD)

    feed(node, GLITCH_QPPS)
    assert node.measured_wheels.rear_right == held
    assert node.feedback_rejects['rear_right'] == 1
    assert node.feedback_rejects['front_left'] == 0
    # 다른 바퀴는 정상 갱신
    assert node.measured_wheels.front_left == pytest.approx(
        2000 * node.invert['front_left'] / QPPS_PER_RAD)
