"""100 Hz 전환용 드라이버 옵션 (2026-10-22) — update_rate_hz, speed_read_mode, parallel_boards, serial_retries.

시리얼 아래만 가짜(amr_base.roboclaw_fake)로 바꾸고 RoboClaw 래퍼·노드 코드는 실제 것을 쓴다.
기본값이 그 전 동작과 같은지도 본다.
"""
from concurrent.futures import ThreadPoolExecutor
import logging
import time

import pytest
import rclpy

from amr_base.roboclaw_fake import FakeRoboClaw
from amr_base.roboclaw_mecanum_node import RoboClaw, RoboClawMecanumNode

QPPS_PER_RAD = 210.08


def wrapped(fake):
    """mock 으로 만든 뒤 내부 드라이버만 가짜 보드로 바꾼다 (포트를 열지 않는다)."""
    claw = RoboClaw('fake', 115200, 0.01, True, True, logging.getLogger('test'))
    claw.mock = False
    claw._driver = fake
    return claw


def spin_up(claw, address, m1, m2, settle=0.6):
    assert claw.set_motor_speeds(address, m1, m2)
    time.sleep(settle)                       # τ 90 ms, 창 30 ms → 0.6 s 면 정상상태


def test_combined_read_decodes_signed_like_per_motor():
    claw = wrapped(FakeRoboClaw(seed=0))
    spin_up(claw, 128, 2100, -1500)
    per = claw.read_motor_speeds(128)
    comb = claw.read_motor_speeds_combined(128)
    assert comb[0] == pytest.approx(per[0], abs=40) and comb[0] > 2000
    assert comb[1] == pytest.approx(per[1], abs=40) and comb[1] < -1400     # 음수 two's complement 복원
    claw.set_motor_speeds(128, 0, 0)


def test_combined_read_unsupported_returns_none():
    claw = wrapped(FakeRoboClaw(supports_getspeeds=False, timeout=0.001, retries=1))
    assert claw.read_motor_speeds_combined(128) == (None, None)


@pytest.fixture
def node():
    rclpy.init()
    n = RoboClawMecanumNode()     # mock_mode 기본 True — 노드만 만들고 보드는 아래에서 꽂는다
    yield n
    n.destroy_node()
    rclpy.shutdown()


def attach_fake_boards(n, **kw):
    n.left_driver = wrapped(FakeRoboClaw(seed=1, **kw))
    n.right_driver = wrapped(FakeRoboClaw(seed=2, **kw))
    n.qpps_per_rad_per_sec = QPPS_PER_RAD


def test_defaults_match_previous_behaviour(node):
    assert node.update_rate_hz == 50.0
    assert node.timer.timer_period_ns == 20_000_000
    assert node.speed_read_mode == 'per_motor'
    assert node.parallel_boards is False and node._board_pool is None


@pytest.mark.parametrize('mode', ['per_motor', 'combined'])
@pytest.mark.parametrize('parallel', [False, True])
def test_feedback_modes_give_same_wheel_speeds(node, mode, parallel):
    attach_fake_boards(node)
    node.speed_read_mode = mode
    node._board_pool = ThreadPoolExecutor(max_workers=2) if parallel else None
    # 출력 매핑(base 기본): FL=left_m2, FR=right_m1, RL=left_m1, RR=right_m2
    spin_up(node.left_driver, node.left_address, 1000, -2000, settle=0)
    spin_up(node.right_driver, node.right_address, 3000, -500)
    node.read_encoder_feedback()
    w = node.measured_wheels
    assert w.rear_left == pytest.approx(1000 / QPPS_PER_RAD, rel=0.03)
    assert w.front_left == pytest.approx(-2000 / QPPS_PER_RAD, rel=0.03)
    assert w.front_right == pytest.approx(3000 / QPPS_PER_RAD, rel=0.03)
    assert w.rear_right == pytest.approx(-500 / QPPS_PER_RAD, rel=0.05)
    assert node.last_read_ms > 0
    if node._board_pool is not None:
        node._board_pool.shutdown()
        node._board_pool = None


def test_parallel_boards_halve_read_time(node):
    attach_fake_boards(node, latency_min=0.002, latency_max=0.002)    # 트랜잭션 2 ms 고정
    node.publish_encoder_speeds = False      # 병렬화만 본다 (켜면 보드당 GetEncoders 가 하나 더 붙는다)
    times = {}
    for parallel in (False, True):
        node._board_pool = ThreadPoolExecutor(max_workers=2) if parallel else None
        samples = []
        for _ in range(10):
            node.read_encoder_feedback()
            samples.append(node.last_read_ms)
        times[parallel] = sorted(samples)[5]
        if node._board_pool is not None:
            node._board_pool.shutdown()
    node._board_pool = None
    assert times[False] == pytest.approx(8.0, abs=1.5)      # 보드 2 × 트랜잭션 2 × 2 ms
    assert times[True] < 0.7 * times[False]                  # 보드별 동시 → 절반 근처


def test_parallel_write_reaches_both_boards(node):
    attach_fake_boards(node)
    node._board_pool = ThreadPoolExecutor(max_workers=2)
    from amr_base.roboclaw_mecanum_node import WheelSpeeds
    assert node.send_wheel_speeds(WheelSpeeds(1.0, 2.0, 3.0, 4.0))
    assert node.left_driver._driver.calls == 1 and node.right_driver._driver.calls == 1
    assert node.last_write_ms > 0
    node._board_pool.shutdown()
    node._board_pool = None


def test_rate_and_mode_parameters_are_validated():
    for args, ok in ((['-p', 'update_rate_hz:=100.0'], True),
                     (['-p', 'update_rate_hz:=0.0'], False),
                     (['-p', 'speed_read_mode:=bogus'], False),
                     (['-p', 'serial_retries:=0'], False)):
        rclpy.init(args=['--ros-args', *args])
        try:
            if ok:
                n = RoboClawMecanumNode()
                assert n.timer.timer_period_ns == 10_000_000
                n.destroy_node()
            else:
                with pytest.raises(ValueError):
                    RoboClawMecanumNode()
        finally:
            rclpy.shutdown()


# ── encoder 모드: 카운트 차분 속도 ─────────────────────────────────────────
def test_encoder_mode_first_sample_then_speed():
    claw = wrapped(FakeRoboClaw(seed=3))
    spin_up(claw, 128, 1500, -2500)
    assert claw.read_motor_speeds_encoder(128) == (None, None)     # 짝이 없다
    time.sleep(0.05)      # 가짜 보드는 카운트를 정수로 자른다 — 10 ms 면 15 counts 라 ±1 이 ±7 % (불안정했다)
    v1, v2 = claw.read_motor_speeds_encoder(128)
    assert v1 == pytest.approx(1500, rel=0.08) and v2 == pytest.approx(-2500, rel=0.08)
    claw.set_motor_speeds(128, 0, 0)


def test_encoder_mode_handles_counter_wraparound(monkeypatch):
    claw = RoboClaw('fake', 115200, 0.01, True, True, logging.getLogger('test'))
    claw.mock = False
    seq = iter([(True, 0x7FFFFF00, 0x00000100), (True, 0x80000064, 0xFFFFFF9C)])   # +356 counts, −356 counts
    claw._driver = type('D', (), {'GetEncoders': lambda self, a: next(seq)})()
    t = iter([0.0, 0.0, 0.01, 0.01])
    monkeypatch.setattr('amr_base.roboclaw_mecanum_node.time.monotonic', lambda: next(t))
    assert claw.read_motor_speeds_encoder(128) == (None, None)
    assert claw.read_motor_speeds_encoder(128) == (35600, -35600)   # 356 counts / 0.01 s


def test_encoder_mode_in_node(node):
    attach_fake_boards(node)
    node.speed_read_mode = 'encoder'
    spin_up(node.left_driver, node.left_address, 1000, -2000, settle=0)
    spin_up(node.right_driver, node.right_address, 3000, -500)
    node.read_encoder_feedback()                  # 첫 샘플 — 실패로 센다
    assert node.feedback_failures['front_left'] == 1
    time.sleep(0.05)
    node.read_encoder_feedback()
    w = node.measured_wheels
    assert w.rear_left == pytest.approx(1000 / QPPS_PER_RAD, rel=0.08)
    assert w.front_right == pytest.approx(3000 / QPPS_PER_RAD, rel=0.08)
    assert w.rear_right == pytest.approx(-500 / QPPS_PER_RAD, rel=0.15)


# ── encoder_wheel_speeds 토픽 (speed_read_mode 와 별개) ────────────────────
def test_encoder_topic_alongside_per_motor(node):
    from unittest.mock import Mock
    attach_fake_boards(node)
    node.encoder_wheel_pub = Mock()
    assert node.speed_read_mode == 'per_motor' and node.publish_encoder_speeds is True
    spin_up(node.left_driver, node.left_address, 1000, -2000, settle=0)
    spin_up(node.right_driver, node.right_address, 3000, -500)
    node.read_encoder_feedback()                      # 엔코더 첫 샘플 — 토픽 안 냄
    node.encoder_wheel_pub.publish.assert_not_called()
    time.sleep(0.05)
    node.read_encoder_feedback()
    fl, fr, rl, rr = node.encoder_wheel_pub.publish.call_args.args[0].data
    assert fl == pytest.approx(-2000 / QPPS_PER_RAD, rel=0.08)   # FL = left_m2
    assert fr == pytest.approx(3000 / QPPS_PER_RAD, rel=0.08)    # FR = right_m1
    assert rl == pytest.approx(1000 / QPPS_PER_RAD, rel=0.08)    # RL = left_m1
    assert rr == pytest.approx(-500 / QPPS_PER_RAD, rel=0.15)    # RR = right_m2
    # 기존 토픽 값은 cmd 18 경로 그대로
    assert node.measured_wheels.front_right == pytest.approx(3000 / QPPS_PER_RAD, rel=0.03)


def test_encoder_topic_skips_cycle_with_failed_wheel(node):
    from unittest.mock import Mock
    node.encoder_wheel_pub = Mock()
    node.qpps_per_rad_per_sec = QPPS_PER_RAD
    node.publish_encoder_wheel_speeds({'left_m1': 100, 'left_m2': None, 'right_m1': 100, 'right_m2': 100})
    node.publish_encoder_wheel_speeds({'left_m1': 100, 'left_m2': 100, 'right_m1': 10**7, 'right_m2': 100})   # 크기 게이트
    node.encoder_wheel_pub.publish.assert_not_called()
    node.publish_encoder_wheel_speeds({'left_m1': 210, 'left_m2': 420, 'right_m1': 630, 'right_m2': -210})
    assert node.encoder_wheel_pub.publish.call_args.args[0].data == pytest.approx([2.0, 3.0, 1.0, -1.0], rel=0.01)
