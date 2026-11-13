"""RoboClaw 속도 필드 복원 — signed 32-bit, status bit 무시.

예전 코드는 status & 0x01 이면 양수 raw 의 부호를 뒤집어, 감속 중인 양수
바퀴가 음수로 읽혔다(velocity_logs 에서 +→- 108 건, -→+ 시작 0 건).
"""
import logging

import pytest

from amr_base.roboclaw_mecanum_node import RoboClaw


@pytest.fixture
def claw():
    return RoboClaw('/dev/null', 115200, 0.01, True, True, logging.getLogger('test'))


def decode(claw, raw, status):
    return claw._read_speed_result(lambda address: (True, raw, status), 128, 'M1')


@pytest.mark.parametrize('status', [0x00, 0x01, 0xFF])
def test_positive_speed_ignores_status_bit(claw, status):
    # 감속 중 양수 바퀴: 이전 코드는 status 0x01 에서 -1200 을 돌려줬다
    assert decode(claw, 1200, status) == 1200


@pytest.mark.parametrize('status', [0x00, 0x01])
def test_negative_speed_is_twos_complement(claw, status):
    assert decode(claw, 0x100000000 - 1200, status) == -1200


def test_extremes(claw):
    assert decode(claw, 0x7FFFFFFF, 0) == 2**31 - 1
    assert decode(claw, 0x80000000, 0) == -2**31
    assert decode(claw, 0, 1) == 0


def test_failed_read_returns_none(claw):
    assert claw._read_speed_result(lambda address: (False, 0, 0), 128, 'M1') is None
