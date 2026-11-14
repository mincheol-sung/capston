"""slip_logger 의 슬립률 정의 (구조도)."""
import numpy as np
import pytest

from amr_mpc.nodes.slip_logger import slip_ratio


def test_no_slip_is_zero():
    w = np.array([10.0, -5.0, 3.0, 0.0])
    assert np.allclose(slip_ratio(w, w, 2.5), 0.0)


def test_spin_and_skid_signs():
    # 헛돎(바퀴가 더 빠름) = +, 끌림(바퀴가 더 느림) = −, 후진에서도 같은 뜻
    lam = slip_ratio([12.0, 8.0, -12.0, -8.0], [10.0, 10.0, -10.0, -10.0], 2.5)
    assert lam[0] == pytest.approx(2 / 12) and lam[1] == pytest.approx(-2 / 10)
    assert lam[2] == pytest.approx(2 / 12) and lam[3] == pytest.approx(-2 / 10)


def test_bounded_and_no_division_blowup():
    lam = slip_ratio([20.0, 0.0, 0.01, 1.0], [0.0, 20.0, 0.0, 1.2], 2.5)
    assert lam[0] == pytest.approx(1.0)        # 몸체는 서 있는데 바퀴만 돈다: 완전 헛돎
    assert lam[1] == pytest.approx(-1.0)       # 바퀴가 멈춘 채 끌려간다: 완전 잠김
    assert abs(lam[2]) < 0.01                  # 거의 정지한 바퀴는 ω_ε 로 눌린다
    assert np.all(np.abs(lam) <= 1.0 + 1e-12)
    assert slip_ratio([-10.0], [10.0], 2.5)[0] == pytest.approx(-2.0)   # 몸체와 반대로 돈다


def test_stale_or_error_wheels_are_left_out():
    """R9 리뷰: 응답이 없거나(fresh false) 에러인 바퀴의 λ 는 NaN (오래된 바퀴 속도로 가짜 슬립을 만들지 않는다)."""
    import math
    from unittest.mock import Mock

    import rclpy
    from nav_msgs.msg import Odometry

    from amr_interfaces.msg import WheelStates
    from amr_mpc.nodes.slip_logger import SlipLoggerNode
    rclpy.init(args=["--ros-args", "-p", "wheel_states_topic:=/r9_slip/ws", "-p", "body_topic:=/r9_slip/odom"])
    try:
        n = SlipLoggerNode()
        n.pub = Mock()
        body = Odometry()
        body.twist.twist.linear.x = 0.5
        n.on_body(body)
        ws = WheelStates()
        ws.velocity = [12.5, 12.5, 12.5, 0.0]           # RR 은 응답이 끊겨 예전 값(0)이 남아 있다
        ws.fresh = [True, True, True, False]
        ws.error = [0, 0, 4, 255]                        # RL 은 드라이버 에러(저전압)
        n.on_wheels(ws)
        lam = n.pub.publish.call_args[0][0].data[1:5]
        assert abs(lam[0]) < 1e-9 and abs(lam[1]) < 1e-9
        assert math.isnan(lam[2]) and math.isnan(lam[3])
        n.destroy_node()
    finally:
        rclpy.shutdown()
