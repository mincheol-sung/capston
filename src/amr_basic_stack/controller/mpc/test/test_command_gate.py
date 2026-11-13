"""keyboard_command_gate — 기본(타이머 재샘플)과 forward_on_receive(받는 즉시 중계) 모드.

forward_on_receive 는 2026-10-22 MPC 100 Hz 용으로 추가했다. 무장·신선도·정지 논리가 두 모드에서 같아야 한다.
"""
from unittest.mock import Mock

import pytest
import rclpy
from std_msgs.msg import Float32MultiArray

from amr_mpc.nodes.keyboard_command_gate import KeyboardCommandGate


def make_gate(forward, armed=True):
    rclpy.init(args=['--ros-args', '-p', 'command_type:=wheels', '-p', 'keyboard_enabled:=false',
                     '-p', f'initially_armed:={str(armed).lower()}',
                     '-p', f'forward_on_receive:={str(forward).lower()}'])
    g = KeyboardCommandGate()
    g.timer.cancel()                      # 타이머는 손으로 부른다
    g.command_publisher = Mock()
    return g


@pytest.fixture
def teardown():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def sent(g):
    return [list(c.args[0].data) for c in g.command_publisher.publish.call_args_list]


def cmd(v):
    return Float32MultiArray(data=[float(v)] * 4)


def test_default_mode_resamples_on_timer(teardown):
    g = make_gate(forward=False)
    g._on_command(cmd(1))
    g._on_command(cmd(2))                 # 타이머 전에 두 개 → 하나는 버려진다 (기존 동작)
    assert sent(g) == []
    g._publish_gated_command()
    g._publish_gated_command()            # 같은 명령을 다시 낸다
    assert sent(g) == [[2.0] * 4, [2.0] * 4]
    g.destroy_node()


def test_forward_mode_relays_every_command_immediately(teardown):
    g = make_gate(forward=True)
    for v in (1, 2, 3):
        g._on_command(cmd(v))
    assert sent(g) == [[1.0] * 4, [2.0] * 4, [3.0] * 4]
    g._publish_gated_command()            # 신선한 명령이 있으면 타이머는 아무것도 안 낸다 (중복 없음)
    assert len(sent(g)) == 3
    g.destroy_node()


def test_forward_mode_times_out_to_zero(teardown):
    g = make_gate(forward=True)
    g._on_command(cmd(5))
    g.latest_command_time -= 1.0          # command_timeout 0.2 s 를 넘긴 것으로
    g._publish_gated_command()
    assert sent(g)[-1] == [0.0] * 4
    g.destroy_node()


def test_forward_mode_disarmed_sends_only_zero(teardown):
    g = make_gate(forward=True, armed=False)
    g._on_command(cmd(5))
    assert sent(g) == []
    g._publish_gated_command()
    assert sent(g) == [[0.0] * 4]
    g.destroy_node()


def test_forward_mode_arming_does_not_release_buffered_command(teardown):
    g = make_gate(forward=True, armed=False)
    g._on_command(cmd(7))                 # 해제 중 받은 명령
    g._set_armed(True, 'test')
    g._publish_gated_command()
    assert sent(g)[-1] == [0.0] * 4       # 무장 직후 묵은 명령은 내지 않는다
    g._on_command(cmd(8))
    assert sent(g)[-1] == [8.0] * 4       # 무장 뒤 새 명령은 바로
    g._set_armed(False, 'test')
    assert sent(g)[-1] == [0.0] * 4
    g.destroy_node()


# ── 토크 모드 (2026-11-11): 정지 신호는 토크 0 이 아니라 MODE_DAMPING ──────────────────────
def make_torque_gate(armed=True):
    rclpy.init(args=['--ros-args', '-p', 'command_type:=torques', '-p', 'keyboard_enabled:=false',
                     '-p', f'initially_armed:={str(armed).lower()}', '-p', 'forward_on_receive:=true'])
    g = KeyboardCommandGate()
    g.timer.cancel()
    g.command_publisher = Mock()
    return g


def torque_cmd(v, sec=123):
    from amr_interfaces.msg import WheelTorqueCommand
    m = WheelTorqueCommand(mode=WheelTorqueCommand.MODE_TORQUE, torque=[float(v)] * 4)
    m.header.stamp.sec = sec
    return m


def test_torque_mode_relays_command_with_stamp_untouched(teardown):
    g = make_torque_gate()
    g._on_command(torque_cmd(0.05, sec=123))
    out = g.command_publisher.publish.call_args_list[-1].args[0]
    assert out.mode == out.MODE_TORQUE and list(out.torque) == [0.05] * 4
    assert out.header.stamp.sec == 123            # 종단 지연 측정용 stamp 는 그대로
    g.destroy_node()


def test_torque_mode_stop_is_damping_not_zero_torque(teardown):
    g = make_torque_gate(armed=False)
    g._on_command(torque_cmd(0.05))               # 해제 중에는 중계하지 않는다
    assert g.command_publisher.publish.call_args_list == []
    g._publish_gated_command()
    out = g.command_publisher.publish.call_args_list[-1].args[0]
    assert out.mode == out.MODE_DAMPING
    assert out.header.stamp.sec > 0               # 정지 신호에는 지금 시각
    g.destroy_node()


def test_torque_mode_timeout_and_disarm_send_damping(teardown):
    g = make_torque_gate()
    g._on_command(torque_cmd(0.05))
    g.latest_command_time -= 1.0                  # 명령 끊김
    g._publish_gated_command()
    assert g.command_publisher.publish.call_args_list[-1].args[0].mode == 1
    g._on_command(torque_cmd(0.07))
    assert g.command_publisher.publish.call_args_list[-1].args[0].mode == 0
    g._set_armed(False, 'test')                   # 's' 키
    assert g.command_publisher.publish.call_args_list[-1].args[0].mode == 1
    g.destroy_node()


def test_shutdown_sends_damping_last(teardown):
    """리뷰 R6: 끌 때 마지막 신호는 DAMPING (rclpy 의 SIGINT 처리기를 꺼서 shutdown() 이 낼 수 있다)."""
    g = make_torque_gate()
    g._on_command(torque_cmd(0.05))
    g.shutdown()
    out = g.command_publisher.publish.call_args_list[-1].args[0]
    assert out.mode == out.MODE_DAMPING
    g.destroy_node()
