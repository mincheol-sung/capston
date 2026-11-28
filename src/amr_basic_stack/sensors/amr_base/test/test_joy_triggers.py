"""joy_teleop_node 트리거 무장 (2026-11-28): RT 축 → /mpc/arm true, LT 축 → /teleop/arm true, B → /mpc/arm false.

Xbox 인덱스 (joy_teleop_xbox.yaml 머리 표, 2026-11-23 실측): 축 4 = RT, 5 = LT (놓음 +1, 끝까지 −1), 버튼 1 = B.
"""
from unittest.mock import Mock

import pytest
import rclpy
from sensor_msgs.msg import Joy

from amr_base.joy_teleop_node import JoyTeleop

RT, LT, B = 4, 5, 1


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'arm_axis:=4', '-p', 'teleop_arm_axis:=5', '-p', 'disarm_button:=1'])
    n = JoyTeleop()
    n._arm_pub, n._teleop_pub = Mock(), Mock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def joy(rt=1.0, lt=1.0, b=0):
    axes = [0.0] * 8
    axes[RT], axes[LT] = rt, lt
    buttons = [0] * 15
    buttons[B] = b
    return Joy(axes=axes, buttons=buttons)


def sent(pub):
    return [c.args[0].data for c in pub.publish.call_args_list]


def test_rt_arms_once_per_pull(node):
    node._joy_cb(joy())
    for rt in (0.0, -0.6, -1.0, -1.0, 1.0, -1.0):      # 0 (안 만진 트리거) 은 당김이 아니다, 놓았다 다시 당기면 한 번 더
        node._joy_cb(joy(rt=rt))
    assert sent(node._arm_pub) == [True, True] and sent(node._teleop_pub) == []


def test_lt_switches_to_teleop(node):
    node._joy_cb(joy())
    node._joy_cb(joy(lt=-1.0))
    node._joy_cb(joy(lt=-1.0))
    assert sent(node._teleop_pub) == [True] and sent(node._arm_pub) == []


def test_lt_wins_over_rt_and_b_wins_over_all(node):
    node._joy_cb(joy())
    node._joy_cb(joy(rt=-1.0, lt=-1.0))                # 둘 다 → 수동 (더 안전한 쪽)
    assert sent(node._teleop_pub) == [True] and sent(node._arm_pub) == []
    node._joy_cb(joy(rt=1.0, lt=-1.0))
    node._joy_cb(joy(rt=-1.0, lt=-1.0))                # LT 를 당기고 있는 동안 RT 는 안 먹는다
    assert sent(node._arm_pub) == []
    node._joy_cb(joy(lt=1.0))
    node._joy_cb(joy(rt=-1.0, b=1))                    # B 가 이긴다
    assert sent(node._arm_pub) == [False]


def test_trigger_held_at_start_does_not_arm(node):
    node._joy_cb(joy(rt=-1.0, lt=-1.0))                # 첫 메시지 = 기준
    node._joy_cb(joy(rt=-1.0, lt=-1.0))
    assert sent(node._arm_pub) == [] and sent(node._teleop_pub) == []
