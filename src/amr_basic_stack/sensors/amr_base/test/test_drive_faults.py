"""Transport/feedback faults must stop without opening a motor device."""

from types import SimpleNamespace as NS
from unittest.mock import Mock

from amr_base.roboclaw_mecanum_node import RoboClaw
from amr_base.roboclaw_mecanum_node import RoboClawMecanumNode
from amr_base.roboclaw_mecanum_node import WheelSpeeds
from geometry_msgs.msg import Twist
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray


def receiver(monkeypatch):
    monkeypatch.setattr('amr_base.roboclaw_mecanum_node.time.monotonic', lambda: 100.)
    n = NS(
        get_clock=lambda: NS(now=lambda: Time(seconds=2.)),
        get_logger=lambda: Mock(), last_cmd_monotonic=0., cmd_timeout_s=.5,
        last_cmd_time=Time(seconds=2.),  # ROS time is frozen; watchdog must still run.
        feedback_timeout_s=.25, stop_pending=False,
        current_wheels=WheelSpeeds(5., 5., 5., 5.),
        left_driver=NS(mock=True, set_motor_speeds=Mock(return_value=True)),
        right_driver=NS(mock=True, set_motor_speeds=Mock(return_value=True)),
        left_address=128, right_address=129,
        feedback_last_accept_time=dict.fromkeys(WheelSpeeds().__dict__, 100.),
    )
    n.stop_all = lambda: RoboClawMecanumNode.stop_all(n)
    n.feedback_is_fresh = lambda: RoboClawMecanumNode.feedback_is_fresh(n)
    for name in ('publish_wheel_speeds', 'read_encoder_feedback',
                 'publish_measured_wheel_speeds', 'publish_measured_qpps',
                 'publish_joint_state', 'publish_measured_odometry',
                 'report_feedback_health'):
        setattr(n, name, Mock())
    return n


def test_stop_ack_failure_retries_even_after_command_is_zero(monkeypatch):
    n = receiver(monkeypatch)
    n.left_driver.set_motor_speeds.side_effect = [False, True]
    RoboClawMecanumNode.update(n)
    assert n.stop_pending and n.current_wheels == WheelSpeeds()
    RoboClawMecanumNode.update(n)
    assert not n.stop_pending
    RoboClawMecanumNode.update(n)
    assert n.left_driver.set_motor_speeds.call_count == 2
    assert n.right_driver.set_motor_speeds.call_count == 2


def test_encoder_expiry_stops_and_suppresses_fresh_state(monkeypatch):
    n = receiver(monkeypatch)
    n.left_driver.mock = n.right_driver.mock = False
    n.last_cmd_monotonic = 100.  # Commands remain fresh during the encoder fault.
    n.feedback_last_accept_time['rear_right'] = 99.
    RoboClawMecanumNode.update(n)
    assert n.current_wheels == WheelSpeeds()
    for name in ('publish_measured_wheel_speeds', 'publish_measured_qpps',
                 'publish_joint_state', 'publish_measured_odometry'):
        getattr(n, name).assert_not_called()
    assert n.last_odom_time == n.get_clock().now()
    assert n.last_joint_time == n.get_clock().now()


def test_short_encoder_glitch_retains_bounded_feedback_grace(monkeypatch):
    n = receiver(monkeypatch)
    n.left_driver.mock = n.right_driver.mock = False
    n.last_cmd_monotonic = 100.
    n.feedback_last_accept_time['rear_right'] = 99.8
    RoboClawMecanumNode.update(n)
    n.left_driver.set_motor_speeds.assert_not_called()
    n.publish_measured_wheel_speeds.assert_called_once()
    n.publish_measured_odometry.assert_called_once()


def test_commands_are_rejected_until_every_encoder_is_valid(monkeypatch):
    n = receiver(monkeypatch)
    n.left_driver.mock = n.right_driver.mock = False
    n.feedback_last_accept_time['rear_right'] = None
    n.send_wheel_speeds = Mock()
    RoboClawMecanumNode.cmd_vel_callback(n, Twist())
    RoboClawMecanumNode.wheel_cmd_callback(n, Float32MultiArray(data=[5.] * 4))
    n.send_wheel_speeds.assert_not_called()
    assert n.left_driver.set_motor_speeds.call_args.args == (128, 0, 0)


def test_mock_transport_reports_success():
    claw = RoboClaw('/dev/null', 115200, .01, True, True, Mock())
    assert claw.set_motor_speeds(128, 0, 0) is True


def test_transport_rejection_and_exception_report_failure():
    claw = RoboClaw('/dev/null', 115200, .01, True, True, Mock())
    claw.mock = False
    claw._driver = NS(SpeedM1M2=Mock(return_value=False))
    assert claw.set_motor_speeds(128, 0, 0) is False
    claw._driver.SpeedM1M2.side_effect = OSError('disconnected')
    assert claw.set_motor_speeds(128, 0, 0) is False
