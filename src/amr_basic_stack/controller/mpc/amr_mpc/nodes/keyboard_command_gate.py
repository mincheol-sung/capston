#!/usr/bin/env python3
"""Gate MPC velocity commands behind an explicit keyboard arm action."""

import copy
import os
import select
import signal
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray

from amr_interfaces.msg import WheelTorqueCommand

# 구조 A 는 몸체 twist, 구조 B 는 바퀴 각속도 4개를 낸다. 게이트의 arming·freshness·
# 정지 논리는 완전히 같으므로 메시지 타입만 갈아끼운다.
#   ★ zero() 가 중요하다. roboclaw_mecanum_node.wheel_cmd_callback 은 길이가 4 가 아니면
#     **무시**한다 — 빈 Float32MultiArray() 를 정지신호로 보내면 드라이버가 버리고 직전
#     명령을 그대로 유지한다. 정지가 정지가 아니게 된다. 반드시 [0,0,0,0] 을 보낸다.
#   'torques' (토크 MPC 스택, 2026-11-11): 정지 신호는 토크 0 이 아니라 MODE_DAMPING 이다.
#     토크 모터에 0 을 보내면 서지 않고 관성으로 굴러간다. DAMPING 은 드라이버가 MIT kp=0, v=0, kd>0 으로 세운다.
#     무장 중 중계하는 명령의 header.stamp 는 건드리지 않는다 (종단 지연 측정용).
#   R6 리뷰 2026-11-13:
#     · 발행은 모두 lock 안에서 한다. 키보드 스레드의 정지('s')와 실행기 스레드의 즉시 중계(forward_on_receive)가
#       엇갈려 정지 신호 **뒤에** 토크 명령이 한 번 더 나가는 일을 막는다.
#     · rclpy 의 SIGINT 처리기를 끈다 (SignalHandlerOptions.NO). 켜 두면 Ctrl+C 때 rclpy 가 먼저 닫혀
#       shutdown() 의 마지막 정지 신호를 못 내고, 토크 모드에서는 마지막 토크 명령이 남는다 (실측).
_COMMAND_TYPES = {
    'twist':  (Twist,              Twist),
    'wheels': (Float32MultiArray,  lambda: Float32MultiArray(data=[0.0] * 4)),
    'torques': (WheelTorqueCommand, lambda: WheelTorqueCommand(mode=WheelTorqueCommand.MODE_DAMPING)),
}


class KeyboardCommandGate(Node):
    """Relay fresh MPC commands only while the gate is armed."""

    def __init__(self):
        super().__init__('mpc_command_gate')
        self.declare_parameter('input_topic', '/mpc/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        # 'twist' = 구조 A (/cmd_vel), 'wheels' = 구조 B (/wheel_cmd, Float32MultiArray[4]),
        # 'torques' = 토크 MPC (/wheel_torque_cmd, WheelTorqueCommand — 정지는 MODE_DAMPING)
        self.declare_parameter('command_type', 'twist')
        self.declare_parameter('arm_topic', '/mpc/arm')
        self.declare_parameter('initially_armed', False)
        self.declare_parameter('keyboard_enabled', True)
        self.declare_parameter('command_timeout', 0.20)
        self.declare_parameter('publish_frequency', 50.0)
        # 2026-10-22: 기본(false)은 타이머가 최근 명령을 publish_frequency 로 다시 낸다 — 입력보다 빠른 MPC 를
        # 거르면 명령을 버리고, 타이머와 MPC 틱이 어긋나 0 ~ 1/publish_frequency 의 무작위 지연이 붙는다
        # (MPC 100 Hz, 게이트 50 Hz 면 명령 절반 유실 + 0~20 ms). true 면 무장 중에는 받는 즉시 내보내고,
        # 타이머는 무장 해제·명령 끊김 때의 0 만 낸다. 무장·신선도·정지 논리는 같다.
        self.declare_parameter('forward_on_receive', False)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        command_type = str(self.get_parameter('command_type').value).lower()
        if command_type not in _COMMAND_TYPES:
            raise ValueError(
                f"command_type={command_type!r} — {sorted(_COMMAND_TYPES)} 중 하나여야 한다"
            )
        self.msg_type, self._zero = _COMMAND_TYPES[command_type]
        arm_topic = str(self.get_parameter('arm_topic').value)
        self.armed = bool(self.get_parameter('initially_armed').value)
        self.keyboard_enabled = bool(
            self.get_parameter('keyboard_enabled').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        publish_frequency = float(
            self.get_parameter('publish_frequency').value
        )
        self.forward_on_receive = bool(self.get_parameter('forward_on_receive').value)

        self.latest_command = self._zero()
        self.latest_command_time = 0.0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.keyboard_thread = None

        self.command_publisher = self.create_publisher(self.msg_type, output_topic, 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_publisher = self.create_publisher(
            Bool,
            '/mpc/armed',
            state_qos,
        )
        self.create_subscription(self.msg_type, input_topic, self._on_command, 10)
        self.create_subscription(Bool, arm_topic, self._on_arm_message, 10)
        self.timer = self.create_timer(
            1.0 / max(publish_frequency, 1.0),
            self._publish_gated_command,
        )

        self._publish_state()
        if self.keyboard_enabled:
            self.keyboard_thread = threading.Thread(
                target=self._keyboard_loop,
                name='mpc-keyboard-arm',
                daemon=True,
            )
            self.keyboard_thread.start()

        initial_state = 'ARMED' if self.armed else 'DISARMED'
        self.get_logger().warning(
            f'MPC command gate is {initial_state}. '
            "Press 'a' to arm; press 's' or SPACE to stop."
        )
        self.get_logger().info(
            f'Command gate [{command_type}]: {input_topic} -> {output_topic}, '
            f'arm topic={arm_topic}, '
            f'{"forward on receive" if self.forward_on_receive else f"resample {publish_frequency:.0f} Hz"}.'
        )

    def _stop_message(self):
        """정지 신호. header 가 있는 타입이면 지금 시각을 찍는다."""
        message = self._zero()
        if hasattr(message, 'header'):
            message.header.stamp = self.get_clock().now().to_msg()
        return message

    def _on_command(self, message):
        with self.lock:
            self.latest_command = copy.deepcopy(message)
            self.latest_command_time = time.monotonic()
            if self.forward_on_receive and self.armed:
                self.command_publisher.publish(message)     # lock 안 — 정지 신호와 순서가 뒤바뀌지 않게

    def _on_arm_message(self, message):
        self._set_armed(bool(message.data), 'ROS arm topic')

    def _set_armed(self, armed, source):
        with self.lock:
            changed = self.armed != armed
            self.armed = armed
            if armed:
                # Require a command received after arming; never release a stale
                # command that was buffered while the robot was disarmed.
                self.latest_command_time = 0.0
            else:
                self.command_publisher.publish(self._stop_message())
        self._publish_state()
        if changed:
            state = 'ARMED' if armed else 'DISARMED'
            self.get_logger().warning(f'MPC command gate {state} by {source}.')

    def _publish_state(self):
        self.state_publisher.publish(Bool(data=self.armed))

    def _publish_gated_command(self):
        now = time.monotonic()
        with self.lock:
            fresh = (
                self.latest_command_time > 0.0
                and now - self.latest_command_time <= self.command_timeout
            )
            if self.armed and fresh:
                if self.forward_on_receive:
                    return              # _on_command 가 이미 받은 즉시 냈다
                command = copy.deepcopy(self.latest_command)
            else:
                command = self._stop_message()
            self.command_publisher.publish(command)

    def _keyboard_loop(self):
        try:
            terminal_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            self.get_logger().warning(
                f'Keyboard arming unavailable: cannot open /dev/tty ({exc}). '
                'Use std_msgs/Bool on /mpc/arm instead.'
            )
            return

        original_settings = None
        try:
            original_settings = termios.tcgetattr(terminal_fd)
            tty.setcbreak(terminal_fd)
            while not self.stop_event.is_set() and rclpy.ok():
                readable, _, _ = select.select([terminal_fd], [], [], 0.10)
                if not readable:
                    continue
                try:
                    key = os.read(terminal_fd, 1).decode(errors='ignore')
                except BlockingIOError:
                    continue
                if key.lower() == 'a':
                    self._set_armed(True, "keyboard 'a'")
                elif key.lower() == 's' or key == ' ':
                    self._set_armed(False, 'keyboard stop key')
        except (OSError, termios.error) as exc:
            if not self.stop_event.is_set():
                self.get_logger().warning(f'Keyboard reader stopped: {exc}')
        finally:
            if original_settings is not None:
                try:
                    termios.tcsetattr(
                        terminal_fd,
                        termios.TCSADRAIN,
                        original_settings,
                    )
                except termios.error:
                    pass
            os.close(terminal_fd)

    def shutdown(self):
        self.stop_event.set()
        if rclpy.ok():
            self._set_armed(False, 'node shutdown')
        else:
            with self.lock:
                self.armed = False
        if self.keyboard_thread is not None:
            self.keyboard_thread.join(timeout=0.5)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)   # 머리주석 R6
    node = KeyboardCommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)    # 정지 신호를 내는 동안 두 번째 Ctrl+C 무시
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
