#!/usr/bin/env python3
"""Gamepad teleop for the mecanum base.

/cmd_vel 소유권
---------------
할 말이 있을 때만 발행한다. 스틱을 놓으면 정지를 **한 번** 보내고 버스를 놓는다.
그 뒤 안전은 드라이버의 cmd_timeout_s(base.yaml, 0.5 s)가 맡는다 — 명령이 끊기면
드라이버가 스스로 멈춘다. 그러니 0 을 계속 쏠 이유가 없다.

왜 이렇게 바꿨나 (2026-09-03)
  joy_node 의 autorepeat_rate 가 20 Hz 라, 게임패드 수신기가 꽂혀 있으면 아무 조작이
  없어도 /joy 가 20 Hz 로 계속 온다. 그래서 _last_joy_time 이 늘 갱신되고 joy_timeout_s
  분기도 안 타서, 이 노드가 중립인 0 을 영원히 20 Hz 로 /cmd_vel 에 쏘고 있었다.
  25 Hz MPC 와 섞이면 드라이버가 받는 명령의 절반 가까이가 0 이 되어, 로봇이 명령의
  일부만 실행하고 MPC 는 "반응이 없다"로 읽어 적분기가 감기며 포화한다.
  실측(제자리 회전 ±0.5 rad/s 계단, /cmd_vel 대비 상호상관):
      /odom wz 이득  단독 0.97  ->  경쟁 0.83     (MPC 는 25 Hz 라 더 나쁘다)
  8자 추종이 이 상태에서 발산했고, 원인 추적에 한나절이 들었다.

※ 여러 노드가 /cmd_vel 을 쓰는 구성에서는 이것만으로 충분하지 않다. 소유권을 강제하려면
  amr_mpc 의 keyboard_command_gate 를 쓰고, autonomy.launch.py 처럼 use_joystick:=false
  로 띄운다. mpc_tracking_node 는 발행자가 둘 이상이면 ERROR 를 낸다.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")

        self.declare_parameter("max_linear_vel", 0.3)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("enable_axis_control", True)
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_z", 3)
        self.declare_parameter("scale_linear_x", 1.0)
        self.declare_parameter("scale_linear_y", 1.0)
        self.declare_parameter("scale_angular_z", 1.0)
        self.declare_parameter("axis_deadzone", 0.08)
        self.declare_parameter("enable_button_control", True)
        self.declare_parameter("button_forward", 3)
        self.declare_parameter("button_backward", 0)
        self.declare_parameter("button_left", 2)
        self.declare_parameter("button_right", 1)
        self.declare_parameter("button_rotate_ccw", 4)
        self.declare_parameter("button_rotate_cw", 5)
        self.declare_parameter("enable_button", -1)
        self.declare_parameter("joy_timeout_s", 0.5)

        self._max_v = float(self.get_parameter("max_linear_vel").value)
        self._max_w = float(self.get_parameter("max_angular_vel").value)
        self._axis_enabled = bool(self.get_parameter("enable_axis_control").value)
        self._axis_x = int(self.get_parameter("axis_linear_x").value)
        self._axis_y = int(self.get_parameter("axis_linear_y").value)
        self._axis_w = int(self.get_parameter("axis_angular_z").value)
        self._scale_x = float(self.get_parameter("scale_linear_x").value)
        self._scale_y = float(self.get_parameter("scale_linear_y").value)
        self._scale_w = float(self.get_parameter("scale_angular_z").value)
        self._axis_deadzone = float(self.get_parameter("axis_deadzone").value)
        self._buttons_enabled = bool(self.get_parameter("enable_button_control").value)
        self._btn_fwd = int(self.get_parameter("button_forward").value)
        self._btn_bwd = int(self.get_parameter("button_backward").value)
        self._btn_left = int(self.get_parameter("button_left").value)
        self._btn_right = int(self.get_parameter("button_right").value)
        self._btn_ccw = int(self.get_parameter("button_rotate_ccw").value)
        self._btn_cw = int(self.get_parameter("button_rotate_cw").value)
        self._enable_button = int(self.get_parameter("enable_button").value)
        self._joy_timeout_s = float(self.get_parameter("joy_timeout_s").value)

        self._cmd = Twist()
        self._last_joy_time = None
        self._was_active = False        # 직전 주기에 0 이 아닌 명령을 냈나

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Joy, "joy", self._joy_cb, 10)
        self.create_timer(0.05, self._publish)

        self.get_logger().info(
            "Joy teleop started: left stick x/y strafe/forward, right stick x rotate, "
            "buttons still enabled "
            f"(linear {self._max_v} m/s, angular {self._max_w} rad/s)"
        )

    @staticmethod
    def _pressed(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def _enabled(self, msg: Joy) -> bool:
        return self._enable_button < 0 or self._pressed(msg, self._enable_button)

    def _axis(self, msg: Joy, index: int) -> float:
        if index < 0 or index >= len(msg.axes):
            return 0.0
        value = float(msg.axes[index])
        return 0.0 if abs(value) < self._axis_deadzone else value

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _joy_cb(self, msg: Joy):
        cmd = Twist()
        if self._enabled(msg):
            if self._axis_enabled:
                cmd.linear.x += self._max_v * self._scale_x * self._axis(msg, self._axis_x)
                cmd.linear.y += self._max_v * self._scale_y * self._axis(msg, self._axis_y)
                cmd.angular.z += self._max_w * self._scale_w * self._axis(msg, self._axis_w)

            # scale_* 을 축과 똑같이 곱한다. 2026-09-16 이전에는 이 세 줄에만
            # scale 이 빠져 있어서 버튼 입력이 max_* 를 그대로 냈다.
            # joy_teleop.yaml 이 axis_angular_z: -1 (축 비활성) 이라 회전은
            # 전부 버튼으로 들어왔고, 결과적으로 scale_angular_z 가 회전에
            # 아무 영향도 못 줬다 — bag map_hq_20260916_001124 에서 회전 명령
            # 875 개가 전부 정확히 1.000 rad/s (= max_angular_vel) 였다.
            if self._buttons_enabled:
                cmd.linear.x += self._max_v * self._scale_x * (
                    self._pressed(msg, self._btn_fwd) - self._pressed(msg, self._btn_bwd)
                )
                cmd.linear.y += self._max_v * self._scale_y * (
                    self._pressed(msg, self._btn_left) - self._pressed(msg, self._btn_right)
                )
                cmd.angular.z += self._max_w * self._scale_w * (
                    self._pressed(msg, self._btn_ccw) - self._pressed(msg, self._btn_cw)
                )

        # 축과 버튼이 동시에 눌리면 합쳐지므로 여기서 한 번 더 자른다.
        # 상한은 max_* 가 아니라 max_* x scale_* 이다 — yaml 의 계약이
        # "max_* 는 하드웨어 한계, scale_* 이 그 중 실제로 쓰는 비율" 이다.
        cmd.linear.x = self._clamp(cmd.linear.x, self._max_v * self._scale_x)
        cmd.linear.y = self._clamp(cmd.linear.y, self._max_v * self._scale_y)
        cmd.angular.z = self._clamp(cmd.angular.z, self._max_w * self._scale_w)
        self._cmd = cmd
        self._last_joy_time = self.get_clock().now()

    @staticmethod
    def _is_idle(cmd: Twist, eps: float = 1e-6) -> bool:
        return (abs(cmd.linear.x) < eps and abs(cmd.linear.y) < eps
                and abs(cmd.angular.z) < eps)

    def _publish(self):
        if self._last_joy_time is None:
            return

        age = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        if age > self._joy_timeout_s:
            self._cmd = Twist()

        if self._is_idle(self._cmd):
            if self._was_active:                  # 방금 놓았다 -> 정지 한 번만 확실히
                self._pub.publish(Twist())
                self._was_active = False
            return                                # 그 뒤엔 버스를 놓는다
        self._pub.publish(self._cmd)
        self._was_active = True


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
