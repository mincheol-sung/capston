#!/usr/bin/env python3
"""amcl_wiggle_node — AMCL 전역 측위를 수렴시킬 최소 이동을 자동으로 만든다.

amcl_bootstrap_node 는 파티클이 수렴하면 /initialpose 를 내지만, 수렴에는 이동이
필요하다. 실제 문턱은 두 곳이다:

    amcl.yaml       update_min_d 0.15 m / update_min_a 0.20 rad
                    -> 이만큼 움직여야 AMCL 이 파티클을 갱신한다
    amcl.yaml       min_travel
                    -> ConvergenceTracker 가 수렴 판정을 시작하는 누적 경로 길이
                       (rotate_only 패턴에서는 0 이어야 한다 — 아래 참조)

amcl_bootstrap_node._on_odometry 는 매 odom 샘플의 hypot 을 더하므로 누적량은
**순변위가 아니라 경로 길이**다. 그래서 제자리에서 왕복만 해도 카운터가 찬다.
0.20 m 짜리 사각형 한 바퀴 = 0.80 m, 두 바퀴 = 1.60 m 로 1.0 m 를 넘긴다.
회전은 이 누적에 기여하지 않는다.

패턴은 세 가지다.

    square        전 -> 우 -> 후 -> 좌. 헤딩 고정이라 출발점으로 정확히 돌아온다.
                  전후 왕복(forward_back)만 하면 그 축 방향 모호성이 안 풀린다 —
                  복도 중앙에서 앞뒤로 20 cm 를 흔들어도 복도 길이 방향 위치
                  후보가 그대로 남아 min_travel 카운터만 차고 position_std 는
                  안 떨어진다. 메카넘이라 측방이 공짜이므로 직교 방향을 같이 넣는다.
    forward_back  측방 공간이 없을 때. laps 를 3 이상으로.
    rotate_only   병진 없이 제자리 회전만. ★ min_travel 0 이 전제다(아래).

★ rotate_only 의 전제와 한계
  누적 이동은 x·y 경로 길이라 제자리 회전은 **0 을 더한다**. 그래서
  amcl.yaml 의 min_travel 이 0 이어야 ConvergenceTracker 가 수렴 판정을 낸다.
  0 이 아니면 파티클이 아무리 모여도 converged 가 영원히 False 다.

  그리고 한 지점에서 본 관측만 쌓인다. 360도 LiDAR 는 제자리에서 돌아도 보는
  장면이 같으므로, 위치 모호성(대칭 복도)은 회전으로 풀리지 않는다. yaw 는
  묶인다 — 스캔이 로봇 프레임 기준이라 회전하면 각도가 시프트되고,
  likelihood_field 정합이 그 자세를 잡아낸다.

  즉 대칭 구간에서는 수렴이 안 되거나(position_std 가 문턱 위에 머문다) 틀린
  곳에 모인다. 전자는 안전한 실패다. 후자를 막는 것이 verify_with_scan 이고,
  180도 뒤집힌 pose 의 실측 점수는 0.200 으로 기준 0.55 에 한참 못 미친다.
  실패하면 square 로 되돌리는 것이 정답이다.

    /odometry/filtered  Odometry              이동량 측정 (구간 완료 판정)
    /initialpose        PoseWithCovarianceStamped
                        <- 이게 오면 즉시 정지하고 물러난다
    /wheel_mpc/cmd_raw  Float32MultiArray[4]  [FL, FR, RL, RR] rad/s
                        게이트(keyboard_command_gate) 의 **입력** 토픽이다
    /mpc/arm            Bool                  게이트 arm

★ 게이트 입력 토픽을 쓰는 이유와 그 대가
  게이트는 disarmed 여도 50 Hz 로 0 을 발행한다(적극적 정지). 그래서 게이트
  출력(/wheel_cmd)에 직접 쓰면 두 발행자가 섞인다. 입력 토픽에 쓰고 arm 하는
  것이 유일한 경로다.
  그런데 wheel_tracking.py 는 reference 가 없어도 같은 입력 토픽에 0 을 계속
  낸다(그 파일 24행 "reference/odom/wheel 끊김 -> 0 출력"). 게이트는 가장 최근
  명령을 내보내므로 둘이 번갈아 나가 덜컹거린다. publish_frequency 를 MPC 보다
  높게 두어 duty 를 확보한다. 20 cm 짜리 캘리브레이션 동작이라 이 정도는
  감당하되, 매끄럽게 하려면 MPC 기동을 /initialpose 이후로 미뤄야 한다.
"""

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from std_msgs.msg import Bool, Float32MultiArray


# 구간 정의: (vx, vy, yaw_rate) 의 단위 방향. 크기는 speed 가 정한다.
# 사각형은 몸체 기준 전 -> 우 -> 후 -> 좌 다. 회전이 없으므로 네 구간이 모두
# 같은 자세에서 일어나고, 출발점으로 정확히 돌아온다.
def _quaternion_yaw(rotation):
    """평면 회전만 남긴 yaw. amcl_bootstrap_node 의 같은 이름 함수와 같은 식."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))


TRANSLATE = 'translate'
ROTATE = 'rotate'

# 사각형은 헤딩을 고정한 채 돈다. 회전이 섞이면 구간마다 몸체 자세가 달라져
# 방향 합이 0 이어도 출발점으로 안 돌아온다.
SQUARE = (
    ('전진', TRANSLATE, (1.0, 0.0, 0.0)),
    ('우측', TRANSLATE, (0.0, -1.0, 0.0)),
    ('후진', TRANSLATE, (-1.0, 0.0, 0.0)),
    ('좌측', TRANSLATE, (0.0, 1.0, 0.0)),
)
FORWARD_BACK = (
    ('전진', TRANSLATE, (1.0, 0.0, 0.0)),
    ('후진', TRANSLATE, (-1.0, 0.0, 0.0)),
)

# 병진이 끝난 뒤 제자리에서 헤딩을 돌린다. 순서가 중요하다 — 회전만 먼저 하면
# amcl_bootstrap_node docstring 의 "대칭 구간에서 엉뚱하게 붙는다" 가 된다.
#
# 왜 회전이 필요한가: 병진만으로는 yaw 공분산이 잘 안 줄어든다. 360° LiDAR 가
# 평행이동만 하면 위치는 잘 묶이지만 자세는 약하게 묶인다. 그런데
# ConvergenceTracker 는 위치와 **yaw 를 함께** 본다 (yaw_std_degrees 8.0).
# 그래서 사각형만 돌면 position_std 는 내려가는데 yaw_std 가 문턱에 걸려
# 수렴 판정이 영원히 안 떨어질 수 있다.
# 회전은 update_min_a(0.20 rad) 를 계속 넘겨 파티클 갱신 횟수도 크게 늘린다 —
# 2 바퀴 = 4π rad 이면 4π / 0.20 ≈ 63 회다.
ROTATION = (
    ('제자리 회전', ROTATE, (0.0, 0.0, 1.0)),
)

# 병진을 아예 하지 않는 패턴. legs = [] + ROTATION 이 되어 회전 구간만 남는다.
#
# ★ 이걸 쓰면 amcl.yaml 의 min_travel 이 반드시 0 이어야 한다.
#   ConvergenceTracker.add_travel 은 /odometry/filtered 의 x·y hypot 누적이라
#   제자리 회전은 0 을 더한다. converged = travel_satisfied and streak >= N
#   이므로 min_travel 이 0 보다 크면 수렴 판정이 영원히 안 난다.
#
# 대칭 구간 모호성은 verify_with_scan 이 대신 막는다 (min_match_score 0.55,
# 180도 뒤집힌 pose 의 실측 점수 0.200).
ROTATE_ONLY = ()

PATTERNS = {
    'square': SQUARE,
    'forward_back': FORWARD_BACK,
    'rotate_only': ROTATE_ONLY,
}


def mecanum_inverse(vx, vy, yaw_rate, wheel_radius, k):
    """몸체 속도를 [FL, FR, RL, RR] rad/s 로 바꾼다.

    amr_mpc.core.kinematics.mecanum_matrices 의 T_inv 와 같은 식·같은 순서다.
    그 모듈을 import 하지 않는 이유는 인지 패키지가 제어 패키지에 의존하지
    않게 하려는 것뿐이다. 부호가 갈리면 로봇이 엉뚱하게 움직이므로
    test_amcl_wiggle.py 가 두 구현을 직접 비교한다.
    """
    return (np.array([
        [1.0, -1.0, -k],   # FL
        [1.0, 1.0, k],     # FR
        [1.0, 1.0, -k],    # RL
        [1.0, -1.0, k],    # RR
    ], dtype=np.float64) @ np.array([vx, vy, yaw_rate], dtype=np.float64)
    ) / float(wheel_radius)


class AmclWiggleNode(Node):
    """AMCL 수렴용 최소 이동을 만들고, /initialpose 가 나오면 물러난다."""

    def __init__(self):
        super().__init__('amcl_wiggle_node')

        odom_topic = self._declare('odometry_topic', '/odometry/filtered')
        self.initialpose_topic = self._declare('initialpose_topic',
                                               '/initialpose')
        wheel_cmd_topic = self._declare('wheel_cmd_topic',
                                        '/wheel_mpc/cmd_raw')
        arm_topic = self._declare('arm_topic', '/mpc/arm')

        # 기하. plant_wheel.yaml geometry 와 같은 값이어야 한다.
        #   wheel_radius_m 0.04 · wheel_base_m 0.1897 · robot_width_m 0.207
        self.wheel_radius = float(self._declare('wheel_radius', 0.04))
        wheel_base = float(self._declare('wheel_base', 0.1897))
        robot_width = float(self._declare('robot_width', 0.207))
        self.k = (wheel_base + robot_width) / 2.0

        self.pattern_name = str(self._declare('pattern', 'square'))
        self.leg_distance = float(self._declare('leg_distance', 0.20))
        self.speed = float(self._declare('speed', 0.10))
        self.laps = int(self._declare('laps', 2))
        # 병진 뒤 제자리 회전. 0 이면 회전 단계를 건너뛴다.
        self.rotate_revolutions = float(
            self._declare('rotate_revolutions', 2.0))
        # 회전 각속도 [rad/s]. 0.5 에서 바퀴는 0.5 × k / r = 2.48 rad/s 로
        # 병진 0.10 m/s 와 같은 크기다(한계 30 의 8 %).
        self.yaw_rate = float(self._declare('yaw_rate', 0.5))
        # 게이트·드라이버가 뜨기 전에 명령을 쏘면 첫 구간이 버려진다.
        self.start_delay = float(self._declare('start_delay', 5.0))
        # 안전 상한. 바퀴가 헛돌거나 odom 이 안 오면 여기서 끝낸다.
        self.max_duration = float(self._declare('max_duration', 90.0))
        # odom 이 이보다 오래 끊기면 즉시 정지한다.
        self.odom_timeout = float(self._declare('odom_timeout', 0.5))
        self.publish_frequency = float(self._declare('publish_frequency',
                                                     100.0))
        # 끝나면 게이트를 다시 내린다. 자율주행 전에 사람이 arm 하게 남긴다.
        self.disarm_when_done = bool(self._declare('disarm_when_done', True))
        # 바퀴 명령 상한. base.yaml / plant_wheel.yaml limits 와 같게.
        self.max_wheel_rad_per_sec = float(
            self._declare('max_wheel_rad_per_sec', 30.0))

        if self.leg_distance <= 0.0:
            raise ValueError('leg_distance must be positive')
        if self.speed <= 0.0:
            raise ValueError('speed must be positive')
        if self.laps < 1:
            raise ValueError('laps must be at least 1')
        if self.rotate_revolutions < 0.0:
            raise ValueError('rotate_revolutions must not be negative')
        if self.rotate_revolutions > 0.0 and self.yaw_rate <= 0.0:
            raise ValueError('yaw_rate must be positive to rotate')
        if self.publish_frequency <= 0.0:
            raise ValueError('publish_frequency must be positive')
        if self.pattern_name not in PATTERNS:
            raise ValueError(
                f'pattern must be one of {sorted(PATTERNS)}, '
                f'got {self.pattern_name!r}')
        if self.pattern_name == 'rotate_only' and self.rotate_revolutions <= 0.0:
            raise ValueError(
                'pattern "rotate_only" 는 rotate_revolutions > 0 이어야 한다 — '
                '병진 구간이 없으므로 회전까지 끄면 할 일이 하나도 없다')

        pattern = PATTERNS[self.pattern_name]
        self.legs = list(pattern) * self.laps
        self.rotate_target = 2.0 * math.pi * self.rotate_revolutions
        if self.rotate_target > 0.0:
            self.legs += list(ROTATION)

        self._leg_index = 0
        self._leg_start = None      # 병진 구간 시작 시점의 odom (x, y)
        self._position = None
        self._yaw = None            # 최신 odom yaw [rad]
        self._turned = 0.0          # 이번 회전 구간의 누적 |Δyaw| [rad]
        self._last_odom = 0.0
        self._travel = 0.0          # 누적 경로 길이. 부트스트랩 노드와 같은 정의
        self._done = False
        self._armed = False
        self._started_at = time.monotonic()

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.wheel_publisher = self.create_publisher(
            Float32MultiArray, wheel_cmd_topic, 10)
        self.arm_publisher = self.create_publisher(Bool, arm_topic, latched)
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic,
            self._on_initial_pose, latched)
        self.create_timer(1.0 / self.publish_frequency, self._tick)

        translate_legs = sum(1 for _, kind, _ in self.legs
                             if kind == TRANSLATE)
        planned = self.leg_distance * translate_legs
        rotation_text = (
            (' 그 뒤' if translate_legs else '')
            + f' 제자리 회전 {self.rotate_revolutions:.1f} 바퀴 '
            f'({math.degrees(self.rotate_target):.0f}°) @ '
            f'{self.yaw_rate:.2f} rad/s.'
            if self.rotate_target > 0.0 else ' 회전 단계 없음.'
        )
        plan_text = (
            f'{self.pattern_name} {self.leg_distance:.2f} m '
            f'× {len(pattern)} 구간 × {self.laps} 바퀴 = 경로 길이 '
            f'{planned:.2f} m, {self.speed:.2f} m/s.'
            if translate_legs else
            f'{self.pattern_name} — 병진 구간 없음(경로 길이 0 m).'
        )
        self.get_logger().info(
            f'amcl_wiggle_node: {plan_text}{rotation_text}'
            f' {self.start_delay:.1f} s 뒤 시작하고 {self.initialpose_topic} 가 '
            f'오면 즉시 정지한다.'
        )
        if not translate_legs:
            self.get_logger().warning(
                '병진 구간이 없다. amcl.yaml 의 min_travel 이 0 이어야 '
                'amcl_bootstrap_node 가 수렴 판정을 낸다 — 누적 이동은 x·y '
                '경로 길이라 제자리 회전은 0 을 더한다. 그리고 한 지점에서 본 '
                '관측만 쌓이므로 대칭 구간 모호성은 verify_with_scan 이 막는다.'
            )
        elif planned < 1.0:
            self.get_logger().warning(
                f'계획 경로 길이 {planned:.2f} m 가 amcl.yaml 의 min_travel '
                f'1.0 m 보다 짧다. laps 를 늘리거나 leg_distance 를 키울 것.'
            )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------ #

    def _on_odometry(self, message):
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self._position is not None:
            self._travel += math.hypot(current[0] - self._position[0],
                                       current[1] - self._position[1])
        self._position = current

        yaw = _quaternion_yaw(message.pose.pose.orientation)
        if self._yaw is not None:
            # 누적은 wrap 한 증분의 절댓값 합이다. yaw 를 그냥 빼면 ±pi 경계에서
            # 2pi 가 튀어 한 바퀴를 돌 때마다 목표를 순간 달성한 것으로 읽는다.
            delta = math.atan2(math.sin(yaw - self._yaw),
                               math.cos(yaw - self._yaw))
            self._turned += abs(delta)
        self._yaw = yaw
        self._last_odom = time.monotonic()

    def _on_initial_pose(self, message):
        """AMCL 이 수렴했다. 더 움직일 이유가 없다."""
        del message
        if self._done:
            return
        self._finish(
            f'{self.initialpose_topic} 수신 — 수렴했다. 누적 경로 '
            f'{self._travel:.2f} m'
        )

    # ------------------------------------------------------------------ #

    def _stop_command(self):
        return [0.0, 0.0, 0.0, 0.0]

    def _publish_wheels(self, values):
        message = Float32MultiArray()
        # rclpy 의 float32[] 는 numpy 스칼라를 받지 않는다. Python float 로.
        message.data = [float(v) for v in values]
        self.wheel_publisher.publish(message)

    def _set_armed(self, armed):
        if self._armed == armed:
            return
        self._armed = armed
        message = Bool()
        message.data = bool(armed)
        self.arm_publisher.publish(message)
        self.get_logger().info(f'게이트 {"arm" if armed else "disarm"}')

    def _finish(self, reason):
        self._done = True
        # 정지 명령을 먼저 내고 나서 arm 을 내린다. 순서를 바꾸면 게이트가
        # 마지막 이동 명령을 들고 있는 채로 disarm 된다.
        self._publish_wheels(self._stop_command())
        if self.disarm_when_done:
            self._set_armed(False)
        self.get_logger().info(f'wiggle 종료 — {reason}')
        if self.disarm_when_done:
            self.get_logger().info(
                '게이트를 내렸다. 자율주행에는 다시 arm 이 필요하다: '
                "ros2 topic pub --once /mpc/arm std_msgs/msg/Bool '{data: true}'"
            )

    # ------------------------------------------------------------------ #

    def _tick(self):
        if self._done:
            return

        elapsed = time.monotonic() - self._started_at
        if elapsed < self.start_delay:
            return
        if elapsed - self.start_delay > self.max_duration:
            self._finish(
                f'max_duration {self.max_duration:.0f} s 초과. 누적 경로 '
                f'{self._travel:.2f} m — 바퀴가 헛돌거나 odom 이 이상하다'
            )
            return

        if self._position is None:
            self.get_logger().warning(
                'odom 을 아직 못 받았다. 이동을 시작하지 않는다.',
                throttle_duration_sec=5.0)
            return
        if time.monotonic() - self._last_odom > self.odom_timeout:
            self._publish_wheels(self._stop_command())
            self.get_logger().warning(
                f'odom 이 {self.odom_timeout:.1f} s 이상 끊겼다. 정지한다.',
                throttle_duration_sec=2.0)
            return

        if self._leg_index >= len(self.legs):
            self._finish(
                f'계획한 구간을 모두 돌았다. 누적 경로 {self._travel:.2f} m. '
                f'그런데 {self.initialpose_topic} 가 아직 안 왔다 — 수렴이 '
                f'덜 됐다면 laps 를 늘리거나 텔레옵으로 더 주행할 것'
            )
            return

        label, kind, direction = self.legs[self._leg_index]

        if self._leg_start is None:
            self._leg_start = self._position
            self._turned = 0.0
            amount = (f'{self.leg_distance:.2f} m' if kind == TRANSLATE
                      else f'{math.degrees(self.rotate_target):.0f}°')
            self.get_logger().info(
                f'구간 {self._leg_index + 1}/{len(self.legs)} {label} {amount}'
            )
            self._set_armed(True)

        if kind == TRANSLATE:
            done = math.hypot(self._position[0] - self._leg_start[0],
                              self._position[1] - self._leg_start[1])
            target = self.leg_distance
            body = (direction[0] * self.speed,
                    direction[1] * self.speed,
                    0.0)
        else:
            # 회전 완료는 누적 |Δyaw| 로 본다. 시작 yaw 와의 차이로 재면 한 바퀴
            # 마다 0 으로 돌아와 영원히 안 끝난다.
            done = self._turned
            target = self.rotate_target
            body = (0.0, 0.0, direction[2] * self.yaw_rate)

        if done >= target:
            self._leg_index += 1
            self._leg_start = None
            self._turned = 0.0
            self._publish_wheels(self._stop_command())
            return

        wheels = mecanum_inverse(
            body[0], body[1], body[2], self.wheel_radius, self.k)
        peak = float(np.max(np.abs(wheels)))
        if peak > self.max_wheel_rad_per_sec:
            wheels = wheels * (self.max_wheel_rad_per_sec / peak)
            self.get_logger().warning(
                f'바퀴 명령이 한계를 넘어 {self.max_wheel_rad_per_sec:.0f} '
                f'rad/s 로 줄였다. speed 를 낮출 것.',
                throttle_duration_sec=5.0)
        self._publish_wheels(wheels)


def main(args=None):
    rclpy.init(args=args)
    node = AmclWiggleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 어떤 경로로 나가든 바퀴를 세우고 게이트를 내린다.
        try:
            node._publish_wheels(node._stop_command())
            node._set_armed(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
