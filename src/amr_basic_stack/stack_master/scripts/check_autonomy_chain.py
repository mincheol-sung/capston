#!/usr/bin/env python3
"""goal_pose 를 찍었는데 로봇이 안 움직일 때, 사슬의 어느 고리가 끊겼는지 찾는다.

    python3 check_autonomy_chain.py            # 5 초 관측
    python3 check_autonomy_chain.py 10         # 10 초 관측

사슬 (구조 B = mpc_structure:=wheel 기본):

    /goal_pose                RViz 2D Goal Pose
        ↓  rviz_theta_star_node
    /global_path              ★ require_initial_pose: true — /initialpose 를 받은
                                적이 없으면 목표를 버리고 **빈 경로**를 낸다
        ↓  dra_mppi_node (10 Hz)
    /dra_mppi/reference_trajectory   ★ 경로가 비면 정지 기준열만 낸다
        ↓  wheel_tracking (구조 B)
    /wheel_mpc/cmd_raw        ★ reference/odom/wheel 중 하나라도 끊기면 0 을 낸다
        ↓  keyboard_command_gate (50 Hz)
    /mpc/armed  /wheel_cmd    ★ disarmed 면 0 을 계속 낸다 (적극적 정지)
        ↓  roboclaw_mecanum_node
    실제 바퀴

여기서 못 잡는 것: 초기위치가 **틀린** 경우(수렴은 했으나 엉뚱한 곳). 그건
tf_jump.py 와 check_localization.py 가 본다.
"""

import sys
import time

from amr_interfaces.msg import PlanarReferenceArray

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

from nav_msgs.msg import Odometry, Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from std_msgs.msg import Bool, Float32MultiArray


LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ChainProbe(Node):
    """사슬의 각 고리를 동시에 구독해 개수·속도·값을 모은다."""

    def __init__(self):
        super().__init__('check_autonomy_chain')
        self.seen = {}
        self.note = {}

        def watch(name, msg_type, qos, summarize=None):
            self.seen[name] = 0
            self.note[name] = ''

            def callback(message, name=name, summarize=summarize):
                self.seen[name] += 1
                if summarize is not None:
                    self.note[name] = summarize(message)

            self.create_subscription(msg_type, name, callback, qos)

        def path_note(message):
            return f'poses={len(message.poses)}'

        def ref_note(message):
            n = len(message.references)
            if n == 0:
                return 'references=0'
            speeds = [abs(r.vx) + abs(r.vy) + abs(r.yaw_rate)
                      for r in message.references]
            return f'references={n} max|v|={max(speeds):.3f}'

        def wheels_note(message):
            values = list(message.data)
            if len(values) != 4:
                return f'len={len(values)} (드라이버가 무시한다)'
            return 'max|w|={:.3f} rad/s'.format(max(abs(v) for v in values))

        def twist_note(message):
            return 'max|v|={:.3f}'.format(max(
                abs(message.linear.x), abs(message.linear.y),
                abs(message.angular.z)))

        def bool_note(message):
            return 'armed=True' if message.data else 'armed=False'

        # RViz 도구는 volatile 이라 과거 메시지는 못 본다 — 관측 중에 다시 찍어야 보인다.
        watch('/initialpose', PoseWithCovarianceStamped, 10)
        watch('/goal_pose', PoseStamped, 10)
        watch('/global_path', Path, LATCHED, path_note)
        watch('/odometry/filtered', Odometry, 20)
        watch('/dra_mppi/reference_trajectory', PlanarReferenceArray, 10,
              ref_note)
        watch('/wheel_mpc/cmd_raw', Float32MultiArray, 10, wheels_note)
        watch('/mpc/armed', Bool, LATCHED, bool_note)
        watch('/wheel_cmd', Float32MultiArray, 10, wheels_note)
        watch('/measured_wheel_speeds', Float32MultiArray, 10, wheels_note)
        watch('/cmd_vel', Twist, 10, twist_note)      # 있으면 안 된다(구조 B)


# 고리별 판정. (토픽, 기대 최소 개수, 끊겼을 때 볼 곳)
EXPECT = [
    ('/odometry/filtered', 1,
     'EKF·드라이버가 죽었다. use_base / mock_mode 와 roboclaw 로그를 볼 것'),
    ('/global_path', 1,
     'rviz_theta_star_node 가 목표를 안 받았거나 계획에 실패했다'),
    ('/dra_mppi/reference_trajectory', 1,
     'dra_mppi_node 가 안 떴다. use_dra_mppi 확인'),
    ('/wheel_mpc/cmd_raw', 1,
     'wheel_tracking 이 안 떴다. start_mpc / mpc_structure 확인'),
    ('/mpc/armed', 1,
     'keyboard_command_gate 가 안 떴다. use_command_gate 확인'),
    ('/wheel_cmd', 1,
     '게이트가 출력을 안 낸다'),
    ('/measured_wheel_speeds', 1,
     '드라이버 encoder 가 안 온다. 구조 B 의 속도 피드백이 없으면 MPC 는 0 을 낸다'),
]


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = ChainProbe()
    print(f'{duration:.0f} 초 관측한다. 그 사이 RViz 에서 2D Goal Pose 를 다시 '
          f'찍으면 /goal_pose 도 잡힌다.\n')
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'{"토픽":34s} {"개수":>6s} {"Hz":>7s}  비고')
    print('-' * 88)
    for name in node.seen:
        count = node.seen[name]
        rate = count / duration
        print(f'{name:34s} {count:6d} {rate:7.1f}  {node.note[name]}')

    print()
    broken = None
    for name, minimum, hint in EXPECT:
        if node.seen[name] < minimum:
            broken = (name, hint)
            break

    if broken is None:
        print('사슬은 전부 살아 있다. 값이 0 인 고리를 위 표의 "비고" 에서 찾을 것.')
        armed = node.note['/mpc/armed'] == 'armed=True'
        path_empty = node.note['/global_path'] == 'poses=0'
        if path_empty:
            print()
            print('★ /global_path 가 **빈 경로**다. rviz_theta_star_node 가 목표를')
            print('  버렸다는 뜻이다. 가장 흔한 원인은 초기위치다 —')
            print('  theta_star.yaml 의 require_initial_pose: true 라서')
            print('  /initialpose 를 한 번도 못 받으면 _on_goal 이 경고만 남기고')
            print('  빈 경로를 낸다. 그 노드 로그에서 이 문장을 찾을 것:')
            print('    "Set RViz 2D Pose Estimate before sending 2D Goal Pose."')
            print('  해결: use_amcl_wiggle:=true 로 수렴을 기다리거나, RViz 의')
            print('        2D Pose Estimate 로 직접 찍는다.')
        if not armed:
            print()
            print('★ 게이트가 disarmed 다. 터미널 a 가 안 먹었을 수 있다 —')
            print('  게이트는 stdin 이 아니라 /dev/tty 를 직접 읽고, ros2 launch')
            print('  와 같은 tty 를 공유하면 키를 놓칠 수 있다. 토픽으로 확실하게:')
            print("    ros2 topic pub --once /mpc/arm std_msgs/msg/Bool '{data: true}'")
        if node.seen['/cmd_vel'] > 0:
            print()
            print('★ /cmd_vel 에 발행자가 있다. 구조 B 에서는 없어야 한다 —')
            print('  드라이버가 cmd_vel 과 wheel_cmd 를 **둘 다** 구독하고 중재 없이')
            print('  마지막 메시지를 실행한다. use_joystick:=false 로 띄울 것.')
    else:
        name, hint = broken
        print(f'★ 여기서 끊겼다: {name}')
        print(f'  {hint}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
