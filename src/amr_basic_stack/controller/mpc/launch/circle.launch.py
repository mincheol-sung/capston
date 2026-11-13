#!/usr/bin/env python3
"""원궤적 자기시험: fake_reference_node + mpc_tracking_node.  (run_circle.sh 가 감싼다)

기동 포즈를 원점으로 반시계 원을 돈다. 플래너 없이 MPC 를 시험하되, 기준열 → 보간 → MPC
경로를 운용과 똑같이 타므로 여기서 나온 수치가 운용에 그대로 읽힌다.
(2026-09-04 이전에는 mpc_circle_node 가 MPC 루프를 따로 갖고 있어 제약·게인이 갈라졌다.)

   실차:  ./run_circle.sh 1.0 0.5 2          ← 부하 예산 검사까지 해준다. 이쪽을 써라.
   시뮬:  ros2 launch amr_mpc circle.launch.py sim:=true radius:=1.0 speed:=0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('amr_mpc'), 'config', 'mpc.yaml')
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='1.0', description='원 반지름 [m].'),
        DeclareLaunchArgument('speed', default_value='0.15', description='접선속도 [m/s].'),
        DeclareLaunchArgument('ramp_s', default_value='1.0', description='0→speed 가감속 시간 [s].'),
        DeclareLaunchArgument('duration', default_value='0.0',
                              description='주행 시간 [s]. 0 이면 계속. 바퀴 수로 주려면 run_circle.sh.'),
        # 자기시험은 /odom 을 쓴다 — model 의 θ 가 /odom 으로 식별됐다. README 참조.
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('log_prefix', default_value='mpc_circle',
                              description='CSV 접두. 비면 저장 안 함.'),
        # 제어 노드를 전용 코어에 고정한다. 부하 있는 실차에서 꼬리(느린 틱)를 직접 줄인다.
        #   예:  mpc_prefix:='taskset -c 10,11'
        #   실측 (12 코어 중 10 개에 CPU 부하 83 %, 8자 sim, 682 틱):
        #     고정 없음   step p50 16.6  p99 25.9  max 31.6 ms   20 ms 초과 116 틱
        #     10,11 고정  step p50  1.8  p99  3.3  max 12.6 ms   20 ms 초과   0 틱
        #   코드는 그대로다 — 느린 틱의 원인이 알고리즘이 아니라 CPU 경쟁이라는 증거다.
        DeclareLaunchArgument('mpc_prefix', default_value='',
                              description="제어 노드 실행 접두. 예: 'taskset -c 10,11'"),
        DeclareLaunchArgument('sim', default_value='false', description='true 면 sim_plant_node 도 띄운다.'),

        Node(package='amr_mpc', executable='sim_plant_node', name='sim_plant_node',
             output='screen', condition=IfCondition(lc('sim')),
             parameters=[{'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc', executable='fake_reference_node', name='fake_reference_node',
             output='screen',
             parameters=[{'radius': lc('radius'), 'speed': lc('speed'),
                          'ramp_s': lc('ramp_s'), 'duration': lc('duration'),
                          'odom_topic': lc('odom_topic')}]),
        Node(package='amr_mpc', executable='mpc_tracking_node', name='mpc_tracking_node',
             # numpy/OpenBLAS 스레드를 1 로 묶는다. 결정변수 15~25, 행렬 최대 100×25 라
             # 스레드 동기화가 계산보다 비싸고, 제한이 없으면 코어 수(12)만큼 스레드를 띄워
             # 다른 노드의 코어를 잠식한다.
             #   실측 (8자 기준궤적, step 600 회):
             #     제한 없음  p50 1.243  p99 1.291  max 2.324 ms   OS 스레드 12
             #     1 스레드   p50 0.664  p99 0.719  max 0.772 ms   OS 스레드  1
             #   1.9 배 빠르면서 코어를 1 개만 쓴다 — 이 크기에서는 순수 이득이다.
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
                             'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'},
             output='screen', prefix=lc('mpc_prefix'),
             parameters=[cfg, {'odom_topic': lc('odom_topic'), 'cmd_vel_topic': lc('cmd_vel_topic'),
                               'log_prefix': lc('log_prefix')}]),
    ])
