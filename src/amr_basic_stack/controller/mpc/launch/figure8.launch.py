#!/usr/bin/env python3
"""8자 경로 주행: figure8_reference_node + mpc_tracking_node.
   기동 포즈가 8자의 교차점이고, 한 바퀴 뒤 그 포즈(위치·헤딩)로 돌아와 정지한다.

   실차:  ros2 launch amr_mpc figure8.launch.py
   시뮬:  ros2 launch amr_mpc figure8.launch.py sim:=true      (sim_plant_node 를 같이 띄운다)
   예:    ros2 launch amr_mpc figure8.launch.py width:=1.2 height:=0.8 speed:=0.2 yaw_mode:=fixed
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
        DeclareLaunchArgument('width', default_value='1.4', description='8자 경계상자 가로 [m].'),
        DeclareLaunchArgument('height', default_value='0.8', description='8자 경계상자 세로 [m].'),
        DeclareLaunchArgument('speed', default_value='0.25', description='순항 접선속도 [m/s].'),
        DeclareLaunchArgument('accel', default_value='0.4', description='호길이 축 가감속 [m/s²].'),
        DeclareLaunchArgument('laps', default_value='1.0', description='바퀴 수.'),
        DeclareLaunchArgument('yaw_mode', default_value='tangent', description='tangent | fixed.'),
        DeclareLaunchArgument('align_s', default_value='1.5', description='tangent 앞뒤 제자리 회전 [s].'),
        DeclareLaunchArgument('start_delay', default_value='3.0', description='첫 odom 뒤 출발까지 대기 [s].'),
        DeclareLaunchArgument('abort_error', default_value='0.35', description='기준과 이만큼 벌어지면 중단 [m]. 0 이하면 끔.'),
        DeclareLaunchArgument('hold_s', default_value='5.0', description='복귀 후 제자리 유지 [s]. <0 이면 무한.'),
        # /odom 이 기본이다 — model.py 의 FOPDT(θ=40 ms, d=1)가 sys_id/wp2_step_logger.py 에서
        # /odom 으로 식별됐고, run_circle.sh·circle.launch.py 도 /odom 을 쓴다.
        # /odometry/filtered 는 EKF(50 Hz, process_noise 미튜닝, yaw 는 자이로 적분만)를 끼워
        # 모델에 없는 지연을 넣는다. 시뮬에서 그 지연 120 ms 를 주면 직선·원까지 부하 100 %,
        # QP 800 회로 발산한다 (2026-09-03 실차 발산과 같은 서명).
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('log_prefix', default_value='mpc_fig8', description='CSV 접두. 비면 저장 안 함.'),
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
             parameters=[{'odom_topic': lc('odom_topic')}]),   # 실차와 같은 토픽으로 내보낸다
        Node(package='amr_mpc', executable='figure8_reference_node', name='figure8_reference_node',
             output='screen',
             parameters=[{'width': lc('width'), 'height': lc('height'), 'speed': lc('speed'),
                          'accel': lc('accel'), 'laps': lc('laps'), 'yaw_mode': lc('yaw_mode'),
                          'align_s': lc('align_s'), 'start_delay': lc('start_delay'),
                          'abort_error': lc('abort_error'),
                          'hold_s': lc('hold_s'),
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
