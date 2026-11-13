from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'amr_mpc'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'tools'), ['tools/setup_can.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smoc',
    maintainer_email='hw030918@pusan.ac.kr',
    description='WP4 trajectory-tracking MPC for the mecanum AMR.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # 운용
            'mpc_tracking_node = amr_mpc.nodes.body_tracking:main',        # 구조 A: 몸체속도 출력 → /cmd_vel
            'wheel_tracking_node = amr_mpc.nodes.wheel_tracking:main',     # 구조 B: 바퀴 각속도 출력 → /wheel_cmd
            'keyboard_command_gate = amr_mpc.nodes.keyboard_command_gate:main',
            # 자기시험·시뮬
            'fake_reference_node = amr_mpc.nodes.fake_reference:main',
            'figure8_reference_node = amr_mpc.nodes.figure8_reference:main',
            'sim_plant_node = amr_mpc.nodes.sim_plant_body:main',
            'sim_plant_wheel_node = amr_mpc.nodes.sim_plant_wheel:main',
            # 토크 MPC 스택 (구조도 v3)
            'sim_plant_torque_node = amr_mpc.nodes.sim_plant_torque:main',
            'torque_output_node = amr_mpc.nodes.torque_output:main',
            'state_estimator_node = amr_mpc.nodes.state_estimator:main',
            'wheel_odometry_node = amr_mpc.nodes.wheel_odometry:main',
            'slip_logger_node = amr_mpc.nodes.slip_logger:main',
            'fault_monitor_node = amr_mpc.nodes.fault_monitor:main',
            # 비교군 A (퓨어 퍼슛 + 바퀴별 PI, 6단계)
            'pure_pursuit_node = amr_mpc.nodes.pure_pursuit:main',
            'wheel_pi_node = amr_mpc.nodes.wheel_pi:main',
            # 실차 모터 (7단계)
            'ak40_can_node = amr_mpc.nodes.ak40_can:main',
        ],
    },
)
