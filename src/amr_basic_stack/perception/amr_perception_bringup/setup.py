import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_perception_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py') + glob('tools/*.sh')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml') + glob('config/*.lua')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yumin',
    maintainer_email='yumin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_to_odom_node = amr_perception_bringup.pose_to_odom_node:main',
            'imu_to_base_node = amr_perception_bringup.imu_to_base_node:main',
            'finish_map = amr_perception_bringup.finish_map:main',
            'set_initial_pose_node = amr_perception_bringup.set_initial_pose_node:main',
            'carto_debug = amr_perception_bringup.carto_debug:main',
            'amcl_bootstrap_node = amr_perception_bringup.amcl_bootstrap_node:main',
            'amcl_wiggle_node = amr_perception_bringup.amcl_wiggle_node:main',
        ],
    },
)
