from glob import glob

from setuptools import find_packages, setup

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
        ('share/' + package_name + '/docs',
            glob('docs/*.md') + glob('docs/observation_timing_validation.txt')),
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
            'rgbd_detection_node = sensor_fusion.rgbd_detection_node:main',
            'detection_markers = sensor_fusion.detection_markers_node:main',
            'lidar_cluster_node = sensor_fusion.lidar_cluster_node:main',
            'camera_lidar_fusion_node = sensor_fusion.camera_lidar_fusion_node:main',
        ],
    },
)
