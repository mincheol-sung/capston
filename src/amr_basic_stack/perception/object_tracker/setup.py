from glob import glob

from setuptools import find_packages, setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yumin',
    maintainer_email='yumin@todo.todo',
    description='Camera-LiDAR 융합 검출 기반 다중객체 추적 '
                '(CV/CT IMM + Mahalanobis association + 속도 chi^2 판별).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_tracker_node = object_tracker.object_tracker_node:main',
        ],
    },
)
