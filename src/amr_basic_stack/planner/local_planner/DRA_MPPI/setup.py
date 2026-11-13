from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dra_mppi'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smoc',
    maintainer_email='smoc@todo.todo',
    description='Dynamic risk-aware MPPI local planner for a holonomic AMR.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'dra_mppi_node = dra_mppi.dra_mppi_node:main',
        ],
    },
)
