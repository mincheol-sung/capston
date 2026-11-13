from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamic_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yumin',
    maintainer_email='yumin@todo.todo',
    description='Causal radius or cluster IMM track masking for dual-LiDAR SLAM',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_filter_node = dynamic_filter.dynamic_filter_node:main',
        ],
    },
)
