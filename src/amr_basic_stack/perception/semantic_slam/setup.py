from glob import glob

from setuptools import Extension, find_packages, setup

package_name = 'semantic_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[Extension('semantic_slam._voxel', ['semantic_slam/_voxel.c'])],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='yumin',
    maintainer_email='yumin@todo.todo',
    description='Persistent object mapping and AMR object-goal navigation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'semantic_slam_node = semantic_slam.semantic_slam_node:main',
            'semantic_navigation_node = semantic_slam.semantic_navigation_node:main',
        ],
    },
)
