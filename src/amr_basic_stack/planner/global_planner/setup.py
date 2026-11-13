from pathlib import Path

from setuptools import find_packages, setup

package_name = 'global_planner'


def collect_map_files():
    data_files = []
    maps_dir = Path('maps')
    if not maps_dir.is_dir():
        return data_files

    for map_dir in sorted(path for path in maps_dir.iterdir() if path.is_dir()):
        files = sorted(str(path) for path in map_dir.iterdir() if path.is_file())
        if files:
            data_files.append((f'share/{package_name}/maps/{map_dir.name}', files))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            [str(path) for path in Path('config').glob('*.yaml')],
        ),
    ] + collect_map_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smoc',
    maintainer_email='smoc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_planner_node = global_planner.global_planner_node:main',
            'global_path_publisher_node = global_planner.global_path_publisher_node:main',
            'rviz_theta_star_node = global_planner.rviz_theta_star_node:main',
            'theta_planner = global_planner.theta_star.planner:main',
        ],
    },
)
