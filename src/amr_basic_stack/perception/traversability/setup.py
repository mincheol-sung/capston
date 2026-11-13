from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'traversability'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'tools'),
            glob('tools/*.py') + glob('tools/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yumin',
    maintainer_email='yumin@todo.todo',
    description='기준 2D 맵과 관측을 합쳐 3-state traversable map 을 Planning 에 제공한다.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traversability_node = traversability.traversability_node:main',
            'depth_preprocess_node = traversability.depth_preprocess_node:main',
        ],
    },
)
