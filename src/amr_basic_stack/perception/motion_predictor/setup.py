from glob import glob

from setuptools import find_packages, setup

package_name = 'motion_predictor'

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
    description='Measurement-free multi-modal prediction from CV/CT IMM tracks',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_predictor_node = motion_predictor.motion_predictor_node:main',
            'prediction_markers_node = '
            'motion_predictor.prediction_markers_node:main',
        ],
    },
)
