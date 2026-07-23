import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'auto_referee'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lq',
    maintainer_email='1595642896@qq.com',
    description='ROS 2 automatic referee for Gazebo Harmonic.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_referee_node = auto_referee.referee_node:main',
        ],
    },
)
