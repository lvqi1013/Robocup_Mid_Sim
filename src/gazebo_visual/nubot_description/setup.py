import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nubot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Weijia Yao; lq',
    maintainer_email='abcgarden@126.com; 1595642896@qq.com',
    description='NUBOT robot model for Gazebo Harmonic simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
