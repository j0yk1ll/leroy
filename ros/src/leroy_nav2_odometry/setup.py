"""ROS 2 Jazzy, M3 synthetic-only mecanum odometry package."""

from glob import glob
import os

from setuptools import setup

name = 'leroy_nav2_odometry'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        (os.path.join('share', name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Leroy contributors',
    maintainer_email='maintainer@example.com',
    description='Synthetic mecanum wheel odometry and TF, no motor authority',
    license='Apache-2.0',
    # Historical synthetic odom/ROS probe entry points were archived.
    entry_points={'console_scripts': []},
)
