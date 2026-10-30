"""ROS package for the passive sensor observer used by Leroy's real IMU source.

Historical synthetic Nav2 integration experiments are archived and are not
part of the current runtime or the supported clean-checkout build.
"""
from setuptools import setup

package_name = 'leroy_nav2_integration'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leroy contributors',
    maintainer_email='maintainer@example.com',
    description='Passive read-only ROS sensor observer used by the STM32 IMU runtime',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'm16_passive_observer = leroy_nav2_integration.m16_passive_observer:main',
    ]},
)
