"""M4: offline-only Nav2 integration fixture package."""

from glob import glob
import os

from setuptools import setup

package_name = 'leroy_nav2_integration'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Leroy contributors',
    maintainer_email='maintainer@example.com',
    description='M4 offline-only synthetic odometry/depth rolling costmap',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'integrated_probe = leroy_nav2_integration.integrated_probe:main',
        'controller_probe = leroy_nav2_integration.controller_probe:main',
        'fault_depth_fixture = leroy_nav2_integration.fault_depth_fixture:main',
        'fault_odom_fixture = leroy_nav2_integration.fault_odom_fixture:main',
        'fault_audit = leroy_nav2_integration.fault_audit:main',
        'shadow_replay = leroy_nav2_integration.shadow_replay:main',
        'synthetic_shadow_candidate = leroy_nav2_integration.synthetic_shadow_candidate:main',
        'ros_shadow_audit = leroy_nav2_integration.ros_shadow_audit:main',
        'mppi_shadow_probe = leroy_nav2_integration.m9_mppi_shadow_probe:main',
        'm10_fake_depth = leroy_nav2_integration.m10_triggered_fake_fault:main_depth',
        'm10_fake_odom = leroy_nav2_integration.m10_triggered_fake_fault:main_odom',
        'm10_active_fault_probe = leroy_nav2_integration.m10_active_fault_probe:main',
        'm11_timed_depth = leroy_nav2_integration.m11_timed_fake_fault:main_depth',
        'm11_timed_odom = leroy_nav2_integration.m11_timed_fake_fault:main_odom',
        'm11_blackout_probe = leroy_nav2_integration.m11_blackout_probe:main',
    ]},
)
