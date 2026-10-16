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
    ]},
)
