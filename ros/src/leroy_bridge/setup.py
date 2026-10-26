from glob import glob
from setuptools import find_packages, setup

package_name = 'leroy_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin',
    maintainer_email='kevin@example.com',
    description='ROS 2 hardware bridge for the Leroy mobile base MCU.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leroy_bridge_node = leroy_bridge.node:main',
            'leroy_motor_controller_signer = leroy_bridge.motor_signer:main_controller',
            'leroy_motor_operator_grant = leroy_bridge.motor_signer:main_operator',
        ],
    },
)
