"""ROS 2 Jazzy pure-Python mecanum kinematics library (no ROS executable)."""

from setuptools import setup

name = 'leroy_nav2_odometry'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Leroy contributors',
    maintainer_email='maintainer@example.com',
    description='Mecanum wheel kinematics library; no odometry publisher or TF authority',
    license='Apache-2.0',
)
