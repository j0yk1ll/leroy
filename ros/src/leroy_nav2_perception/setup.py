"""ROS 2 Jazzy package for Leroy's opt-in diagnostic depth adapter."""

from setuptools import setup

package_name = 'leroy_nav2_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/depth_diagnostics.yaml',
            'config/live_diagnostic_costmap.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/live_depth_costmap.launch.py',
            'launch/live_depth_pipeline.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    # Colcon uses this metadata to choose pytest rather than setup.py test.
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Leroy contributors',
    maintainer_email='maintainer@example.com',
    description='Diagnostic DepthART-to-PointCloud2 geometry adapter, no motor authority',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'depth_obstacles = leroy_nav2_perception.depth_obstacles:main',
        'live_pipeline_probe = leroy_nav2_perception.live_pipeline_probe:main',
        'live_costmap_probe = leroy_nav2_perception.live_costmap_probe:main',
    ]},
)
