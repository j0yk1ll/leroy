"""Real camera -> DepthART -> optical cloud -> Nav2 diagnostic costmap.

Uses existing PROVISIONAL sensor mounts, not asserted validated geometry.
No odometry, planners, controller_server, ROS command authority or motors.
Only launch in an isolated single-container motor-disconnected experiment.
"""

# TODO(AUTONOMY) #013 [navigation costmaps]: Replace the current diagnostic
# costmap fixture and provisional static TF with measured base geometry,
# dynamic odom/map transforms and all-sided obstacle sensing. Configure
# separate appropriate local/global Nav2 layers with safe unknown-space
# behavior, clearing, inflation and recovery handling only after validation.
# Acceptance: replayed and real moving obstacles, TF gaps and sensor loss
# produce bounded stop/replan behavior without changing sensor-only defaults.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _guard(_context, *args, **kwargs):
    required = {
        "LEROY_LIVE_PERCEPTION_ONLY": "1",
        "LEROY_LIVE_COSTMAP_DIAGNOSTIC": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError("blocked nonisolated live depth costmap launch")
    if os.path.exists("/dev/ttyACM0"):
        raise RuntimeError("STM32 serial device must not be mapped")
    return []


def _include(package, launch_file, *, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), "launch", launch_file)),
        launch_arguments=(launch_arguments or {}).items())


def generate_launch_description():
    share = get_package_share_directory("leroy_nav2_perception")
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        _include("leroy_nav2_perception", "live_depth_pipeline.launch.py"),
        # Physical mount figures in sensor_mounts.yaml are PROVISIONAL.
        # Use them ONLY to exercise Nav2 TF routing with real sensor messages.
        # Colcon --symlink-install makes the installed config a symlink,
        # but description.load_sensor_transforms correctly refuses symlinks.
        # Pass the immutable-at-build COPY as a real regular file instead.
        _include(
            "leroy_description", "description.launch.py",
            launch_arguments={
                "sensor_mounts_file": "/etc/leroy/sensor_mounts.yaml",
            }),
        Node(
            package="leroy_nav2_perception",
            executable="costmap_fixture",
            name="costmap",
            output="screen",
            parameters=[os.path.join(
                share, "config", "live_diagnostic_costmap.yaml")],
        ),
        # Lifecycle autostart intentionally omitted: configure and activate
        # manually after observing the sensor pipeline and TF.
    ])
