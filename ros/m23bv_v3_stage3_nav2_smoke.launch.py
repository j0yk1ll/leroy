"""Stage 3: real Nav2 nodes, unconfigured and command-output remapped.

Hardware-free application startup only; never use on a robot or with a bridge.
No synthetic odometry, costmap obstacle clearing, navigation goals, or actuation.
"""
import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


PROPOSED = "/leroy_nav2/diagnostics/proposed_cmd_vel"


def _guard(_context, *args, **kwargs):
    required = {
        "M23BV_STAGE3_NO_DEVICE_APP_SMOKE": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError("Stage 3 Nav2 app smoke requires pinned isolated graph")
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/video2"):
        raise RuntimeError("Stage 3 Nav2 app smoke forbids device mappings")
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            remappings=[("cmd_vel", PROPOSED), ("/cmd_vel", PROPOSED),
                        ("cmd_vel_nav", PROPOSED),
                        ("/cmd_vel_nav", PROPOSED)],
        ),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_stage3_no_device",
            output="screen",
            parameters=[{
                "autostart": False,
                "node_names": [
                    "controller_server", "planner_server", "bt_navigator"],
            }],
        ),
    ])
