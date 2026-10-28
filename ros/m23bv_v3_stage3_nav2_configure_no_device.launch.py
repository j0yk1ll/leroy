"""Stage 3 Nav2 lifecycle configure-only diagnostic. NO DEVICES OR GOALS.

All parameters in simulation.yaml remain fictional and never represent
measured Leroy geometry/speeds or a safe physical control configuration.
"""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

DIAG = "/leroy_nav2/diagnostics/proposed_cmd_vel"


def _guard(_context, *args, **kwargs):
    required = {
        "M23BV_STAGE3_NAV2_CONFIGURE_NO_DEVICE": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    if any(os.environ.get(k) != value for k, value in required.items()):
        raise RuntimeError("BLOCKED_STAGE3_NAV2_CONFIGURE_ENV")
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/video2"):
        raise RuntimeError("BLOCKED_STAGE3_NAV2_CONFIGURE_DEVICE")
    return []


def generate_launch_description():
    # Never feed these fictional simulation parameters into a live robot.
    config = str(Path(get_package_share_directory("leroy_nav2")) /
                 "config" / "simulation.yaml")
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", parameters=[config], output="screen",
            remappings=[
                ("cmd_vel", DIAG), ("/cmd_vel", DIAG),
                ("cmd_vel_nav", DIAG), ("/cmd_vel_nav", DIAG),
                ("cmd_vel_smoothed", DIAG), ("/cmd_vel_smoothed", DIAG),
            ],
        ),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
        ),
        # No lifecycle_manager: only the audited ChangeState client below
        # may send Configure and Cleanup; Activate is never requested.
    ])
