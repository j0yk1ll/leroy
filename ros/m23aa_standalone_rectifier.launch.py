"""M23AA isolated no-device standalone control: same stock Jazzy rectifier.

Only the process/manager lifetime differs from the component control.
No LD_PRELOAD, hardware, motor nodes, or alternate rectification code.
"""
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    rectifier = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="/camera",
        output="screen",
        remappings=[
            ("image", "image_raw"),
            ("camera_info", "camera_info"),
        ],
    )
    return LaunchDescription([
        RegisterEventHandler(OnProcessStart(
            target_action=rectifier,
            on_start=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_START")])),
        RegisterEventHandler(OnProcessExit(
            target_action=rectifier,
            on_exit=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_EXIT")])),
        rectifier,
    ])
