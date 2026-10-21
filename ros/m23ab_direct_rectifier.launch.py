"""M23AB direct-linked stock RectifyNode, synthetic-only diagnostic.

This node uses the new directly linked main, not the image_proc-generated
class_loader component-factory main. It still runs the same stock librectify.
No physical camera, device drivers, IMU, SLAM, or motor processes.
"""
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    rectifier = Node(
        package="leroy_m23ab_direct_rectifier",
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
