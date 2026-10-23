"""M23AA isolated no-device component control: stock Jazzy RectifyNode.

Deliberately no LD_PRELOAD and no real camera / motor nodes.
"""
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectifier = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="m23aa_image_proc_container",
        namespace="",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
                namespace="/camera",
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
            ),
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
