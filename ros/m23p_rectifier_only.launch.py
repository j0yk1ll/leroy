"""M23P: device-free standalone image_proc rectifier component lifecycle.

No physical sensor node is launched here. A separate synthetic
publisher can connect to the same camera topics in synthetic mode.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="m23p_image_proc_container",
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
                )
            ],
        )
    ])
