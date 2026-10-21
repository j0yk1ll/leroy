"""M23AC synthetic-only component launch: identical binary, controlled policy."""
import os

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    policy = os.environ.get("M23AC_ARM")
    if policy not in ("control", "nodelete"):
        raise ValueError("M23AC_ARM must be control or nodelete")

    component = ComposableNodeContainer(
        package="leroy_m23ac_loaded_rectify_lifetime",
        executable="component_container",
        name="m23ac_image_proc_container",
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
        SetEnvironmentVariable(name="M23AC_LIBRARY_POLICY", value=policy),
        RegisterEventHandler(OnProcessStart(
            target_action=component,
            on_start=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_START")],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=component,
            on_exit=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_EXIT")],
        )),
        component,
    ])
