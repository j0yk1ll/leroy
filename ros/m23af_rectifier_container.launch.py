"""M23AF: same-process /proc/self/maps snapshots; CONTROL only, no devices."""
import os
import re

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    policy = os.environ.get("M23AC_ARM")
    run_id = os.environ.get("M23AF_RUN_ID", "")
    if policy != "control" or re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
        raise ValueError("M23AF launch requires explicit control + unique run ID")

    component = ComposableNodeContainer(
        package="leroy_m23af_shutdown_maps",
        executable="component_container",
        name="m23af_image_proc_container",
        namespace="",
        output="screen",
        # The nonce is also in the process command line. A future host
        # coredump must prove it came from THIS run, not the prior cores.
        arguments=["--ros-args", "-p", "m23af_run_id:=" + run_id],
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
                namespace="/camera",
                remappings=[("image", "image_raw"),
                            ("camera_info", "camera_info")],
            ),
        ],
    )
    return LaunchDescription([
        SetEnvironmentVariable(name="M23AC_LIBRARY_POLICY", value="control"),
        RegisterEventHandler(OnProcessStart(
            target_action=component,
            on_start=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_START")])),
        RegisterEventHandler(OnProcessExit(
            target_action=component,
            on_exit=[LogInfo(msg="M23AA_LAUNCH_EVENT=RECTIFIER_PROCESS_EXIT")])),
        component,
    ])
