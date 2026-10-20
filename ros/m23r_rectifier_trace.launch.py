"""M23R: instrument only the M23P no-device RectifyNode component child.

Inherited M23P runner still supplies graph/publisher/observer gating and
evaluates component_container exit separately from ros2 launch exit.
Never start usb_cam, a device driver, IMU, recorder, or motor interface.
"""
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


TRACE_LIBRARY = "/opt/m23r/libm23r_loader_trace.so"


def generate_launch_description():
    component = ComposableNodeContainer(
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
            ),
        ],
    )
    return LaunchDescription([
        # Only this launch's child processes inherit the preload.
        # M23P publisher/observer are siblings started by its Python
        # runner, so neither has LD_PRELOAD applied to it.
        SetEnvironmentVariable(name="LD_PRELOAD", value=TRACE_LIBRARY),
        RegisterEventHandler(OnProcessStart(
            target_action=component,
            on_start=[LogInfo(msg="M23R_LAUNCH_EVENT=CONTAINER_PROCESS_START")],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=component,
            on_exit=[LogInfo(msg="M23R_LAUNCH_EVENT=CONTAINER_PROCESS_EXIT")],
        )),
        RegisterEventHandler(OnShutdown(
            on_shutdown=[LogInfo(msg="M23R_LAUNCH_EVENT=SHUTDOWN_REQUEST")],
        )),
        component,
    ])
