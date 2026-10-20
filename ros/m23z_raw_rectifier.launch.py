"""M23Z raw-only mapping capture: synthetic-only stock Jazzy rectifier; no physical inputs.

Same M23R component-only tracer and M23S publisher/observer throughout.
The sole intended difference is the output publisher allowlist in the raw arm.
Do not deploy this experimental launch to a real camera.
"""
from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

TRACE_LIBRARY = "/opt/m23z/libm23z_loader_map_trace.so"

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
                parameters=[{"image_rect.enable_pub_plugins": ["image_transport/raw"]}],
            ),
        ],
    )
    return LaunchDescription([
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
