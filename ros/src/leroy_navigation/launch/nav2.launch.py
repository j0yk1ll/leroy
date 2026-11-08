"""Nav2 Jazzy native action stack; isolated, non-actuating command output.

Do not launch the custom goal_controller at the same time.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory("leroy_navigation"),
                          "config", "nav2.yaml")
    sim = LaunchConfiguration("use_sim_time")
    managed = ["controller_server", "planner_server", "behavior_server",
               "bt_navigator", "waypoint_follower"]
    servers = []
    for package, executable, name in (
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
    ):
        # Controller and recovery outputs are not wired to the MCU or signer.
        remappings = [("cmd_vel", "/leroy/navigation/nav2_cmd_vel"),
                      ("cmd_vel_nav", "/leroy/navigation/nav2_cmd_vel")]
        servers.append(Node(package=package, executable=executable, name=name,
                            output="screen", parameters=[params, {"use_sim_time": sim}],
                            remappings=remappings))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        *servers,
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": sim, "autostart": True,
                          "node_names": managed}]),
    ])
