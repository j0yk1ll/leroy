"""Leroy's six fixed sensor TF edges, configured entirely from YAML.

No URDF, Xacro, robot_state_publisher, MCU, camera or motor interface.
This launch starts only standard tf2_ros/static_transform_publisher nodes.
Do not launch a second TF owner for these children at the same time.
"""

# TODO(AUTONOMY) #008 [TF authority]: Replace the provisional mount YAML with
# measured sensor-to-body transforms and validate exact single ROS publisher
# ownership of every static edge in the integrated navigation graph.
# Audit separate dynamic odom->base_link and map->odom publishers for
# correct axis conventions, timestamp validity and discontinuity handling.
# Acceptance: no duplicate TF, zero placeholder extrinsics, backwards-time
# transforms or uncalibrated pose frame can authorize production navigation.

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Parent/child frame IDs are invariant (not configurable by YAML), preventing
# an accidental competing odom, map, or navigation authority.
FRAME_EDGES = (
    ("mounts", "imu", "base_link", "imu_link"),
    ("mounts", "front_camera", "base_link", "front_camera_link"),
    ("fixed_frames", "front_camera_optical", "front_camera_link",
     "front_camera_optical_frame"),
    ("fixed_frames", "camera_optical_alias", "front_camera_optical_frame",
     "camera_optical_frame"),
    ("mounts", "workspace_camera", "base_link", "workspace_camera_link"),
    ("fixed_frames", "workspace_camera_optical", "workspace_camera_link",
     "workspace_camera_optical_frame"),
)


def load_sensor_transforms(config_file):
    """Validate YAML and return six fixed (parent, child, xyz, rpy) edges."""
    path = Path(config_file)
    if not path.is_file() or path.is_symlink():
        raise RuntimeError("BLOCKED_SENSOR_TF_CONFIG_NOT_REGULAR_FILE")
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or set(cfg) != {
            "status", "mounts", "fixed_frames"}:
        raise RuntimeError("BLOCKED_SENSOR_TF_TOP_LEVEL_CONFIG")
    if cfg["status"] != "provisional":
        # Config status never lifts VIO pose/map/motor permissions.
        raise RuntimeError("BLOCKED_SENSOR_TF_UNREVIEWED_CALIBRATION_STATUS")

    expected_sections = {
        "mounts": {"imu", "front_camera", "workspace_camera"},
        "fixed_frames": {"front_camera_optical",
                         "camera_optical_alias",
                         "workspace_camera_optical"},
    }
    for section, keys in expected_sections.items():
        if (not isinstance(cfg[section], dict)
                or set(cfg[section]) != keys):
            raise RuntimeError("BLOCKED_SENSOR_TF_UNKNOWN_OR_MISSING_" +
                               section.upper())

    edges = []
    children = set()
    for section, name, parent, child in FRAME_EDGES:
        entry = cfg[section][name]
        if not isinstance(entry, dict) or set(entry) != {"xyz", "rpy"}:
            raise RuntimeError("BLOCKED_SENSOR_TF_ENTRY_" + name)
        vectors = []
        for field in ("xyz", "rpy"):
            value = entry[field]
            if (not isinstance(value, list) or len(value) != 3
                    or any(type(v) not in (float, int)
                           or not math.isfinite(v) for v in value)):
                raise RuntimeError("BLOCKED_SENSOR_TF_VECTOR_" +
                                   name + "_" + field)
            vectors.append(tuple(float(v) for v in value))
        if parent == child or child in children:
            raise RuntimeError("BLOCKED_SENSOR_TF_FRAME_GRAPH")
        children.add(child)
        edges.append((parent, child, *vectors))

    if edges[0][3] != (0.0, 0.0, 0.0):
        # /imu/data_raw is ALREADY robot-axis-aligned by MCU firmware.
        raise RuntimeError("BLOCKED_SENSOR_TF_DOUBLE_IMU_ROTATION")
    optical = (-math.pi / 2, 0.0, -math.pi / 2)
    for edge in (edges[2], edges[5]):
        if (edge[2] != (0.0, 0.0, 0.0)
                or any(abs(a-b) > 1e-12
                       for a, b in zip(edge[3], optical))):
            raise RuntimeError("BLOCKED_SENSOR_TF_OPTICAL_CONVENTION")
    if edges[3][2:] != ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)):
        raise RuntimeError("BLOCKED_SENSOR_TF_CAMERA_ALIAS")

    return tuple(edges)


def publisher_arguments(edge):
    """Modern Jazzy CLI option form: x/y/z and roll/pitch/yaw (radians)."""
    parent, child, xyz, rpy = edge
    return [
        "--x", repr(xyz[0]), "--y", repr(xyz[1]), "--z", repr(xyz[2]),
        "--roll", repr(rpy[0]), "--pitch", repr(rpy[1]), "--yaw", repr(rpy[2]),
        "--frame-id", parent, "--child-frame-id", child,
    ]


def make_publishers(config_file):
    edges = load_sensor_transforms(config_file)
    return [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="leroy_static_tf_" + child,
            arguments=publisher_arguments(edge),
            output="screen",
        )
        for edge in edges
        for child in (edge[1],)
    ]


def _start_configured_publishers(context, *args, **kwargs):
    config_file = LaunchConfiguration("sensor_mounts_file").perform(context)
    return make_publishers(config_file)


def generate_launch_description():
    installed_yaml = str(
        Path(get_package_share_directory("leroy_description")) /
        "config" / "sensor_mounts.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_mounts_file", default_value=installed_yaml,
            description="Absolute path to fixed sensor TF YAML"),
        OpaqueFunction(function=_start_configured_publishers),
    ])
