#!/usr/bin/env python3
"""M23BG: device-free static XRSLAM -> depth -> costmap interface audit.

Inert by default. Source contracts only, NOT a real ROS integration run,
sensor correctness proof, hardware permission, or production readiness.
"""
import argparse
import json
from pathlib import Path
import sys

MAX_FILE_BYTES = 120_000

PATHS = {
    "xrslam": "ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp",
    "depth_obstacles": (
        "ros/src/leroy_nav2_perception/leroy_nav2_perception/depth_obstacles.py"
    ),
    "costmap_config": "ros/src/leroy_nav2_integration/config/rolling_costmap.yaml",
    "synthetic_probe": (
        "ros/src/leroy_nav2_integration/leroy_nav2_integration/integrated_probe.py"
    ),
    "real_input_gate": (
        "ros/src/leroy_nav2_integration/leroy_nav2_integration/m15_real_input_gate.py"
    ),
    "passive_observer": (
        "ros/src/leroy_nav2_integration/leroy_nav2_integration/m16_passive_contract.py"
    ),
    "robot_launch": "ros/src/leroy_bringup/launch/robot.launch.py",
}

REQUIRED = {
    "xrslam": (
        'create_publisher<nav_msgs::msg::Odometry>("/odom"',
        'declare_parameter<bool>("pose_convention_validated",false)',
        'declare_parameter<bool>("enable_map_tf",false)',
        'XRSLAM_RESULT_BODY_POSE',
        'tf_broadcaster_.sendTransform(tf);',
        '"/xrslam/tracking_status"',
        '"/xrslam/localization_valid"',
    ),
    "depth_obstacles": (
        "'depth_topic': '/depthart/depth'",
        "'camera_info_topic': '/camera/camera_info'",
        "'obstacle_cloud_topic': '/leroy_nav2/diagnostics/obstacle_points'",
        "'publish_obstacles': False",
        "'extrinsics_validated': False",
        "'ground_reference_validated': False",
        "'base_link', msg.header.frame_id",
        "self._obstacle_pub.publish(",
    ),
    "costmap_config": (
        "global_frame: odom",
        "robot_base_frame: base_link",
        "track_unknown_space: true",
        "marking: true",
        "clearing: false",
        "topic: /leroy_nav2/diagnostics/obstacle_points",
        "Arbitrary footprint, NOT a measurement of Leroy",
    ),
    "synthetic_probe": (
        "OccupancyGrid, '/costmap'",
        "'/leroy_nav2/diagnostics/synthetic_odom'",
    ),
    "real_input_gate": (
        "'odom': '/odom'",
        "'cloud': '/leroy_nav2/diagnostics/obstacle_points'",
        "'map': '/local_costmap/costmap'",
        "OUTPUT = '/leroy_nav2/diagnostics/m15_gated_cmd_vel'",
        "if '/cmd_vel' in topics",
    ),
    "passive_observer": (
        "'camera_image': ('/camera/image_rect'",
        "'camera_info': ('/camera/camera_info'",
        "'depth_image': ('/depthart/depth'",
        "'imu': ('/imu/data_raw'",
        "'odom': ('/odom'",
        "'obstacle_cloud': ('/leroy_nav2/diagnostics/obstacle_points'",
    ),
    "robot_launch": (
        "bridge = IncludeLaunchDescription(",
        "FindPackageShare('leroy_bridge')",
        "'leroy_bridge.launch.py'",
        "        bridge,",
        "DeclareLaunchArgument('use_xrslam', default_value='false')",
        "OpaqueFunction(function=_launch_xrslam, condition=IfCondition(use_xrslam))",
    ),
}


class Blocked(ValueError):
    pass


def audit_sources(sources):
    if set(sources) != set(PATHS):
        raise Blocked("seven exact integration source files required")
    for name, anchors in REQUIRED.items():
        content = sources[name]
        if not isinstance(content, str) or not content:
            raise Blocked("source missing: " + name)
        for anchor in anchors:
            if anchor not in content:
                raise Blocked("source interface changed: " + name +
                              " / " + anchor[:70])

    # These are separate fixture and real-gate consumers of two DIFFERENT
    # topics. Do NOT silently mark them as a connected costmap pipeline.
    if ("OccupancyGrid, '/costmap'" not in sources["synthetic_probe"] or
            "'map': '/local_costmap/costmap'" not in sources["real_input_gate"]):
        raise Blocked("costmap endpoint provenance cannot be established")
    return {
        "verdict": "OFFLINE_SOURCE_INTERFACES_AUDITED_ONLY",
        "component_count": len(PATHS),
        "xrslam_provides_gated_odom_tf": True,
        "xrslam_metric_pose_verified_on_robot": False,
        "xrslam_map_alignment_code_conditional": True,
        "persistent_mapping_verified": False,
        "depth_obstacles_source_topic_matches_costmap": True,
        "depth_obstacle_publication_enabled": False,
        "depth_ground_and_extrinsics_verified": False,
        "m4_fixture_costmap_topic": "/costmap",
        "m15_real_gate_costmap_topic": "/local_costmap/costmap",
        "actual_costmap_endpoint_alignment_verified": False,
        "bridge_free_stationary_live_launch_present": False,
        "normal_robot_launch_includes_motor_capable_bridge": True,
        "camera_stamp_integrity_verified": False,
        "camera_imu_physical_offset_verified": False,
        "real_vio_tracking_verified": False,
        "real_obstacle_costmap_verified": False,
        "hardware_command_authority_verified": False,
        "next_milestone": "BRIDGE_FREE_STATIONARY_SENSOR_LOCALIZATION_COSTMAP",
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def load_sources(root):
    if root.is_symlink() or not root.is_dir():
        raise Blocked("checkout root missing or symlinked")
    result = {}
    for name, rel in PATHS.items():
        path = root / rel
        if path.is_symlink() or not path.is_file():
            raise Blocked("source file absent/nonregular/symlink: " + rel)
        size = path.stat().st_size
        if not 0 < size <= MAX_FILE_BYTES:
            raise Blocked("source size outside contract: " + rel)
        contents = path.read_text(encoding="utf-8")
        if path.stat().st_size != size:
            raise Blocked("source changed while reading: " + rel)
        result[name] = contents
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check", action="store_true")
    p.add_argument("--root", type=Path)
    args = p.parse_args(argv)
    if not args.check and args.root is None:
        print("M23BG_STATUS=INERT_NO_ROS_NO_DEVICE_NO_WRITES", flush=True)
        return 0
    try:
        if not args.check or args.root is None:
            raise Blocked("--check and an explicit checkout --root are required")
        report = audit_sources(load_sources(args.root))
        print("M23BG_RESULT_JSON=" + json.dumps(report, sort_keys=True), flush=True)
        print("M23BG_SOURCE_INTERFACE_GATE=PASS", flush=True)
        print("M23BG_REAL_ROBOT_INTEGRATION=BLOCKED", flush=True)
        print("M23BG_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 0
    except (Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23BG_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:175],
              flush=True)
        print("M23BG_REAL_ROBOT_INTEGRATION=BLOCKED", flush=True)
        print("M23BG_CAMERA_RETEST_AUTHORIZED=FALSE", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
