#!/usr/bin/env python3
"""Read-only audit of Stage 3 YAML TF overlay inside the real XRSLAM image.

No actual robot launch, device, camera, bridge, STM32, Nav2 or XRSLAM
nodes are invoked. Only the six already-accepted tf2_ros static
publishers start, through the existing observer.
"""
import importlib.util
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

EXPECTED_OVERLAY = Path("/opt/leroy_stage3_tf_ws/install")
EXPECTED_SOURCE = Path("/opt/leroy_stage3_tf_ws/src/leroy_description")
OBSERVER = Path("/opt/leroy_tf_live_verify.py")


def require(condition, marker):
    if not condition:
        raise RuntimeError("BLOCKED_" + marker)


def main():
    require(os.environ.get("ROS_DOMAIN_ID") == "170", "DOMAIN")
    require(os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST",
            "DISCOVERY")
    require(os.environ.get("RMW_IMPLEMENTATION") == "rmw_fastrtps_cpp",
            "DDS_BACKEND")
    for device in ("/dev/video2", "/dev/ttyACM0", "/dev/ttyUSB0"):
        require(not Path(device).exists(), "DEVICE_VISIBLE")

    # Prove that the source candidate is being used in the FULL runtime
    # package environment, not the previous self-contained tmpfs test.
    prefix = Path(get_package_prefix("leroy_description"))
    require(prefix == EXPECTED_OVERLAY, "DESCRIPTION_NOT_OVERLAID")
    share = Path(get_package_share_directory("leroy_description"))
    require(share == EXPECTED_OVERLAY / "share/leroy_description",
            "DESCRIPTION_SHARE_NOT_OVERLAID")
    require((share / "config/sensor_mounts.yaml").read_bytes() ==
            (EXPECTED_SOURCE / "config/sensor_mounts.yaml").read_bytes(),
            "INSTALLED_YAML_DRIFT")
    require((share / "launch/description.launch.py").read_bytes() ==
            (EXPECTED_SOURCE / "launch/description.launch.py").read_bytes(),
            "INSTALLED_LAUNCH_DRIFT")
    require(not (share / "urdf").exists(), "OBSOLETE_OVERLAY_URDF")
    print("STAGE3_TF_RUNTIME_OVERLAY=PASS_NEW_YAML_DESCRIPTION_RESOLVED",
          flush=True)

    for package in ("leroy_bringup", "leroy_bridge", "leroy_depthart",
                    "leroy_xrslam"):
        other_prefix = Path(get_package_prefix(package))
        require(str(other_prefix).startswith("/leroy_ws/install/"),
                "INHERITED_PACKAGE_PATH_" + package)
    bringup_share = Path(get_package_share_directory("leroy_bringup"))
    bringup_launch = bringup_share / "launch/robot.launch.py"
    require(bringup_launch.is_file(), "ORIGINAL_BRINGUP_MISSING")
    require("FindPackageShare('leroy_description')" in
            bringup_launch.read_text(), "BRINGUP_NOT_LINKED_TO_DESCRIPTION")
    print("STAGE3_TF_RUNTIME_STACK=PASS_BRINGUP_BRIDGE_DEPTHART_XRSLAM_RETAINED",
          flush=True)
    print("STAGE3_TF_RUNTIME_BRINGUP=NOT_LAUNCHED", flush=True)

    # Reuse the already operator-accepted TF2 observer without modifying
    # its source. Override its two expected test paths for this image,
    # preserving the same six-frame/quaternion/DDS/shutdown assertions.
    spec = importlib.util.spec_from_file_location("stage3_accepted_tf_observer",
                                                   OBSERVER)
    require(spec is not None and spec.loader is not None,
            "OBSERVER_IMPORT")
    observer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(observer)
    observer.INSTALL = EXPECTED_OVERLAY
    observer.SOURCE = EXPECTED_SOURCE
    observer.main()
    print("STAGE3_TF_RUNTIME_LIVE_TF=PASS_SIX_EDGES_IN_FULL_RUNTIME_IMAGE",
          flush=True)
    print("STAGE3_TF_RUNTIME_MCU_CAMERA_BRIDGE_NAV2=NOT_STARTED", flush=True)
    print("STAGE3_TF_RUNTIME_GEOMETRY=PROVISIONAL", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("STAGE3_TF_RUNTIME=BLOCKED_" + repr(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
