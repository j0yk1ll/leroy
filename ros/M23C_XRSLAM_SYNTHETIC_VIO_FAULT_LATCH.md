# M23C: Isolated synthetic XRSLAM VIO and IMU gap latch

Status: source implemented, runtime acceptance **PENDING**. No hardware.

M23B PR #96 merged into main at
93785ab0ccae9e6a1856d202f242a949c11cdf57.
Operator-reported M23B acceptance proved 16 synthetic monocular CDR
frames reached the native XRSLAM C++ adapter, with zero odom messages,
dynamic TF, /cmd_vel publishers or device access. This does not prove
real VIO.

## Scope and invariants

- Real native XRSLAM runs in monocular_imu mode with 16 synthetic mono8
  camera frames, 16 matching CameraInfo and 104 fake 50Hz IMU samples.
- The exact M23A CDR reader preflights the bag in monocular_imu mode.
- XRSLAM must discover all three input subscriptions, initialize and
  log a first actual VIO-path camera submission.
- A separate fake 500ms IMU timestamp dropout, deliberately OUTSIDE
  the preflighted healthy bag, must drive the C++ terminal imu_gap
  diagnostic and remain latched after subsequent fake IMU data.
- The *only* static TF in the test consists of fabricated identities
  base_link -> camera_optical_frame and base_link -> imu_link.
- An upstream EuRoC test-only YAML copy, NOT Leroy calibration, is used.
  camera_imu_offset_sec:=0.0 is a synthetic arbitrary value; it is NOT
  measured synchronization. pose_convention_validated:=false and
  enable_map_tf:=false are mandatory.
- ZERO odom messages, ZERO dynamic TF, ZERO /cmd_vel publishers and
  never localization_valid=true are required. The XRSLAM node may
  advertise /odom without emitting messages.
- No physical cameras, MCU serial, motor output or robot launch.
  Opening STM32 /dev/ttyACM* alone may energize wheel/manipulator
  motors. A physical STOP ACK is not established. Keep all hardware
  unconnected and the old leroy-ros production container exited.

## Step A: Arch Python-only tests (NO devices/ROS required)

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git status --short
  git fetch origin
  git switch feature/m23c-isolated-xrslam-synthetic-vio-fault-latch
  git pull --ff-only origin feature/m23c-isolated-xrslam-synthetic-vio-fault-latch
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23c_synthetic_vio_contract_test.py
  python3 -u m23b_monocular_isolation_contract_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23c_offline_contract.log
M23C_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23C_OFFLINE_EXIT_CODE=$M23C_OFFLINE_EXIT"
~~~

Require 16/16 M23C + 12/12 M23B + 21/21 M23A = 49/49 PASS,
M23C_SYNTHETIC_VIO_STATIC_CONTRACT=PASS,
M23C_REAL_XRSLAM_VIO_RUNTIME_VALIDATED=FALSE,
M23C_DEVICE_ACCESS=NONE, M23C_OFFLINE_EXIT_CODE=0.

## Step B: Offline local-image build

Uses previously accepted LOCAL M23B Podman image, without pulls.

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m23b-xrslam-monocular-test
  podman build --network none --no-cache \
    -f Dockerfile.m23c-selftest \
    -t leroy-sensors:m23c-xrslam-synthetic-vio .
) 2>&1 | tee m23c_no_device_build.log
M23C_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23C_BUILD_EXIT_CODE=$M23C_BUILD_EXIT"
~~~

Require M23C_SYNTHETIC_VIO_STATIC_CONTRACT=PASS,
M23C_JAZZY_VIO_IMPORT=PASS and M23C_BUILD_EXIT_CODE=0.
Original runner and Dockerfile are copied into /opt/m23c beside the
source-inspection test, preventing the M23B packaging omission.

## Step C: Disconnected real XRSLAM VIO simulation

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m23c-synthetic-vio-only \
    --network none --ipc private --pid private \
    --shm-size 512m --cap-drop all \
    --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23c-xrslam-synthetic-vio
) 2>&1 | tee m23c_no_device_ros_replay.log
M23C_SIM_EXIT=${PIPESTATUS[0]}
echo "M23C_SIM_EXIT_CODE=$M23C_SIM_EXIT"
~~~

Required report markers (NOT yet observed):

~~~text
M23C_SYNTHETIC_VIO_BAG_PREFLIGHT=PASS
M23C_REAL_XRSLAM_VIO_SUBSCRIPTIONS=PASS
M23C_REAL_XRSLAM_VIO_INITIALIZING=PASS
M23C_REAL_XRSLAM_SYNTHETIC_VIO_SUBMITTED=PASS
M23C_REAL_XRSLAM_IMU_GAP_LATCH=PASS
M23C_VIO_FAULT_STAYS_LATCHED_NO_ODOM=PASS
M23C_REAL_XRSLAM_SYNTHETIC_VIO_SELFTEST=PASS
M23C_VERDICT=REAL_XRSLAM_SYNTHETIC_VIO_FAULT_ONLY
M23C_DEVICE_ACCESS=NONE
M23C_SIM_EXIT_CODE=0
~~~

Also require cdr_images_replayed=16, cdr_camera_info_replayed=16,
cdr_imu_replayed=104, imu_gap_latched=true, odom_messages=0,
dynamic_tf_messages=0, cmd_vel_publishers=0,
camera_imu_time_offset_calibrated=false, extrinsics_validated=false,
recording_is_real=false, xrslam_tracking_accuracy_validated=false,
physical_motion_authorized=false, physical_device_access=false.

If any marker fails, investigate fake-only configs and native-library
errors. Do not enable pose, map TF, robot hardware, motor commands or
physical USB. The remaining gates are a real complete camera/IMU bag,
independently measured time offset, spatial/noise calibration, reliable
VIO tracking, pose conventions and physical safe-stop integrity.
