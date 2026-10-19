# M23H — bounded real three-topic ROS 2 Jazzy bag capture

**STATUS:** capture code committed for REVIEW. No real camera/IMU
bag was captured by creating this PR. All host/runtime acceptance
tests, operator approval and device identity checks are still pending.

M23G readiness PR #101 is the **base** of this stacked branch.
Do NOT merge M23H to main until #101 is merged, then retarget.

## Engineering scope

Run the existing already-tested M20 `leroy-m20-sensor-run`
(sensor-only, real front USB camera, image_proc rectification, real
MCU-sourced /imu/data_raw) and a separate Jazzy `ros2 bag record`
process inside **one** isolated ROS_DOMAIN_ID=170 container.

1. First start the recorder, specifying EXACT topics:
   /camera/image_rect, /camera/camera_info, /imu/data_raw.
   Force sqlite3 CDR + M23G best-effort/volatile QoS override YAML.
   Do not record all topics, odom, TF, /cmd_vel, motor controls,
   bridge, DepthART, XRSLAM or Nav2.
2. Allow 3s for recorder startup, then invoke M20 sensor-only
   runner, which starts the rectifier, waits for it, runs the real
   bounded 120s M18 IMU stream, checks forbidden ROS graph publishers,
   and tries to stop the MCU stream with a verified STOP ACK.
3. Wait for M20 to exit normally, or bounded 175s parent timeout.
   On failure, request SIGTERM on the M20 **parent shell** only:
   its own trap attempts graceful IMU STOP, then camera shutdown.
   Do NOT send process-group SIGTERM to M20's children and bypass the
   ordered STOP handler. If M20 never exits after 24s,
   mark failure and retain physical control over hardware state.
4. Send SIGINT to rosbag recorder process group and wait at most
   50s for metadata + sqlite flush. NEVER claim a complete bag
   if recorder is still running, is SIGKILLed or fails shutdown.
5. Check that a metadata.yaml and regular sqlite3 .db3 files exist.
   Query the **closed** sqlite3 files read-only, verify PRAGMA
   quick_check and ONLY the 3 permitted topic names. Minimum counts
   are 400 images, 400 CameraInfo and 5600 IMU; counts alone
   do not validate header times, image payloads or camera–IMU
   timing. The later M23F/M23E/M23D/M23A audit may still BLOCK.

The runner requires explicit env
`M23H_SELFTEST=0`,
`M23H_OPERATOR_CAPTURE_APPROVED=YES` and
`M23H_SESSION_ID=<safe identifier>` to enable the physical capture
path. Otherwise it defaults to hardware-free selftests, and rejects
mapping devices into a self-test. This environmental opt-in is not
evidence that motor power has been disabled or that the E-stop works.

## Safety details inseparable from correct operation

Opening the actual STM32's /dev/ttyACM* can reset controller firmware
and cause actuator movement without a motor command. Previous real
M20 runs were explicitly physically immobilized with wheel and
manipulator motor power inhibited and verified E-stop. Operator has
stated that their current isolated test environment is safe; that
assessment alone cannot verify physical E-stop or power state.

Before the first real run, independently establish BOTH wheel and
manipulator actuator-power inhibition or another independently
verified physical condition preventing harm during USB reset,
immobilization and hardware E-stop; verify no person/objects are
in moving pathways. Verify **current** exact STM32 by-id and physical
front-camera identity by non-opening host discovery tools. Never
blindly assume ttyACM0 or video6/video8 will retain their identity.
Production leroy-ros must remain exited; there must be no second
serial/video owner. Ensure no ROS bridge or motor command clients.
If the IMU stream STOP ACK is unavailable, keep the robot's
physical safe state until recovery; SIGTERM and software cleanup
can never guarantee a STOP after port loss or abrupt termination.

This milestone does not start the hardware automatically. A human
operator must expressly run the capture command after these checks.

## A. Pure Python offline regression tests (no ROS, no hardware)

~~~bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m23h-bounded-live-rosbag-capture
  git pull --ff-only origin feature/m23h-bounded-live-rosbag-capture
  cd ros
  export PYTHONDONTWRITEBYTECODE=1
  python3 -u m23h_bounded_capture_test.py
  python3 -u m23g_live_recording_preflight_test.py
  python3 -u m23f_existing_recording_inventory_test.py
  python3 -u m23e_offline_evidence_snapshot_test.py
  python3 -u m23d_recorded_bag_readiness_test.py
  python3 -u m23a_xrslam_rosbag_preflight_test.py
) 2>&1 | tee m23h_offline_contract.log
M23H_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M23H_OFFLINE_EXIT_CODE=$M23H_OFFLINE_EXIT"
~~~

Expected **114/114 PASS**: M23H 20 + M23G 19 + M23F 18 +
M23E 19 + M23D 17 + M23A 21.
Requires M23H_OFFLINE_CAPTURE_LIFECYCLE_CONTRACT=PASS,
M23H_OFFLINE_EXIT_CODE=0 and M23H_DEVICE_ACCESS=NONE.

## B. Rootless, no-network build from ACCEPTED local M20 image

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)/ros"
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m20-camera-imu
  podman build --network none --no-cache \
    -f Dockerfile.m23h-capture \
    -t leroy-sensors:m23h-bounded-recording .
) 2>&1 | tee m23h_no_device_build.log
M23H_BUILD_EXIT=${PIPESTATUS[0]}
echo "M23H_BUILD_EXIT_CODE=$M23H_BUILD_EXIT"
~~~

This fails closed if rosbag2 CLI, yaml or rosbag2_py are not in
the local M20 base image. Never silently download packages.
Require M23H_JAZZY_RECORDER_CLI=PASS,
M23H_OFFLINE_CAPTURE_LIFECYCLE_CONTRACT=PASS, and build exit 0.

## C. Isolated no-device Podman selftest

~~~bash
set +e
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm --name leroy-m23h-no-device-test \
    --network none --ipc private --pid private \
    --cap-drop all --security-opt no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m23h-bounded-recording
) 2>&1 | tee m23h_no_device_test.log
M23H_SIM_EXIT=${PIPESTATUS[0]}
echo "M23H_SIM_EXIT_CODE=$M23H_SIM_EXIT"
~~~

Require M23H_OFFLINE_CAPTURE_LIFECYCLE_CONTRACT=PASS,
M23H_FAKE_RECORDER_SUBSCRIPTIONS=PASS,
M23H_SYNTHETIC_ROS2_RECORDER_SQLITE=PASS,
M23H_SYNTHETIC_CLI_RECORDER_SELFTEST=PASS,
M23H_DEVICE_ACCESS=NONE and M23H_SIM_EXIT_CODE=0.
The first test uses MOCK subprocesses and fake SQLite files;
the second runs the **actual Jazzy ros2 bag CLI** with three
deliberately fabricated ROS publishers inside an isolated no-device
container. The synthetic data is discarded with the /tmp directory.
No physical camera or serial device is mapped or accessed.
Neither test is a live recording or physical calibration.

## D. Physical operator-controlled capture — HOLD before checks pass

The executable M23H capture path is intentionally separate.
After the independent hardware safe-state and current device
identities are verified and all offline gates pass, choose an existing
writable offline host directory with >=9 GiB free for one 120s run.
Mount only that directory at /recordings:rw.
The previously used verified host devices are **historical**:
M20 used host /dev/video6 as in-container /dev/video2 and the
verified STM32 host /dev/ttyACM0 as same. They can re-enumerate.
Do not run with stale device numbers.

The operator's eventual Podman invocation uses the reviewed M23H
container with:
- `--network none --ipc private --pid private`
- `--cap-drop all --security-opt no-new-privileges --read-only`
- `--tmpfs /tmp:rw,nosuid,nodev,size=512m --shm-size 512m`
- precisely verified camera `--device host-video:/dev/video2`
- precisely verified STM32 CDC `--device host-tty:/dev/ttyACM0`
- offline writable `--volume host-existing-root:/recordings:rw`
- `-e M23H_SELFTEST=0 -e M23H_OPERATOR_CAPTURE_APPROVED=YES`
- `-e M23H_SESSION_ID=<unique_safe_id>`
- base image: `localhost/leroy-sensors:m23h-bounded-recording`.

The operator host wrapper `m23h_operator_host_capture.sh` supplies
these Podman isolation arguments and checks the stopped production
container, current hardware by-id, and competing device owners.
After independent physical safe-state and offline build verification:

~~~bash
cd "$(git rev-parse --show-toplevel)/ros"
export M23H_HOST_RECORDINGS="/absolute/existing/offline/directory"
export M23H_HOST_CAMERA="/dev/videoN"  # replace with current verified path
export M23H_MCU_BY_ID="/dev/serial/by-id/usb-VERIFIED-CURRENT-STM32-ID"
export M23H_SESSION_ID="capture001"
export M23H_OPERATOR_CAPTURE_APPROVED=YES
bash ./m23h_operator_host_capture.sh
~~~

These camera and MCU values are EXAMPLES, not currently verified
physical device identities. The host wrapper will refuse invalid
paths, existing output sessions, concurrent device owners, a running
production container or missing interactive approval.

Do not run physical capture if production leroy-ros is running.
Use a session ID unique per attempt. Recorder creates
`/recordings/m23g_live_<id>` and log is
`/recordings/m23h_capture_<id>.log`. No overwrite.
Attempted capture failures may leave partial SQLite; they MUST
be treated as unverified and not replayed as certified data.
Untrusted names or absent recorder flush result BLOCK acceptance.

## E. After capturing, offline evidence gate only

Take the CLOSED bag directory, unmount devices, ensure the production
bridge remains stopped, and run merged M23F against this bag under a
read-only `/input/recordings:ro` mount in a no-network container.
M23F should report exact image/CameraInfo/IMU counts and any faults.
The camera receive gap previously seen in real M20 (1.351s)
would **FAIL** the strict M23A >250ms gap guard if it appears in
the final recorded message header sequence. Do not bypass the
fault to claim VIO readiness.

The permitted provisional M23H success is
`CLOSED_SENSOR_BAG_RECORDED_PENDING_M23F`, not real VIO
validation. Full-frame payload, clock alignment, camera intrinsics,
camera–IMU extrinsics, IMU noise, XRSLAM scale/pose accuracy and
physical safe stop remain separately unverified.
