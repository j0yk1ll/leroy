# M23AU — FIRST camera-only pthreads trial, REVIEW ONLY

**Current state: PREPARED, NOT EXECUTED OR AUTHORIZED.**
A user has reported M23AT read-only host identity/image check PASSED
after reboot and attested the original 92 ChArUco images were taken
with the same Wed Camera and its optical/crop settings have not changed.
Stored calibration M23AS has 92 source 640x480 PNGs, no rejected
views, reprojection RMS 0.3184px, and deployed K/D/R/P match original.
**Motor and manipulator supply disconnection must be re-confirmed
AFTER reboot and immediately prior to any actual camera access.**
A source test, YAML match, or setting an environment flag is not
proof of a motor's physical electrical state.

M23AK/AL/AM/AN/AO/AP synthetic pthreads image ID:
334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025.
Stock rclcpp_components component_container passed 900/900 and real
child exit 0 (M23AP). Nominal host capacity 16 logical CPUs,
21,596,412 KiB MemAvailable. The M23AO diagnostic process consumed
an average of ~1.28 logical CPUs and high-water RSS 50.83MiB;
live driver/DDS/host overhead and camera latency remain unverified.

## Critical physical isolation

The normal robot.launch.py unconditionally starts leroy_bridge,
which may open the MCU CDC serial port and energize actuators even
though commands are software limited. **NEVER use robot.launch.py
or ros/exec.sh for this experiment.** Do not attach or map any serial,
IMU, robot, actuator, DepthART, XRSLAM, Nav2, recorder, host
network or entire /dev directory. Only one specifically verified
Wed Camera capture node may be passed into a fresh rootless Podman.

Original live M23K/M23O camera-only experiments with the distro TBB
OpenCV repeatedly showed SIGSEGV at component SIGINT, and the
raw/rect DDS observers reported low rates/gaps despite native V4L2
MJPEG capturing about 29.81fps. The pthreads mitigation has NOT
yet been tested on a physical camera. A clean synthetic M23AP
cannot guarantee a clean live trial or >25 Hz observations.

## What this PR adds

The uninstalled, inert-by-default Python wrapper
ros/m23au_bounded_camera_probe.py:
- invokes **the already accepted M23K/M23O 45-second real-camera
  supervisor and C++ header-only DDS observer**;
- substitutes ONLY the candidate M23AQ two-node camera launch,
  containing patched-overlay usb_cam and stock image_proc::RectifyNode
  in rclcpp_components/component_container;
- selects M23O reliable_100 graph/QoS audit and continuity gate:
  no raw/rect header gaps >250ms, raw+rect observed rate >=25Hz,
  >=95% header pairing with no unexplained rect-only stamps;
- requires actual clean exit markers for both camera and component
  children after the original M23K SIGINT process-group shutdown;
- preserves original M23K/M23O summary JSON on stdout even if
  the throughput gate blocks. Child-crash context is surfaced by
  original M23K code. It does NOT record pixel data or ROS bag.

Failing throughput can reflect separate observer DDS deserialization
drops or CPU contention and does NOT prove the capture driver
itself is slow. Report as a **blocked/incomplete diagnostic**, do
not weaken >=25Hz or 250ms limit, and do not automatically retry.

This first physical test does NOT validate the causal TBB unload
hypothesis; it is intended to test the practical pthreads mitigation
on the actual stock ROS container plus raw camera path.

## Stage A — source-only, SAFE TO RUN NOW

After reopening a terminal after reboot, create a new persistent
sparse source checkout from the exact branch (do not assume /tmp
worktrees survive):

~~~bash
(
  set -euo pipefail
  set -C
  umask 077
  test "$(id -u)" -ne 0
  DIR="$HOME/.local/state/leroy-m23au-source"
  test ! -e "$DIR"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23au-camera-only-live-protocol-review \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23aq_camera_only.launch.py' \
    '/ros/m23au_bounded_camera_probe.py' \
    '/ros/m23au_source_test.py' \
    '/ros/config/camera/front_camera.yaml'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23au_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23au_bounded_camera_probe.py
)
~~~

Require all 14 offline tests and
M23AU_STATUS=NO_ROS_OR_CAMERA_BY_DEFAULT.
These tests do not start a ROS node, execute Podman, or touch devices.
No actual camera trial is approved by executing Stage A.

## Stage B — mandatory renewed PRECONDITIONS and operator approval

DO NOT EXECUTE STAGE C until all of the following are independently
checked and the actual camera-only procedure has been reviewed:
1. After reboot and again immediately before launch, human explicitly
   attests **both motor and manipulator power supplies are physically
   disconnected**, including no alternative power source; robot
   immobilized. Do not rely on software disabled wheels.
2. Resolve the known Wed Camera /dev/v4l/by-id/ video-index0 link
   to a single current /dev/videoN capture device using udev
   vendor/product 32e6:9221 and V4L capture property. Do not
   disclose the full by-id serial string in logs or output. Verify
   the node is not already owned by another process.
3. Original robot/serial processes are not running; rootless
   podman ps is empty. No tty/serial devices are mapped. No
   preexisting running guest container may retain this camera.
4. Pinned pthreads image ID and its internal
   /config/camera/front_camera.yaml content matches source
   SHA256 from the M23AS validated checkout. Keep image immutable,
   no installs/rebuilds and network disabled.
5. Avoid duplicate evidence: host log
   ~/.local/state/leroy-m23au/live-camera-01.log must not exist.
6. Physical operator explicitly approves this **one** bounded
   camera-only observation and understands SIGSEGV, apparent
   rate <25Hz, wrong mode, >250ms gaps or child cleanup failure
   is a STOP, not permission to repeat. Outer timeout bounds
   approximately 45s observation plus warmup/cleanup (<100s);
   operator can issue podman stop to the named disposable guest
   from another terminal. No automatic rerun.

For the first approved trial, an appropriate host script should
perform these read-only preflight checks anew, require an explicit
interactive power-isolation/approval acknowledgment, verify the
camera/YAML hash and invoke ONLY the pinned image with:
- --rm --pull=never, --network=none --ipc=private --pid=private;
- --cap-drop=all, --security-opt=no-new-privileges, --read-only;
- --shm-size 512m, private /tmp tmpfs, memory and PID caps;
- --group-add keep-groups if required for rootless video read;
- --device "$VERIFIED_CURRENT_VIDEO_NODE":/dev/video2 ONLY;
- read-only binds for these exact M23AQ and M23AU files;
- ROS Jazzy and workspace env, LD_LIBRARY_PATH with
  /opt/m23aj-opencv/lib first, M23AU/M23AQ explicit gates;
- actual wrapper --run in place of default inert mode.

**This PR does not include an immediately runnable live shell
invocation.** A separate final human confirmation of post-reboot
electrical isolation and approval is required to release one.

## Evidence/acceptance for the eventual authorized one-shot test

Require M23O_RECT_QOS_VERDICT=RECTIFIED_CAMERA_QOS_DIAGNOSTIC_ONLY,
M23AU_VERDICT=BOUNDED_CAMERA_ONLY_DIAGNOSTIC_COMPLETE,
zero >250ms header gaps, camera and rectifier both cleanly
exited, plus outer host exit 0. Preserve the complete private
host stdout/stderr and original M23O metrics. Any failed
predicate means diagnostic remains blocked; do not overstate
camera's true published frame rate from two independent
subscribers. If the camera node rejects MJPEG mode or gives
lower rate, stop and review, not auto-switch camera resolution,
QoS or pixel format.

Physical camera data may validate camera-only behavior; it
does not authorize XRSLAM, DepthART, serial MCU/IMU,
recording, active navigation, motor power restoration or
production use.

M23AU_PHYSICAL_CAMERA_AUTHORIZED=FALSE
M23AU_PRODUCTION_ACCEPTANCE=BLOCKED
