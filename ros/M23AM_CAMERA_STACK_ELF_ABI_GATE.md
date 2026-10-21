# M23AM — device-free camera-stack ELF compatibility gate

## Accepted background

M23AK 1/1 stock composable rectifier clean and M23AL 5/5 distinct
synthetic no-TBB OpenCV shutdown runs clean. Each M23AL trial required
actual ROS child exit 0, 900/900 synthetic raw/rect/info, graph/QoS,
matching stamps, publisher drain, M23AF mapping phases, replacement
OpenCV 4.6 selection and no observed TBB executable mapping.

M23AJ pthreads image pinned ID:
`334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`.

The next goal is NOT physical activation. It is to test that the
specific **Leroy patched usb_cam 0.8.1 overlay** and ROS camera
libraries resolve dynamic symbols against the replacement OpenCV
and do not pull in TBB or distro OpenCV via another ELF dependency.
This checks the relevant camera-only path more narrowly than
all robot packages.

Project Dockerfile declares the patched overlay via
`/leroy_ws/install/usb_cam`, whose component is
`libusb_cam_node.so` and library `libusb_cam.so`. The camera
timestamp patch is important; do not accidentally test the distro
usb_cam executable or bypass this overlay.

## Stage A — no-runtime tests

From the M23AL source checkout:

~~~bash
cd /tmp/leroy-m23al
git fetch origin feature/m23am-camera-stack-pthreads-elf-abi-audit
git worktree add --detach /tmp/leroy-m23am   origin/feature/m23am-camera-stack-pthreads-elf-abi-audit
cd /tmp/leroy-m23am
git status --porcelain
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23am_source_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23am_camera_stack_elf_audit.py
~~~

Expect eleven offline tests and
`M23AM_STATUS=NO_ELF_READ_OR_COMMAND_BY_DEFAULT`.

## Stage B — EXACT saved image, one isolated loader-only audit

This uses *one* rootless Podman instance started from the immutable
M23AJ image ID, with network disabled, no device binds, read-only
root and empty temporary directory. It does not run any ROS nodes,
capture images, alter the image or execute any camera code. The
trusted ELF dynamic linker is allowed to resolve relocations with
`ldd -r`; that is **not** equivalent to full application ABI proof.

The Python script checks:
- `ros2 pkg prefix usb_cam` is exactly the patched overlay;
- six ELF targets: usb_cam executable, its component and support
  library, cv_bridge, image_geometry and stock librectify;
- `ldd -r` returns zero with no missing DT_NEEDED files,
  unresolved relocations, or TBB dependencies in their ELF closure;
- every `libopencv_*.so` resolves from
  `/opt/m23aj-opencv/lib/` and uses the expected `.so.406`
  SONAME; all six camera-pipeline OpenCV modules are present
  in the aggregate dependency closures.

One new owner-private log `compatibility-01.log` must not overwrite
earlier evidence. Any unresolved symbol, missing dependency,
wrong overlay, mixed OpenCV, oneTBB, or launch failure is a stop
condition—do NOT automatically install packages or rebuild.

This audit does not test Python cv2/DepthART, IMU, serial, VIO,
robot control, dynamic runtime loading outside the inspected
ELF closure, full OpenCV ABI or performance. Those remain
outside camera-only preflight.

## Next gate

After this passes, benchmark realistic synthetic resolution/FPS
against the camera's expected operating mode and confirm timing,
CPU and memory headroom. Then review a separately constrained
camera-only physical launch with real calibration, explicit device
allowlist, safe stop behavior and motors physically disabled.

No camera/video devices, STM32/IMU/serial, DepthART/XRSLAM,
recording, Nav2, motors, motion or production in M23AM.
Physical camera and production authorization remain BLOCKED.
