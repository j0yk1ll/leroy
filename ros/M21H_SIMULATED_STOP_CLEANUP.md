# M21H — M20 serial STOP cleanup regression with FAKE CDC port

## Prerequisite

Operator M21G fault-path simulation passed: 6/6 tests,
M21G_SOURCE_FAULT_SELFTEST=PASS, M21G_DEVICE_ACCESS=NONE,
M21G_SIM_EXIT_CODE=0. PR #72 was verified merged into main
at a25ee10fef8fec3b44c312e8bdcaae2cdd5e6b7f.
Real camera–IMU physical acquisition offset remains UNVALIDATED.

## Concrete source issue and smallest fix

In M20, serial.Serial(...) originally opened the MCU port, followed
by time.sleep(0.3) and reset_input_buffer(), THEN entered a try/finally
that sends set_imu_stream_hz 0 (STOP). An input-buffer reset failure
could therefore leave the opened port without a STOP attempt.

M21H moves only the try boundary to immediately after successfully
opening the port. The existing settle, reset, START, observation loop,
STOP finally and ACK logic remain otherwise unchanged. This does NOT
guarantee STOP acknowledgment in the presence of process death, USB
disconnection or failed writes. After a serial-open failure, no handle
exists to send STOP; this software patch cannot make MCU state certain.

## Five ROS fault-injection regressions

The tests call the actual M20.run() with serial.Serial monkeypatched to
an in-memory FakeCdc. The fake permits ONLY previously allowlisted START
(set_imu_stream_hz 50) and STOP (set_imu_stream_hz 0), then supplies
synthetic ACKs or controlled errors. No real tty or camera is opened.

1. Input buffer reset fails after opening: STOP still attempted/ACKed;
   START never issued, M20 rejects the run.
2. An opt-in M18 metadata callback throws: source faults, then
   M20 attempts STOP and rejects the run.
3. Serial read fails after START ACK: STOP attempted/ACKed.
4. STOP ACK missing: M20 reports UNVERIFIED and refuses acceptance.
5. Forbidden ROS graph before port access: serial port never opened.

A PASS shows cleanup-control-flow behavior in these exact simulated
conditions ONLY, not physical STM32 recovery or safety certification.

## Build M21H no-device image on Arch

The host must have previously accepted M21G image.
Production leroy-ros MUST remain exited. No device flags.

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21h-runner-cleanup-tests
  git pull --ff-only origin feature/m21h-runner-cleanup-tests
  cd ros
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman image exists localhost/leroy-sensors:m21g-fault-latch-selftest
  podman build --network none --no-cache \
    -f Dockerfile.m21h-selftest \
    -t leroy-sensors:m21h-cleanup-selftest .
) 2>&1 | tee m21h_cleanup_build.log
M21H_BUILD_EXIT=${PIPESTATUS[0]}
echo "M21H_BUILD_EXIT_CODE=$M21H_BUILD_EXIT"
```

Require M21H_M20_IMPORT_SELFTEST=PASS and M21H_BUILD_EXIT_CODE=0.

## Run isolated fake CDC regression

Only after build success:

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  test "$(podman inspect leroy-ros --format '{{.State.Status}}')" = exited
  podman run --rm \
    --name leroy-m21h-cleanup-only \
    --network none --ipc private --pid private \
    --shm-size 512m \
    --cap-drop all --security-opt no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=512m \
    localhost/leroy-sensors:m21h-cleanup-selftest
) 2>&1 | tee m21h_cleanup_simulation.log
M21H_SIM_EXIT=${PIPESTATUS[0]}
echo "M21H_SIM_EXIT_CODE=$M21H_SIM_EXIT"
```

Required: Ran 5 tests; OK; M21H_STOP_CLEANUP_SELFTEST=PASS;
M21H_VERDICT=SIMULATED_M20_SERIAL_CLEANUP_ONLY;
M21H_DEVICE_ACCESS=NONE; M21H_SIM_EXIT_CODE=0.

## Physical safety / limits

Do NOT run the physical M20 container or open /dev/ttyACM0 under
this procedure. Even STOP ACK of an in-memory simulated port
is not proof that a real MCU will acknowledge STOP after reset,
USB failure or control process termination. No motor, wheel,
manipulator, normal bridge, /cmd_vel, odometry, TF, Nav2, XRSLAM
or DepthART is enabled or authorized. For any later separately
approved hardware session: PHYSICALLY inhibit both wheel AND
manipulator power, verify hardware E-stop, immobilize the robot,
check exact current device IDs/ownership, require exclusive
isolated least-privilege container and verified START/STOP ACKs.
On uncertain STOP, KEEP motor power inhibited and inspect MCU state.
DepthART CPU optimization remains deferred.
