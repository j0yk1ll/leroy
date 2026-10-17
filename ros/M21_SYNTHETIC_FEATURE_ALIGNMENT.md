# M21B — Hardware-free scalar motion-feature alignment

## Scope

M20 physical sensor contract is already accepted. M21A offline audit found:
- 2,082 rectified images at 17.36 Hz; maximum image receipt gap 1.351 seconds.
- 3,578 CameraInfo messages at 29.81 Hz.
- 6,001 IMU messages at 50.00 Hz.
- No invalid messages; 119.886 seconds common receipt window; host-relative MCU clock-rate estimate -41.08 ppm.

This M21B implementation tests **synthetic scalar camera and IMU motion features**.
It does not access ROS, USB, serial, Podman or the robot. It cannot estimate an
offset from the existing M20 summary log, which contains no timestamped optical
motion feature series and no raw per-sample IMU history.

The sign convention is: effective lag = IMU feature timestamp minus camera
feature timestamp for the same motion feature. This is NOT a physical
exposure-to-MCU-acquisition offset measurement.

The pure-Python estimator searches plus/minus 200 ms, interpolates the IMU
stream only across intervals of at most 75 ms, clips extreme feature values,
checks correlation and peak ambiguity, and compares split-window candidates.
Missing camera frames are never reconstructed. The simulated trace includes
a roughly 1.4-second camera outage resembling the M20 maximum gap.

The scalar feature approach discards rotation-axis information and can be
ambiguous under periodic motion. In physical scenes, optical flow can also
depend on translation, scene depth and rolling shutter. A high synthetic
correlation is only an algorithmic development result, without an absolute
timing uncertainty bound or calibrated spatial extrinsics.

## Arch Linux host: no-hardware validation

Run from your existing Leroy repository Bash shell:

    set +e
    set +u
    set +o pipefail
    (
      set -euo pipefail
      cd "$(git rev-parse --show-toplevel)"
      git fetch origin
      git switch feature/m21-synthetic-feature-alignment
      git pull --ff-only origin feature/m21-synthetic-feature-alignment
      cd ros
      PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s . -p m21_synthetic_time_alignment_test.py -v
      PYTHONDONTWRITEBYTECODE=1 python3 m21_synthetic_time_alignment.py --demo
    ) 2>&1 | tee m21_synthetic_feature_alignment.log
    M21_SYNTHETIC_EXIT=${PIPESTATUS[0]}
    echo "M21_SYNTHETIC_EXIT_CODE=$M21_SYNTHETIC_EXIT"

Expected: 13 tests passing; SYNTHETIC_KNOWN_LAG_RECOVERED; injected +47 ms
recovered within 10 ms; M21_SYNTHETIC_EXIT_CODE=0. Synthetic results do NOT
verify a robot physical offset.

## Optional already-extracted feature input

The --input mode accepts a local JSON file containing two time-indexed arrays:

    {
      "camera_motion": [[100.000, 1.21], [100.058, 1.26]],
      "imu_motion": [[99.980, 1.18], [100.000, 1.20]]
    }

This illustrative snippet is too short for estimation. Valid offline feature
sets require at least 80 camera samples, 180 IMU samples and seven seconds of
nonstationary and uniquely varying observations. This program DOES NOT capture
camera pixels, extract optical flow or read gyro hardware. Even if --input
passes, it reports OFFLINE_FEATURE_ALIGNMENT_CANDIDATE_UNCALIBRATED and
physical_camera_imu_offset_validated=false.

## Next step: separately approved physical method

First implement and review a passive per-frame recorder and physical
time-correlated event plan. Retain camera exposure-related header stamp,
host receipt stamp, optical motion estimates, IMU MCU sequence/t_us and mapped
header stamp, gyro waveform, and clock diagnostics. Locate large frame gaps
and characterize their timing. A stationary scene without motion cannot
identify a visual–inertial time lag.

Physical correlated excitation requires a separately approved method using
a safe, constrained sensor fixture, not wheel or manipulator motor movement.
Before any MCU serial access physically inhibit BOTH wheel and manipulator
motor power, engage and verify hardware E-stop, immobilize Leroy, verify camera
and STM32 identities and exclusive ownership, and keep leroy-ros exited.
Use only isolated, device-limited disposable sensor containers and require
START and STOP ACKs; a missing STOP ACK leaves MCU state unverified.

Never publish /cmd_vel, launch the normal motor bridge, start Nav2 control,
or enable XRSLAM odometry/TF. DepthART CPU optimization remains deferred.
