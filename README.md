# Leroy — ROS 2 robot software

Leroy is an unfinished ROS 2 Jazzy mobile robot. The maintained source is organized by runtime responsibility, not by development milestone. **Autonomous operation and motor actuation are not commissioned.** No command-line option, parameter or environment variable currently enables unattended motion.

## Operator workflow

From `ros/`:

```bash
bash ./leroy.sh check    # source-only checks; no hardware
bash ./leroy.sh build    # build production base image with rootless Podman
bash ./leroy.sh status   # inspect production container
bash ./leroy.sh run      # intentionally denied until motion supervision is commissioned
bash ./leroy.sh stop     # does not report verified physical stopping
```

The platform must not be moved under power based solely on offline test results. MCU acceptance of a STOP command is **not** proof that motors have physically stopped. Keep the external power-cut E-stop and physically disconnected motor rails during sensor development.

## ROS packages

| Package | Responsibility |
|---|---|
| `leroy_bringup` | ROS launch composition, startup-readiness model and lifecycle integration points |
| `leroy_mcu` | STM32 protocol, clock sync, command allowlist, signed session handling, motion authority, motor control and stop/fault handling |
| `leroy_sensors` | IMU parsing, acquisition and sensor validity |
| `leroy_perception` | DepthART inference, depth-to-obstacle conversion and geometry |
| `leroy_localization` | Relative heading, calibration, kinematics, startup-localization state and XRSLAM adapter |
| `leroy_navigation` | Navigation configuration and a hardware deployment manifest that remains disabled |
| `leroy_description` | Robot description, TF ownership and sensor mounting configuration |
| `leroy_msgs` | Shared ROS interfaces |

Tests live under `ros/tests/` and package-local `test/` directories. The code and tests are *development evidence*, not a physical safety certification.

## Missing production requirements

The primary remaining work is: (1) physical sensor extrinsics and timing; (2) validated metric odometry and localization; (3) measured all-around obstacle coverage; (4) single STM32 serial ownership and verified hardware stop/driver-power interlocks; (5) wheel/motor commissioning; (6) a fault-latched lifecycle supervisor; and (7) a separately reviewed, physically accepted navigation deployment. Never change `ros/src/leroy_navigation/config/hardware.disabled.yaml` to enabled just to make a launch succeed.

`ros/src/leroy_mcu/leroy_mcu/command_ingress.py` deliberately rejects nonzero wheel commands. Preserve that boundary until the motor authorization path passes independent hardware acceptance.

The current source retains compatibility identifiers used by firmware/ROS protocols; they are not indications of implementation readiness. Schema changes require coordinated migration rather than blind string replacement.
