# M21C — Offline passive-sensor metadata contract

M20 physical concurrent camera and IMU message contract passed. M21A offline
readiness passed. M21B recovered +46.5 ms from a SYNTHETIC +47 ms injection.
None of these measurements establish Leroy's physical exposure-to-IMU offset.

This PR adds only a pure Python read-only JSONL format validator and twelve
offline tests. NO live recorder, camera device, serial port, ROS, or container
is started by this code. The previous M20 aggregate log cannot be used as
input because it does not contain individual frame and IMU packet histories.

## JSONL contract v1

First row: {"kind":"session","schema":"leroy_m21c_v1","evidence_type":"SYNTHETIC"}
Evidence types: SYNTHETIC or REAL_UNCALIBRATED only. The latter is only an
untrusted self-label, never cryptographic provenance or calibration evidence.

Allowed event kinds: camera_image, camera_info and imu. Common fields:
kind, header_ros_ns, receipt_mono_ns, frame_id and publisher.
Raw image pixels or extra fields are rejected. Camera events require
640x480 width and height, camera_optical_frame, and publisher nodes
/camera/rectify_node or /camera/usb_cam respectively.
camera_image also carries a monotonically increasing local frame_index.
IMU events carry uint32 mcu_seq, uint32 mcu_t_us and a finite 3-vector
gyro_xyz_rad_s, frame imu_link, publisher /m18_imu_source.

IMPORTANT: Existing ROS sensor_msgs/msg/Imu does NOT expose MCU seq or t_us.
A future recorder must record those values explicitly at the source; it
must never fabricate them from header timestamps or M20 aggregate metrics.

header_ros_ns and receipt_mono_ns refer to DIFFERENT clock domains and must
not be subtracted. MCU seq and t_us are unwrapped only to check continuity
(including uint32 wrap). Per-stream gaps and receipt overlap are reported.
Camera frame_index is a local callback count, not a source exposure index.
Even REAL_UNCALIBRATED records are NEVER treated as camera–IMU calibration.

## Arch host hardware-free test command

```bash
set +e
set +u
set +o pipefail
(
  set -euo pipefail
  cd "$(git rev-parse --show-toplevel)"
  git fetch origin
  git switch feature/m21c-offline-sensor-metadata-contract
  git pull --ff-only origin feature/m21c-offline-sensor-metadata-contract
  cd ros
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s . -p m21_record_contract_test.py -v
  TMP_M21C="$(mktemp)"
  trap 'rm -f "$TMP_M21C"' EXIT
  PYTHONDONTWRITEBYTECODE=1 python3 - "$TMP_M21C" <<'PY'
import json, sys
from m21_record_contract_test import fixture
with open(sys.argv[1], 'w', encoding='utf-8') as stream:
    for entry in fixture():
        stream.write(json.dumps(entry) + '\n')
PY
  PYTHONDONTWRITEBYTECODE=1 python3 m21_record_contract.py "$TMP_M21C"
) 2>&1 | tee m21c_offline_metadata_contract.log
M21C_OFFLINE_EXIT=${PIPESTATUS[0]}
echo "M21C_OFFLINE_EXIT_CODE=$M21C_OFFLINE_EXIT"
```

Expected: Ran 12 tests, OK; JSON verdict M21C_OFFLINE_METADATA_FORMAT_VALID,
evidence_type SYNTHETIC, physical_camera_imu_offset_validated false, exit 0.

## Future work / no physical authorization

Later design a passive per-frame camera+IMU metadata collector and independent
physical correlated-event protocol. Analyze camera frame gaps and callback
overload. Scalar gyro metadata alone does not produce an optical angular
motion feature. A future feature extractor will need image-frame processing;
this PR deliberately does not implement that or capture raw pixels.

Do not start the production leroy-ros container, motor bridge, /cmd_vel,
Nav2, XRSLAM odometry, depth mapping or DepthART. Future MCU access requires
separate operator authorization, BOTH wheel and manipulator power physically
inhibited, hardware E-stop verified, immobilized robot, device identity and
ownership checks, isolated disposable container and valid START/STOP ACKs.
The previous host front camera was /dev/video8 but must be reidentified.
DepthART CPU performance optimization remains deferred.
