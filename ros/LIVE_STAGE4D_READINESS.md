# Stage 4D — offline recorder load and fail-closed fault evidence

**Status: CODE CANDIDATE; NO PHYSICAL RUN, NO CALIBRATION AUTHORITY.**

This is deliberately stacked on the still-draft Stage 4C source branch.
It does not change the accepted Stage 4B MCU/serial source, STOP-ACK
behavior, motor safety, calibration constants, ROS TF, XRSLAM or navigation.

## Prior operator-verified gates, not reinterpreted

The operator reported Stage 4C **31/31** pure-Python tests PASS
and a disconnected synthetic DDS/SQLite bag round-trip:
`STAGE4C_SYNTHETIC_DDS_BAG_INTEGRATION=PASS_NO_DEVICE`,
`STAGE4C_SYNTHETIC_IMU_META_MATCHED=12`,
`STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE`,
`STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE`.
Stage 4C CI later passed **36/36** source-only tests. The 12-sample
DDS test demonstrates only a small synthetic end-to-end path;
it is NOT a rate/load qualification or physical phase measurement.

## Stage 4D scope

1. `live_stage4d_capture_ingress.py` is a pure-Python reusable
   message receipt + serialized SQLite-writer contract.
   **The hard-disabled Stage 4C passive recorder now invokes this same
   ingress**, rather than maintaining a separate, untested
   message-validation copy. It validates:
   - strictly whitelisted ROS sensor topics and frame stamps;
   - Image width/height/step/encoding and full row data;
   - CameraInfo finite, positive focal lengths and geometry;
   - IMU finite linear acceleration and angular rates;
   - original MCU `seq/t_us` metadata JSON and time correspondence;
   - serialized message, monotonic receipt and bounded record/byte limits.
   Each unexpected message, serializer exception or writer error
   **latches a terminal fault**, denies further samples and forces the
   owning recorder to abandon the attempt. This code has NO ROS
   initialization, serial/USB access, file open or motor API.
2. `live_stage4d_capture_test.py` evaluates the shared ingress
   using **virtual event time**: 75 simulated seconds, 3,750 IMU
   messages at 50 Hz and 2,250 640x480 mono8 images + CameraInfo
   at 30 Hz, each IMU with original-shaped synthetic MCU metadata.
   The result is **12,000 callbacks**, with exact header-stamp
   matching, all within the existing 20,000-record / 900 MB
   serialized-payload budget. This is a **pure Python in-memory
   workload**, *not* real 75-second DDS, camera, SQLite or physical
   throughput evidence. Tests also inject disk-full writer errors,
   invalid camera geometry/intrinsics, nonfinite IMU values, invalid
   MCU ticks, oversize CDR payload and serializer exceptions.
3. `live_stage4d_synthetic_load.py` is a **short, real DDS + SQLite
   performance smoke** with an entirely fake camera and M18-named
   source, no actual device or MCU source module. It runs 250 IMU
   samples at a paced 50 Hz and 150 synthetic 640x480 images plus
   CameraInfo at 30 Hz *mean* over nominal 5 seconds, and 250
   original-shaped metadata records. It demands exact subscriber
   receipt counts, exact stamp-matched metadata and SHA-256 replay
   matching, then checks that the SQLite file is at most 96 MB.
   This is a limited smoke, not a 75s stressed runtime qualification.
   The actual writable /tmp is independently limited to 256 MB
   by the operator's disposable Podman invocation.
4. `live_stage4d_synthetic_load_test.py` and the new Stage 4D
   GitHub workflow check the offline executable's source,
   no-device preflight, strict source identity, ingress linkage,
   safe container requirements and still-hard-blocked live recorder.

**Not exercised:** The Stage 4C live recorder's real 75s node lifecycle,
real CameraInfo/USB exposure semantics, actual STM32 callback with raw
acquisition ticks, physical camera–IMU offset, sustained under-load
DDS delivery with hardware and the MCU STOP acknowledgment. The
new fake M18 node cannot authenticate the real MCU. The 900MB
serialized-payload guard does not by itself guarantee a hard SQLite
filesystem quota; journal/metadata overhead remains. An additional
source/safety/disk-failure review is required before modifying any
physical capture launcher.

## Verified source-only CI result

On the Stage 4D draft PR, the new regression workflow passed
**15/15 Stage 4D tests** (virtual 75-second 12,000-message
accounting and fault injection) and separately reran **36/36 Stage 4C
tests** successfully: **51/51 source-only tests total**.
The other two triggered repository checks also passed.
This is GitHub source-only CI; it does **not** run the native
paced DDS+SQLite smoke or authenticate any physical data.
The latter remains **OPERATOR TEST REQUIRED**.

## Offline acceptance commands — no devices

The Stage 4D branch is a *new* worktree stacked on the Stage 4C source:

```bash
cd /home/kevin/Code/leroy
git fetch origin feature/stage4d-offline-recorder-load-faults
git worktree add --detach ../leroy-stage4d \
  origin/feature/stage4d-offline-recorder-load-faults
cd ../leroy-stage4d/ros

PYTHONDONTWRITEBYTECODE=1 python3 -Werror::SyntaxWarning -m unittest -v \
  live_stage4d_capture_test live_stage4d_synthetic_load_test

PYTHONDONTWRITEBYTECODE=1 python3 -Werror::SyntaxWarning -m unittest -v \
  live_stage4c_capture_test live_stage4c_source_meta_test \
  live_stage4c_source_meta_ros_test live_stage4c_synthetic_dds_test
```

The above tests use host Python without `cv2`, ROS, cameras,
motors or containers. In the same `ros` folder, run the
paced synthetic DDS+bag test using the already built
`localhost/leroy-ros:stage4c-capture-offline` image; **no rebuild
and no new privileged container are needed**:

```bash
podman run --rm --pull=never \
  --network=none --ipc=private --pid=private \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  -v "$PWD:/offline:ro" \
  -e PYTHONPATH=/offline:/opt/stage4c \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e ROS_DOMAIN_ID=171 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_STAGE4C_SYNTHETIC_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/stage4d-ros \
  -e XDG_CACHE_HOME=/tmp/stage4d-cache \
  --entrypoint bash localhost/leroy-ros:stage4c-capture-offline -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    exec python3 -u /offline/live_stage4d_synthetic_load.py
  '
```

The *expected* result if the operator's actual local ROS/DDS and
SQLite run succeeds is:

```text
STAGE4D_SYNTHETIC_DDS_LOAD=PASS_NO_DEVICE
STAGE4D_DDS_COUNTS_JSON={"/camera/camera_info": 150, "/camera/image_rect": 150, "/imu/data_raw": 250, "/stage4c/imu_source_meta": 250}
STAGE4D_DDS_CALLBACK_DELAY_MS_P95=<measured number>
STAGE4D_LIVE_CAPTURE_AUTHORIZED=FALSE
STAGE4D_PHYSICAL_CALIBRATION_VALIDATED=FALSE
```

No real-run output is claimed until the operator posts their
result. An interrupted run, incomplete count, DDS graph change,
bad bag replay or disk-full is a **BLOCK**, not a reason to relax
the hard gate.

## Absolute safety and authorization boundaries

`LIVE_CAPTURE_COMMISSIONED = False` remains hardcoded in
`live_stage4c_passive_recorder.py`, and the candidate image
still defaults to `CMD ["/bin/false"]`.
Do not run `leroy-ros`, `live_stage4_imu_sidecar.sh`,
`utils/imu/calibrate_time_offset.py`, or any physical
calibration motion. Opening `/dev/ttyACM0` can reset the
motor-capable STM32 even without publishing motion commands.
A future physical session needs an independent, explicitly
authorized plan, both motor-driver and manipulator rails
physically isolated, secured chassis, verified E-stop,
operator presence, correct START/STOP allowlist and
mandatory **STOP ACK**. Missing STOP ACK means leave both
power rails isolated and do not retry.

Stage 4D acceptance is **offline recorder confidence only**.
It cannot validate camera exposure timestamp semantics,
camera/IMU extrinsics, calibrated IMU noise, physical temporal
offset, XRSLAM VIO, metric odometry, map TF or navigation.
