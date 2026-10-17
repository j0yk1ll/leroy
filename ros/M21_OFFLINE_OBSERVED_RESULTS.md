# M21 — Observed offline M20 log evidence (2026-10-17)

## Provenance and acceptance

Operator ran the merged M21 offline audit on the complete saved M20 log.
The Python test suite passed (9/9) and the offline audit exited 0:

    M21_OFFLINE_VERDICT=READY_FOR_M21_CORRELATED_TIMING_EXPERIMENT_DESIGN
    M21_OFFLINE_EXIT_CODE=0

The terminal output had TWO M21_OFFLINE_REPORT_JSON records: the first was
the synthetic fixture printed by test_read_only_log_roundtrip, the second
was the real log audit. The fixture values are not physical observations.
This corrective branch captures fixture CLI output inside the test.

## Real observed per-topic summary (second JSON report)

| Metric | Rectified image | CameraInfo | IMU |
| --- | ---: | ---: | ---: |
| Accepted | 2082 | 3578 | 6001 |
| Invalid | 0 | 0 | 0 |
| Mean receipt rate | 17.36 Hz | 29.81 Hz | 50.00 Hz |
| Min reported header age | 35.3 ms | 33.4 ms | 1.2 ms |
| Max reported header age | 58.8 ms | 42.5 ms | 13.9 ms |
| Max receive gap | 1.351 s | 0.039 s | 0.028 s |

All expected stream frames and publishers matched the M21 contract.
120-second M20 observation and 119.886-second receive overlap are retained.
Host-relative MCU clock-rate estimate = -41.08 ppm.

## Interpretation

- The rectified camera is considerably slower than CameraInfo. The
  camera's 1.351-second maximum receive gap needs characterization before
  claiming steady-state temporal continuity. Its position within the
  capture is unknown because the summary contains no per-frame timeline.
- 17.36 Hz was accepted by M20's narrow message contract; it is not
  evidence that the camera satisfies XRSLAM timing requirements.
- Camera and IMU age values are derived through different timestamp
  conventions and transport paths. Their difference is NOT a measured
  physical exposure-to-IMU acquisition offset.
- M20 / M21's results do not establish camera-IMU extrinsics, XRSLAM
  initialization, pose convention, dynamic TF, or motor/navigation safety.

## Recommended next engineering step

Develop a separately reviewed, bounded passive recorder to capture
per-frame camera stamp and host monotonic receipt time, derived optical
rotation features, and timestamped MCU gyro values with clock diagnostics.
First exercise synthetic replay tests, including deliberately missing image
frames, offset injection, and outlier handling. Hardware observation must
retain all original motor power and E-stop restrictions. Any physically
correlated motion requires a separately approved immobilization-safe
experimental fixture and procedure; do not move the robot or run actuators
under the existing stationary capture authorization.

Only a future event-based calibration with sufficiently characterized
timing and uncertainties can establish a defensible exposure-to-IMU
physical acquisition offset. Continue deferring DepthART CPU optimization.
