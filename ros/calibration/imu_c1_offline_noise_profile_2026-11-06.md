# Leroy C1: offline-only IMU noise commissioning profile (2026-11-06)

**Status: exploratory, NOT an authorized XRSLAM runtime configuration.**
All VIO pose authority, `/odom`, map TF, motor access, and Nav2 remain disabled.
The camera source must retain the separate `usb_cam` epoch-microseconds fix.

## Evidence and convention

Source bag: `/leroy_ws/calibration/leroy_imu_stationary_long_01`,
202,118 `/imu/data_raw` samples over 2021.082 s, about 100.005 Hz.
Analyzer: `ros2 run leroy_localization imu_allan_diagnostics`.
Header interval max 10.042 ms; irregular intervals false. Device assumed motionless.
These observations are **not** a complete calibration.

The actual XRSLAM source on branch `feature/ros-tracked-features-export`
(`xrslam/src/xrslam/estimation/preintegrator.h`,
`xrslam/src/xrslam/estimation/preintegrator.cpp`) defines
`cov_w`, `cov_a`, `cov_bg`, `cov_ba` as continuous-time covariance
coefficients. It integrates measurement noise as `cov_w / dt` and
`cov_a / dt`, and bias random walk as `cov_bg * dt` and
`cov_ba * dt`. The Leroy ROS adapter passes single diagonal scalars
directly into `imu.noise.cov_{g,a,bg,ba}`.

**Hence square a suitable white-noise density to obtain the corresponding
continuous-time covariance diagonal. Do NOT multiply by 100 Hz.**
The adapter is currently isotropic (one scalar for each of the four 3x3
matrices), so any aggregate necessarily discards axis-specific information.

## Allan-derived measured/estimated terms

All six short-tau slopes at ~0.02 s are near -0.5, consistent with white
measurement noise. Densities are *estimates* subject to sampling, filtering,
temperature, quantization and scene-stationarity assumptions.

| Axis | Gyro density rad/s/sqrt(Hz) | Accel density (m/s²)/sqrt(Hz) |
| --- | ---: | ---: |
| X | 0.000282305 | 0.003006743 |
| Y | 0.000264764 | 0.003522775 |
| Z | 0.000236019 | 0.003330351 |

The corresponding *axis-specific* continuous-time covariance estimates
(density squared) are approximately:

| Axis | Gyro `cov_g` | Accel `cov_a` |
| --- | ---: | ---: |
| X | 7.96961e-8 | 9.04050e-6 |
| Y | 7.01000e-8 | 1.24100e-5 |
| Z | 5.57050e-8 | 1.10912e-5 |

A possible **isotropic exploratory aggregate** is the arithmetic mean
of those three *variances*, not the square of the average density:
`imu_cov_g_diagonal ~ 6.8500e-8` and
`imu_cov_a_diagonal ~ 1.08545e-5`.
These are *candidate measurement-noise inputs*, not validated navigation
parameters and not a YAML/launch preset.

Bias noise is **not identified**: gyro X/Y/Z and accel Y/Z showed no
qualifying +0.5-slope region; accel X has only a tentative local slope
+0.3488 at ~79.7 s (25 clusters). Do not automatically copy that candidate
into any axis. Thus:

| Parameter | Status |
| --- | --- |
| `imu_cov_g_diagonal` | Exploratory isotropic measurement variance density available |
| `imu_cov_a_diagonal` | Exploratory isotropic measurement variance density available |
| `imu_cov_bg_diagonal` | **Unknown. No number assigned.** |
| `imu_cov_ba_diagonal` | **Unknown. No number assigned.** |
| `camera_imu_offset_sec` | Provisional signed/magnitude candidates near zero; not certified |

## Next experiment (no controls or odometry)

1. Preserve raw Allan JSON, record temperature/stationarity assumptions,
   and verify the installed native library corresponds to the inspected
   source revision. The runtime directory `/opt/xrslam` has only headers,
   library and configs, not the estimator sources.
2. If offline VIO initialization is desired before bias drift is characterized,
   construct an **explicitly labeled sensitivity sweep** of positive
   `cov_bg` and `cov_ba` hypotheses, justified separately and never
   hardcoded as measured calibration. Keep all pose publishing authority
   disabled. Do not assume EuRoC bias coefficients represent Leroy.
3. Analyze initialization/tracking status and comparison against independent
   planar-motion ground truth. Stationary yaw or monocular rotation alone
   does not establish metric scale. A controlled translation may be needed.
4. Do not promote any parameter or pose into the navigation stack without
   separate extrinsic, timing, scale, stability and safety validation.

The local `usb_cam` timestamp-conversion source change also requires a
reproducible build patch (see `ros/patches/usb_cam/`) before any image rebuild.
