# Handoff 1 — XRSLAM: ROS 2 Jazzy, Persistent Mapping and Semantic Search

## 1. Mission

Continue development of the XRSLAM fork at:

**Repository:** `https://github.com/j0yk1ll/xrslam`  
**Last verified `main` commit:** `bf7886dd05617d32e330a69e76fa10df04f5777b`  
**Development environment:** Arch Linux, `/home/kevin/Code/xrslam`, Python `.venv`, CMake build directory `build-eigenplaces-cpu`  
**ROS target:** ROS 2 Jazzy  
**Robot:** Leroy, an indoor mecanum robot with a front camera, IMU and partially faulty wheel encoders.

**Primary objective:** XRSLAM must own visual-inertial odometry, persistent localization, keyframe-based semantic memory and semantic search. Nav2 will be used separately for navigation and obstacle avoidance, communicating only through ROS 2 interfaces.

Do **not** introduce RTAB-Map, directly integrate Nav2 into XRSLAM, or restore the removed XRSLAM occupancy mapper.

## 2. Existing implementation and validated functionality

The following components already exist:

| Subsystem | Current state |
|---|---|
| Visual-inertial SLAM | Existing XRSLAM estimator, sliding-window optimization, keyframes and landmarks |
| Global place descriptors | EigenPlaces TorchScript |
| Place retrieval | USearch |
| Geometric relocalization | ORB descriptors, PnP, multi-frame verification and VIO recovery |
| Persistent reference map | Preserved across tracking-session resets **within a process** |
| Global pose corrections | Output-only 4-DoF correction; estimator remains untouched |
| Semantic segmentation | Asynchronous PIDNet TorchScript worker |
| Semantic archives | Keyframe masks, confidence masks, poses and sampled landmarks |
| 3D semantic landmark fusion | Session-local, confidence-weighted, multi-view |
| Dense semantic point cloud | Registered metric RGB-D sampling and voxel fusion |
| Dense quality filters | Depth-edge rejection, spatial filtering and multi-view evidence |
| PLY visualization | RGB or class-colored semantic point clouds |
| 3D occupancy mapper | **Removed** in favor of Nav2's Voxel Layer |

The preferred dense reconstruction settings are:

- Voxel size: **0.02 m**
- Pixel stride: **2**
- Quality filtering: enabled

Relevant environment variables include `XRSLAM_DENSE_VOXEL_SIZE_M`, `XRSLAM_DENSE_PIXEL_STRIDE`, `XRSLAM_DENSE_QUALITY_ENABLE`, `XRSLAM_SEMANTICS_ENABLE`, `XRSLAM_PIDNET_MODEL` and `XRSLAM_PIDNET_DEVICE`.

The present PIDNet model uses **Cityscapes-19**, which does not include `towel`. The intended future deployment will use a fine-tuned indoor taxonomy. Semantic metadata must therefore not assume Cityscapes classes permanently.

### Validated regressions

The established regression is:

`env XRSLAM_SEMANTICS_ENABLE=0 bash ~/Downloads/0121p_graph_preservation_regressions.sh both`

Previously validated results:

- Healthy EuRoC V2_03 replay: 1,813 samples, no false automatic-loss triggers.
- Synthetic 16-frame dropout: one automatic loss trigger, one confirmed historical PnP seed, one successful handoff, one restarted VIO session and 481 subsequent tracking steps.
- All healthy/dropout local/global trajectory comparisons achieved 100% timestamp matching and zero translation difference at printed precision, with maximum rotation difference approximately 0.000003°.

Dense reconstruction was successfully tested on TUM Freiburg1 XYZ. The 0123d occupancy functionality was validated and subsequently removed intentionally. Preserve the functioning dense semantic map and its regression tests.

## 3. Current source-code entry points

Important repository files:

| File | Role |
|---|---|
| `xrslam-interface/include/XRSLAM.h` | Public C API |
| `xrslam-interface/src/XRSLAMInternal.cpp` | C API dispatch |
| `xrslam-interface/src/XRSLAMManager.cpp` | Runtime creation, sensor ingestion, output poses and semantic-worker setup |
| `xrslam/src/xrslam/core/detail.*` | XRSLAM runtime and worker ownership |
| `xrslam/src/xrslam/core/frontend_worker.*` | Tracking lifecycle and recovery |
| `xrslam/src/xrslam/core/sliding_window_tracker.*` | Optimized keyframe archive and semantic snapshot submission |
| `xrslam/src/xrslam/core/keyframe_archive.h` | Value-owned optimized keyframe geometry |
| `xrslam/src/xrslam/core/persistent_relocalization_map.h` | Historical in-process VIO-session reference storage |
| `xrslam/include/xrslam/place_recognition.h` | Descriptors, place keyframes and place database interfaces |
| `xrslam/include/xrslam/semantic_mapping.h` | Semantic keyframe/depth snapshot types |
| `xrslam/include/xrslam/semantic_landmark_map.h` | Sparse semantic landmark fusion |
| `xrslam/include/xrslam/dense_semantic_voxel_map.h` | Dense semantic voxel fusion |
| `xrslam-extra/include/xrslam/extra/pidnet_semantic_worker.h` | Worker, semantic records and snapshot APIs |
| `xrslam-extra/src/xrslam/extra/pidnet_semantic_worker.cpp` | Semantic inference and fusion |
| `xrslam-extra/src/xrslam/extra/yaml_config.cpp` | Camera/IMU configuration loading |
| `xrslam-ros/src/xrslam-ros/` | **Legacy ROS 1 code; not a ROS 2 implementation** |

The legacy ROS node uses `roscpp`, catkin and hard-coded EuRoC camera values. Implement a fresh Jazzy adapter rather than treating that node as already ported.

The current public C API includes `XRSLAMCreate`, `XRSLAMRequiresIMU`, `XRSLAMPushSensorData`, `XRSLAMRunOneFrame`, `XRSLAMGetResult`, `XRSLAMDestroy`, and read-only sparse/dense semantic-map snapshot functions.

**Important:** The current `XRSLAMCreate` signature accepts five arguments, including `void **config`. Do not reuse the outdated invocation in the ROS 1 node.

Also note that `XRSLAM_RESULT_GLOBAL_BODY_POSE` can fall back to the local pose when no usable graph correction exists. Equality between local and global poses is **not proof of valid persistent-map localization**.

## 4. Milestone A — Standalone `xrslam_ros2` Jazzy adapter

Create a new ROS 2 package, preferably named `xrslam_ros2`, using `ament_cmake` and `rclcpp`. Keep it separate from the legacy ROS 1 package.

### Inputs

- `/imu/data_raw`: `sensor_msgs/msg/Imu`, currently 100 Hz, best-effort QoS, frame `imu_link`.
- Front-camera rectified image: `sensor_msgs/msg/Image`; topic must be configurable and discovered from the live graph.
- Camera calibration: `sensor_msgs/msg/CameraInfo`, configured according to the active camera driver/rectification pipeline.
- Static robot geometry: TF2, published by Leroy's `robot_state_publisher`.
- Optional RGB-D input only when its metric scale and registration are verified.

Wheel encoder observations are **not required**. Do not modify the estimator to fuse faulty wheel readings.

### Static calibration ownership

The canonical source for mounted sensor positions and orientations is **Leroy's URDF**, not an independent XRSLAM extrinsics YAML.

Before XRSLAM initialization, obtain:

- `base_link ← imu_link`
- `base_link ← camera_optical_frame`

Use TF2 lookups such as `lookupTransform("base_link", "imu_link", tf2::TimePointZero)` and the corresponding camera lookup.

The XRSLAM device configuration currently reads `cam0.extrinsic.q_bc`, `cam0.extrinsic.p_bc`, `imu.extrinsic.q_bi`, `imu.extrinsic.p_bi`, and `cam0.intrinsics` from YAML.

Implement a controlled runtime configuration override or temporary generated configuration that injects:

- Verified TF-derived camera and IMU extrinsics.
- Intrinsics from the actual rectified `CameraInfo`.
- Existing XRSLAM estimator configuration, noise parameters and explicitly calibrated camera/IMU time offsets.

**Do not guess transform direction.** Confirm the mathematical conventions of `q_bc`, `q_bi`, camera optical axes, quaternions and body poses using a nonidentity transform test.

If transforms, camera calibration or a required sensor are missing, do not initialize with placeholder identity geometry.

### Timestamp and sensor ingestion requirements

- Respect ROS message header timestamps rather than arrival times.
- Consume both angular velocity and linear acceleration from each IMU message.
- The IMU reports no orientation estimate; `orientation_covariance[0] = -1` is expected and does not prevent VIO.
- Handle delayed/out-of-order input explicitly.
- Support best-effort sensor-data QoS.
- Handle grayscale and color image encodings correctly, including channel count, stride and buffer lifetime.
- Never apply a second undistortion step to an already rectified image.
- Gate camera processing until its prerequisite IMU interval is available where required.
- Verify time alignment between MCU-derived IMU timestamps and camera timestamps.

### Outputs owned by XRSLAM

| Output | ROS 2 type | Requirement |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | Continuous, locally smooth robot-body odometry |
| `/tf`: `odom → base_link` | TF2 dynamic transform | **XRSLAM is the sole publisher** |
| `/xrslam/pose` | `geometry_msgs/msg/PoseStamped` | Diagnostic pose output |
| `/xrslam/tracking_status` | Status/diagnostic message | Initialization, tracking loss and recovery |
| `/xrslam/trajectory` | `nav_msgs/msg/Path`, optional | Debugging, preferably bounded |

Odometry must represent `base_link`, not camera optical coordinates.

Publish meaningful timestamps, frame IDs, velocity estimates and defensible covariance. Do not invent high-confidence covariance values or silently publish zero velocity as though it were measured.

### Tracking-loss behavior

The established recovery pipeline can create a new VIO session. The adapter must not let a session-origin change silently jump `odom → base_link`.

Required behavior:

- During initialization, withhold valid odometry.
- During tracking, publish local continuous odometry.
- During loss, publish invalid/degraded status and stop producing falsely valid pose updates.
- During recovery, only resume using a verified alignment that preserves odometry continuity.
- Provide a clear safety signal for consumers to inhibit motion if localization is unavailable.

Global corrections and relocalization effects must not be applied directly to the smooth `odom → base_link` transform.

## 5. Milestone B — Persistent XRSLAM map storage

The existing `PersistentRelocalizationMap` survives tracking resets **only in memory**. The project does not yet have complete disk-backed save/load for localization.

Implement versioned save/load including:

- Archived keyframe IDs, timestamps, optimized body/camera poses and intrinsics.
- Local landmark observations and ORB descriptors, including descriptor-to-observation association.
- EigenPlaces global descriptors and their place keys.
- VIO session IDs and coordinate-frame identities.
- Verified local-to-global alignments, when available.
- Semantic records and masks, class statistics and associated robot-body poses.
- Semantic taxonomy names/IDs, checkpoint identity or hash, and confidence thresholds.

Restore the place-retrieval database by rebuilding USearch from saved descriptors if necessary.

**Do not merge unrelated session-local frames.** Save the transform provenance or explicitly mark historical poses as unaligned.

Required tests:

1. Record a reference map and semantic keyframes.
2. Save the map and terminate the process.
3. Restart XRSLAM and reload the data.
4. Verify keyframe, descriptor and semantic metadata integrity.
5. Recover localization against the saved map, with geometric verification.
6. Confirm that the map-frame pose of historical keyframes is unchanged or corrected through explicit validated transforms.

A PLY export alone is insufficient for reloading a relocalization-capable visual map.

## 6. Milestone C — Semantic keyframe search

**The user's desired search result is the robot pose where an object class was observed, not the object's reconstructed 3D coordinates.**

Use the existing PIDNet semantic masks and keyframe archive.

For each keyframe, maintain an index of:

- Taxonomy and semantic class ID/name.
- Class presence, confidence and pixel coverage.
- Keyframe timestamp, frame ID and tracking session ID.
- Robot **body pose at observation time**.
- Associated image/mask references if available.
- Valid map-frame alignment status.

Provide a query interface such as:

**Request:** `class_name = "towel"`, `min_confidence = 0.70`, optional minimum pixel coverage and result limit.

**Response:** Matching keyframes, robot poses, timestamps, confidence summaries, session/map identities and whether each pose is suitable for map-frame navigation.

Nearby or repeated observations may optionally be grouped to reduce duplicates. Keep the original records accessible.

The first implementation requires only **semantic class indexing**, not instance segmentation, object tracking or 3D object localization.

Current Cityscapes-19 PIDNet cannot recognize `towel`. Support a future fine-tuned indoor model with configurable taxonomy and model/version metadata. Never reinterpret stored class ID 7 as a different label after changing checkpoints.

The semantic search ROS 2 interface must not depend on Nav2.

## 7. Milestone D — Map-relative ROS localization

Eventually publish verified `map → odom`, separate from the continuous `odom → base_link`.

Requirements:

- One stable persistent map identity.
- Validated alignment between the active VIO session and the saved map.
- Explicit validity signaling when global localization is lost.
- No unverified identity transform that makes an unknown map location appear valid.
- Correct TF timestamp handling and transform direction.

The application can take a semantic-search result in the verified `map` frame and send it to Nav2 using the standard `NavigateToPose` action. XRSLAM itself must not initiate that action.

## 8. Nav2 interface boundary

XRSLAM must have **no Nav2 build dependency**.

Nav2 will independently consume:

- `/odom`
- `odom → base_link`
- A verified `map → odom`
- A persistent 2D navigation occupancy grid from a separate provider
- A live, correctly calibrated metric `PointCloud2` for its Voxel Layer

**The semantic point-cloud PLY is not a Nav2 occupancy grid.** Because the custom 3D occupancy mapper was removed, a separate navigation-map generation/persistence solution is still required.

## 9. Regression and acceptance requirements

- Preserve the semantics-disabled EuRoC healthy/dropout behavior and local/global trajectory parity.
- Preserve 0123b 2 cm dense-map outputs and semantic class-coloring functionality.
- Never introduce inference into XRSLAM's estimator thread.
- Keep semantic worker teardown deterministic; accepted inference jobs must be drained where export requires it.
- Verify ROS 2 build with `colcon` on Jazzy.
- Verify TF static calibration with nonidentity transforms.
- Ensure there is exactly one `odom → base_link` publisher.
- Verify camera/IMU timestamp synchronization and odometry frame conventions.
- Verify no odometry discontinuity after loss/recovery.
- Validate map save/restart/reload and semantic search against the saved map.
- Keep motors disabled during initial physical-robot integration.

**Current task status:** The ROS 2 Jazzy adapter was planned but has **not yet been implemented or tested** in this handoff. Start by reviewing current `main`, creating the standalone adapter, and validating static TF/CameraInfo initialization before altering persistence or navigation behavior.