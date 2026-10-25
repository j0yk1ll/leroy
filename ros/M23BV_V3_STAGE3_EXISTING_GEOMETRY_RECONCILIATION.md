# Stage 3: reconcile existing FreeCAD geometry with ROS sensor frames

**Scope: existing artifact inventory only.** The operator has already
specified body frames, IMU axes, and camera mounting information in earlier
Leroy development. Do not request new measurements until all original
records have been located and reconciled.

## Current sources of truth and limits

1. **Firmware sensor data axes:** The MCU's `imu_read_robot()` outputs
   robot-aligned acceleration and gyro values. The established package
   axis map is `forward=-ICM X`, `left=-ICM Y`,
   `up=+ICM Z`. PR #155 moves these signs into a reviewed
   compile-time hardware profile without changing the resulting telemetry;
   the host-only transform tests and full STM32F401 cross-build PASSED.
   **Do not apply a second IMU orientation correction in ROS TF.**
2. **ROS canonical TF:** `ros/src/leroy_description/urdf/leroy.urdf.xacro`
   defines `base_link` axes, the `imu_link` transform and camera
   optical-frame convention. Its 95 mm forward / 95 mm right IMU
   offset, 30-degree front camera pitch, and zero camera translation
   remain labeled **PROVISIONAL**. They are source parameters,
   not independently demonstrated physical mount center locations.
3. **Hardware CAD:** `hardware/parts.FCStd` (Git blob
   `136b5ab4a14d236f43551addf058ac4eb494870b`) is a
   checked-in FreeCAD model. It may contain existing part labels and
   placements for camera/IMU hardware. A placement in this model's
   coordinate system is **not** automatically a calibrated
   `base_link -> sensor` transform, nor does a printed mount origin
   equal the optical center or IMU sensing die origin.
4. **Historical ChArUco work:** `utils/imu/README.md` and
   `utils/imu/*` include earlier real calibration and diagnostic
   attempts. The adjacent-frame yaw estimates (about −42, −58,
   −66 ms) and orientation-based estimates (about −52, −90,
   −114 ms) disagree and are explicitly described as **not
   validated time offsets**. The ChArUco utility's sign convention
   is camera header minus IMU header for the *same physical event*;
   **XRSLAM's device time-offset sign must be independently verified
   before any value is transferred**.
5. **Active driver:** Operator-verified Stage 3 pinned image selects
   Leroy's previously patched `usb_cam` source/linked library.
   The distro underlay's tagged bug is not a demonstrated active
   regression. Camera exposure-event timing remains unmeasured.

## Narrow, no-device CAD metadata inspection

The new `ros/m23bv_v3_stage3_existing_geometry_cad_audit.py` reads
only the exact tracked `hardware/parts.FCStd` ZIP's
`Document.xml` and the canonical Xacro. It searches existing CAD
object labels/names for camera, IMU, chassis, mounts and frame-related
components and prints up to 40 candidate object records, including
any FreeCAD placement attributes explicitly present. It does not
load FreeCAD workbenches, export meshes, start ROS, open video or
serial ports, or write to hardware. No third-party Python packages
are needed.

From the `ros/` directory **after switching back to the Stage 3
branch** (PR #155 can remain draft on its separate branch):

```bash
git switch feature/m23bv-v3-stage3-fullstack-diagnostic
git pull --ff-only
python3 -B ./m23bv_v3_stage3_existing_geometry_cad_audit.py
```

Expected when the read-only inventory succeeds (not yet run by
the operator):

```text
STAGE3_EXISTING_GEOMETRY_FREECAD_OBJECT_COUNT=...
STAGE3_EXISTING_GEOMETRY_CANDIDATE_COUNT=...
STAGE3_EXISTING_GEOMETRY_CAD_PLACEMENTS=MODEL_SPACE_ONLY_UNVERIFIED
STAGE3_EXISTING_GEOMETRY_CAMERA_OPTICAL_CENTER=NOT_DERIVED
STAGE3_EXISTING_GEOMETRY_BASE_LINK_ORIGIN=NOT_DERIVED
STAGE3_EXISTING_GEOMETRY_IMU_DIE_CENTER=NOT_DERIVED
STAGE3_EXISTING_GEOMETRY=PASS_READ_ONLY_SOURCE_INVENTORY
STAGE3_EXISTING_GEOMETRY_HARDWARE_ACCESS=NONE
```

The candidate records will help determine whether the original
geometry is embedded in CAD and how it relates to the URDF. If
they are merely loose mount parts, the inventory is still a PASS
for read-only inspection but **geometry reconciliation remains
unresolved**; do not convert model-space placements to verified
sensor extrinsics without common datum and mount/origin evidence.
Do not replace existing measurement definitions with the Xacro
placeholders.

## After inspection: decision gate

- If a uniquely referenced, complete CAD assembly transform and
  sensor-center offset are available and tied to the already defined
  `base_link` datum, cite those artifacts for a **separate proposed**
  URDF update. No direct update or acceptance from label matches alone.
- If the CAD contains individual printable components only, locate
  the previously supplied assembled-robot geometry record instead.
  Do not ask the operator to remeasure the chassis by default.
- In either case, absolute camera–IMU phase is **still not
  calibrated** and observed native XRSLAM state was
  `initializing` under stationary diagnostic conditions.
  No navigation, pose/TF, recording or motor permissions follow
  from this static review.

**Safety:** No MCU serial opening, wheel/manipulator power,
camera startup, new recording, motion, flash or physical sensor
manipulation authorized. Retain the accepted pose/TF/Nav2 gates.
