#!/usr/bin/env bash
# Install a modern OpenCV wheel into a private directory for Leroy calibration.
# The container's ROS/DepthART Python and system OpenCV are NOT modified.
set -euo pipefail

TARGET="/tmp/leroy-calibration-opencv"
PACKAGE="opencv-contrib-python-headless==4.10.0.84"

python3 - <<'PY'
import numpy
major = int(numpy.__version__.split(".")[0])
print("System NumPy:", numpy.__version__)
if major != 1:
    raise SystemExit(
        "Unexpected NumPy major version; verify wheel/ROS compatibility "
        "before installing calibration OpenCV."
    )
PY

python3 -m pip install --break-system-packages --no-deps --only-binary=:all: \
  --target "$TARGET" "$PACKAGE"

PYTHONPATH="$TARGET${PYTHONPATH:+:$PYTHONPATH}" python3 - <<'PY'
import cv2
import numpy
from pathlib import Path
print("Calibration OpenCV:", cv2.__version__, "at", cv2.__file__)
print("System NumPy:", numpy.__version__)
expected = Path("/tmp/leroy-calibration-opencv").resolve()
actual = Path(cv2.__file__).resolve()
if expected not in actual.parents:
    raise SystemExit("OpenCV was not imported from the isolated overlay!")
if not cv2.__version__.startswith("4.10.0"):
    raise SystemExit("Unexpected calibration OpenCV version")
if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
    raise SystemExit("OpenCV ArUco/ChArUco modules are unavailable")
print("Calibration OpenCV overlay ready.")
PY

cat <<'NOTE'
For calibration commands only, after sourcing ROS 2, run:
    export PYTHONPATH="/tmp/leroy-calibration-opencv:${PYTHONPATH:-}"
Do not set this in the robot's main launch environment.
NOTE
