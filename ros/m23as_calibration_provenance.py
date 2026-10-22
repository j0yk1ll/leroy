#!/usr/bin/env python3
"""M23AS: offline calibration provenance comparison, WITHOUT video/ROS access.

Compares numeric K/D/R/P of the deployed camera YAML against the original
ChArUco calibration YAML, validates 92 referenced PNG headers are 640x480.
No camera identity, changed-focus, crop, live mode or physical accuracy proof.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import sys

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / "utils/charuco/calibration.yaml"
DEPLOYED = ROOT / "ros/config/camera/front_camera.yaml"
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
NUM = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?")


class Blocked(ValueError):
    pass


def dimension(yaml_content, name):
    match = re.search(r"(?m)^" + re.escape(name) + r":\s*(\d+)\s*$",
                      yaml_content)
    if match is None:
        raise Blocked("missing " + name)
    return int(match.group(1))


def matrix(content, name):
    # Select only the indented YAML block following a top-level matrix key.
    match = re.search(r"(?m)^" + re.escape(name) +
                      r":\s*\n((?:[ \t]+[^\n]*(?:\n|$))*)", content)
    if match is None:
        raise Blocked("matrix not found: " + name)
    body = match.group(1)
    rows = re.search(r"(?m)^\s+rows:\s*(\d+)\s*$", body)
    cols = re.search(r"(?m)^\s+cols:\s*(\d+)\s*$", body)
    match_data = re.search(r"(?m)^\s+data:\s*(.*)$", body)
    if rows is None or cols is None or match_data is None:
        raise Blocked("matrix rows/columns/data invalid: " + name)
    start = match_data.start(1)
    data_text = body[start:]
    values = [float(s) for s in NUM.findall(data_text)]
    shape = (int(rows.group(1)), int(cols.group(1)))
    if len(values) != shape[0] * shape[1]:
        raise Blocked("matrix element count mismatch: " + name)
    return shape, values


def png_dimensions(path):
    if path.is_symlink() or not path.is_file():
        raise Blocked("missing/non-regular calibration image: " + path.name)
    with path.open("rb") as fp:
        header = fp.read(24)
    if len(header) != 24 or header[:8] != PNG_MAGIC or header[12:16] != b"IHDR":
        raise Blocked("invalid calibration image PNG IHDR: " + path.name)
    width, height = struct.unpack(">II", header[16:24])
    if not 0 < width <= 16384 or not 0 < height <= 16384:
        raise Blocked("invalid calibration image resolution")
    return width, height


def validate(source, deployed, image_dims, image_names):
    expected = (640, 480)
    if (dimension(source, "image_width"),
        dimension(source, "image_height")) != expected:
        raise Blocked("original ChArUco calibration is not 640x480")
    if (dimension(deployed, "image_width"),
        dimension(deployed, "image_height")) != expected:
        raise Blocked("deployed calibration is not 640x480")
    for name in ("camera_matrix", "distortion_coefficients",
                 "rectification_matrix", "projection_matrix"):
        a_shape, a = matrix(source, name)
        b_shape, b = matrix(deployed, name)
        if a_shape != b_shape or any(abs(x-y) > 0.000005 for x,y in zip(a,b)):
            raise Blocked("deployed calibration differs from ChArUco: " + name)
    for text in ("plumb_bob",):
        for content in (source, deployed):
            if not re.search(r"(?m)^distortion_model:\s*" +
                             re.escape(text) + r"\s*$", content):
                raise Blocked("distortion model differs")
    views = dimension(source, "views_used")
    rejected = dimension(source, "views_rejected")
    if (views, rejected) != (92, 0):
        raise Blocked("unexpected calibration view count")
    rms = re.search(r"(?m)^rms_reprojection_error_px:\s*(" + NUM.pattern + r")\s*$",
                    source)
    if rms is None or not (0 < float(rms.group(1)) < 0.5):
        raise Blocked("RMS accuracy metadata missing/invalid")
    if len(image_names) != views or len(set(image_names)) != views:
        raise Blocked("calibration image reference count mismatch")
    if any(dim != expected for dim in image_dims):
        raise Blocked("one or more source calibration images not 640x480")
    return {
        "verdict": "STORED_CALIBRATION_NUMERICAL_PROVENANCE_MATCH",
        "deployed_matches_source_matrices": True,
        "source_charuco_image_count": views,
        "source_rejected_images": rejected,
        "source_reprojection_rms_px": round(float(rms.group(1)), 4),
        "stored_png_dimensions_all_640x480": True,
        "physical_camera_which_acquired_images_proven": False,
        "current_camera_mode_verified": False,
        "lens_focus_or_crop_unchanged_proven": False,
        "physical_camera_authorized": False,
    }


def run():
    if SOURCE.is_symlink() or DEPLOYED.is_symlink():
        raise Blocked("calibration YAML symlink forbidden")
    source = SOURCE.read_text()
    deployed = DEPLOYED.read_text()
    # The source YAML is generated and its used-images key is a list of paths
    # relative to utils/charuco/. Do not read arbitrary external paths.
    header = source.split("\nused_images:\n")
    if len(header) != 2:
        raise Blocked("missing or duplicate used_images provenance list")
    image_lines = header[1].split("\nper_view_reprojection_error_px:", 1)[0]
    image_names = []
    for line in image_lines.splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"- calibration_images/(calib_[0-9]{3}[.]png)", line)
        if match is None:
            raise Blocked("unexpected used_images path")
        image_names.append(match.group(1))
    actual_dims = [
        png_dimensions(ROOT / "utils/charuco/calibration_images" / filename)
        for filename in image_names
    ]
    result = validate(source, deployed, actual_dims, image_names)
    print("M23AS_RESULT_JSON=" + json.dumps(result, sort_keys=True), flush=True)
    print("M23AS_VERDICT=" + result["verdict"], flush=True)
    print("M23AS_PHYSICAL_CAMERA_AUTHORIZED=FALSE", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    if not args.check:
        print("M23AS_STATUS=NO_FILE_OR_DEVICE_READ_BY_DEFAULT", flush=True)
        return 0
    try:
        return run()
    except (Blocked, OSError, ValueError, UnicodeError) as exc:
        print("M23AS_BLOCKED=" + type(exc).__name__ + ":" + str(exc)[:160])
        print("M23AS_PHYSICAL_CAMERA_AUTHORIZED=FALSE")
        return 2


if __name__ == "__main__":
    sys.exit(main())
