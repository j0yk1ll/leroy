#!/usr/bin/env python3
"""M21I offline synthetic full-window metadata workload; NO ROS or hardware.

Message counts/timing are deliberately modeled on M20's summary, but every
event is GENERATED, not replayed physical sensor measurements. This program
cannot estimate physical camera exposure-to-MCU acquisition phase.
"""
import argparse
import heapq
import json
import math
from pathlib import Path
import sys

from m21d_metadata_capture import MetadataCapture

CAM_IMAGES = 2082
CAM_INFO = 3578
IMU_SAMPLES = 6001
DURATION_NS = 120_000_000_000
CAM_START_NS = 50_000_000
CAM_OVERLAP_NS = 119_886_000_000
IMAGE_GAP_NS = 1_351_000_000
IMAGE_GAP_AFTER = 850  # One deliberately injected, missing-frame interval.
MCU_START_TICK = (1 << 32) - 70_000
BASE_RECEIPT_MONO_NS = 12_000_000_000_000
BASE_ROS_NS = 1_800_000_000_000_000_000
TOTAL = CAM_IMAGES + CAM_INFO + IMU_SAMPLES


def camera_times():
    # Exactly 2082 images, 119.886-second span, and a 1.351-second gap.
    # The other 2080 camera intervals split the remaining span evenly.
    normal_span = CAM_OVERLAP_NS - IMAGE_GAP_NS
    for i in range(CAM_IMAGES):
        normal_index = i if i <= IMAGE_GAP_AFTER else i - 1
        offset = IMAGE_GAP_NS if i > IMAGE_GAP_AFTER else 0
        yield CAM_START_NS + (normal_span * normal_index // (CAM_IMAGES - 2)) + offset


def events():
    image = ((t, 0, "camera_image", i) for i, t in enumerate(camera_times()))
    info = ((DURATION_NS * i // (CAM_INFO - 1), 1, "camera_info", i)
            for i in range(CAM_INFO))
    imu = ((20_000_000 * i, 2, "imu", i) for i in range(IMU_SAMPLES))
    yield from heapq.merge(image, info, imu)


def fill(capture):
    """Append synthesized, globally chronological events using source API."""
    for t, _order, kind, index in events():
        receipt = BASE_RECEIPT_MONO_NS + t
        if kind == "camera_image":
            capture.image(
                frame_index=index,
                stamp_ns=BASE_ROS_NS + t - 48_000_000,
                receipt_mono_ns=receipt, frame_id="camera_optical_frame",
                width=640, height=480)
        elif kind == "camera_info":
            capture.camera_info(
                stamp_ns=BASE_ROS_NS + t - 34_000_000,
                receipt_mono_ns=receipt, frame_id="camera_optical_frame",
                width=640, height=480)
        else:
            capture.imu_source(
                mcu_seq=index,
                mcu_t_us=(MCU_START_TICK + index * 20_000) & 0xffffffff,
                stamp_ns=BASE_ROS_NS + t - 2_000_000,
                receipt_mono_ns=receipt,
                gyro_xyz_rad_s=[
                    round(0.12 * math.sin(index * 0.016), 6),
                    round(0.08 * math.cos(index * 0.011), 6),
                    round(0.03 * math.sin(index * 0.028), 6)])


def assess(report):
    kinds = report["stream_diagnostics"]
    if (report["evidence_type"] != "SYNTHETIC"
            or report["verdict"] != "M21C_OFFLINE_METADATA_FORMAT_VALID"
            or report["recorder_event_count"] != TOTAL
            or kinds["camera_image"]["messages"] != CAM_IMAGES
            or kinds["camera_info"]["messages"] != CAM_INFO
            or kinds["imu"]["messages"] != IMU_SAMPLES
            or kinds["imu"]["sequence_missing_samples"] != 0
            or abs(report["receipt_overlap_s"] - 119.886) > 0.00001
            or abs(kinds["camera_image"]["max_receive_gap_s"] - 1.351) > 0.00001
            or not report["large_camera_gap_over_250ms"]
            or report["record_file_bytes"] > 8 * 1024 * 1024
            or report["physical_camera_imu_offset_validated"] is not False
            or report["physical_camera_imu_offset_s"] is not None
            or report["recorder_device_access"] is not False):
        raise AssertionError("M21I synthetic workload contract failed")
    return report


def run(path):
    # Only existing metadata APIs; no source adapter, ROS, Podman or serial.
    capture = MetadataCapture("SYNTHETIC", max_events=12000)
    fill(capture)
    report = assess(capture.finish(path))
    print("M21I_FULL_WINDOW_REPORT_JSON=" + json.dumps(report, sort_keys=True))
    print("M21I_FULL_WINDOW_VERDICT=SYNTHETIC_M20_SCALE_METADATA_CONTRACT")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="new JSONL file; never overwrite")
    args = parser.parse_args(argv)
    try:
        return run(Path(args.output))
    except (OSError, ValueError, AssertionError) as exc:
        print("M21I_FULL_WINDOW_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
