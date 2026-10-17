#!/usr/bin/env python3
"""M21J offline 120s metadata-budget and post-STOP finalization gate.

No ROS, Podman, physical serial or camera access. A caller-supplied STOP
function in this module is used ONLY in mocked/synthetic tests. It is not a
hardware launcher or a substitute for motor-power isolation.
"""
import argparse
import json
import math
from pathlib import Path
import tempfile

from m21d_metadata_capture import MAX_EVENTS, MAX_OUTPUT_BYTES, MetadataCapture


class AdmissionError(ValueError):
    pass


def _positive_number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise AdmissionError(label + " must be a positive finite number")
    return value


def admission(duration_s=120, image_hz=30, camera_info_hz=30, imu_hz=50,
              reserve_pct=20, capacity=MAX_EVENTS):
    """Reserve full rates plus endpoint samples, then 20% headroom by default.

    The byte projection is a conservative planning estimate, NOT a proven
    maximum row length. M21D.finish enforces the actual 8 MiB output bound.
    """
    _positive_number(duration_s, "duration_s")
    if duration_s > 120:
        raise AdmissionError("capture duration exceeds reviewed 120s window")
    for name, value in (("image_hz", image_hz), ("camera_info_hz", camera_info_hz),
                        ("imu_hz", imu_hz)):
        _positive_number(value, name)
    if type(reserve_pct) not in (int, float) or not math.isfinite(reserve_pct) or not 10 <= reserve_pct <= 100:
        raise AdmissionError("reserve_pct must be between 10 and 100")
    if type(capacity) is not int or not 6 <= capacity <= MAX_EVENTS:
        raise AdmissionError("capacity must be between 6 and M21D max_events")
    counts = {
        "camera_image": math.ceil(duration_s * image_hz) + 1,
        "camera_info": math.ceil(duration_s * camera_info_hz) + 1,
        "imu": math.ceil(duration_s * imu_hz) + 1,
    }
    projected = sum(counts.values())
    required = math.ceil(projected * (1.0 + reserve_pct / 100.0))
    # 480 bytes/event is only an admission heuristic; actual JSON is checked
    # by the writer and may still exceed its hard file-size limit.
    projected_bytes = required * 480 + 512
    ready = required <= capacity and projected_bytes <= MAX_OUTPUT_BYTES
    return {
        "milestone": "M21J",
        "evidence_type": "PLANNING_ONLY",
        "duration_s": duration_s,
        "projected_per_stream": counts,
        "projected_event_count": projected,
        "reserve_pct": reserve_pct,
        "required_event_capacity": required,
        "configured_event_capacity": capacity,
        "remaining_reserved_events": capacity - required,
        "projected_output_bytes_upper_heuristic": projected_bytes,
        "hard_output_bytes_limit": MAX_OUTPUT_BYTES,
        "capacity_ready": ready,
        "physical_camera_imu_offset_validated": False,
        "verdict": ("SYNTHETIC_CAPACITY_ADMISSION_READY" if ready
                    else "CAPACITY_ADMISSION_REJECTED"),
    }


def finalize_after_stop(capture, output_path, stop_request):
    """Try STOP *before* file serialization, even for unverified STOP.

    The return value is a separate lifecycle verdict. Persisting a valid
    metadata file does NOT imply safe MCU state when STOP ACK is missing.
    This function does not open MCU serial or know a motor's physical state.
    """
    if not isinstance(capture, MetadataCapture):
        raise TypeError("M21J expects MetadataCapture")
    if not callable(stop_request):
        raise TypeError("M21J requires an injected callable stop_request")
    stop_acked = False
    stop_error = None
    try:
        ack = stop_request()
        if type(ack) is not bool:
            raise ValueError("STOP acknowledgement must be explicitly boolean")
        stop_acked = ack
    except Exception as exc:
        stop_error = type(exc).__name__ + ": " + str(exc)

    # STOP has already been attempted. Save forensic metadata if possible,
    # even if MCU state is unverified, without accepting the lifecycle.
    saved = False
    capture_error = None
    capture_report = None
    try:
        capture_report = capture.finish(output_path)
        saved = True
    except Exception as exc:
        capture_error = type(exc).__name__ + ": " + str(exc)
    accepted = bool(stop_acked and saved and
                    capture_report["verdict"] == "M21C_OFFLINE_METADATA_FORMAT_VALID")
    return {
        "milestone": "M21J",
        "evidence_type": capture.rows[0]["evidence_type"],
        "stop_attempted": True,
        "stop_acknowledged": stop_acked,
        "stop_error": stop_error,
        "metadata_saved": saved,
        "capture_error": capture_error,
        "saved_event_count": (capture_report["recorder_event_count"]
                              if saved else None),
        "physical_camera_imu_offset_validated": False,
        "verdict": ("SIMULATED_POST_STOP_FINALIZATION_READY" if accepted
                    else "UNVERIFIED_STOP_OR_METADATA_CAPTURE"),
    }


def demonstration():
    plan = admission()
    if not plan["capacity_ready"]:
        raise RuntimeError("default capture capacity not admitted")
    print("M21J_CAPACITY_REPORT_JSON=" + json.dumps(plan, sort_keys=True))
    with tempfile.TemporaryDirectory(prefix="m21j-offline-") as tmp:
        capture = MetadataCapture("SYNTHETIC", max_events=20_000)
        for i in range(3):
            t = 1_800_000_000_000_000_000 + i * 100_000_000
            steady = 5_000_000_000 + i * 100_000_000
            capture.image(frame_index=i, stamp_ns=t, receipt_mono_ns=steady,
                          frame_id="camera_optical_frame", width=640, height=480)
            capture.camera_info(stamp_ns=t + 1000,
                                receipt_mono_ns=steady + 1000,
                                frame_id="camera_optical_frame",
                                width=640, height=480)
            capture.imu_source(mcu_seq=i, mcu_t_us=i * 100_000,
                               stamp_ns=t + 2000, receipt_mono_ns=steady + 2000,
                               gyro_xyz_rad_s=[0.1, 0.2, 0.3])
        order = []
        def simulated_stop():
            order.append("STOP")
            return True
        result = finalize_after_stop(capture, Path(tmp) / "test.jsonl",
                                     simulated_stop)
        order.append("FINALIZED")
        if not (result["metadata_saved"] and result["stop_acknowledged"]
                and order == ["STOP", "FINALIZED"]
                and result["evidence_type"] == "SYNTHETIC"):
            raise RuntimeError("M21J synthetic STOP-before-persistence failed")
        print("M21J_FINALIZATION_REPORT_JSON=" + json.dumps(result, sort_keys=True))
    print("M21J_VERDICT=OFFLINE_CAPACITY_AND_STOP_ORDER_READY")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", required=True,
                        help="offline-only synthetic capacity and STOP-order demo")
    parser.parse_args(argv)
    return demonstration()


if __name__ == "__main__":
    raise SystemExit(main())
