#!/usr/bin/env python3
"""M21K offline-only recorder lifecycle: STOP, quiesce writers, then save.

All callbacks and threads are supplied by the caller; this module never
creates a ROS node, opens serial/video, starts Podman or commands hardware.
Tests inject fake callbacks. NOT a live capture runner or safety approval.
"""
import json
import math
from pathlib import Path
import tempfile
import threading
import time

from m21d_metadata_capture import MetadataCapture


class QuiescenceError(ValueError):
    pass


def finish_after_quiescence(capture, output_path, stop_request,
                            request_producer_shutdown, producer_threads,
                            join_timeout_s=1.0):
    """Attempt STOP first, stop/join ALL registered producers, then save.

    A missing STOP ACK can still produce a diagnostic file; never a positive
    lifecycle verdict. If any producer is still alive or never started, do
    NOT finalize the recorder, preventing late writes against a closed file.
    A live implementation must separately prove every callback executor and
    IMU source writer is registered, drained, and prevented from restarting.
    """
    if not isinstance(capture, MetadataCapture) or capture.closed:
        raise QuiescenceError("requires an open MetadataCapture")
    if not callable(stop_request) or not callable(request_producer_shutdown):
        raise QuiescenceError("injected STOP and shutdown callbacks required")
    if not isinstance(producer_threads, (tuple, list)) or len(producer_threads) > 8:
        raise QuiescenceError("expected at most eight registered producer threads")
    threads = tuple(producer_threads)
    if len({id(t) for t in threads}) != len(threads):
        raise QuiescenceError("duplicate producer thread")
    if any(not isinstance(t, threading.Thread) or t is threading.current_thread()
           for t in threads):
        raise QuiescenceError("unjoinable producer thread")
    if type(join_timeout_s) not in (int, float) or not math.isfinite(join_timeout_s) or not 0 < join_timeout_s <= 5:
        raise QuiescenceError("bounded join timeout must be in (0,5] seconds")

    stop_acked = False
    stop_error = None
    try:
        ack = stop_request()  # The first lifecycle action, NEVER file I/O.
        if type(ack) is not bool:
            raise ValueError("STOP acknowledgement must be bool")
        stop_acked = ack
    except Exception as exc:
        stop_error = type(exc).__name__ + ": " + str(exc)

    quiesce_error = None
    try:
        request_producer_shutdown()
    except Exception as exc:
        quiesce_error = type(exc).__name__ + ": " + str(exc)
    deadline = time.monotonic() + join_timeout_s
    unstarted = []
    for worker in threads:
        if worker.ident is None:
            unstarted.append(worker.name)
            continue
        try:
            worker.join(max(0.0, deadline - time.monotonic()))
        except RuntimeError:
            unstarted.append(worker.name)
    still_alive = [worker.name for worker in threads if worker.is_alive()]
    quiescent = not unstarted and not still_alive

    saved = False
    save_error = None
    event_count = None
    bytes_written = None
    # The metadata writer owns its own lock. Thread join must come first.
    if quiescent:
        try:
            record_report = capture.finish(Path(output_path))
            saved = True
            event_count = record_report["recorder_event_count"]
            bytes_written = record_report["record_file_bytes"]
        except Exception as exc:
            save_error = type(exc).__name__ + ": " + str(exc)
    accepted = stop_acked and quiescent and quiesce_error is None and saved
    return {
        "milestone": "M21K",
        "evidence_type": capture.rows[0]["evidence_type"],
        "stop_attempted": True,
        "stop_acknowledged": stop_acked,
        "stop_error": stop_error,
        "quiescence_requested": True,
        "quiescence_error": quiesce_error,
        "producers_quiescent": quiescent,
        "unstarted_producers": unstarted,
        "active_producers": still_alive,
        "metadata_saved": saved,
        "save_error": save_error,
        "saved_event_count": event_count,
        "saved_bytes": bytes_written,
        "physical_camera_imu_offset_validated": False,
        "physical_stop_acknowledgement_validated": False,
        "verdict": ("SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT" if accepted
                    else "STOP_QUIESCENCE_OR_SAVE_UNVERIFIED"),
    }


def demo():
    capture = MetadataCapture("SYNTHETIC", max_events=100)
    for index in range(3):
        t = 1_800_000_000_000_000_000 + index * 100_000_000
        receipt = 5_000_000_000 + index * 100_000_000
        capture.image(frame_index=index, stamp_ns=t,
                      receipt_mono_ns=receipt,
                      frame_id="camera_optical_frame", width=640, height=480)
        capture.camera_info(stamp_ns=t + 1000, receipt_mono_ns=receipt + 1000,
                            frame_id="camera_optical_frame", width=640, height=480)
        capture.imu_source(mcu_seq=index, mcu_t_us=index * 100_000,
                           stamp_ns=t + 2000, receipt_mono_ns=receipt + 2000,
                           gyro_xyz_rad_s=[0.1, 0.2, 0.3])
    order = []
    with tempfile.TemporaryDirectory(prefix="m21k-synthetic-") as directory:
        result = finish_after_quiescence(
            capture, Path(directory) / "synthetic.jsonl",
            lambda: (order.append("stop"), True)[1],
            lambda: order.append("quiesce"),
            [])
        if (result["verdict"] != "SYNTHETIC_STOP_QUIESCE_SAVE_CONTRACT"
                or result["saved_event_count"] != 9
                or order != ["stop", "quiesce"]):
            raise RuntimeError("offline lifecycle contract failed")
        print("M21K_LIFECYCLE_REPORT_JSON=" + json.dumps(result, sort_keys=True))
    print("M21K_VERDICT=OFFLINE_QUIESCENCE_CONTRACT_READY")


if __name__ == "__main__":
    demo()
