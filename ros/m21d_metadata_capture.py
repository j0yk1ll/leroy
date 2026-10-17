#!/usr/bin/env python3
"""M21D bounded metadata-only capture buffer; no ROS, serial, video or motors.

Data can be supplied ONLY by separately reviewed source adapters. Output is
M21C-compatible JSONL, never a physical camera/IMU offset calibration.
"""
import json
import os
import threading
from pathlib import Path

from m21_record_contract import FIELDS, EXPECTED, integer, summarize

MAX_EVENTS = 20000
MAX_OUTPUT_BYTES = 8 * 1024 * 1024


class CaptureError(ValueError):
    pass


class MetadataCapture:
    def __init__(self, evidence_type="SYNTHETIC", max_events=MAX_EVENTS):
        if evidence_type not in ("SYNTHETIC", "REAL_UNCALIBRATED"):
            raise CaptureError("unrecognized evidence type")
        integer(max_events, "max_events", 6, MAX_EVENTS)
        self.rows = [{"kind": "session", "schema": "leroy_m21c_v1",
                      "evidence_type": evidence_type}]
        self.max_events = max_events
        self.closed = False
        self._lock = threading.RLock()

    @staticmethod
    def stamp_ns(seconds, nanoseconds):
        sec = integer(seconds, "ROS sec", 0, (1 << 32) - 1)
        ns = integer(nanoseconds, "ROS nanosec", 0, 999999999)
        result = sec * 1000000000 + ns
        integer(result, "ROS header epoch ns", 1)
        return result

    def _append(self, record):
        with self._lock:
            if self.closed:
                raise CaptureError("capture already finished")
            if len(self.rows) - 1 >= self.max_events:
                raise CaptureError("bounded event capacity exhausted")
            kind = record.get("kind")
            if kind not in FIELDS or set(record) != FIELDS[kind]:
                raise CaptureError("invalid metadata keys")
            integer(record["receipt_mono_ns"], "receipt_mono_ns", 1)
            integer(record["header_ros_ns"], "header_ros_ns", 1)
            if (record["frame_id"], record["publisher"]) != EXPECTED[kind]:
                raise CaptureError("unexpected frame or graph publisher claim")
            if kind != "imu" and (record["width"], record["height"]) != (640, 480):
                raise CaptureError("unexpected camera dimensions")
            # Full temporal integrity and MCU sequence checks happen in finish().
            self.rows.append(record)

    def image(self, *, frame_index, stamp_ns, receipt_mono_ns, frame_id,
              width, height, publisher="/camera/rectify_node"):
        integer(frame_index, "frame_index", 0)
        self._append(dict(kind="camera_image", frame_index=frame_index,
                          header_ros_ns=stamp_ns, receipt_mono_ns=receipt_mono_ns,
                          frame_id=frame_id, width=width, height=height,
                          publisher=publisher))

    def camera_info(self, *, stamp_ns, receipt_mono_ns, frame_id, width, height,
                    publisher="/camera/usb_cam"):
        self._append(dict(kind="camera_info", header_ros_ns=stamp_ns,
                          receipt_mono_ns=receipt_mono_ns, frame_id=frame_id,
                          width=width, height=height, publisher=publisher))

    def imu_source(self, *, mcu_seq, mcu_t_us, stamp_ns, receipt_mono_ns,
                   gyro_xyz_rad_s, frame_id="imu_link",
                   publisher="/m18_imu_source"):
        integer(mcu_seq, "mcu_seq", 0, (1 << 32) - 1)
        integer(mcu_t_us, "mcu_t_us", 0, (1 << 32) - 1)
        if (not isinstance(gyro_xyz_rad_s, (tuple, list))
                or len(gyro_xyz_rad_s) != 3):
            raise CaptureError("gyro_xyz_rad_s must have three entries")
        self._append(dict(kind="imu", mcu_seq=mcu_seq, mcu_t_us=mcu_t_us,
                          header_ros_ns=stamp_ns,
                          receipt_mono_ns=receipt_mono_ns,
                          gyro_xyz_rad_s=list(gyro_xyz_rad_s),
                          frame_id=frame_id, publisher=publisher))

    def finish(self, path):
        """Verify all observations, then create a new bounded JSONL file.

        Never overwrite a prior capture; error leaves capture available for
        diagnosis, no hardware cleanup or serial state is managed here.
        """
        with self._lock:
            if self.closed:
                raise CaptureError("capture already finished")
            report = summarize(self.rows)
            payload = "".join(json.dumps(row, separators=(",", ":"), allow_nan=False)
                              + "\n" for row in self.rows).encode("utf-8")
            if len(payload) > MAX_OUTPUT_BYTES:
                raise CaptureError("output exceeds configured byte limit")
            target = Path(path)
            with target.open("xb") as fh:
                fh.write(payload)
                fh.flush()
                os.fsync(fh.fileno())
            self.closed = True
            report["record_file_bytes"] = len(payload)
            report["recorder_event_count"] = len(self.rows) - 1
            report["recorder_device_access"] = False
            return report
