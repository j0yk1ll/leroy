#!/usr/bin/env python3
"""M17 non-actuating MCU IMU query audit: NO ROS, NO motor commands.

Exactly one outbound firmware command is permitted: get_imu_robot.
This request reads an IMU snapshot. Unlike streamed MCU telemetry, the
response has NO MCU acquisition timestamp or sequence number and cannot
be treated as qualified XRSLAM ROS IMU input.
"""
import argparse
import json
import math
import re
import sys
import time

import serial


QUERY = b"get_imu_robot\r\n"
PORT = "/dev/ttyACM0"
POLL_PERIOD_S = 0.5
POLL_RESPONSE_TIMEOUT_S = 0.35
MIN_SECONDS = 5
MAX_SECONDS = 30
FIELDS = (
    "accel_forward_mps2", "accel_left_mps2", "accel_up_mps2",
    "gyro_roll_radps", "gyro_pitch_radps", "gyro_yaw_radps",
)
# Firmware print_fixed_milli() emits signed decimal numbers with three
# fractional digits; use the exact protocol format rather than accepting
# arbitrary strings (including NaN, infinity or unexpected extra fields).
MILLI = re.compile(r"^-?[0-9]+\.[0-9]{3}$")


def parse_imu_response(raw):
    """Return (calibrated, six finite SI values) or a specific rejection."""
    text = raw.strip()
    if text in ("imu_robot=unavailable", "imu_robot=read_error"):
        raise ValueError(text)
    parts = text.split(" ")
    if not parts or parts[0] != "imu_robot=ok":
        raise ValueError("unexpected_response")
    items = {}
    for part in parts[1:]:
        if "=" not in part:
            raise ValueError("malformed_field")
        key, value = part.split("=", 1)
        if key in items:
            raise ValueError("duplicate_field")
        items[key] = value
    if set(items) != set(FIELDS) | {"calibrated"}:
        raise ValueError("unexpected_or_missing_field")
    if items["calibrated"] not in ("0", "1"):
        raise ValueError("invalid_calibration_flag")
    if not all(MILLI.fullmatch(items[k]) for k in FIELDS):
        raise ValueError("invalid_fixed_milli_value")
    values = tuple(float(items[k]) for k in FIELDS)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite_measurement")
    return items["calibrated"] == "1", values


def self_test():
    good = ("imu_robot=ok calibrated=1 "
            "accel_forward_mps2=0.010 accel_left_mps2=-0.020 "
            "accel_up_mps2=9.807 gyro_roll_radps=0.001 "
            "gyro_pitch_radps=-0.002 gyro_yaw_radps=0.003")
    calibrated, sample = parse_imu_response(good)
    assert calibrated and sample == (0.01, -0.02, 9.807, 0.001, -0.002, 0.003)
    for invalid in (
        good.replace("calibrated=1", "calibrated=2"),
        good.replace("9.807", "nan"),
        good.replace("0.003", "0.003 extra=1"),
        good.replace("gyro_yaw_radps=0.003", ""),
        good.replace("gyro_yaw_radps=0.003", "gyro_yaw_radps=0.003 gyro_yaw_radps=0.003"),
        "imu_robot=unavailable",
        "imu_robot=read_error",
        "ok set_wheels",
    ):
        try:
            parse_imu_response(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("malformed MCU response accepted")
    assert QUERY == b"get_imu_robot\r\n"
    print("M17_IMU_QUERY_SELFTEST=PASS", flush=True)


def observe(port, seconds):
    # The only serial write below sends QUERY. This is still a USB CDC
    # operation against motor-capable firmware; motor power must be physically
    # inhibited. pyserial opening the port may change DTR/reset the MCU.
    counters = {
        "queries_sent": 0, "valid_samples": 0, "calibrated_samples": 0,
        "unavailable": 0, "read_error": 0, "invalid_response": 0,
        "timeout": 0, "other_lines": 0, "nonascii_lines": 0,
    }
    acceleration_norms = []
    gyro_norms = []
    delays = []
    first_valid_mono = None
    last_valid_mono = None

    with serial.Serial(
        port=port, baudrate=115200, timeout=0.10,
        write_timeout=0.25, exclusive=True,
    ) as device:
        # Give USB CDC time to settle; serial port open may reset the MCU.
        time.sleep(0.3)
        device.reset_input_buffer()
        start = time.monotonic()
        next_query = start
        while time.monotonic() - start < seconds:
            pause = next_query - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            if time.monotonic() - start >= seconds:
                break
            sent_at = time.monotonic()
            device.write(QUERY)
            device.flush()
            counters["queries_sent"] += 1
            deadline = sent_at + POLL_RESPONSE_TIMEOUT_S
            got_answer = False
            while time.monotonic() < deadline:
                data = device.readline()
                if not data:
                    continue
                if len(data) > 256:
                    counters["invalid_response"] += 1
                    continue
                try:
                    line = data.decode("ascii").strip()
                except UnicodeDecodeError:
                    counters["nonascii_lines"] += 1
                    continue
                if not line.startswith("imu_robot="):
                    counters["other_lines"] += 1
                    continue
                got_answer = True
                try:
                    calibrated, values = parse_imu_response(line)
                except ValueError as exc:
                    key = str(exc)
                    if key == "imu_robot=unavailable":
                        counters["unavailable"] += 1
                    elif key == "imu_robot=read_error":
                        counters["read_error"] += 1
                    else:
                        counters["invalid_response"] += 1
                else:
                    counters["valid_samples"] += 1
                    counters["calibrated_samples"] += int(calibrated)
                    acceleration_norms.append(round(math.sqrt(
                        sum(x*x for x in values[:3])), 4))
                    gyro_norms.append(round(math.sqrt(
                        sum(x*x for x in values[3:])), 4))
                    received_at = time.monotonic()
                    if first_valid_mono is None:
                        first_valid_mono = received_at
                    last_valid_mono = received_at
                    delays.append(received_at - sent_at)
                break
            if not got_answer:
                counters["timeout"] += 1
            next_query += POLL_PERIOD_S

    valid = counters["valid_samples"]
    verdict = (
        "OBSERVED_UNSTAMPED_IMU_QUERY_SAMPLES"
        if valid >= 5 and counters["invalid_response"] == 0
        else "INCOMPLETE_OR_UNVERIFIED_IMU_QUERY"
    )
    report = {
        "milestone": "M17",
        "query_only": True,
        "motor_commands_sent": 0,
        "serial_command_allowlist": ["get_imu_robot"],
        "no_ros_publishers": True,
        "mcu_acquisition_timestamps_observed": False,
        "ros_imu_contract_validated": False,
        "xrslam_ready": False,
        "duration_requested_s": seconds,
        "poll_period_s": POLL_PERIOD_S,
        "counters": counters,
        "mean_valid_response_rate_hz": round(
            (valid - 1) / (last_valid_mono - first_valid_mono), 3)
            if valid > 1 and last_valid_mono > first_valid_mono else None,
        "acceleration_magnitude_mps2_min_max": [
            min(acceleration_norms), max(acceleration_norms)
        ] if acceleration_norms else None,
        "gyro_magnitude_radps_min_max": [
            min(gyro_norms), max(gyro_norms)
        ] if gyro_norms else None,
        "max_query_roundtrip_ms": round(max(delays)*1000, 2) if delays else None,
        "all_valid_samples_gyro_calibrated": (
            counters["calibrated_samples"] == valid and valid > 0),
        "verdict": verdict,
    }
    print("M17_IMU_QUERY_REPORT_JSON=" + json.dumps(report, sort_keys=True),
          flush=True)
    print("M17_IMU_QUERY_VERDICT=" + verdict, flush=True)
    return verdict == "OBSERVED_UNSTAMPED_IMU_QUERY_SAMPLES"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--duration-sec", type=int, default=15)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not MIN_SECONDS <= args.duration_sec <= MAX_SECONDS:
        parser.error("duration must be 5..30 seconds")
    if not PORT == "/dev/ttyACM0":
        raise RuntimeError("serial device guard violated")
    try:
        return 0 if observe(PORT, args.duration_sec) else 2
    except (serial.SerialException, OSError) as exc:
        print("M17_IMU_QUERY_ERROR=" + type(exc).__name__ + ": " + str(exc),
              file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
