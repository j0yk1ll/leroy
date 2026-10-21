#!/usr/bin/env python3
"""M23AN: one stock composable TBB-free device-free timing observation.

Inert by default. --run inherits M23AK 900/900 graph/QoS/stamp/drain
and actual child shutdown gates. Observer monotonic receipt timestamps are
measured only AFTER M23AK success. They are NOT camera capture timestamps
or direct OpenCV rectify CPU times.
"""
import argparse
import contextlib
import io
import json
import os
from pathlib import Path
import re
import statistics
import sys

SAMPLE = re.compile(r"^M23K_SAMPLE (raw|rect|info) ([0-9]+) ([0-9]+)$")
KINDS = ("raw", "rect", "info")
COUNT = 900
PAYLOAD_BYTES = 640 * 480 * 3
MAX_RECEIPT_GAP_NS = 250_000_000


class Blocked(ValueError):
    pass


def quantile(values, numerator, denominator=100):
    if not values:
        raise Blocked("empty observed sample set")
    sorted_values = sorted(values)
    rank = (len(sorted_values) - 1) * numerator / denominator
    lo, hi = int(rank), min(len(sorted_values)-1, int(rank)+1)
    return sorted_values[lo] + (rank - lo) * (sorted_values[hi] - sorted_values[lo])


def analyze(observer, publisher):
    # Original M23K observer prints monotonic steady-clock receipt ns for
    # each subscribed callback plus original synthetic header timestamp.
    stamps = {kind: {} for kind in KINDS}
    for line in observer.splitlines():
        if not line.startswith("M23K_SAMPLE"):
            continue
        match = SAMPLE.fullmatch(line)
        if match is None:
            raise Blocked("malformed observer receipt record")
        kind, stamp_str, receive_str = match.groups()
        stamp, receive = int(stamp_str), int(receive_str)
        if stamp <= 0 or receive <= 0 or stamp in stamps[kind]:
            raise Blocked("invalid/duplicate observer header or receipt")
        stamps[kind][stamp] = receive
    if any(len(stamps[kind]) != COUNT for kind in KINDS):
        raise Blocked("not exactly 900 observed messages per stream")
    keys = set(stamps["raw"])
    if any(set(stamps[kind]) != keys for kind in KINDS):
        raise Blocked("image/rect/info header stamps differ")

    payload_lines = [
        x.split("=", 1)[1] for x in publisher.splitlines()
        if x.startswith("M23P_PUBLISHER_PAYLOAD_BYTES=")]
    if len(payload_lines) != 1 or payload_lines[0] != str(PAYLOAD_BYTES):
        raise Blocked("not original 640x480 RGB8 synthetic payload")
    rect_receipts = sorted(stamps["rect"].values())
    gaps = [b-a for a,b in zip(rect_receipts, rect_receipts[1:])]
    if any(g <= 0 for g in gaps):
        raise Blocked("non-monotonic rectified receipts")
    elapsed_ns = rect_receipts[-1] - rect_receipts[0]
    if elapsed_ns <= 0:
        raise Blocked("zero synthetic observed duration")

    skew = [stamps["rect"][stamp] - stamps["raw"][stamp]
            for stamp in sorted(keys)]
    abs_skew = [abs(v) for v in skew]
    return {
        "dimensions": {"width": 640, "height": 480, "encoding": "rgb8",
                       "payload_bytes": PAYLOAD_BYTES},
        "counts": {k: len(stamps[k]) for k in KINDS},
        "paired_header_stamps": len(keys),
        "observed_rectified_receipt_fps": round((COUNT-1)*1e9/elapsed_ns, 3),
        "observed_rectified_receive_elapsed_s": round(elapsed_ns/1e9, 3),
        "rect_receipt_gap_p50_ms": round(quantile(gaps, 50)/1e6, 3),
        "rect_receipt_gap_p99_ms": round(quantile(gaps, 99)/1e6, 3),
        "rect_receipt_gap_max_ms": round(max(gaps)/1e6, 3),
        "rect_receipt_gaps_over_250ms": sum(g > MAX_RECEIPT_GAP_NS for g in gaps),
        "raw_to_rect_receipt_skew_p50_ms": round(quantile(skew, 50)/1e6, 3),
        "raw_to_rect_receipt_skew_p95_ms": round(quantile(skew, 95)/1e6, 3),
        "raw_to_rect_receipt_skew_p99_ms": round(quantile(skew, 99)/1e6, 3),
        "raw_to_rect_abs_skew_p99_ms": round(quantile(abs_skew, 99)/1e6, 3),
        "raw_to_rect_negative_skew_count": sum(v < 0 for v in skew),
        "latency_measurement_type": "paired_ROS_observer_callback_receipt_delta",
        "actual_rectify_processing_latency_measured": False,
        "child_cpu_usage_measured": False,
        "child_peak_rss_measured": False,
    }


def run():
    if os.environ.get("M23AN_ARM") != "timing":
        raise Blocked("explicit M23AN_ARM=timing required")
    import m23ak_composable_compare as m23ak
    import m23p_no_device_rectifier_shutdown as m23p
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        code = m23ak.run()
    sys.stdout.write(captured.getvalue())
    sys.stdout.flush()
    if code != 0 or captured.getvalue().count(
            "M23AK_VERDICT=TBB_FREE_PTHREADS_COMPOSABLE_SYNTHETIC_CLEAN") != 1:
        raise Blocked("inherited M23AK stock component success absent")
    measured = analyze(
        m23p.COUNTER_LOG.read_text(errors="replace"),
        m23p.PUBLISHER_LOG.read_text(errors="replace"))
    if measured["rect_receipt_gaps_over_250ms"] != 0:
        raise Blocked("rect subscriber callback receipt gap exceeded 250ms")
    print("M23AN_RESULT_JSON=" + json.dumps({
        "verdict": "SYNTHETIC_OBSERVER_TIMING_RECORDED",
        "observations": measured,
        "inherited_stock_composable_pthreads": True,
        "physical_camera_authorized": False,
        "performance_budget_fully_accepted": False,
        "production_acceptance": "BLOCKED",
    }, sort_keys=True), flush=True)
    print("M23AN_VERDICT=SYNTHETIC_OBSERVER_TIMING_RECORDED", flush=True)
    print("M23AN_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    if not args.run:
        print("M23AN_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT", flush=True)
        return 0
    try:
        return run()
    except (Blocked, OSError, RuntimeError, ValueError, KeyError, TypeError) as e:
        print("M23AN_BLOCKED=" + type(e).__name__ + ":" + str(e)[:170], flush=True)
        print("M23AN_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
