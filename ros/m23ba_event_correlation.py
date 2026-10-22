#!/usr/bin/env python3
"""M23BA: OFFLINE text-only M23AX pre-send / M23K subscriber correlation.

Default is inert; no camera, ROS, Podman, devices, or log creation.
A successful comparison is forensic consistency, NOT production acceptance.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys

CAP = 5000
MAX_LOG_BYTES = 4_000_000
GAP_NS = 250_000_000
STREAMS = ("raw", "rect", "info")
PRE_SEND = re.compile(
    r"M23AX_PUBLISH pub_index=(\d+) fresh=([01])"
    r" status=([a-z_]+) dequeue_count=(\d+)"
    r" v4l2_seq=(\d+) v4l2_timestamp_us=(-?\d+)"
    r" ros_stamp_ns=(\d+)\s*$"
)
SAMPLE = re.compile(r"^M23K_SAMPLE (raw|rect|info) (\d+) (\d+)$")
COUNTS = re.compile(r"^M23K_COUNTER_COUNTS=(\d+),(\d+),(\d+)$")
STATUS = {"mmap_dequeued", "select_eintr",
          "mmap_g_fmt_eagain", "mmap_dqbuf_eagain"}


class Blocked(ValueError):
    pass


@dataclass(frozen=True)
class PreSend:
    index: int
    fresh: bool
    status: str
    dequeues: int
    v4l2_seq: int
    v4l2_timestamp_us: int
    stamp_ns: int


def parse_publisher(text):
    """Require exact, complete, contiguous pre-send events; no inferred writes."""
    if not isinstance(text, str) or not text:
        raise Blocked("publisher log empty")
    events = []
    for line in text.splitlines():
        if "M23AX_FATAL=" in line or "Segmentation fault" in line or "process has died" in line:
            raise Blocked("publisher fatal or process crash")
        if "M23AX_PUBLISH" not in line:
            continue
        if line.count("M23AX_PUBLISH") != 1:
            raise Blocked("ambiguous pre-send marker line")
        match = PRE_SEND.search(line)
        if match is None:
            raise Blocked("malformed pre-send event")
        i, fresh, status, count, seq, ts, stamp = match.groups()
        event = PreSend(int(i), fresh == "1", status, int(count), int(seq),
                        int(ts), int(stamp))
        if event.index != len(events) + 1:
            raise Blocked("publisher index not contiguous or not starting at one")
        if event.index > CAP:
            raise Blocked("publisher bound exceeded")
        if event.status not in STATUS:
            raise Blocked("unexpected acquisition status")
        if (event.fresh != (event.status == "mmap_dequeued")
                or event.stamp_ns <= 0 or event.v4l2_seq > 0xFFFFFFFF
                or event.v4l2_timestamp_us < -1):
            raise Blocked("invalid pre-send acquisition event")
        previous = events[-1] if events else None
        prior_dequeues = previous.dequeues if previous else 0
        if event.dequeues != prior_dequeues + int(event.fresh):
            raise Blocked("dequeue counter differs from acquisition freshness")
        if event.fresh and event.v4l2_timestamp_us < 0:
            raise Blocked("fresh buffer missing kernel timestamp")
        if not event.fresh:
            if previous is None:
                raise Blocked("initial nonfresh positive-stamp event cannot be audited")
            if (event.stamp_ns != previous.stamp_ns
                    or event.v4l2_seq != previous.v4l2_seq
                    or event.v4l2_timestamp_us != previous.v4l2_timestamp_us):
                raise Blocked("nonfresh event unexpectedly changed stored frame")
        events.append(event)
    if not events:
        raise Blocked("no M23AX pre-send events")
    return events


def marker_once(lines, prefix, expected):
    matches = [line[len(prefix):] for line in lines if line.startswith(prefix)]
    if len(matches) != 1 or matches[0] != expected:
        raise Blocked("missing, duplicate or wrong observer marker: " + prefix)


def parse_observer(text):
    """Parse the ACTUAL M23K counter format and preserve every occurrence."""
    if not isinstance(text, str) or not text:
        raise Blocked("observer log empty")
    samples = {key: [] for key in STREAMS}
    summaries = []
    lines = text.splitlines()
    marker_once(lines, "M23K_COUNTER_VARIANT=", "FULL")
    marker_once(lines, "M23M_COUNTER_QOS_PROFILE=", "reliable_100")
    for line in lines:
        if ("M23K_COUNTER_FATAL=" in line or "Segmentation fault" in line
                or "process has died" in line):
            raise Blocked("observer fatal or process crash")
        if line.startswith("M23K_SAMPLE"):
            found = SAMPLE.fullmatch(line)
            if found is None:
                raise Blocked("malformed subscriber sample")
            kind, stamp, receipt = found.groups()
            stamp, receipt = int(stamp), int(receipt)
            if stamp <= 0 or receipt <= 0 or len(samples[kind]) >= CAP:
                raise Blocked("invalid sample bound or timestamp")
            if samples[kind] and receipt <= samples[kind][-1][1]:
                raise Blocked("nonincreasing steady-clock receipt: " + kind)
            samples[kind].append((stamp, receipt))
        elif line.startswith("M23K_COUNTER_COUNTS="):
            match = COUNTS.fullmatch(line)
            if match is None:
                raise Blocked("malformed counter summary")
            summaries.append(tuple(int(v) for v in match.groups()))
    if len(summaries) != 1:
        raise Blocked("missing or duplicated counter completion summary")
    actual = tuple(len(samples[k]) for k in STREAMS)
    if summaries[0] != actual:
        raise Blocked("subscriber sample log truncated or counts mismatch")
    if any(c < 3 for c in actual):
        raise Blocked("insufficient subscriber stream coverage")
    return samples


def nonpositive_intervals(stamps):
    return sum(b <= a for a, b in zip(stamps, stamps[1:]))


def over_gap_intervals(stamps):
    return sum(b - a > GAP_NS for a, b in zip(stamps, stamps[1:]))


def duplicates(stamps):
    counts = Counter(stamps)
    return {s: n for s, n in sorted(counts.items()) if n > 1}


def correlate(publisher, observer):
    presend = parse_publisher(publisher)
    samples = parse_observer(observer)
    pub_stamps = [e.stamp_ns for e in presend]
    subscriber_stamps = {k: [s for s, _ in samples[k]] for k in STREAMS}
    pub_counts = Counter(pub_stamps)
    raw_counts = Counter(subscriber_stamps["raw"])
    pub_dup = duplicates(pub_stamps)
    raw_dup = duplicates(subscriber_stamps["raw"])
    stale = [e.index for e in presend if not e.fresh]
    shared_repeated = sorted(set(pub_dup) & set(raw_dup))
    if shared_repeated:
        pattern = "PRESEND_AND_RAW_REPEATED_STAMP_OBSERVED"
    elif pub_dup:
        pattern = "PRESEND_REPEAT_WITHOUT_MATCHED_RAW_REPEAT"
    elif raw_dup:
        pattern = "RAW_OBSERVER_REPEAT_WITHOUT_RECORDED_PRESEND_REPEAT"
    else:
        pattern = "NO_REPEAT_IN_PRESERVED_EVENTS"

    return {
        "verdict": "OFFLINE_EVENT_CORRELATION_DIAGNOSTIC_ONLY",
        "observed_pattern": pattern,
        "pre_send_attempts": len(presend),
        "pre_send_repeated_stamp_excess": sum(n - 1 for n in pub_dup.values()),
        "pre_send_nonfresh_attempts": len(stale),
        "pre_send_nonfresh_indexes": stale[:20],
        "pre_send_nonpositive_header_intervals": nonpositive_intervals(pub_stamps),
        "pre_send_header_gaps_over_250ms": over_gap_intervals(pub_stamps),
        "raw_callbacks": len(samples["raw"]),
        "rect_callbacks": len(samples["rect"]),
        "info_callbacks": len(samples["info"]),
        "raw_repeated_stamp_excess": sum(n - 1 for n in raw_dup.values()),
        "raw_nonpositive_header_intervals":
            nonpositive_intervals(subscriber_stamps["raw"]),
        "raw_header_gaps_over_250ms":
            over_gap_intervals(subscriber_stamps["raw"]),
        "rect_nonpositive_header_intervals":
            nonpositive_intervals(subscriber_stamps["rect"]),
        "rect_header_gaps_over_250ms":
            over_gap_intervals(subscriber_stamps["rect"]),
        "repeated_stamps_in_both_pre_send_and_raw": shared_repeated,
        "raw_stamp_occurrences_exceeding_pre_send_count": [
            {"stamp_ns": s, "raw_observations": count,
             "pre_send_attempts": pub_counts.get(s, 0)}
            for s, count in sorted(raw_counts.items())
            if count > pub_counts.get(s, 0)
        ][:25],
        "raw_stamp_excess_record_count": sum(
            count > pub_counts.get(s, 0) for s, count in raw_counts.items()),
        "rect_stamps_not_seen_in_raw": len(
            set(subscriber_stamps["rect"]) - set(subscriber_stamps["raw"])),
        "individual_occurrences_preserved_in_source_text": True,
        "identical_stamp_occurrences_are_not_uniquely_pairable": True,
        "pre_send_log_does_not_prove_publish_delivery": True,
        "publisher_and_observer_capture_window_equivalence_verified": False,
        "dds_redelivery_or_reordering_proven": False,
        "usb_cam_duplicate_delivered_to_raw_subscriber_proven": False,
        "physical_camera_retest_authorized": False,
        "production_acceptance": "BLOCKED",
    }


def read_bounded_file(path):
    if path.is_symlink() or not path.is_file():
        raise Blocked("evidence file missing, symlinked, or nonregular")
    if not 0 < path.stat().st_size <= MAX_LOG_BYTES:
        raise Blocked("evidence file empty or above bound")
    return path.read_text(encoding="utf-8")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--publisher", type=Path)
    p.add_argument("--observer", type=Path)
    args = p.parse_args(argv)
    if args.publisher is None and args.observer is None:
        print("M23BA_STATUS=INERT_NO_ROS_NO_DEVICE", flush=True)
        return 0
    try:
        if args.publisher is None or args.observer is None:
            raise Blocked("both independent evidence files required")
        if args.publisher.resolve() == args.observer.resolve():
            raise Blocked("publisher and observer file must be distinct")
        result = correlate(read_bounded_file(args.publisher),
                           read_bounded_file(args.observer))
        print("M23BA_RESULT_JSON=" + json.dumps(result, sort_keys=True), flush=True)
        print("M23BA_VERDICT=OFFLINE_EVENT_CORRELATION_DIAGNOSTIC_ONLY",
              flush=True)
        print("M23BA_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 0
    except (OSError, UnicodeError, ValueError, TypeError, KeyError) as e:
        print("M23BA_BLOCKED=" + type(e).__name__ + ":" + str(e)[:190], flush=True)
        print("M23BA_PRODUCTION_ACCEPTANCE=BLOCKED", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
