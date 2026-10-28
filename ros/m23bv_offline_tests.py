#!/usr/bin/env python3
"""Standalone M23BV regressions; no ROS installation, Podman or devices."""
import json
import contextlib
import io
import importlib.util
import os
import sys
import types
from pathlib import Path
from types import SimpleNamespace as N
import tempfile
import unittest
from unittest import mock

import yaml
import m23bv_window as w
import m23bv_qos_guard as g
import m23bv_derive_closed_bag as d

ROOT = Path(__file__).resolve().parent
BASE = 1_800_000_000_000_000_000
IMU = "/imu/data_raw"
IMAGE = "/camera/image_rect"
INFO = "/camera/camera_info"


def event(topic, i, *, t=None, width=640):
    ns = BASE + i * 20_000_000 if t is None else t
    return w.Event(topic, ns + i * 1_000, ns,
                   "imu_link" if topic == IMU else "camera_optical_frame",
                   width if topic != IMU else 0,
                   480 if topic != IMU else 0)


def happy_events():
    # 0..(20 * 20ms), images/info begin later and end earlier than IMU.
    out = []
    for i in range(30):
        out.append(event(IMU, i))
        if i < 2 or i > 27:     # original raw acquisition is wider
            out.append(event(IMAGE, i))
            out.append(event(INFO, i))
        elif 2 <= i <= 27:
            out.append(event(IMAGE, i))
            out.append(event(INFO, i))
    return sorted(out, key=lambda e: (e.bag_ns, e.topic))


def graph():
    def ep(name, qos="RELIABLE"):
        return {"node_name": name, "node_namespace": "/camera",
                "qos": {"reliability": qos, "durability": "VOLATILE",
                        "depth": 0, "history": "UNKNOWN"}}
    return {"rmw_implementation": "rmw_fastrtps_cpp",
            "topics": {
                g.RAW: {"publishers": [ep("usb_cam")],
                        "subscriptions": [ep("rectify_node")]},
                g.IMAGE: {"publishers": [ep("rectify_node")],
                          "subscriptions": [ep("rosbag2_recorder")]},
                g.INFO: {"publishers": [ep("usb_cam")],
                         "subscriptions": [ep("rectify_node", "BEST_EFFORT")]}}}


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.minimum = {"image": 3, "info": 3, "imu": 4}

    def test_clean_window(self):
        events = happy_events()
        # Hand-built extended camera stream is within IMU so no trim here.
        keep, report = w.select(events, min_counts=self.minimum)
        self.assertEqual(report["image_gaps_over_250ms"], 0)
        self.assertEqual(sum(keep), len(events))
        self.assertEqual(report["max_image_gap_ms"], 20)

    def test_head_and_tail_camera_frames_are_excluded(self):
        rows = [event(IMAGE, -2), event(INFO, -2), event(IMAGE, -1), event(INFO, -1)]
        rows += happy_events()
        rows += [event(IMAGE, 30), event(INFO, 30)]
        rows = sorted(rows, key=lambda x: (x.bag_ns, x.topic))
        flags, report = w.select(rows, min_counts=self.minimum)
        self.assertEqual(report["excluded_counts"], {"image": 3, "info": 3})
        self.assertEqual(sum(flags), len(happy_events()))
        self.assertEqual(report["retained_counts"]["imu"], 30)

    def test_large_gap_cannot_be_hidden(self):
        rows = [e for e in happy_events() if not (
            e.topic in (IMAGE, INFO) and 5 <= (e.header_ns-BASE)//20_000_000 <= 20)]
        with self.assertRaisesRegex(ValueError, "BLOCKED_IMAGE_STAMP_GAP"):
            w.select(rows, min_counts=self.minimum)

    def test_missing_camera_info_blocks(self):
        rows = [e for e in happy_events() if not (e.topic == INFO and e.header_ns == BASE + 15*20_000_000)]
        with self.assertRaisesRegex(ValueError, "BLOCKED_UNMATCHED_CAMERA_INFO"):
            w.select(rows, min_counts=self.minimum)

    def test_no_imu_blocks(self):
        with self.assertRaisesRegex(ValueError, "BLOCKED_NO_IMU"):
            w.select([e for e in happy_events() if e.topic != IMU], min_counts=self.minimum)

    def test_imu_gap_blocks(self):
        rows = [e for e in happy_events() if e.topic != IMU or (e.header_ns-BASE)//20_000_000 not in range(5, 24)]
        with self.assertRaisesRegex(ValueError, "BLOCKED_IMU_STAMP_GAP"):
            w.select(rows, min_counts=self.minimum)

    def test_bad_camera_frame_blocks(self):
        rows = happy_events()
        e = rows[3]
        rows[3] = w.Event(e.topic, e.bag_ns, e.header_ns, "wrong", e.width, e.height)
        with self.assertRaisesRegex(ValueError, "BLOCKED_UNEXPECTED_FRAME_ID"):
            w.select(rows, min_counts=self.minimum)

    def test_no_fabricated_timestamps(self):
        rows = [event(IMU, i) for i in range(10)]
        rows += [event(IMAGE, i) for i in range(10)]
        rows += [event(INFO, i) for i in range(10)]
        rows = sorted(rows, key=lambda e: (e.bag_ns, e.topic))
        flags, report = w.select(rows, min_counts=self.minimum)
        self.assertTrue(all(rows[i].header_ns == rows[i].header_ns for i,x in enumerate(flags) if x))
        self.assertFalse(report["physical_camera_imu_offset_calibrated"])

    def test_strict_count_gate_not_relaxed(self):
        with self.assertRaisesRegex(ValueError, "BLOCKED_INSUFFICIENT_RETAINED"):
            w.select(happy_events())


class QoSAndSafetyTests(unittest.TestCase):
    def test_valid_endpoint_inventory(self):
        self.assertTrue(g.validate_camera_graph(graph()))

    def test_no_recorder_reliable_endpoint_blocks(self):
        q = graph(); q["topics"][g.IMAGE]["subscriptions"][0]["qos"]["reliability"] = "BEST_EFFORT"
        with self.assertRaisesRegex(ValueError, "RECORDER_IMAGE"):
            g.validate_camera_graph(q)

    def test_best_effort_camera_raw_blocks(self):
        q = graph(); q["topics"][g.RAW]["publishers"][0]["qos"]["reliability"] = "BEST_EFFORT"
        with self.assertRaisesRegex(ValueError, "RELIABLE_ENDPOINT"):
            g.validate_camera_graph(q)

    def test_unknown_rmw_blocks(self):
        q = graph(); q["rmw_implementation"] = ""
        with self.assertRaisesRegex(ValueError, "UNKNOWN_RMW"):
            g.validate_camera_graph(q)

    def test_additive_qos_policy(self):
        a = yaml.safe_load((ROOT / "m23bv_sensor_qos.yaml").read_text())
        self.assertEqual(set(a), set(d.MESSAGE_TYPES))
        self.assertEqual(a[IMAGE]["reliability"], "reliable")
        self.assertEqual(a[IMAGE]["depth"], 100)
        self.assertEqual(a[IMU]["reliability"], "best_effort")

    def test_rectifier_process_is_separate(self):
        s = (ROOT / "m23bv_camera_only.launch.py").read_text()
        self.assertIn('executable="rectify_node"', s)
        self.assertIn('additional_env={', s)
        self.assertIn('ALT_OPENCV', s)
        self.assertNotIn("ComposableNodeContainer(", s)
        self.assertIn('"LEROY_NAV2_DRY_RUN": "1"', s)

    def test_m20_observer_source_unmodified(self):
        runner = (ROOT / "m23bv_sensor_only_run.sh").read_text()
        self.assertIn('python3 -u /opt/m20/m20_combined_observer.py &', runner)
        self.assertIn('kill -TERM "$pid"', runner)
        self.assertIn('for name in imu camera', runner)
        self.assertIn('if ! python3 -u /opt/m23bv/m23bv_qos_guard.py', runner)
        self.assertLess(runner.index('m23bv_qos_guard.py'), runner.index('m20_combined_observer.py &'))
        self.assertNotIn("leroy-ros", runner)
        self.assertNotIn("set_wheels", runner)

    def test_no_hardware_host_launcher_supplied(self):
        self.assertFalse((ROOT / "m23bv_operator_host_capture.sh").exists())


class FakeBag:
    messages = {}
    written = {}

    class StorageOptions:
        def __init__(self, **kwargs): self.__dict__.update(kwargs)
    class ConverterOptions:
        def __init__(self, a, b): self.types=(a,b)
    class SequentialReader:
        def open(self, storage, options):
            self.rows = list(FakeBag.messages[storage.uri])
            self.i = 0
        def get_all_topics_and_types(self):
            return [N(name=k, type=v) for k,v in d.MESSAGE_TYPES.items()]
        def has_next(self): return self.i < len(self.rows)
        def read_next(self):
            v=self.rows[self.i];self.i+=1;return v
        def close(self): pass
    class SequentialWriter:
        def open(self, storage, options):
            self.path=Path(storage.uri)
            self.path.mkdir()
            self.rows=[]
            self.topic=[]
        def create_topic(self, entry): self.topic.append(entry.name)
        def write(self, topic, blob, receipt): self.rows.append((topic,blob,receipt))
        def close(self):
            (self.path/"bag_0.db3").write_bytes(b"".join(blob for _,blob,_ in self.rows))
            (self.path/"metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n"
                "  storage_identifier: sqlite3\n"
                "  relative_file_paths: [bag_0.db3]\n")
            FakeBag.written[str(self.path)] = self.rows


def fake_decode(_topic, payload):
    obj = json.loads(payload)
    stamp = int(obj["ns"])
    return N(header=N(stamp=N(sec=stamp // 1_000_000_000,
                              nanosec=stamp % 1_000_000_000),
                      frame_id=obj["frame"]),
             width=obj.get("width",0), height=obj.get("height",0))


class DerivedBagTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root=Path(self.tmp.name)
        self.source=root/"source"
        self.output=root/"derived"
        self.source.mkdir()
        (self.source/"bag_0.db3").write_bytes(b"immutable-original-database")
        (self.source/"metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  storage_identifier: sqlite3\n"
            "  relative_file_paths: [bag_0.db3]\n")
        FakeBag.written={}
        self.setup_rows(happy_events())

    def setup_rows(self, events):
        FakeBag.messages={str(self.source): [
            (e.topic, json.dumps({"ns":e.header_ns,"frame":e.frame_id,
                                  "width":e.width,"height":e.height}).encode(), e.bag_ns)
            for e in events]}

    def test_derived_bag_copies_cdr_and_receipt_stamps(self):
        before=d.fingerprints(self.source)
        report=d.derive(self.source,self.output,baglib=FakeBag,
                        decode=fake_decode,min_counts={"image":3,"info":3,"imu":4})
        self.assertEqual(before,d.fingerprints(self.source))
        self.assertEqual(report["verdict"],"DERIVED_BAG_CLOSED_PENDING_M23A_M23I_M23F_M23J")
        self.assertEqual(FakeBag.written[str(self.output)], FakeBag.messages[str(self.source)])
        self.assertTrue((self.output/"metadata.yaml").is_file())

    def test_source_retained_on_bad_gap_and_no_output_created(self):
        ev=[e for e in happy_events() if not (e.topic in (IMAGE,INFO) and
            5 <= (e.header_ns-BASE)//20_000_000 <= 20)]
        self.setup_rows(ev)
        before=d.fingerprints(self.source)
        with self.assertRaisesRegex(ValueError,"BLOCKED_IMAGE_STAMP_GAP"):
            d.derive(self.source,self.output,baglib=FakeBag,
                     decode=fake_decode,min_counts={"image":3,"info":3,"imu":4})
        self.assertFalse(self.output.exists())
        self.assertEqual(before,d.fingerprints(self.source))

    def test_output_collision_refuses_overwrite(self):
        self.output.mkdir()
        (self.output/"important").write_text("preserve")
        with self.assertRaisesRegex(ValueError,"BLOCKED_OUTPUT_EXISTS"):
            d.derive(self.source,self.output,baglib=FakeBag,
                     decode=fake_decode,min_counts={"image":3,"info":3,"imu":4})
        self.assertEqual((self.output/"important").read_text(),"preserve")

    def test_metadata_traversal_rejected(self):
        (self.source/"metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n"
            "  storage_identifier: sqlite3\n"
            "  relative_file_paths: [../outside.db3]\n")
        with self.assertRaisesRegex(ValueError,"BLOCKED_UNSAFE_STORAGE_REFERENCE"):
            d.fingerprints(self.source)



def load_capture_with_stubs():
    # Runtime-independent adapter for the existing M23H/M23G imports.
    gmod = types.ModuleType("m23g_live_recording_preflight")
    gmod.TOPICS = tuple(d.MESSAGE_TYPES)
    gmod.preflight = lambda root, sid, duration_seconds: {
        "disk_capacity_sufficient": True,
        "output_directory": str(Path(root) / ("m23g_live_" + sid)),
    }
    hmod = types.ModuleType("m23h_bounded_capture")
    hmod.check_devices = lambda: None
    hmod.closed_sqlite_counts = lambda target: {t: 6000 for t in gmod.TOPICS}
    hmod.assess_counts = lambda counts: set(counts) == set(gmod.TOPICS)
    hmod.SENSOR_PROCESS_TIMEOUT_S = 175.0
    hmod.SENSOR_GRACE_S = 24.0
    hmod.ROSBAG_FLUSH_TIMEOUT_S = 50.0
    hmod.RECORD_START_SETTLE_S = 3.0
    events = []
    def stop(proc, signum, grace, process_group=False):
        events.append((proc.name, signum, process_group))
        return proc.code, True
    hmod._signal_and_wait = stop
    spec = importlib.util.spec_from_file_location(
        "m23bv_synthetic_capture", ROOT / "m23bv_bounded_capture.py")
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, {
            "m23g_live_recording_preflight": gmod,
            "m23h_bounded_capture": hmod}):
        spec.loader.exec_module(module)
    return module, events


class FakeProcess:
    def __init__(self, name, code=0):
        self.name=name
        self.code=code
    def poll(self): return None
    def wait(self, timeout=None): return self.code


class CaptureLifecycleTests(unittest.TestCase):
    def _exercise(self, *, sensor_code=0, recorder_code=0):
        module, events = load_capture_with_stubs()
        calls=[]
        def spawn(argv, **kwargs):
            calls.append(list(argv))
            return FakeProcess("recorder",recorder_code) if len(calls)==1 else FakeProcess("sensor",sensor_code)
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.dict(os.environ, module.EXPECTED_ENV, clear=True):
                # Avoid printing a fake CLOSED candidate as if it were a real run.
                with contextlib.redirect_stdout(io.StringIO()):
                    rc=module.capture(Path(tmp), "mock", spawn=spawn, pause=lambda _:None)
            log=Path(tmp)/"m23bv_capture_mock.log"
            self.assertTrue(log.exists())
        return rc,calls,events

    def test_recorder_starts_first_and_uses_reliable_qos(self):
        rc,calls,events=self._exercise()
        self.assertEqual(rc,0)
        self.assertEqual(calls[0][:3],["ros2","bag","record"])
        self.assertEqual(calls[0][calls[0].index("--qos-profile-overrides-path")+1],
                         "/opt/m23bv/m23bv_sensor_qos.yaml")
        self.assertEqual(calls[1],["/usr/local/bin/leroy-m23bv-sensor-run"])

    def test_imu_cleanup_order_and_bag_sigint(self):
        import signal
        rc,_,events=self._exercise()
        self.assertEqual(rc,0)
        self.assertEqual(events,[("sensor",signal.SIGTERM,False),
                                 ("recorder",signal.SIGINT,True)])

    def test_nonzero_sensor_fails(self):
        rc,_,_=self._exercise(sensor_code=2)
        self.assertEqual(rc,2)

    def test_nonzero_recorder_fails(self):
        rc,_,_=self._exercise(recorder_code=2)
        self.assertEqual(rc,2)

    def test_unapproved_capture_fails_before_spawning(self):
        module,_=load_capture_with_stubs()
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.dict(os.environ, {}, clear=True):
                with self.assertRaisesRegex(ValueError,"approval/environment"):
                    module.capture(Path(tmp), "mock", spawn=lambda *a,**k:
                                   self.fail("should never spawn"))



if __name__ == "__main__":
    rc=unittest.main(verbosity=2,exit=False).result.wasSuccessful()
    print("M23BV_OFFLINE_UNIT_TESTS=" + ("PASS" if rc else "FAIL"),flush=True)
    print("M23BV_DEVICE_ACCESS=NONE",flush=True)
    raise SystemExit(0 if rc else 2)
