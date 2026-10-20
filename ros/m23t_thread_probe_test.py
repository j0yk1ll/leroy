"""M23T no-device source and negative evidence contracts; no ROS execution."""
import ast
import json
from pathlib import Path
import unittest

from m23t_thread_probe_runner import (
    THREAD_PROFILES, selected_profile, setup_from_log,
)
from m23t_compare_thread_logs import classify, compare

ROOT = Path(__file__).resolve().parent
D = "-0.08,0.01,0.001,-0.001,0"


def env(profile="default"):
    return {
        "M23T_THREAD_PROBE_ONLY": "1",
        "M23T_OPENCV_THREAD_PROFILE": profile,
        "M23S_AB_ONLY": "1", "M23S_DISTORTION_PROFILE": "rectify",
        "M23P_PROFILE": "synthetic", "M23P_SYNTHETIC_ONLY": "1",
        "M23R_TRACE_ONLY": "1", "M23K_PROBE_MODE": "selftest",
        "M23N_CAMERA_QOS_ONLY": "0", "M23O_RECT_QOS_ONLY": "0",
        "LEROY_NAV2_DRY_RUN": "1", "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }


def fixture(profile="default", crash=True, delivered=900):
    before = 64
    after = 64 if profile == "default" else 1
    p = {
        "device_access": "NONE", "profile": "synthetic",
        "actual_graph_qos_verified": True, "synthetic_calibration_only": True,
        "physical_motion_authorized": False, "synthetic_publisher_exit_code": 0,
        "synthetic_observer_exit_code": 0,
        "component_exit_class":
            "BLOCKED_RECTIFIER_CHILD_CRASH" if crash else "CLEAN_COMPONENT_SHUTDOWN",
        "faults": ["BLOCKED_RECTIFIER_CHILD_CRASH"] if crash else [],
        "observations": {
            "synthetic_published_image_count": 900,
            "actual_graph_qos_verified": True,
            "synthetic_calibration": "FABRICATED_NONZERO_DISTORTION",
            "synthetic_observer_counts": {"raw": delivered, "rect": delivered, "info": 900},
            "unmatched_rectified_header_stamps": 0,
            "raw_rect_identical_header_stamp_pairs": delivered,
            "synthetic_rect_max_header_gap_ms": 33.524,
        },
    }
    r = {
        "device_access": "NONE", "faults": [],
        "m23p_runner_exit": 2 if crash else 0,
        "verdict": "BLOCKED_COMPONENT_LIFECYCLE" if crash else
                   "CLEAN_OBSERVED_UNDER_TRACE_ONLY",
        "module_root_cause_verified": False,
        "trace": {"process_count": 1, "event_counts": {"CONSTRUCTOR": 1}},
    }
    s = {
        "distortion_profile": "rectify", "source_d": D,
        "publisher_calibration_verified": True, "faults": [],
        "device_access": "NONE", "production_acceptance": "BLOCKED",
        "m23r_runner_exit": 2 if crash else 0,
        "component_child_exit_code_if_reported": -11 if crash else None,
    }
    t = {
        "thread_profile": profile, "distortion_profile": "rectify",
        "source_d": D, "source_component": "image_proc::RectifyNode",
        "device_access": "NONE", "production_acceptance": "BLOCKED",
        "root_cause_verified": False, "opencv_backend_verified": False,
        "faults": [], "m23s_runner_exit": 2 if crash else 0,
        "verdict": "BLOCKED_M23T_INHERITED_GATES" if crash else
                   "CLEAN_UNDER_THREAD_PROBE_ONLY",
        "opencv_setup": {"profile": profile, "api_threads_before": before,
                         "api_threads_after": after, "threading_backend_verified": False,
                         "actual_worker_threads_verified": False},
    }
    return ("M23P_LIFECYCLE_JSON=" + json.dumps(p) + "\n"
            + "M23R_TRACE_JSON=" + json.dumps(r) + "\n"
            + "M23S_AB_RESULT_JSON=" + json.dumps(s) + "\n"
            + "M23T_THREADS_JSON=" + json.dumps(t) + "\n"
            + "M23T_DEVICE_ACCESS=NONE\n"
            + "M23T_PRODUCTION_ACCEPTANCE=BLOCKED\n")


def mutate(log, key, apply):
    lines = log.splitlines()
    for i, line in enumerate(lines):
        if line.startswith(key):
            item = json.loads(line[len(key):])
            apply(item)
            lines[i] = key + json.dumps(item)
            return "\n".join(lines) + "\n"
    raise AssertionError("missing fixture marker")


class M23TContracts(unittest.TestCase):
    def test_only_two_opencv_profiles(self):
        self.assertEqual(THREAD_PROFILES, ("default", "single"))
        for p in THREAD_PROFILES:
            self.assertEqual(selected_profile(env(p)), p)
        for p in ("", "opencv_tbb", "no_opencv", "3", "rectify", None):
            e = env()
            if p is None:
                e.pop("M23T_OPENCV_THREAD_PROFILE")
            else:
                e["M23T_OPENCV_THREAD_PROFILE"] = p
            with self.subTest(profile=p), self.assertRaises(ValueError):
                selected_profile(e)

    def test_every_child_and_no_device_guard_required(self):
        for name in env():
            e = env()
            e.pop(name)
            with self.subTest(key=name), self.assertRaises(ValueError):
                selected_profile(e)

    def test_global_preload_forbidden(self):
        with self.assertRaises(ValueError):
            selected_profile({**env(), "LD_PRELOAD": "/tmp/lib.so"})

    def test_must_use_nonzero_distortion(self):
        with self.assertRaises(ValueError):
            selected_profile({**env(), "M23S_DISTORTION_PROFILE": "passthrough"})

    def test_opencv_markers_verified_exactly_once(self):
        for p, after in (("default", 64), ("single", 1)):
            sample = (
                "[component_container-1] M23R_TRACE_EVENT=CONSTRUCTOR "
                "mono_ns=15 pid=32 tid=32 handle=(nil) status=0 module=component_container\n"
                "[component_container-1] M23T_OPENCV_THREAD_SETUP "
                f"profile={p} before=64 after={after} pid=32\n"
            )
            result = setup_from_log(sample, p)
            self.assertEqual(result["api_threads_after"], after)
            self.assertIs(result["threading_backend_verified"], False)
            self.assertIs(result["actual_worker_threads_verified"], False)

    def test_opencv_marker_missing_or_duplicate_blocks(self):
        prefix = ("M23R_TRACE_EVENT=CONSTRUCTOR mono_ns=15 "
                  "pid=32 tid=32 handle=(nil) status=0 module=component_container\n")
        body = "M23T_OPENCV_THREAD_SETUP profile=single before=64 after=1 pid=32\n"
        for invalid in (
            prefix, prefix + body + body,
            prefix + body.replace("pid=32", "pid=90"),
            prefix + body.replace("after=1", "after=5"),
            prefix + body.replace("profile=single", "profile=default"),
            prefix + body.replace("before=64", "before=-1"),
        ):
            with self.subTest(payload=invalid[-70:]), self.assertRaises(ValueError):
                setup_from_log(invalid, "single")

    def test_default_cannot_mutate_thread_count(self):
        log = ("M23R_TRACE_EVENT=CONSTRUCTOR mono_ns=15 pid=32 tid=32 handle=(nil) "
               "status=0 module=component_container\n"
               "M23T_OPENCV_THREAD_SETUP profile=default before=64 after=1 pid=32\n")
        with self.assertRaises(ValueError):
            setup_from_log(log, "default")

    def test_source_is_same_original_rectify_component(self):
        launch = (ROOT / "m23t_rectifier_thread_trace.launch.py").read_text()
        previous = (ROOT / "m23r_rectifier_trace.launch.py").read_text()
        a = launch.split("    component = ComposableNodeContainer(", 1)[1].split(
            "    return LaunchDescription(", 1)[0]
        b = previous.split("    component = ComposableNodeContainer(", 1)[1].split(
            "    return LaunchDescription(", 1)[0]
        self.assertEqual(a, b)
        self.assertIn('plugin="image_proc::RectifyNode"', launch)
        self.assertEqual(launch.count("ComposableNode("), 1)
        self.assertIn('value=TRACE_LIBRARY + ":" + OPENCV_LIBRARY', launch)
        self.assertNotIn("m23t::", launch)

    def test_runner_reuses_m23s_m23r_m23p_verbatim(self):
        src = (ROOT / "m23t_thread_probe_runner.py").read_text()
        self.assertIn("m23r.TRACE_LAUNCH = THREAD_LAUNCH", src)
        self.assertIn("status = m23s.run()", src)
        self.assertIn("m23p.prohibit_devices()", src)
        self.assertIn("m23r.preconditions(os.environ)", src)
        self.assertIn('M23T_PRODUCTION_ACCEPTANCE=BLOCKED', src)
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                name = (node.names[0].name if isinstance(node, ast.Import)
                        else node.module or "")
                self.assertNotIn(name.split(".")[0],
                                 ("subprocess", "rclpy", "serial"))
            if isinstance(node, ast.Call):
                self.assertNotIn(ast.unparse(node.func),
                                 ("subprocess.Popen", "os.system", "os.killpg"))

    def test_opencv_probe_is_child_gated_and_no_ros(self):
        src = (ROOT / "m23t_opencv_thread_probe.cpp").read_text()
        self.assertIn('readlink("/proc/self/exe"', src)
        self.assertIn('"component_container"', src)
        self.assertIn('std::getenv("M23T_THREAD_PROBE_ONLY")', src)
        self.assertIn('cv::setNumThreads(1)', src)
        self.assertIn('cv::getNumThreads()', src)
        self.assertNotIn("rclcpp", src)
        for x in ("fork(", "execve(", "system(", "kill(", "/dev/video", "ttyACM"):
            self.assertNotIn(x, src)

    def test_classifier_default_sigsegv_single_clean(self):
        result = compare(fixture("default", True),
                         fixture("single", False))
        self.assertEqual(result["pair_verdict"],
                         "THREAD_LIMIT_ASSOCIATION_SUPPORTED_NOT_CAUSAL")
        self.assertFalse(result["causation_verified"])
        self.assertEqual(result["production_acceptance"], "BLOCKED")

    def test_classifier_both_crash(self):
        result = compare(fixture("default", True), fixture("single", True))
        self.assertEqual(result["pair_verdict"],
                         "THREAD_LIMIT_DOES_NOT_PREVENT_SIGSEGV_IN_PAIR")

    def test_classifier_both_clean_not_fix(self):
        result = compare(fixture("default", False), fixture("single", False))
        self.assertEqual(result["pair_verdict"],
                         "BASELINE_CRASH_NOT_REPRODUCED_UNDER_PROBE")
        self.assertFalse(result["causation_verified"])

    def test_classifier_reverse_not_causal(self):
        result = compare(fixture("default", False), fixture("single", True))
        self.assertEqual(result["pair_verdict"],
                         "REVERSE_PATTERN_INCONCLUSIVE")

    def test_wrong_D_qos_or_weak_rect_blocks(self):
        for bad in (
            mutate(fixture(), "M23S_AB_RESULT_JSON=",
                   lambda d: d.update(source_d="0,0,0,0,0")),
            mutate(fixture(), "M23P_LIFECYCLE_JSON=",
                   lambda d: d.update(actual_graph_qos_verified=False)),
            fixture(delivered=4),
            mutate(fixture(), "M23P_LIFECYCLE_JSON=",
                   lambda d: d["observations"].update(
                       synthetic_rect_max_header_gap_ms=251.0)),
        ):
            with self.assertRaises(ValueError):
                classify(bad, "default")

    def test_missing_duplicate_marker_or_no_device_blocks(self):
        good = fixture()
        for bad in (
            good.replace("M23T_THREADS_JSON=", "M23T_BAD_JSON="),
            good + good.splitlines()[0] + "\n",
            good.replace("M23T_DEVICE_ACCESS=NONE", "M23T_DEVICE_ACCESS=CAMERA"),
        ):
            with self.assertRaises(ValueError):
                classify(bad, "default")

    def test_child_signal_unknown_blocks_even_on_zero_parent(self):
        bad = mutate(fixture(), "M23S_AB_RESULT_JSON=",
                     lambda d: d.update(component_child_exit_code_if_reported=None))
        with self.assertRaises(ValueError):
            classify(bad, "default")

    def test_thread_setup_wrong_profile_or_unverified_blocks(self):
        for bad in (
            mutate(fixture(), "M23T_THREADS_JSON=",
                   lambda d: d.update(thread_profile="single")),
            mutate(fixture(), "M23T_THREADS_JSON=",
                   lambda d: d["opencv_setup"].update(
                       threading_backend_verified=True)),
            mutate(fixture("single", True), "M23T_THREADS_JSON=",
                   lambda d: d["opencv_setup"].update(api_threads_after=12)),
        ):
            expected = "single" if '"thread_profile": "single"' in bad else "default"
            with self.assertRaises(ValueError):
                classify(bad, expected)

    def test_docker_build_contains_sources_and_no_network_install(self):
        docker = (ROOT / "Dockerfile.m23t-opencv-thread").read_text()
        self.assertIn(
            "FROM localhost/leroy-sensors:m23s-rectification-ab", docker)
        self.assertIn("M23T_NO_DEVICE_THREAD_IMAGE_CONTRACT=PASS", docker)
        for x in (
            "m23t_opencv_thread_probe.cpp",
            "m23t_rectifier_thread_trace.launch.py",
            "m23t_thread_probe_runner.py",
            "m23t_compare_thread_logs.py",
            "m23t_thread_probe_test.py",
            "M23T_OPENCV_THREAD_LIFETIME.md",
            "Dockerfile.m23t-opencv-thread",
        ):
            self.assertIn("COPY " + x + " /opt/m23t/" + x, docker)
        for x in ("--device ", "--privileged", "apt-get ", "curl ", "wget "):
            self.assertNotIn(x, docker)

    def test_runbook_preserves_isolation_and_nonclaims(self):
        doc = (ROOT / "M23T_OPENCV_THREAD_LIFETIME.md").read_text()
        for x in ("--network none", "--ipc private", "--pid private",
                  "--pull=never", "--read-only", "--cap-drop all",
                  "--security-opt no-new-privileges",
                  "M23T_PRODUCTION_ACCEPTANCE=BLOCKED"):
            self.assertIn(x, doc)
        for x in ("--device ", "--privileged", "--network host"):
            self.assertNotIn(x, doc)


if __name__ == "__main__":
    r = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(M23TContracts))
    if r.wasSuccessful():
        print("M23T_NO_DEVICE_THREAD_SOURCE_CONTRACT=PASS", flush=True)
        print("M23T_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
