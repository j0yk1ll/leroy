"""M23S: offline-only A/B source, safety, publisher and evidence tests."""
import ast
import json
from pathlib import Path
import unittest

import m23p_no_device_rectifier_shutdown as m23p
from m23s_rectification_ab import (
    CALIBRATION, D_VECTORS, PROFILES, child_exit_from_log,
    selected_profile, validate_publisher_log,
)
from m23s_compare_ab_logs import (
    classify_run, compare_payloads, sole_json,
)

ROOT = Path(__file__).resolve().parent


def environment(profile="passthrough"):
    return {
        "M23S_AB_ONLY": "1",
        "M23S_DISTORTION_PROFILE": profile,
        "M23P_PROFILE": "synthetic",
        "M23P_SYNTHETIC_ONLY": "1",
        "M23R_TRACE_ONLY": "1",
        "M23K_PROBE_MODE": "selftest",
        "M23N_CAMERA_QOS_ONLY": "0",
        "M23O_RECT_QOS_ONLY": "0",
        "LEROY_NAV2_DRY_RUN": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }


def publisher_log(profile):
    return "\n".join((
        "M23S_PUBLISHER_DISTORTION_PROFILE=" + profile,
        "M23S_PUBLISHER_D_VECTOR=" + D_VECTORS[profile],
        "M23P_SYNTHETIC_CALIBRATION=" + CALIBRATION[profile],
        "M23P_PUBLISHER_GRAPH_READY=YES",
        "M23P_PUBLISHER_FRAMES_STARTED=YES",
        "M23P_PUBLISHER_COUNTS=900,900",
        "M23P_PUBLISHER_START_SPAN_NS=29966666667",
        "M23P_PUBLISHER_MAX_CALLBACK_NS=1050000",
        "M23P_PUBLISHER_PAYLOAD_BYTES=921600",
    )) + "\n"


def paired_log(profile="passthrough", crash=False):
    obs = {
        "actual_graph_qos_verified": True,
        "synthetic_calibration": CALIBRATION[profile],
        "synthetic_observer_counts": {"info": 900, "raw": 899, "rect": 899},
        "synthetic_published_image_count": 900,
        "raw_rect_identical_header_stamp_pairs": 899,
        "unmatched_rectified_header_stamps": 0,
        "synthetic_rect_max_header_gap_ms": 33.524,
    }
    p = {
        "profile": "synthetic",
        "device_access": "NONE",
        "synthetic_calibration_only": True,
        "physical_motion_authorized": False,
        "actual_graph_qos_verified": True,
        "observations": obs,
        "synthetic_publisher_exit_code": 0,
        "synthetic_observer_exit_code": 0,
        "component_launch_exit_code": 0,
        "component_exit_class": (
            "BLOCKED_RECTIFIER_CHILD_CRASH" if crash else
            "CLEAN_COMPONENT_SHUTDOWN"),
        "faults": (["BLOCKED_RECTIFIER_CHILD_CRASH"] if crash else []),
    }
    r = {
        "profile": "synthetic",
        "device_access": "NONE",
        "module_root_cause_verified": False,
        "physical_motion_authorized": False,
        "m23p_runner_exit": 2 if crash else 0,
        "verdict": ("BLOCKED_COMPONENT_LIFECYCLE" if crash
                    else "CLEAN_OBSERVED_UNDER_TRACE_ONLY"),
        "faults": [],
        "trace": {"process_count": 1, "diagnostic_only": True,
                  "event_counts": {"CONSTRUCTOR": 1, "DLOPEN_RETURN": 40}},
    }
    s = {
        "distortion_profile": profile,
        "source_d": D_VECTORS[profile],
        "publisher_calibration_verified": True,
        "m23r_runner_exit": 2 if crash else 0,
        "component_child_exit_code_if_reported": -11 if crash else None,
        "verdict": ("BLOCKED_M23S_COMPONENT_LIFECYCLE" if crash
                    else "SYNTHETIC_PROFILE_CLEAN_UNDER_TRACE_ONLY"),
        "faults": [],
        "device_access": "NONE",
        "production_acceptance": "BLOCKED",
        "ab_causation_verified": False,
        "synthetic_only": True,
    }
    return ("M23P_LIFECYCLE_JSON=" + json.dumps(p) + "\n"
            + "M23R_TRACE_JSON=" + json.dumps(r) + "\n"
            + "M23S_AB_RESULT_JSON=" + json.dumps(s) + "\n"
            + "M23S_DEVICE_ACCESS=NONE\n"
            + "M23S_PRODUCTION_ACCEPTANCE=BLOCKED\n")


def mutate_json(payload, marker, change):
    lines = payload.splitlines()
    for n, line in enumerate(lines):
        if line.startswith(marker):
            item = json.loads(line.split("=", 1)[1])
            change(item)
            lines[n] = marker + json.dumps(item)
            return "\n".join(lines) + "\n"
    raise AssertionError("fixture marker not found")


class M23SABContracts(unittest.TestCase):
    def test_explicit_two_profiles_only(self):
        self.assertEqual(PROFILES, ("passthrough", "rectify"))
        for profile in PROFILES:
            self.assertEqual(selected_profile(environment(profile)), profile)
        for mode in ("", "idle", "live", "raw", "nonzero", None):
            env = environment()
            if mode is None:
                env.pop("M23S_DISTORTION_PROFILE")
            else:
                env["M23S_DISTORTION_PROFILE"] = mode
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                selected_profile(env)

    def test_every_no_device_gate_mandatory(self):
        for key in environment():
            env = environment()
            env.pop(key)
            with self.subTest(key=key), self.assertRaises(ValueError):
                selected_profile(env)

    def test_global_preload_is_forbidden(self):
        with self.assertRaises(ValueError):
            selected_profile({**environment(), "LD_PRELOAD": "/tmp/a.so"})

    def test_true_publisher_calibration_markers_both_profiles(self):
        for profile in PROFILES:
            with self.subTest(profile=profile):
                result = validate_publisher_log(
                    publisher_log(profile), profile,
                    m23p.parse_m23p_publisher_log)
                self.assertEqual(result["image"], 900)
                self.assertEqual(result["info"], 900)
                self.assertEqual(result["payload_bytes"], 921600)
                self.assertEqual(result["synthetic_calibration"],
                                 CALIBRATION[profile])

    def test_profile_or_distortion_mismatch_blocks(self):
        for profile in PROFILES:
            other = "rectify" if profile == "passthrough" else "passthrough"
            for payload in (
                publisher_log(profile).replace(
                    "M23S_PUBLISHER_DISTORTION_PROFILE=" + profile,
                    "M23S_PUBLISHER_DISTORTION_PROFILE=" + other),
                publisher_log(profile).replace(
                    "M23S_PUBLISHER_D_VECTOR=" + D_VECTORS[profile],
                    "M23S_PUBLISHER_D_VECTOR=" + D_VECTORS[other]),
                publisher_log(profile).replace(
                    "M23P_SYNTHETIC_CALIBRATION=" + CALIBRATION[profile],
                    "M23P_SYNTHETIC_CALIBRATION=" + CALIBRATION[other]),
            ):
                with self.subTest(profile=profile), self.assertRaises(ValueError):
                    validate_publisher_log(
                        payload, profile, m23p.parse_m23p_publisher_log)

    def test_missing_duplicate_marker_or_missing_newline_blocks(self):
        good = publisher_log("passthrough")
        for payload in (
            good.replace("M23S_PUBLISHER_D_VECTOR=" +
                         D_VECTORS["passthrough"] + "\n", ""),
            good + "M23S_PUBLISHER_DISTORTION_PROFILE=passthrough\n",
            good.replace("M23P_PUBLISHER_COUNTS=900,900",
                         "M23P_PUBLISHER_COUNTS=899,900"),
            good.replace("M23P_PUBLISHER_PAYLOAD_BYTES=921600",
                         "M23P_PUBLISHER_PAYLOAD_BYTES=12"),
        ):
            with self.assertRaises(ValueError):
                validate_publisher_log(
                    payload, "passthrough", m23p.parse_m23p_publisher_log)

    def test_original_M23P_publisher_metrics_check_remains_strict(self):
        for profile in PROFILES:
            for damaged in (
                publisher_log(profile).replace(
                    "M23P_PUBLISHER_GRAPH_READY=YES",
                    "M23P_PUBLISHER_GRAPH_READY=NO"),
                publisher_log(profile).replace(
                    "M23P_PUBLISHER_FRAMES_STARTED=YES",
                    "M23P_PUBLISHER_FRAMES_STARTED=NO"),
            ):
                with self.assertRaises(ValueError):
                    validate_publisher_log(
                        damaged, profile, m23p.parse_m23p_publisher_log)

    def test_child_exit_exact_log_classification(self):
        self.assertEqual(child_exit_from_log(
            "[ERROR] [component_container-1]: process has died "
            "[pid 32, exit code -11]\n"), -11)
        self.assertIsNone(child_exit_from_log(
            "[INFO] [launch]: process has finished cleanly\n"))
        self.assertIsNone(child_exit_from_log(
            "[component_container-1]: process has died [exit code -11]\n"
            "[component_container-1]: process has died [exit code -11]\n"))

    def test_cpp_publisher_exact_payload_timers_and_intrinsics(self):
        original_path = (ROOT / "m23p_cpp/src/m23p_calibrated_publisher.cpp")
        if not original_path.is_file():
            original_path = Path(
                "/opt/m23p/m23p_cpp/src/m23p_calibrated_publisher.cpp")
        original = original_path.read_text()
        modified = (ROOT / "m23s_cpp/src/m23s_ab_publisher.cpp").read_text()
        self.assertEqual(
            original.split("    auto qos =", 1)[1].split("    info_.k =", 1)[0],
            modified.split("    auto qos =", 1)[1].split("    info_.k =", 1)[0])
        self.assertEqual(
            original.split("    info_.k =", 1)[1].split(
                "    // Nonzero D forces", 1)[0],
            modified.split("    info_.k =", 1)[1].split(
                "    // M23S:", 1)[0])
        self.assertEqual(
            original.split(" private:\n  void on_frame()", 1)[1].split(
                "};\n\nint main(", 1)[0],
            modified.split(" private:\n  void on_frame()", 1)[1].split(
                "};\n\nint main(", 1)[0])
        self.assertIn('getenv("M23S_DISTORTION_PROFILE")', modified)
        self.assertIn('std::strcmp(profile, "passthrough")', modified)
        self.assertIn('std::strcmp(profile, "rectify")', modified)
        self.assertIn("info_.d = {0.0, 0.0, 0.0, 0.0, 0.0}", modified)
        self.assertIn("info_.d = {-0.08, 0.01, 0.001, -0.001, 0.0}", modified)

    def test_no_new_ros_launch_or_custom_shutdown(self):
        runner = (ROOT / "m23s_rectification_ab.py").read_text()
        self.assertIn("m23p.PUBLISHER = PUBLISHER", runner)
        self.assertIn("code = m23r.run()", runner)
        self.assertIn("m23p.prohibit_devices()", runner)
        self.assertIn("m23r.preconditions(os.environ)", runner)
        self.assertIn("M23S_PRODUCTION_ACCEPTANCE=BLOCKED", runner)
        # Analyze executable syntax, not comments: the runner's own
        # explanatory text may mention the inherited ROS launch parent.
        tree = ast.parse(runner)
        banned_imports = {"subprocess", "rclpy", "serial"}
        banned_calls = {
            "subprocess.Popen", "subprocess.run", "subprocess.call",
            "os.killpg", "os.system", "serial.Serial",
        }
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn(alias.name.split(".")[0],
                                     banned_imports)
            elif isinstance(node, ast.ImportFrom):
                self.assertNotIn((node.module or "").split(".")[0],
                                 banned_imports)
            elif isinstance(node, ast.Call):
                self.assertNotIn(ast.unparse(node.func), banned_calls)
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                # A literal ROS launch command is forbidden in wrapper code,
                # but a comment mentioning that phrase is not an invocation.
                self.assertNotIn("ros2 launch", node.value)
        self.assertNotIn("usb_cam", runner)

    def test_pair_classifies_clean_passthrough_crashing_rectify(self):
        result = compare_payloads(
            paired_log("passthrough", False), paired_log("rectify", True))
        self.assertEqual(
            result["comparison"],
            "DISTORTION_BRANCH_ASSOCIATION_SUPPORTED_NOT_CAUSAL")
        self.assertEqual(result["production_acceptance"], "BLOCKED")
        self.assertIs(result["causation_verified"], False)

    def test_pair_both_sigsegv_inconclusive_shared_teardown(self):
        self.assertEqual(compare_payloads(
            paired_log("passthrough", True), paired_log("rectify", True))
            ["comparison"],
            "SHARED_COMPONENT_TEARDOWN_ASSOCIATION_SUPPORTED_NOT_CAUSAL")

    def test_pair_both_clean_never_certifies_production(self):
        result = compare_payloads(
            paired_log("passthrough", False), paired_log("rectify", False))
        self.assertEqual(result["comparison"],
                         "NO_CRASH_REPRODUCED_UNDER_TRACE")
        self.assertFalse(result["component_lifecycle_accepted"])

    def test_pair_reverse_pattern_is_inconclusive(self):
        self.assertEqual(compare_payloads(
            paired_log("passthrough", True), paired_log("rectify", False))
            ["comparison"], "REVERSE_ASSOCIATION_INCONCLUSIVE")

    def test_missing_or_duplicate_json_marker_blocks(self):
        good = paired_log()
        for bad in (good.replace("M23S_AB_RESULT_JSON=", "BAD="),
                    good + good.splitlines()[0] + "\n"):
            with self.assertRaises(ValueError):
                classify_run(bad, "passthrough")

    def test_profile_and_source_d_mismatch_blocks(self):
        good = paired_log()
        for bad in (
            mutate_json(good, "M23S_AB_RESULT_JSON=",
                        lambda d: d.update(source_d=D_VECTORS["rectify"])),
            mutate_json(good, "M23S_AB_RESULT_JSON=",
                        lambda d: d.update(distortion_profile="rectify")),
            mutate_json(good, "M23P_LIFECYCLE_JSON=",
                        lambda d: d["observations"].update(
                            synthetic_calibration=CALIBRATION["rectify"])),
        ):
            with self.assertRaises(ValueError):
                classify_run(bad, "passthrough")

    def test_failed_delivery_or_qos_blocks_before_comparison(self):
        good = paired_log()
        for mutator in (
            lambda d: d.update(actual_graph_qos_verified=False),
            lambda d: d["observations"].update(
                synthetic_observer_counts={"info": 900, "raw": 899, "rect": 0}),
            lambda d: d["observations"].update(
                raw_rect_identical_header_stamp_pairs=1),
            lambda d: d["observations"].update(
                unmatched_rectified_header_stamps=1),
            lambda d: d["observations"].update(
                synthetic_rect_max_header_gap_ms=251.0),
            lambda d: d.update(synthetic_publisher_exit_code=2),
        ):
            with self.subTest(mutator=str(mutator)):
                bad = mutate_json(good, "M23P_LIFECYCLE_JSON=", mutator)
                with self.assertRaises(ValueError):
                    classify_run(bad, "passthrough")

    def test_unknown_crash_signal_and_launch_parent_cannot_pass(self):
        bad = mutate_json(
            paired_log("rectify", True), "M23S_AB_RESULT_JSON=",
            lambda d: d.update(component_child_exit_code_if_reported=None))
        with self.assertRaises(ValueError):
            classify_run(bad, "rectify")
        bad2 = mutate_json(
            paired_log("rectify", True), "M23P_LIFECYCLE_JSON=",
            lambda d: d.update(component_launch_exit_code=-11))
        with self.assertRaises(ValueError):
            classify_run(bad2, "rectify")

    def test_device_or_production_acceptance_violation_blocks(self):
        good = paired_log()
        for bad in (
            good.replace("M23S_DEVICE_ACCESS=NONE",
                         "M23S_DEVICE_ACCESS=CAMERA"),
            mutate_json(good, "M23S_AB_RESULT_JSON=",
                        lambda d: d.update(production_acceptance="PASS")),
            mutate_json(good, "M23R_TRACE_JSON=",
                        lambda d: d.update(device_access="CAMERA")),
        ):
            with self.assertRaises(ValueError):
                classify_run(bad, "passthrough")

    def test_offline_build_contains_all_required_files(self):
        docker = (ROOT / "Dockerfile.m23s-rectification-ab").read_text()
        self.assertIn(
            "FROM localhost/leroy-sensors:m23r-shutdown-trace", docker)
        self.assertIn("M23S_NO_DEVICE_AB_IMAGE_CONTRACT=PASS", docker)
        self.assertIn("COPY m23s_cpp /opt/m23s/m23s_cpp", docker)
        for file in (
            "m23s_rectification_ab.py", "m23s_compare_ab_logs.py",
            "m23s_rectification_ab_test.py", "M23S_RECTIFICATION_AB.md",
            "Dockerfile.m23s-rectification-ab",
        ):
            with self.subTest(file=file):
                self.assertIn("COPY " + file + " /opt/m23s/" + file, docker)
        self.assertIn("--packages-select leroy_m23s_rectification_ab", docker)
        self.assertIn("test -x /opt/m23s-cpp-install", docker)

    def test_doc_exposes_strictly_isolated_independent_profile_runs(self):
        readme = (ROOT / "M23S_RECTIFICATION_AB.md").read_text()
        docker = (ROOT / "Dockerfile.m23s-rectification-ab").read_text()
        for required in (
            "--network none", "--ipc private", "--pid private", "--pull=never",
            "--cap-drop all", "--read-only",
            "--security-opt no-new-privileges",
            "M23S_DISTORTION_PROFILE=passthrough",
            "M23S_DISTORTION_PROFILE",
            "M23S_PRODUCTION_ACCEPTANCE=BLOCKED",
        ):
            self.assertIn(required, readme)
        for forbidden in (
            "--device ", "--privileged", "--network host",
            "apt-get ", "wget ", "curl ",
        ):
            self.assertNotIn(forbidden, readme)
            self.assertNotIn(forbidden, docker)

    def test_offline_pair_analyzer_does_not_initiate_ros(self):
        source = (ROOT / "m23s_compare_ab_logs.py").read_text()
        self.assertIn("def compare_payloads(", source)
        self.assertIn("M23S_PAIR_BLOCKED=", source)
        for forbidden in (
            "subprocess.", "rclpy", "ros2 ", "podman run", "ttyACM",
            "/dev/video", "os.system(",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(M23SABContracts))
    if result.wasSuccessful():
        print("M23S_NO_DEVICE_AB_SOURCE_CONTRACT=PASS", flush=True)
        print("M23S_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
