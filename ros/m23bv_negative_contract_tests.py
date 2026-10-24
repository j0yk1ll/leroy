#!/usr/bin/env python3
"""M23BV no-device negative cases and additive-source parity regressions.

Source-only/mocked tests: no rclpy init, no ROS graph, no video/serial, Podman,
MCU, motors, navigation, or external recording access.
"""
import ast
import contextlib
import hashlib
import io
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock

import m23bv_qos_guard as guard
import m23bv_window as window
from m23bv_m23i_independent_preflight import ReplayInputAudit, MAX_PENDING
from m23bv_offline_tests import FakeProcess, graph, load_capture_with_stubs

HERE = Path(__file__).resolve().parent
START = 1_800_000_000_000_000_000
STEP = 33_333_333


def header(stamp, frame="camera_optical_frame"):
    return N(stamp=N(sec=stamp // 1_000_000_000,
                     nanosec=stamp % 1_000_000_000),
             frame_id=frame)


def info(stamp, *, baseline=0.0):
    return N(header=header(stamp), width=4, height=3,
             p=[200., 0., 2., baseline, 0., 200., 1.5, 0.,
                0., 0., 1., 0.],
             roi=N(do_rectify=False))


def image(stamp):
    return N(header=header(stamp), width=4, height=3,
             encoding="mono8", step=4, data=bytes(12))


def scan_long(*, count=3906, pick=None, missing_info=None):
    auditor = ReplayInputAudit(mode="monocular", provenance="synthetic")
    selected = 0
    for i in range(count):
        ts = START + i * STEP
        if i != missing_info:
            auditor.ingest("info", ts, info(ts))
        if pick is None or i in pick:
            auditor.ingest("image", ts + 1, image(ts))
            selected += 1
    return auditor, auditor.finish(), selected


def git_blob_sha(path):
    payload = path.read_bytes()
    return hashlib.sha1(
        b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()


def camera_ast(path):
    script = ast.parse(path.read_text())
    func = next(n for n in script.body
                if isinstance(n, ast.FunctionDef) and n.name == "_camera_only")
    call = next(n.value for n in func.body
                if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "camera"
                        for t in n.targets))
    return {kw.arg: kw.value for kw in call.keywords}


class M23BVLongBagNegativeTests(unittest.TestCase):
    def test_3906_info_all_images_cache_bounded(self):
        auditor, report, count = scan_long()
        self.assertEqual(count, 3906)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertEqual(report["matched_image_info_pairs"], 3906)
        self.assertEqual(report["camera_info_cache_evictions"], 3906 - MAX_PENDING)
        self.assertLessEqual(len(auditor.info_by_stamp), MAX_PENDING)
        self.assertFalse(report["physical_motion_authorized"])

    def test_3906_info_sparse_images_still_valid(self):
        chosen = set(range(0, 3906, 5))
        auditor, report, selected = scan_long(pick=chosen)
        self.assertEqual(report["counts"]["info"], 3906)
        self.assertEqual(report["matched_image_info_pairs"], selected)
        self.assertEqual(report["verdict"], "REPLAY_INPUT_CONTRACT_PASS")
        self.assertLessEqual(len(auditor.info_by_stamp), MAX_PENDING)

    def test_evictions_do_not_hide_missing_camera_info(self):
        _auditor, report, _count = scan_long(missing_info=2300)
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn("BLOCKED_UNMATCHED_CAMERA_INFO", report["faults"])
        self.assertGreater(report["camera_info_cache_evictions"], 0)

    def test_long_info_stream_cannot_mask_image_gap(self):
        chosen = set(range(3906)) - set(range(1500, 1530))
        _auditor, report, _count = scan_long(pick=chosen)
        self.assertEqual(report["verdict"], "BLOCKED_REPLAY_INPUT")
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", report["faults"])
        self.assertGreater(report["max_header_gap_ms"]["image"], 250)

    def test_bad_calibration_projection_still_blocks(self):
        audit = ReplayInputAudit(mode="monocular", provenance="synthetic")
        for i in range(5):
            ts = START + i * STEP
            audit.ingest("info", ts, info(ts, baseline=0.125 if i == 2 else 0.0))
            audit.ingest("image", ts + 1, image(ts))
        report = audit.finish()
        self.assertIn("BLOCKED_UNRECTIFIED_OR_INVALID_CAMERA_INFO", report["faults"])

    def test_independent_auditor_does_not_authorize_tracking(self):
        _auditor, report, _count = scan_long(count=15)
        for field in ("device_access", "provenance_authenticated",
                      "xrslam_executed", "xrslam_tracking_validated",
                      "extrinsics_validated", "camera_imu_physical_offset_validated",
                      "physical_motion_authorized"):
            self.assertIs(report[field], False, field)

    def test_production_window_thresholds_remain_unchanged(self):
        self.assertEqual(window.MIN_COUNTS, {
            "image": 400, "info": 400, "imu": 5600})
        self.assertEqual(window.GAP_IMAGE_NS, 250_000_000)
        self.assertEqual(window.GAP_IMU_NS, 300_000_000)


class M23BVSourceAndLifecycleTests(unittest.TestCase):
    def test_m20_and_m23h_original_sources_pinned(self):
        expected = {
            "m20_camera_only.launch.py": "a72000f97b72ec3b773879fb1066405b505f764a",
            "m20_sensor_only_run.sh": "88c300b174f6e53bd536467f2e445d22e5ed1f8c",
            "m20_combined_observer.py": "5cf2f88cb2f79be80821888401b57a6393ee5e3b",
            "m23h_bounded_capture.py": "eb40aa6e636e7ba602554136a7f8272e9b407025",
        }
        for filename, expected_sha in expected.items():
            with self.subTest(filename=filename):
                self.assertEqual(git_blob_sha(HERE / filename), expected_sha)

    def test_actual_camera_node_parameters_match_m20(self):
        old = camera_ast(HERE / "m20_camera_only.launch.py")
        new = camera_ast(HERE / "m23bv_camera_only.launch.py")
        self.assertEqual(set(old), set(new))
        for key, old_value in old.items():
            self.assertEqual(ast.dump(old_value), ast.dump(new[key]), key)

    def test_opencv_path_is_rectifier_private_only(self):
        launch = (HERE / "m23bv_camera_only.launch.py").read_text()
        self.assertIn('executable="rectify_node"', launch)
        self.assertIn('"LD_LIBRARY_PATH": ALT_OPENCV + ":"', launch)
        self.assertIn("additional_env={", launch)
        self.assertNotIn('os.environ["LD_LIBRARY_PATH"] =', launch)
        self.assertNotIn("ComposableNodeContainer(", launch)

    def test_serial_observer_starts_only_after_qos_guard(self):
        runner = (HERE / "m23bv_sensor_only_run.sh").read_text()
        self.assertLess(runner.index("ros2 launch /opt/m23bv/m23bv_camera_only.launch.py"),
                        runner.index("m23bv_qos_guard.py"))
        self.assertLess(runner.index("m23bv_qos_guard.py"),
                        runner.index("python3 -u /opt/m20/m20_combined_observer.py &"))
        self.assertIn('for name in imu camera', runner)
        self.assertIn('kill -TERM "$pid"', runner)
        self.assertNotIn("set_wheels", runner)

    def test_best_effort_rectifier_publisher_blocks(self):
        q = graph()
        q["topics"][guard.IMAGE]["publishers"][0]["qos"]["reliability"] = "BEST_EFFORT"
        with self.assertRaisesRegex(ValueError, "BLOCKED_RELIABLE_ENDPOINT"):
            guard.validate_camera_graph(q)

    def test_missing_camera_info_publisher_blocks(self):
        q = graph()
        q["topics"][guard.INFO]["publishers"] = []
        with self.assertRaisesRegex(ValueError, "BLOCKED_CAMERAINFO_PUBLISHER"):
            guard.validate_camera_graph(q)

    def test_nonvolatile_raw_endpoint_blocks(self):
        q = graph()
        q["topics"][guard.RAW]["publishers"][0]["qos"]["durability"] = "TRANSIENT_LOCAL"
        with self.assertRaisesRegex(ValueError, "BLOCKED_RELIABLE_ENDPOINT"):
            guard.validate_camera_graph(q)

    def _mock_capture_with_failed_stop(self, fail):
        module, events = load_capture_with_stubs()
        argv = []
        def spawn(cmd, **kwargs):
            argv.append(cmd)
            return FakeProcess("recorder" if len(argv) == 1 else "sensor")
        def stop(proc, signum, grace, process_group=False):
            events.append((proc.name, signum, process_group))
            return (None, False) if proc.name == fail else (0, True)
        from tempfile import TemporaryDirectory
        from pathlib import Path
        with TemporaryDirectory() as tmp:
            with mock.patch.dict(os.environ, module.EXPECTED_ENV, clear=True):
                with mock.patch.object(module, "_signal_and_wait", side_effect=stop):
                    with contextlib.redirect_stdout(io.StringIO()) as capture:
                        rc = module.capture(Path(tmp), "mock", spawn=spawn, pause=lambda _: None)
        return rc, events, capture.getvalue()

    def test_launch_tree_synthetic_source_never_executes_physical_launcher(self):
        source = (HERE / "m23bv_synthetic_launch_tree_probe.py").read_text()
        self.assertIn('ast.unparse(call)', source)
        self.assertIn('rectifier = rectifier_expression()', source)
        self.assertIn('["ros2", "launch", str(launch_path)]', source)
        self.assertIn('BLOCKED_PHYSICAL_DEVICE_PRESENT', source)
        self.assertIn('M23BV_SYNTHETIC_LAUNCH_LIFECYCLE=PASS', source)
        self.assertNotIn('ros2 launch /opt/m23bv/m23bv_camera_only.launch.py', source)
        self.assertNotIn('set_wheels(', source)

    def test_camera_ready_gate_requires_reliable_matched_frames_and_zero_exit(self):
        source = (HERE / "m23bv_synthetic_launch_tree_probe.py").read_text()
        self.assertIn('STARTUP_MODES = ("as_is", "camera_first_3s", "camera_ready")', source)
        self.assertIn('len(images.intersection(infos)) >= 10', source)
        self.assertIn('ReliabilityPolicy.RELIABLE', source)
        self.assertIn('M23BV_CAMERA_READINESS=BLOCKED_NO_RELIABLE_MATCHED_SOURCE', source)
        self.assertIn('"        if event.returncode == 0:', source)
        self.assertIn('Shutdown(reason=\'BLOCKED_CAMERA_READINESS\')', source)
        self.assertIn('M23BV_GRAPH_READY_GATED_RECTIFIER_START=PASS', source)

    def test_launch_tree_wrapper_is_pinned_and_device_free(self):
        runner = (HERE / "m23bv_synthetic_launch_tree_preflight.sh").read_text()
        self.assertIn('EXPECTED_ID=57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec', runner)
        for setting in ('--network none', '--ipc private', '--pid private',
                        '--cap-drop all', '--read-only', 'ROS_DOMAIN_ID=179'):
            with self.subTest(setting=setting):
                self.assertIn(setting, runner)
        self.assertNotIn('--device ', runner)
        self.assertNotIn('--privileged', runner)
        self.assertNotIn('--volume', runner)

    def test_separate_readiness_candidate_camera_parity(self):
        original = camera_ast(HERE / "m23bv_camera_only.launch.py")
        candidate = camera_ast(HERE / "m23bv_camera_ready_candidate.launch.py")
        self.assertEqual(set(original), set(candidate))
        for key in original:
            self.assertEqual(ast.dump(original[key]), ast.dump(candidate[key]), key)

    def test_separate_readiness_candidate_rectifier_parity(self):
        def rectifier(source):
            tree = ast.parse((HERE / source).read_text())
            f = next(n for n in tree.body
                     if isinstance(n, ast.FunctionDef) and n.name == "_camera_only")
            return next(ast.dump(n.value) for n in f.body
                        if isinstance(n, ast.Assign)
                        and any(isinstance(t, ast.Name) and t.id == "rectifier"
                                for t in n.targets))
        self.assertEqual(rectifier("m23bv_camera_only.launch.py"),
                         rectifier("m23bv_camera_ready_candidate.launch.py"))

    def test_candidate_launch_only_after_zero_readiness_and_shutdown_on_camera_loss(self):
        source = (HERE / "m23bv_camera_ready_candidate.launch.py").read_text()
        self.assertIn('M23BV_CAMERA_READY_CANDIDATE', source)
        self.assertIn('if event.returncode == 0:', source)
        self.assertIn('return [rectifier]', source)
        self.assertIn('BLOCKED_M23BV_CAMERA_READY', source)
        self.assertIn('BLOCKED_M23BV_CAMERA_EXIT', source)
        self.assertIn('OnProcessExit(target_action=camera', source)
        self.assertIn('OnProcessExit(target_action=ready', source)
        self.assertNotIn('TimerAction', source)
        self.assertNotIn('set_wheels', source)

    def test_candidate_readiness_and_image_layer_not_auto_selected(self):
        gate = (HERE / "m23bv_camera_ready_gate.py").read_text()
        docker = (HERE / "Dockerfile.m23bv-camera-ready-candidate").read_text()
        runner = (HERE / "m23bv_sensor_only_run.sh").read_text()
        self.assertIn('len(matched) >= PAIR_THRESHOLD', gate)
        self.assertIn('WAIT_SECONDS = 15.0', gate)
        self.assertIn('ReliabilityPolicy.RELIABLE', gate)
        self.assertIn('M23BV_CANDIDATE_CAMERA_READINESS=BLOCKED_TIMEOUT_OR_QOS', gate)
        self.assertNotIn('import serial', gate)
        self.assertNotIn('serial.Serial', gate)
        self.assertNotIn('create_publisher(', gate)
        self.assertIn('FROM ${M23BV_BASE}', docker)
        self.assertIn('COPY m23bv_camera_ready_gate.py', docker)
        self.assertIn('COPY m23bv_camera_ready_candidate.launch.py', docker)
        self.assertNotIn('COPY m23bv_sensor_only_run.sh', docker)
        self.assertNotIn('ENTRYPOINT', docker.splitlines()[-1])
        self.assertIn('ros2 launch /opt/m23bv/m23bv_camera_only.launch.py &', runner)
        self.assertNotIn('m23bv_camera_ready_candidate.launch.py', runner)

    def test_ready_candidate_runtime_uses_exact_candidate_ast_without_device(self):
        source = (HERE / "m23bv_ready_candidate_runtime_probe.py").read_text()
        self.assertIn('EXPECTED_BLOB = "bce577236278768308b1f475dd85e10a5da287c0"', source)
        self.assertIn('git_blob(payload) != EXPECTED_BLOB', source)
        self.assertIn('assigns["rectifier"]', source)
        self.assertIn('assigns["ready"]', source)
        self.assertIn('ast.unparse(funcs[name])', source)
        self.assertIn('len(received) >= 90 and rect_graph(observer)', source)
        self.assertIn('BLOCKED_RECTIFIER_STARTED_AFTER_FAILED_READINESS', source)
        self.assertIn('M23BV_READY_TEST_CHILDREN_REAPED=PASS', source)
        self.assertIn('BLOCKED_DEVICE_PRESENT', source)
        self.assertNotIn('ros2 launch /opt/m23bv/m23bv_camera_ready_candidate.launch.py', source)

    def test_candidate_runtime_generated_launch_returns_launch_description(self):
        # Regression for Jazzy launch error: a plain OpaqueFunction action
        # list is valid inside the candidate, never as the synthetic top-level
        # generate_launch_description() return value.
        source = (HERE / "m23bv_ready_candidate_runtime_probe.py").read_text()
        self.assertIn('f"    return LaunchDescription({returned_expr})\\n"', source)
        self.assertNotIn('f"    return {returned_expr}\\n"', source)
        self.assertIn('BLOCKED_INVALID_SYNTHETIC_LAUNCH_RETURN_TYPE', source)
        self.assertIn('M23BV_CANDIDATE_SYNTHETIC_LAUNCH_DESCRIPTION=PASS', source)

    def test_ready_candidate_runtime_wrapper_pins_image_and_has_no_devices(self):
        source = (HERE / "m23bv_ready_candidate_runtime_preflight.sh").read_text()
        self.assertIn('EXPECTED_ID=2a557fc07aead1ba680cc4024d27a5d67227cf659754114487911f6264690c40', source)
        for required in ('--network none', '--ipc private', '--pid private',
                         '--cap-drop all', '--read-only', '--pull=never',
                         'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'):
            with self.subTest(setting=required):
                self.assertIn(required, source)
        for forbidden in ('--device ', '--privileged', '--volume', '/dev/video2:/dev'):
            self.assertNotIn(forbidden, source)
        self.assertIn('healthy|best_effort|missing_info|camera_exit', source)
        self.assertIn('M23BV_PRODUCTION_OR_PHYSICAL_CAPTURE=NOT_AUTHORIZED', source)

    def test_sigint_v2_checker_preserves_readiness_logic(self):
        # Only the versioned startup shim/docstring may change; the ROS
        # graph, camera calibration checks and safety predicates stay exact.
        base = ast.parse((HERE / "m23bv_camera_ready_gate.py").read_text())
        v2 = ast.parse((HERE / "m23bv_camera_ready_gate_sigint_v2.py").read_text())
        def core(tree):
            return [ast.dump(node) for node in tree.body
                    if not isinstance(node, ast.If)
                    and not (isinstance(node, ast.Expr)
                             and isinstance(node.value, ast.Constant)
                             and isinstance(node.value.value, str))]
        self.assertEqual(core(base), core(v2))

    def test_sigint_v2_checker_exits_nonzero_without_traceback(self):
        # Execute ONLY the small __main__ AST in mocked globals. Do not
        # import rclpy or open any real topic, image or serial device.
        source = HERE / "m23bv_camera_ready_gate_sigint_v2.py"
        parsed = ast.parse(source.read_text())
        main = next(node for node in parsed.body
                    if isinstance(node, ast.If) and
                    isinstance(node.test, ast.Compare) and
                    isinstance(node.test.left, ast.Name) and
                    node.test.left.id == "__name__")
        main_code = compile(ast.fix_missing_locations(
            ast.Module(body=[main], type_ignores=[])), str(source), "exec")
        import sys
        for expected, test_run in (
            (130, lambda: (_ for _ in ()).throw(KeyboardInterrupt())),
            (0, lambda: 0),
            (2, lambda: 2),
        ):
            with self.subTest(expected=expected):
                out, err = io.StringIO(), io.StringIO()
                with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                    with self.assertRaises(SystemExit) as raised:
                        exec(main_code, {"__name__": "__main__",
                                         "run": test_run, "sys": sys})
                self.assertEqual(raised.exception.code, expected)
                if expected == 130:
                    self.assertIn(
                        "M23BV_CANDIDATE_CAMERA_READINESS=INTERRUPTED_SIGINT",
                        out.getvalue())
                else:
                    self.assertNotIn("INTERRUPTED_SIGINT", out.getvalue())
                self.assertNotIn("Traceback", out.getvalue() + err.getvalue())

    def test_sigint_v2_offline_image_layer_cannot_replace_prior_tags(self):
        docker = (HERE / "Dockerfile.m23bv-camera-ready-sigint-v2").read_text()
        builder = (HERE / "m23bv_sigint_v2_build_offline.sh").read_text()
        self.assertIn("FROM ${M23BV_SIGINT_V2_BASE}", docker)
        copy_rows = [line for line in docker.splitlines()
                     if line.lstrip().startswith("COPY ")]
        self.assertEqual(copy_rows, [
            "COPY m23bv_camera_ready_gate_sigint_v2.py /opt/m23bv/m23bv_camera_ready_gate.py"])
        self.assertNotIn("ENTRYPOINT [", docker)
        self.assertIn("EXPECTED_ID=2a557fc07aead1ba680cc4024d27a5d67227cf659754114487911f6264690c40", builder)
        self.assertIn("OUTPUT=localhost/leroy-sensors:m23bv-camera-ready-sigint-v2", builder)
        self.assertIn('podman image exists "$OUTPUT"', builder)
        for policy in ("--network none", "--pull=never", "--ipc private",
                       "--pid private", "--cap-drop all", "--read-only"):
            self.assertIn(policy, builder)
        self.assertNotIn("--device ", builder)
        self.assertNotIn("--volume ", builder)
        self.assertNotIn("--privileged", builder)

    def test_sigint_v2_runtime_requires_real_nonzero_interrupt_evidence(self):
        wrapper = (HERE / "m23bv_sigint_v2_runtime_preflight.sh").read_text()
        probe = (HERE / "m23bv_sigint_v2_runtime_probe.py").read_text()
        self.assertIn('M23BV_SIGINT_V2_EXPECTED_IMAGE_ID', wrapper)
        self.assertIn('EXPECTED_CHECKER_BLOB = "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"', probe)
        self.assertIn('BLOCKED_SIGINT_V2_CHECKER_SOURCE_MISMATCH', probe)
        self.assertIn('BLOCKED_SIGINT_V2_INTERRUPT_MARKER_MISSING', probe)
        self.assertIn('BLOCKED_SIGINT_V2_NONZERO_EXIT_130_MISSING', probe)
        self.assertIn('BLOCKED_SIGINT_V2_READINESS_TRACEBACK', probe)
        self.assertIn('BLOCKED_RECTIFIER_STARTED_AFTER_FAILED_READINESS', probe)
        self.assertIn('M23BV_SIGINT_V2_NO_TRACEBACK=PASS', wrapper)
        self.assertIn('M23BV_SIGINT_V2_INTERRUPTED_NONZERO=PASS', wrapper)
        self.assertNotIn('--device ', wrapper)
        self.assertNotIn('--volume ', wrapper)

    def test_sigint_v2_timeout_preserves_gate_and_prints_diagnostics(self):
        source = (HERE / "m23bv_sigint_v2_runtime_probe.py").read_text()
        # Do not silently increase acceptance time or reduce the required
        # image count to turn the observed healthy timeout green.
        self.assertIn('end = time.monotonic() + (19 if case == "healthy" else 25)', source)
        self.assertIn("len(received) >= 90 and rect_graph(observer)", source)
        self.assertIn('raise RuntimeError("BLOCKED_EXPECTED_LAUNCH_RESULT_TIMEOUT")', source)
        self.assertIn("M23BV_SIGINT_V2_HEALTHY_RECT_FRAMES=", source)
        self.assertIn("M23BV_SIGINT_V2_HEALTHY_GRAPH_OK=", source)
        self.assertIn("M23BV_SIGINT_V2_HEALTHY_GRAPH_JSON=", source)
        self.assertIn("M23BV_SIGINT_V2_HEALTHY_LAUNCH_PARENT_ACTIVE=", source)
        self.assertIn("json.dumps(graph, sort_keys=True)", source)
        self.assertIn("M23BV_SIGINT_V2_NO_TRACEBACK=PASS", source)

    def test_v2_order_diagnostic_never_claims_as_pinned_acceptance(self):
        probe = (HERE / "m23bv_sigint_v2_runtime_probe.py").read_text()
        runner = (HERE / "m23bv_v2_qos_order_diagnostic.sh").read_text()
        self.assertIn('QOS_ORDER_MODES = ("as_pinned", "ready_then_3s")', probe)
        self.assertIn('return [TimerAction(period=3.0, actions=[rectifier])]', probe)
        self.assertIn('M23BV_V2_QOS_ORDER_HANDLER_EXACT=FALSE', probe)
        self.assertIn('M23BV_V2_QOS_ORDER_TIMER_TEST_ONLY=TRUE', probe)
        self.assertIn('M23BV_V2_QOS_ORDER_DIAGNOSTIC=PASS_ONLY', probe)
        self.assertIn('M23BV_V2_QOS_ORDER_HEALTHY=PASS_ONLY', probe)
        # The only generic candidate-runtime PASS may be inside the
        # exact as_pinned branch. Timer mode may emit PASS_ONLY, never PASS.
        self.assertIn(
            'if qos_order_mode == "as_pinned":\n'
            '            print("M23BV_READY_CANDIDATE_RUNTIME_" + case.upper() + "=PASS", flush=True)',
            probe)
        self.assertIn(
            'if qos_order_mode == "as_pinned":\n'
            '                print("M23BV_READY_TEST_HEALTHY=PASS", flush=True)',
            probe)

        self.assertIn('M23BV_AS_PINNED_V2_HEALTHY=UNRESOLVED', probe)
        self.assertIn('len(received) >= 90 and rect_graph(observer)', probe)
        self.assertIn('end = time.monotonic() + (19 if case == "healthy" else 25)', probe)
        self.assertIn('-e M23BV_V2_QOS_ORDER_MODE=ready_then_3s', runner)
        self.assertIn('M23BV_V2_QOS_ORDER_DIAGNOSTIC=PASS_ONLY', runner)
        self.assertIn('M23BV_TEST_ONLY_TIMER_NOT_A_FIX=TRUE', runner)
        self.assertIn("BLOCKED_ONLY_HEALTHY_SUPPORTED", runner)
        self.assertNotIn('--device ', runner)
        self.assertNotIn('--volume ', runner)
        for required in ('--network none', '--read-only', '--cap-drop all',
                         '--pid private', '--ipc private', '--pull=never'):
            self.assertIn(required, runner)

    def test_v3_derived_jazzy_rectifier_has_fixed_reliable_input(self):
        source = (HERE / "m23bv_reliable_rectifier_proto/src/rectify.cpp").read_text()
        self.assertIn("source Git blob e7cbd9b1e1a0c42fff53241a0094d80988979623", source)
        self.assertIn("Redistribution and use in source and binary forms", source)
        self.assertEqual(source.count(
            "auto qos_profile = rmw_qos_profile_sensor_data;"), 1)
        self.assertEqual(source.count(
            "qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;"), 1)
        self.assertEqual(source.count(
            "qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;"), 1)
        self.assertNotIn("getTopicQosProfile(this, image_topic_)", source)
        self.assertIn("image_transport::create_camera_subscription(", source)
        self.assertIn("model_.rectifyImage(image, rect, interpolation_)", source)
        self.assertIn("RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::RectifyNode)", source)

    def test_v3_candidate_preserves_camera_and_fail_closed_handlers(self):
        before = HERE / "m23bv_camera_ready_candidate.launch.py"
        after = HERE / "m23bv_reliable_rectifier_v3_candidate.launch.py"
        original_camera = camera_ast(before)
        candidate_camera = camera_ast(after)
        self.assertEqual(set(original_camera), set(candidate_camera))
        for key in original_camera:
            with self.subTest(camera_parameter=key):
                self.assertEqual(ast.dump(original_camera[key]),
                                 ast.dump(candidate_camera[key]))
        def handlers(path):
            f = next(n for n in ast.parse(path.read_text()).body
                     if isinstance(n, ast.FunctionDef) and n.name == "_camera_only")
            return {n.name: ast.dump(n) for n in f.body
                    if isinstance(n, ast.FunctionDef)}
        self.assertEqual(handlers(before), handlers(after))
        source = after.read_text()
        self.assertIn('M23BV_RELIABLE_RECTIFIER_PROTO_V3', source)
        self.assertIn('package="m23bv_reliable_rectifier"', source)
        self.assertIn('executable="m23bv_rectify_reliable_node"', source)
        self.assertIn('name="rectify_node"', source)
        self.assertNotIn('package="image_proc", executable="rectify_node"', source)
        self.assertIn('if event.returncode == 0:', source)
        self.assertNotIn("TimerAction", source)
        self.assertNotIn("set_wheels", source)
        runner = (HERE / "m23bv_sensor_only_run.sh").read_text()
        self.assertNotIn("m23bv_reliable_rectifier_v3_candidate", runner)

    def test_v3_offline_package_and_builder_are_separate(self):
        source = HERE / "m23bv_reliable_rectifier_proto"
        cmake = (source / "CMakeLists.txt").read_text()
        package = (source / "package.xml").read_text()
        docker = (HERE / "Dockerfile.m23bv-reliable-rectifier-v3").read_text()
        builder = (HERE / "m23bv_reliable_v3_build_offline.sh").read_text()
        self.assertIn("project(m23bv_reliable_rectifier)", cmake)
        self.assertIn("rclcpp_components_register_node(", cmake)
        self.assertIn("EXECUTABLE m23bv_rectify_reliable_node", cmake)
        self.assertIn("<license>BSD-3-Clause</license>", package)
        self.assertIn("FROM ${M23BV_V3_BASE}", docker)
        self.assertIn("COPY m23bv_reliable_rectifier_proto/", docker)
        self.assertIn("COPY m23bv_reliable_rectifier_v3_candidate.launch.py", docker)
        self.assertIn("colcon build --packages-select m23bv_reliable_rectifier", docker)
        self.assertNotIn("apt-get", docker)
        self.assertNotIn("ENTRYPOINT [", docker)
        self.assertIn("EXPECTED_ID=5cdf67f984995003180741c6d14883caf97acfa2b9611e37b34f6015c0dcbc9e", builder)
        self.assertIn("OUTPUT=localhost/leroy-sensors:m23bv-reliable-rectifier-v3", builder)
        self.assertIn('podman image exists "$OUTPUT"', builder)
        for required in ("--network none", "--pull=never", "--no-cache",
                         "--cap-drop all", "--pid private", "--ipc private",
                         "--read-only"):
            self.assertIn(required, builder)
        for forbidden in ("--device ", "--volume", "--privileged"):
            self.assertNotIn(forbidden, builder)

    def test_v3_actual_cpp_and_candidate_launch_blobs_pinned(self):
        source = (HERE / "m23bv_reliable_v3_runtime_probe.py").read_text()
        self.assertIn('EXPECTED_BLOB = "a0ae75a62da39235b113e3d2c324efda5fcfc6e1"', source)
        self.assertIn('EXPECTED_CPP_BLOB = "7e5d199e633f7cf0dac266a656ae70849df07f71"', source)
        self.assertIn('EXPECTED_CHECKER_BLOB = "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"', source)
        self.assertIn('BLOCKED_V3_COMPILED_SOURCE_PIN', source)
        self.assertIn('BLOCKED_CANDIDATE_SOURCE_BLOB_MISMATCH', source)
        self.assertIn('len(received) >= 90 and rect_graph(observer)', source)
        self.assertIn('end = time.monotonic() + (19 if case == "healthy" else 25)', source)
        self.assertIn('BLOCKED_SIGINT_V2_NONZERO_EXIT_130_MISSING', source)
        self.assertIn('BLOCKED_RECTIFIER_STARTED_AFTER_FAILED_READINESS', source)
        self.assertIn('M23BV_V3_CANDIDATE_RUNTIME_', source)
        self.assertIn("M23BV_SIGINT_V2_NO_TRACEBACK=PASS", source)

    def test_v3_early_subscriber_diagnostic_cannot_masquerade_as_candidate(self):
        source = (HERE / "m23bv_reliable_v3_runtime_probe.py").read_text()
        runner = (HERE / "m23bv_reliable_v3_runtime_preflight.sh").read_text()
        self.assertIn('QOS_ORDER_MODES = ("as_pinned", "rectifier_first")', source)
        self.assertIn('TimerAction(period=4.0, actions=[camera])', source)
        self.assertIn('M23BV_V3_RELIABLE_BEFORE_CAMERA=PASS', source)
        self.assertIn('BLOCKED_V3_EARLY_READER_UNRELIABLE', source)
        self.assertIn('M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY', source)
        self.assertIn('M23BV_V3_AS_PINNED_ACCEPTANCE=NOT_ESTABLISHED_BY_DIAGNOSTIC', source)
        self.assertIn('if qos_order_mode == "as_pinned":', source)
        self.assertIn('M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID', runner)
        self.assertIn('m23bv_reliable_v3_runtime_probe.py', runner)
        self.assertIn('source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash', runner)
        self.assertIn('M23BV_RELIABLE_RECTIFIER_PROTO_V3=1', runner)
        self.assertIn('M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY', runner)
        for required in ("--network none", "--cap-drop all", "--pid private",
                         "--ipc private", "--read-only", "--pull=never"):
            self.assertIn(required, runner)
        for forbidden in ("--device ", "--volume", "--privileged"):
            self.assertNotIn(forbidden, runner)

    def test_v3_closed_bag_auditor_rejects_corrupted_synthetic_cdr(self):
        # Compile just the pure payload auditor AST; no rclpy, rosbag2,
        # hardware, subprocess or DDS graph is initialized on the host.
        source = HERE / "m23bv_reliable_v3_bag_probe.py"
        module = ast.parse(source.read_text(), filename=str(source))
        f = next(n for n in module.body
                 if isinstance(n, ast.FunctionDef)
                 and n.name == "audit_recorded_v3")
        code = compile(ast.fix_missing_locations(
            ast.Module(body=[f], type_ignores=[])), str(source), "exec")
        globals_ = {"INFO": "/camera/camera_info",
                    "IMAGE_RECT": "/camera/image_rect",
                    "IMU": "/imu/data_raw"}
        exec(code, globals_)
        audit = globals_["audit_recorded_v3"]
        from copy import deepcopy

        def fixture():
            stamps = [START + i * STEP for i in range(90)]
            raw = {stamp: i % 256 for i, stamp in enumerate(stamps)}
            infos = []
            rects = []
            for stamp in stamps:
                ci = info(stamp)
                ci.height, ci.width = 16, 16
                ci.distortion_model = "plumb_bob"
                ci.d = [0.03, -0.01, 0., 0., 0.]
                ci.k = [100., 0., 8., 0., 100., 8., 0., 0., 1.]
                ci.p = [100., 0., 8., 0., 0., 100., 8., 0., 0., 0., 1., 0.]
                infos.append(ci)
                px = raw[stamp]
                rects.append(N(header=header(stamp), width=16, height=16,
                               encoding="mono8", is_bigendian=0, step=16,
                               data=bytes([px]) * 256))
            return ({"/camera/image_rect": rects,
                     "/camera/camera_info": infos,
                     "/imu/data_raw": [N()] * 700}, raw)

        records, source_pixels = fixture()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(audit(records, source_pixels), 90)

        mutations = (
            ("bad_pixel", lambda rec, raw:
             setattr(rec["/camera/image_rect"][12], "data", bytes([255]) * 256)),
            ("bad_stamp", lambda rec, raw:
             setattr(rec["/camera/image_rect"][12], "header", header(START - STEP))),
            ("missing_info", lambda rec, raw:
             rec["/camera/camera_info"].pop()),
            ("wrong_frame", lambda rec, raw:
             setattr(rec["/camera/image_rect"][12].header, "frame_id", "wrong")),
            ("wrong_encoding", lambda rec, raw:
             setattr(rec["/camera/image_rect"][12], "encoding", "rgb8")),
            ("wrong_calibration", lambda rec, raw:
             setattr(rec["/camera/camera_info"][12], "p", [0.] * 12)),
            ("missing_image", lambda rec, raw:
             rec["/camera/image_rect"].pop()),
            ("duplicate_image", lambda rec, raw:
             rec["/camera/image_rect"].append(rec["/camera/image_rect"][0])),
            ("too_few_imu", lambda rec, raw:
             rec["/imu/data_raw"].pop()),
        )
        for name, mutate in mutations:
            with self.subTest(case=name):
                rec, raw = deepcopy(records), dict(source_pixels)
                mutate(rec, raw)
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, "BLOCKED_V3_"):
                        audit(rec, raw)

    def test_v3_closed_bag_wrapper_pins_image_and_independent_modules(self):
        wrapper = (HERE / "m23bv_reliable_v3_bag_preflight.sh").read_text()
        probe = (HERE / "m23bv_reliable_v3_bag_probe.py").read_text()
        self.assertIn("M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID", wrapper)
        self.assertIn("IMAGE=localhost/leroy-sensors:m23bv-reliable-rectifier-v3", wrapper)
        self.assertIn("base64 -w0 m23bv_m23i_independent_preflight.py", wrapper)
        self.assertIn("cat m23bv_reliable_v3_bag_probe.py", wrapper)
        self.assertIn("M23BV_V3_BAG_STAMP_INFO_LAYOUT_AND_CENTER_PIXEL=PASS", wrapper)
        self.assertIn("M23BV_DERIVED_CDR_AND_RECEIPT_EXACT_MATCH=PASS", wrapper)
        self.assertIn("M23BV_M23I_INDEPENDENT_DERIVED_AUDIT=PASS", wrapper)
        self.assertIn("M23BV_V3_BAG_DATA_INTEGRITY=PASS", wrapper)
        self.assertIn("M23BV_V3_BAG_CPP_SOURCE_PIN=PASS", wrapper)
        self.assertIn("M23BV_V3_PHYSICAL_GEOMETRIC_RECTIFICATION=UNVERIFIED", wrapper)
        for required in ("--network none", "--ipc private", "--pid private",
                         "--cap-drop all", "--read-only", "--pull=never",
                         "--tmpfs /tmp"):
            self.assertIn(required, wrapper)
        for forbidden in ("--device ", "--privileged", "--volume ", "apt-get",
                          "m23bv_sensor_only_run.sh"):
            self.assertNotIn(forbidden, wrapper)
        self.assertIn("EXPECTED_CPP_BLOB = '7e5d199e633f7cf0dac266a656ae70849df07f71'", probe)
        self.assertIn("m23bv_rectify_reliable_node", probe)
        self.assertIn("validate_camera_graph(last_graph)", probe)
        self.assertIn("BLOCKED_V3_RECTIFIED_CENTER_PIXEL", probe)
        self.assertIn("BLOCKED_V3_CAMERA_INFO_GEOMETRY", probe)
        self.assertIn("BLOCKED_V3_IMAGE_STAMP_ASSOCIATION", probe)
        self.assertIn("len(rectified) < 90", probe)
        self.assertIn("min_counts={'image': 90, 'info': 90, 'imu': 700}", probe)
        self.assertIn("M23BV_V3_BAG_CLOSED_SQLITE_CDR=PASS", probe)
        self.assertIn("M23BV_V3_BAG_DATA_INTEGRITY=PASS", probe)
        self.assertIn("M23BV_DERIVED_CDR_AND_RECEIPT_EXACT_MATCH=PASS", probe)
        self.assertIn("M23BV_M23I_INDEPENDENT_DERIVED_AUDIT=PASS", probe)

    def test_v3_geometric_oracle_spatial_fidelity_and_corruption(self):
        # Pure stdlib independent math: no cv2, ROS nodes or devices.
        import m23bv_v3_geometry_oracle as geo
        from copy import deepcopy
        self.assertEqual((geo.WIDTH, geo.HEIGHT), (80, 60))
        self.assertNotEqual(geo.D[0], 0.)
        self.assertNotEqual(geo.D[2], 0.)
        self.assertNotEqual(geo.D[3], 0.)
        self.assertEqual(geo.source_pixel(0, 0, 0), 32)
        self.assertEqual(geo.source_pixel(79, 59, 12), 241)
        self.assertEqual(geo.distorted_coordinate(geo.CX, geo.CY),
                         (geo.CX, geo.CY))
        self.assertNotEqual(geo.distorted_coordinate(0, 0), (0., 0.))
        self.assertGreater(abs(geo.reference_pixel(39, 30, 5) -
                               geo.source_pixel(39, 30, 5)), 0.)

        stamp = START
        phase = 7
        pixels = bytes(min(255, round(geo.reference_pixel(x, y, phase)))
                       for y in range(geo.HEIGHT) for x in range(geo.WIDTH))
        calibration = N(
            header=header(stamp), width=geo.WIDTH, height=geo.HEIGHT,
            distortion_model="plumb_bob", d=list(geo.D), k=list(geo.K),
            r=list(geo.R), p=list(geo.P), roi=N(do_rectify=False))
        rect = N(header=header(stamp), width=geo.WIDTH, height=geo.HEIGHT,
                 encoding="mono8", is_bigendian=0, step=geo.WIDTH,
                 data=pixels)
        metrics = geo.audit_image(rect, calibration, phase)
        self.assertEqual(metrics["pixels"], 4800)
        self.assertGreaterEqual(metrics["warped_pixels"], geo.MIN_WARPED_PIXELS)
        self.assertGreaterEqual(metrics["zero_border_pixels"],
                                geo.MIN_FULL_BORDER_PIXELS)

        def mutate_pixel(dst):
            data = bytearray(dst.data)
            data[30 * geo.WIDTH + 40] = 255 - data[30 * geo.WIDTH + 40]
            dst.data = bytes(data)

        mutations = (
            ("single_wrong_pixel", lambda im, ci, p: mutate_pixel(im)),
            ("unrectified_identity_image", lambda im, ci, p:
             setattr(im, "data", geo.source_image(p))),
            ("bad_camera_info_stamp", lambda im, ci, p:
             setattr(ci, "header", header(stamp + STEP))),
            ("bad_image_frame", lambda im, ci, p:
             setattr(im.header, "frame_id", "incorrect")),
            ("bad_distortion", lambda im, ci, p:
             setattr(ci, "d", [0.] * 5)),
            ("bad_camera_matrix", lambda im, ci, p:
             setattr(ci, "k", [0.] * 9)),
            ("wrong_rectification_matrix", lambda im, ci, p:
             setattr(ci, "r", [0.] * 9)),
            ("bad_projection", lambda im, ci, p:
             setattr(ci, "p", [0.] * 12)),
            ("bad_step", lambda im, ci, p:
             setattr(im, "step", geo.WIDTH - 1)),
            ("bad_encoding", lambda im, ci, p:
             setattr(im, "encoding", "rgb8")),
            ("wrong_phase", lambda im, ci, p: None),
            ("truncated_image", lambda im, ci, p:
             setattr(im, "data", im.data[:-1])),
        )
        for name, mutate in mutations:
            with self.subTest(case=name):
                im, ci = deepcopy(rect), deepcopy(calibration)
                mutate(im, ci, phase)
                chosen_phase = phase + 4 if name == "wrong_phase" else phase
                with self.assertRaisesRegex(ValueError,
                                            "BLOCKED_V3_GEOMETRY_"):
                    geo.audit_image(im, ci, chosen_phase)

    def test_v3_geometric_cdr_wrapper_never_touches_hardware(self):
        geo = (HERE / "m23bv_v3_geometry_oracle.py").read_text()
        probe = (HERE / "m23bv_v3_geometry_probe.py").read_text()
        wrapper = (HERE / "m23bv_v3_geometry_preflight.sh").read_text()
        self.assertNotIn("import cv2", geo)
        self.assertNotIn("import rclpy", geo)
        self.assertNotIn("initUndistortRectifyMap", geo)
        self.assertNotIn("cv2.remap", geo)
        self.assertIn("MAX_ERROR = 4.0", geo)
        self.assertIn("MIN_WARPED_PIXELS", geo)
        self.assertIn("MIN_FULL_BORDER_PIXELS", geo)
        self.assertIn("M23BV_V3_GEOMETRY_FULL_IMAGE_CDR_ORACLE=PASS", probe)
        self.assertIn("M23BV_V3_GEOMETRY_CHECKED_PIXELS=", probe)
        self.assertIn("M23BV_V3_GEOMETRY_MIN_WARPED_PIXELS_PER_FRAME=", probe)
        self.assertIn("m23bv_rectify_reliable_node", probe)
        self.assertIn("EXPECTED_CPP_BLOB = '7e5d199e633f7cf0dac266a656ae70849df07f71'", probe)
        self.assertIn("M23BV_DERIVED_CDR_AND_RECEIPT_EXACT_MATCH=PASS", probe)
        self.assertIn("M23BV_M23I_INDEPENDENT_DERIVED_AUDIT=PASS", probe)
        self.assertIn("M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID", wrapper)
        self.assertIn("base64 -w0 m23bv_v3_geometry_oracle.py", wrapper)
        self.assertIn("base64 -w0 m23bv_m23i_independent_preflight.py", wrapper)
        self.assertIn("cat m23bv_v3_geometry_probe.py", wrapper)
        for expected in ("M23BV_V3_GEOMETRY_FULL_IMAGE_CDR_ORACLE=PASS",
                         "M23BV_V3_GEOMETRY_DATA_INTEGRITY=PASS",
                         "M23BV_DDS_GRAPH_QOS=PASS",
                         "M23BV_M23I_INDEPENDENT_DERIVED_AUDIT=PASS"):
            self.assertIn(expected, wrapper)
        for token in ("--network none", "--ipc private", "--pid private",
                      "--cap-drop all", "--read-only", "--pull=never", "--tmpfs /tmp"):
            self.assertIn(token, wrapper)
        for forbidden in ("--device ", "--volume ", "--privileged", "apt-get",
                          "m23bv_sensor_only_run.sh"):
            self.assertNotIn(forbidden, wrapper)

    def test_sensor_shutdown_timeout_blocks(self):
        import signal
        rc, events, output = self._mock_capture_with_failed_stop("sensor")
        self.assertEqual(rc, 2)
        self.assertEqual(events, [("sensor", signal.SIGTERM, False),
                                  ("recorder", signal.SIGINT, True)])
        self.assertIn("BLOCKED_CAPTURE_OR_FINALIZATION", output)

    def test_recorder_finalization_timeout_blocks(self):
        import signal
        rc, events, output = self._mock_capture_with_failed_stop("recorder")
        self.assertEqual(rc, 2)
        self.assertEqual(events, [("sensor", signal.SIGTERM, False),
                                  ("recorder", signal.SIGINT, True)])
        self.assertIn("BLOCKED_CAPTURE_OR_FINALIZATION", output)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(
            __import__(__name__)))
    print("M23BV_NEGATIVE_SOURCE_REGRESSIONS=" +
          ("PASS" if result.wasSuccessful() else "FAIL"), flush=True)
    print("M23BV_NEGATIVE_TEST_DEVICE_ACCESS=NONE", flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 2)
