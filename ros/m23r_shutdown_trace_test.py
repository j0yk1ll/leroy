"""M23R: offline-only lifecycle trace parser and safety source contracts."""
import os
from pathlib import Path
from unittest.mock import patch
import unittest

from m23r_shutdown_trace_runner import (
    EVENT_RE, LAUNCH_MARKERS, STAGES, preconditions, summarize_trace,
)


def event(stage, ns=123456789, pid=42, tid=51,
          handle="0x1", status=0, module="/opt/ros/jazzy/lib/librectify.so"):
    return (f"M23R_TRACE_EVENT={stage} mono_ns={ns} pid={pid} tid={tid} "
            f"handle={handle} status={status} module={module}")


def traced_log(*extra):
    return "\n".join((
        "[INFO] " + LAUNCH_MARKERS[0],
        "[component_container-1] " + event(
            "CONSTRUCTOR", handle="(nil)", module="component_container"),
        "[component_container-1] " + event("DLOPEN_RETURN"),
        *extra,
        "[INFO] " + LAUNCH_MARKERS[2],
        "[INFO] " + LAUNCH_MARKERS[1],
    ))


class M23RTraceContracts(unittest.TestCase):
    def test_valid_scoped_component_lifecycle_and_loader_markers(self):
        info = summarize_trace(traced_log(
            event("DLCLOSE_BEGIN"),
            event("DLCLOSE_RETURN"),
        ))
        self.assertEqual(info["event_counts"]["CONSTRUCTOR"], 1)
        self.assertEqual(info["event_counts"]["DLCLOSE_BEGIN"], 1)
        self.assertEqual(info["event_counts"]["DLCLOSE_RETURN"], 1)
        self.assertEqual(info["dlclose_begun_not_returned"], 0)
        self.assertEqual(info["process_count"], 1)
        self.assertFalse(info["library_unload_cause_verified"])
        self.assertFalse(info["originating_caller_verified"])

    def test_incomplete_dlclose_does_not_claim_a_root_cause(self):
        info = summarize_trace(traced_log(event("DLCLOSE_BEGIN")))
        self.assertEqual(info["dlclose_begun_not_returned"], 1)
        self.assertFalse(info["library_unload_cause_verified"])

    def test_missing_or_duplicate_component_initializer_blocks(self):
        cases = (
            traced_log().replace("CONSTRUCTOR", "DLOPEN_RETURN"),
            traced_log(event(
                "CONSTRUCTOR", handle="(nil)", module="component_container")),
        )
        for payload in cases:
            with self.subTest(payload=payload[-70:]):
                with self.assertRaises(ValueError):
                    summarize_trace(payload)

    def test_second_process_trace_blocks(self):
        payload = traced_log(event("DLOPEN_RETURN", pid=99))
        with self.assertRaises(ValueError):
            summarize_trace(payload)

    def test_malformed_truncated_or_unpaired_dlclose_blocks(self):
        cases = (
            traced_log("M23R_TRACE_EVENT=DLCLOSE_BEGIN pid=42"),
            traced_log(event("DLCLOSE_RETURN")),
            traced_log(event("DLOPEN_RETURN", ns=0)),
            traced_log(event("DLOPEN_RETURN", handle="bad")),
        )
        for payload in cases:
            with self.subTest(payload=payload[-80:]):
                with self.assertRaises(ValueError):
                    summarize_trace(payload)

    def test_missing_or_duplicate_launch_start_blocks(self):
        payload = traced_log()
        for candidate in (
            payload.replace(LAUNCH_MARKERS[0], "M23R_LAUNCH_EVENT=NOT_STARTED"),
            payload + "\n" + LAUNCH_MARKERS[0],
        ):
            with self.assertRaises(ValueError):
                summarize_trace(candidate)

    def test_no_dynamic_activity_fails_closed(self):
        payload = traced_log().replace("DLOPEN_RETURN", "DESTRUCTOR")
        with self.assertRaises(ValueError):
            summarize_trace(payload)

    def test_no_global_ld_preload_is_accepted(self):
        with patch("m23r_shutdown_trace_runner.Path.is_file",
                   return_value=True):
            preconditions({"M23R_TRACE_ONLY": "1"})
            for env in (
                {},
                {"M23R_TRACE_ONLY": "0"},
                {"M23R_TRACE_ONLY": "1", "LD_PRELOAD": "/tmp/libfoo.so"},
            ):
                with self.assertRaises(ValueError):
                    preconditions(env)

    def test_missing_launch_or_trace_lib_blocks(self):
        with patch("m23r_shutdown_trace_runner.Path.is_file",
                   return_value=False):
            with self.assertRaises(ValueError):
                preconditions({"M23R_TRACE_ONLY": "1"})

    def test_launch_m23p_plugin_and_child_scoped_preload(self):
        root = Path(__file__).resolve().parent
        launch = (root / "m23r_rectifier_trace.launch.py").read_text()
        runner = (root / "m23r_shutdown_trace_runner.py").read_text()
        self.assertIn("image_proc::RectifyNode", launch)
        self.assertIn("SetEnvironmentVariable(name=\"LD_PRELOAD\"", launch)
        self.assertEqual(launch.count("ComposableNode("), 1)
        self.assertIn("m23p.LAUNCH = TRACE_LAUNCH", runner)
        self.assertIn("status = m23p.run()", runner)
        self.assertIn("M23R_PRODUCTION_ACCEPTANCE=BLOCKED", runner)
        self.assertNotIn("subprocess.Popen(", runner)

    def test_interposer_only_wraps_library_lifecycle(self):
        root = Path(__file__).resolve().parent
        source = (root / "m23r_loader_trace.c").read_text()
        self.assertIn('strcmp(name, "component_container")', source)
        self.assertIn('getenv("M23R_TRACE_ONLY")', source)
        self.assertIn('readlink("/proc/self/exe"', source)
        self.assertIn("DLCLOSE_BEGIN", source)
        self.assertIn("DLCLOSE_RETURN", source)
        for forbidden in (
            "ttyACM", "/dev/video", "usb_cam", "serial/by-id",
            "execve(", "system(", "fork(", "kill(",
        ):
            self.assertNotIn(forbidden, source)

    def test_compiler_warning_regression_in_interposer(self):
        # Keep the two -Werror fixes tied to the exact affected source.
        source = (Path(__file__).resolve().parent /
                  "m23r_loader_trace.c").read_text()
        self.assertIn("ssize_t written = write(STDERR_FILENO, buffer, length)",
                      source)
        self.assertIn("if (written < 0 || (size_t)written != length)", source)
        self.assertNotIn("(void)write(STDERR_FILENO", source)
        self.assertNotIn("handle != NULL", source)
        self.assertIn("if (enabled && !in_trace)", source)
        self.assertIn("int status = next_dlclose(handle)", source)

    def test_build_image_contains_every_file_read_by_offline_contracts(self):
        # The source tests run *inside* the image, not only from the
        # repository checkout: missing COPY directives fail builds.
        root = Path(__file__).resolve().parent
        docker = (root / "Dockerfile.m23r-shutdown-trace").read_text()
        required = (
            "m23r_loader_trace.c",
            "m23r_rectifier_trace.launch.py",
            "m23r_shutdown_trace_runner.py",
            "m23r_shutdown_trace_test.py",
            "M23R_NO_DEVICE_SHUTDOWN_TRACE.md",
            "Dockerfile.m23r-shutdown-trace",
        )
        for source in required:
            with self.subTest(source=source):
                self.assertIn(
                    f"COPY {source} /opt/m23r/{source}", docker)

    def test_offline_container_and_operator_safety_guards(self):
        root = Path(__file__).resolve().parent
        docker = (root / "Dockerfile.m23r-shutdown-trace").read_text()
        readme = (root / "M23R_NO_DEVICE_SHUTDOWN_TRACE.md").read_text()
        self.assertIn(
            "FROM localhost/leroy-sensors:m23p-rectifier-shutdown", docker)
        self.assertIn("M23R_NO_DEVICE_TRACE_IMAGE_CONTRACT=PASS", docker)
        for required in (
            "--network none", "--ipc private", "--pid private",
            "--read-only", "--cap-drop all", "--security-opt no-new-privileges",
        ):
            self.assertIn(required, readme)
        for disallowed in (
            "--device ", "--privileged", "--network host",
            "apt-get ", "curl ", "wget ",
        ):
            self.assertNotIn(disallowed, docker)
            self.assertNotIn(disallowed, readme)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(M23RTraceContracts))
    if result.wasSuccessful():
        print("M23R_NO_DEVICE_TRACE_SOURCE_CONTRACT=PASS", flush=True)
        print("M23R_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
