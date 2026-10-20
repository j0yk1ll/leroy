#!/usr/bin/env python3
"""M23Z offline source contracts + fake-log parser tests. Never starts ROS."""
import ast
import importlib.util
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent


class Contract(unittest.TestCase):
    def test_tracer_exact_delta(self):
        before = (HERE / "m23r_reference.c").read_text()
        after = (HERE / "m23z_loader_map_trace.c").read_text()
        self.assertEqual(after.count("M23Z_MAP_BEGIN=YES"), 1)
        self.assertEqual(after.count("M23Z_MAP_END count="), 1)
        self.assertEqual(after.count("M23Z_EXEC_MAP start="), 1)
        self.assertIn("__sync_bool_compare_and_swap", after)
        self.assertIn("dl_iterate_phdr(m23z_capture_exec", after)
        self.assertIn("state->count >= 400U", after)
        # Drop only the specifically scoped code additions; all other
        # original M23R tracing semantics must be byte-for-byte unchanged.
        restored = after.replace(
            "#include <dlfcn.h>\n#include <elf.h>\n#include <stdint.h>\n",
            "#include <dlfcn.h>\n", 1)
        first = restored.index("/* M23Z: snapshot executable PT_LOAD ranges")
        last = restored.index("__attribute__((constructor))", first)
        restored = restored[:first] + restored[last:]
        restored = restored.replace(
            '    event("DLCLOSE_BEGIN", path, handle, 0);\n'
            '    m23z_snapshot_before_first_close();\n',
            '    event("DLCLOSE_BEGIN", path, handle, 0);\n', 1)
        self.assertEqual(restored, before)

    def test_launch_single_difference(self):
        reference = (HERE / "m23x_raw_reference.launch.py").read_text()
        candidate = (HERE / "m23z_raw_rectifier.launch.py").read_text()
        self.assertEqual(
            candidate.replace(
                '"""M23Z raw-only mapping capture: synthetic-only',
                '"""M23X raw: synthetic-only', 1).replace(
                "/opt/m23z/libm23z_loader_map_trace.so",
                "/opt/m23r/libm23r_loader_trace.so", 1),
            reference,
        )
        self.assertIn('["image_transport/raw"]', candidate)
        ast.parse(candidate)

    def test_runner_scope_and_image(self):
        script = (HERE / "m23z_snapshot_runner.py").read_text()
        ast.parse(script)
        for expected in (
            'os.environ.get("M23Z_MAP_ONLY") != "1"',
            'os.environ.get("M23X_ARM") != "raw"',
            'm23x.ARMS["raw"] = LAUNCH',
            'm23x.ARMS["raw"] = original_launch',
            'status = m23y.run()',
            'M23Z_PRIVATE_EXEC_MAP=',
            'M23Z_MAP_BEGIN=YES',
            'M23Z_EXEC_MAP start=',
        ):
            self.assertIn(expected, script)
        self.assertNotIn("subprocess.", script)
        self.assertNotIn("os.system(", script)

    def test_image_scope(self):
        docker = (HERE / "Dockerfile.m23z-child-exec-map").read_text()
        self.assertIn(
            "FROM localhost/leroy-sensors:m23y-synthetic-drain-v2", docker)
        self.assertIn("M23Z_MAP_ONLY=1", docker)
        self.assertIn("M23X_ARM=raw", docker)
        self.assertIn("m23z_snapshot_runner.py", docker)
        self.assertIn("m23z_source_contract_test.py", docker)
        self.assertIn("m23r_reference.c", docker)
        self.assertNotIn("apt-get ", docker)
        self.assertNotIn("curl ", docker)

    def test_fake_snapshot_parser(self):
        spec = importlib.util.spec_from_file_location(
            "m23z_snapshot_runner", HERE / "m23z_snapshot_runner.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        prefix = "[component_container-1] "
        header = prefix + "M23R_TRACE_EVENT=DLCLOSE_BEGIN mono_ns=10"
        begin = prefix + "M23Z_MAP_BEGIN=YES"
        mapping = prefix + (
            "M23Z_EXEC_MAP start=0x1000 end=0x2000 module=libexample.so")
        end = prefix + "M23Z_MAP_END count=1 truncated=0"
        self.assertEqual(
            module.parse_snapshot("\n".join((header, begin, mapping, end))),
            [(0x1000, 0x2000, "libexample.so")])
        bad = (
            "\n".join((header, begin, mapping, mapping, end)),
            "\n".join((header, begin, mapping, end.replace("count=1", "count=2"))),
            "\n".join((header, begin, mapping, end.replace("truncated=0", "truncated=1"))),
            "\n".join((header, begin, mapping.replace("end=0x2000", "end=0x1000"), end)),
            "\n".join((begin, mapping, end)),
        )
        for case in bad:
            with self.assertRaises(ValueError):
                module.parse_snapshot(case)


if __name__ == "__main__":
    result = unittest.main(exit=False, verbosity=0).result
    print("M23Z_OFFLINE_SOURCE_CONTRACT=" +
          ("PASS" if result.wasSuccessful() else "BLOCKED"), flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 2)
