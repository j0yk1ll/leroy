#!/usr/bin/env python3
"""M23X source-contract tests: offline, no ROS import or runtime process."""
import ast
from pathlib import Path
import unittest

from m23x_graph_param import inspect_values

ROOT = Path(__file__).resolve().parent


def launch_tree(arm):
    text = (ROOT / ("m23x_" + arm + "_rectifier.launch.py")).read_text()
    tree = ast.parse(text)
    matches = [n for n in ast.walk(tree)
               if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
               and n.func.id == "ComposableNode"]
    if len(matches) != 1:
        raise ValueError("unexpected rectifier component cardinality")
    return tree, matches[0]


class M23XSourceContracts(unittest.TestCase):
    def test_plugin_allowlist_value_is_class_stem_not_transport_hint(self):
        self.assertTrue(inspect_values(
            "baseline", "/camera/rectify_node", 9,
            ["image_transport/raw", "image_transport/theora"], 9))
        self.assertTrue(inspect_values(
            "raw", "/camera/rectify_node", 9, ["image_transport/raw"], 9))
        for arm, values in (
            ("raw", ["raw"]),
            ("raw", ["image_transport/raw", "image_transport/theora"]),
            ("baseline", ["image_transport/raw"]),
            ("baseline", ["raw", "image_transport/zstd"]),
            ("baseline", ["image_transport/raw", "image_transport/raw"]),
        ):
            with self.subTest(arm=arm, values=values):
                with self.assertRaises(ValueError):
                    inspect_values(arm, "/camera/rectify_node", 9, values, 9)
        with self.assertRaises(ValueError):
            inspect_values("raw", "/camera/rectify_node", 4,
                           ["image_transport/raw"], 9)

    def test_launches_differ_only_by_raw_output_allowlist(self):
        baseline_tree, baseline = launch_tree("baseline")
        raw_tree, raw = launch_tree("raw")
        keys_a = {kw.arg: kw.value for kw in baseline.keywords}
        keys_b = {kw.arg: kw.value for kw in raw.keywords}
        self.assertEqual(
            set(keys_b) - set(keys_a), {"parameters"})
        self.assertNotIn("parameters", keys_a)
        self.assertEqual(ast.literal_eval(keys_b["parameters"]), [{
            "image_rect.enable_pub_plugins": ["image_transport/raw"]
        }])
        self.assertEqual(
            ast.literal_eval(keys_b["remappings"]),
            [("image", "image_raw"), ("camera_info", "camera_info")])
        self.assertEqual(
            {k: ast.dump(v) for k, v in keys_a.items()},
            {k: ast.dump(keys_b[k]) for k in keys_a})
        # Entire launch AST must match beyond the sole parameter.
        raw.keywords = [k for k in raw.keywords if k.arg != "parameters"]
        for tree in (baseline_tree, raw_tree):
            if isinstance(tree.body[0], ast.Expr):
                tree.body.pop(0)  # mode-specific module docstring only
        self.assertEqual(ast.dump(baseline_tree), ast.dump(raw_tree))

    def test_jazzy_parameter_client_api_and_sanitized_failure_stage(self):
        # rclpy Jazzy rclpy/parameter_client.py uses singular
        # AsyncParameterClient; the plural spelling silently failed
        # inside the previous captured diagnostic subprocess.
        observer = (ROOT / "m23x_graph_param.py").read_text()
        runner = (ROOT / "m23x_allowlist_ab.py").read_text()
        tree = ast.parse(observer)
        imports = [n for n in ast.walk(tree)
                   if isinstance(n, ast.ImportFrom)
                   and n.module == "rclpy.parameter_client"]
        self.assertEqual(len(imports), 1)
        self.assertEqual([alias.name for alias in imports[0].names],
                         ["AsyncParameterClient"])
        self.assertIn("client = AsyncParameterClient(node, TARGET)", observer)
        self.assertNotIn("AsyncParametersClient", observer)
        for stage in ("GRAPH_QOS", "PARAMETER_SERVICE",
                      "PARAMETER_RESPONSE", "ALLOWLIST_CONTENT"):
            self.assertIn(stage, observer)
        self.assertIn("M23X_PARAM_FAILURE_STAGE=", observer)
        self.assertIn("M23X_PARAM_FAILURE_STAGE=", runner)

    def test_only_child_tracer_and_stock_rectifier(self):
        for arm in ("baseline", "raw"):
            tree, node = launch_tree(arm)
            vals = {k.arg: k.value for k in node.keywords}
            self.assertEqual(ast.literal_eval(vals["plugin"]),
                             "image_proc::RectifyNode")
            self.assertEqual(ast.literal_eval(vals["namespace"]), "/camera")
            calls = [
                n for n in ast.walk(tree)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == "SetEnvironmentVariable"
            ]
            self.assertEqual(len(calls), 1)
            self.assertIn('TRACE_LIBRARY = "/opt/m23r/libm23r_loader_trace.so"',
                          (ROOT / ("m23x_" + arm + "_rectifier.launch.py")).read_text())

    def test_runner_fail_closed_and_inherits_m23s(self):
        runner = (ROOT / "m23x_allowlist_ab.py").read_text()
        tree = ast.parse(runner)
        self.assertIn("m23p.prohibit_devices()", runner)
        self.assertIn("m23s.run()", runner)
        self.assertIn("raise KeyboardInterrupt(", runner)
        self.assertIn("m23r.TRACE_LAUNCH = ARMS[arm]", runner)
        self.assertIn("m23p.inspect_synthetic_graph = lambda:", runner)
        for symbol in ("usb_cam", "serial.Serial", "cv::setNumThreads"):
            self.assertNotIn(symbol, runner)
        self.assertTrue(
            any(isinstance(n, ast.Import) and
                any(a.name == "subprocess" for a in n.names)
                for n in ast.walk(tree)))
        # subprocess in runner is ONLY for the passive graph inspector,
        # not for launching a second ROS/container experiment.
        calls = [n for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and isinstance(n.func.value, ast.Name)
                 and n.func.value.id == "subprocess"]
        self.assertEqual([n.func.attr for n in calls], ["run"])


if __name__ == "__main__":
    unittest.main()
