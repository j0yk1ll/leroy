#!/usr/bin/env python3
"""M23X passive pre-frame graph + effective output publisher parameter audit.

NO publisher/subscriber, hardware, hardware permissions, or image processing.
Called from M23P's original pre-frame graph-inspection seam, identically in
both arms. Runtime use is NOT authorized by merely building this source.
"""
import json
import os
import sys
import time

PARAM = "image_rect.enable_pub_plugins"
RAW = "image_transport/raw"
TARGET = "/camera/rectify_node"
ARMS = ("baseline", "raw")


def inspect_values(arm, node_name, param_type, names, string_array_code):
    """Pure function: never infer effective value from source/CLI options."""
    if arm not in ARMS or node_name != TARGET:
        raise ValueError("unexpected arm or node")
    if param_type != string_array_code or not isinstance(names, (tuple, list)):
        raise ValueError("parameter must be a ROS string array")
    if any(type(n) is not str or not n for n in names):
        raise ValueError("invalid plugin name")
    if len(names) != len(set(names)):
        raise ValueError("duplicate plugin entries")
    if arm == "raw":
        if list(names) != [RAW]:
            raise ValueError("raw arm effective publisher allowlist mismatch")
    elif RAW not in names or len(names) < 2:
        raise ValueError("baseline plugin fan-out not verified")
    return True


def main():
    arm = os.environ.get("M23X_ARM")
    if arm not in ARMS or os.environ.get("M23S_DISTORTION_PROFILE") != "rectify":
        print("M23X_PARAM_VERDICT=BLOCKED_PRECONDITION", flush=True)
        return 2

    import rclpy
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.utilities import get_rmw_implementation_identifier
    from rcl_interfaces.msg import ParameterType
    from m23o_rect_qos_graph import capture_graph
    from m23p_qos_graph import validate_synthetic_graph

    rclpy.init()
    node = rclpy.create_node("m23x_graph_param_inspector",
                             start_parameter_services=False,
                             enable_rosout=False)
    graph = None
    values = None
    stage = "GRAPH_QOS"
    try:
        # Strict pre-frame sequence; the inherited M23P runner rejects a
        # snapshot whose acquisition overlaps first published frames.
        error = ValueError("missing expected ROS graph endpoints")
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
            graph = capture_graph(node, get_rmw_implementation_identifier())
            try:
                validate_synthetic_graph(graph)
                error = None
                break
            except ValueError as exc:
                error = exc
            time.sleep(0.05)
        if error:
            raise error

        stage = "PARAMETER_SERVICE"
        client = AsyncParameterClient(node, TARGET)
        if not client.wait_for_services(timeout_sec=1.0):
            raise ValueError("rectifier parameter service unavailable")
        stage = "PARAMETER_RESPONSE"
        future = client.get_parameters([PARAM])
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done() or future.exception() is not None:
            raise ValueError("rectifier parameter response incomplete")
        result = future.result()
        if result is None or len(result.values) != 1:
            raise ValueError("rectifier parameter response cardinality")
        parameter = result.values[0]
        values = list(parameter.string_array_value)
        stage = "ALLOWLIST_CONTENT"
        inspect_values(arm, TARGET, parameter.type, values,
                       ParameterType.PARAMETER_STRING_ARRAY)
        print("M23P_GRAPH_JSON=" + json.dumps(graph, sort_keys=True), flush=True)
        print("M23X_PARAM_JSON=" + json.dumps({
            "arm": arm,
            "node": TARGET,
            "parameter": PARAM,
            "type": "string_array",
            "effective_values": values,
            "verified": True,
        }, sort_keys=True), flush=True)
        print("M23X_PARAM_VERDICT=PASS", flush=True)
        return 0
    except (ValueError, RuntimeError, TypeError, OSError, AttributeError) as exc:
        print("M23X_PARAM_FAILURE_STAGE=" + stage, flush=True)
        # Keep private component/host data out of exported summary.
        print("M23X_PARAM_VERDICT=BLOCKED_" + type(exc).__name__, flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
