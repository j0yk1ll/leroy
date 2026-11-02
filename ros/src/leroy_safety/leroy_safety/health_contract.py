"""Versioned, source-bound health report parser. No ROS or actuator imports.

Receipt freshness is measured by the aggregator using its own steady clock.
A remotely asserted verified=True is never trusted as physical qualification.
"""
import json

COMPONENTS = ("sensors", "perception", "localization", "mcu", "stop")
SCHEMA = "leroy_health_v1"
TOPICS = {c: "/leroy/health/" + c for c in COMPONENTS}
OWNERS = {"sensors": "leroy_sensors_health", "perception": "leroy_perception_health",
          "localization": "leroy_localization_health", "mcu": "leroy_mcu_health"}
MAX_BYTES = 2048


def encode(component, *, observed, reason):
    if component not in OWNERS or type(observed) is not bool:
        raise ValueError("invalid component report")
    if type(reason) is not str or not reason or len(reason) > 128:
        raise ValueError("invalid health reason")
    return json.dumps({"schema": SCHEMA, "component": component,
                       "source": OWNERS[component], "observed": observed,
                       "verified": False, "reason": reason}, sort_keys=True)


def decode(raw, component):
    if type(raw) is not str or len(raw.encode("utf-8")) > MAX_BYTES:
        return None
    try:
        value = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if (type(value) is not dict or set(value) !=
            {"schema", "component", "source", "observed", "verified", "reason"}):
        return None
    if (value["schema"] != SCHEMA or value["component"] != component
            or component not in OWNERS or value["source"] != OWNERS[component]
            or type(value["observed"]) is not bool
            or value["verified"] is not False
            or type(value["reason"]) is not str
            or not value["reason"] or len(value["reason"]) > 128):
        return None
    return value
