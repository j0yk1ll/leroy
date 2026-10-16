"""Read-only ROS graph identity contract for the M14 DIAGNOSTIC gate.

ROS 2 may report an endpoint before its DDS node identity is resolved.
An UNKNOWN identity is a *pending* result only, NEVER verification.
A wrong identity, wrong type, or duplicate publisher is a hard failure.

Source names are ROS graph metadata, NOT cryptographic authentication.
No ROS import or output publisher exists in this pure helper.
"""

from .m9_mppi_source import UNKNOWN_NODE, UNKNOWN_NAMESPACE


EXPECTED_GATE_NODE = 'm14_software_gate'
EXPECTED_GATE_NAMESPACE = '/'


def inspect_gate_publisher(endpoints, *, expected_type):
    """Return (verified, pending, evidence) for one M14 diagnostic output."""
    evidence = [
        (getattr(p, 'node_name', None),
         getattr(p, 'node_namespace', None),
         getattr(p, 'topic_type', None))
        for p in endpoints
    ]
    if not evidence:
        return False, True, evidence
    if len(evidence) != 1:
        return False, False, evidence

    name, namespace, topic_type = evidence[0]
    if topic_type != expected_type:
        return False, False, evidence
    if name == EXPECTED_GATE_NODE and namespace == EXPECTED_GATE_NAMESPACE:
        return True, False, evidence

    # Accept incomplete graph discovery only as time-bounded PENDING.
    # None and empty names are also unknown, not evidence of trust.
    pending_names = (EXPECTED_GATE_NODE, UNKNOWN_NODE, None, '')
    pending_namespaces = (EXPECTED_GATE_NAMESPACE, UNKNOWN_NAMESPACE, None, '')
    if (name in pending_names and namespace in pending_namespaces
            and (name != EXPECTED_GATE_NODE
                 or namespace != EXPECTED_GATE_NAMESPACE)):
        return False, True, evidence
    return False, False, evidence
