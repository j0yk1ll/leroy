"""Read-only M8 ROS graph publisher identity contract and failure evidence.

Intentionally independent of ROS imports so the exact endpoint rules can be
unit tested with fabricated endpoint records. It authenticates NOTHING:
ROS node names and graph metadata are not cryptographic identities.
"""

EXPECTED_NODE = 'm8_synthetic_candidate'
EXPECTED_TYPE = 'geometry_msgs/msg/Twist'


def endpoint_report(endpoints):
    """Show discovered graph endpoints without hiding duplicates or wrong names."""
    return [
        {
            'node_name': str(getattr(ep, 'node_name', '<missing>')),
            'node_namespace': str(getattr(ep, 'node_namespace', '<missing>')),
            'topic_type': str(getattr(ep, 'topic_type', '<missing>')),
        }
        for ep in endpoints
    ]


def publisher_problem(endpoints, *, required):
    """Keep the M8 invariant: one correctly named, correctly typed publisher.

    Graph discovery may be empty during startup; only the final PASS check
    requires presence. Do not widen this to accept unknown or extra nodes.
    """
    if not endpoints and not required:
        return None
    if (len(endpoints) == 1
            and getattr(endpoints[0], 'node_name', None) == EXPECTED_NODE
            and getattr(endpoints[0], 'topic_type', None) == EXPECTED_TYPE):
        return None
    return (
        f'expected exactly one node={EXPECTED_NODE!r} '
        f'with type={EXPECTED_TYPE!r}; '
        f'discovered_count={len(endpoints)}; '
        f'discovered={endpoint_report(endpoints)!r}'
    )
