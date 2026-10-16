"""Read-only M8 ROS graph publisher identity contract and failure evidence.

Pure Python: ROS graph metadata is NOT cryptographic source identity.
A DDS endpoint can appear before its node name/namespace are discovered.
Neither an unknown identity nor a missing endpoint may be accepted.
"""

EXPECTED_NODE = 'm8_synthetic_candidate'
EXPECTED_NAMESPACE = '/'
EXPECTED_TYPE = 'geometry_msgs/msg/Twist'
UNKNOWN_NODE = '_NODE_NAME_UNKNOWN_'
UNKNOWN_NAMESPACE = '_NODE_NAMESPACE_UNKNOWN_'


def endpoint_report(endpoints):
    """Expose discovered endpoint count/names/types in a failed audit."""
    return [
        {
            'node_name': str(getattr(ep, 'node_name', '<missing>')),
            'node_namespace': str(getattr(ep, 'node_namespace', '<missing>')),
            'topic_type': str(getattr(ep, 'topic_type', '<missing>')),
        }
        for ep in endpoints
    ]


def publisher_discovery_pending(endpoints):
    """Only an empty or ONE plausible-but-unresolved DDS endpoint can wait.

    A second endpoint, wrong type, or known wrong name/namespace is
    immediately rejected by publisher_problem(), never treated as a
    discovery placeholder. This is a transient WAIT state, NOT trust.
    """
    if not endpoints:
        return True
    if len(endpoints) != 1:
        return False
    ep = endpoints[0]
    name = getattr(ep, 'node_name', None)
    namespace = getattr(ep, 'node_namespace', None)
    typ = getattr(ep, 'topic_type', None)
    return (
        typ == EXPECTED_TYPE
        and name in (EXPECTED_NODE, UNKNOWN_NODE)
        and namespace in (EXPECTED_NAMESPACE, UNKNOWN_NAMESPACE)
        and (name == UNKNOWN_NODE or namespace == UNKNOWN_NAMESPACE)
    )


def publisher_problem(endpoints, *, required):
    """Accept only exactly one named, namespace-correct fake publisher.

    Startup may have no endpoints before discovery; this helper preserves
    that compatibility, but main() separately withholds all positive
    audit results until the publisher is *verified*. No unknown fallback.
    """
    if not endpoints and not required:
        return None
    if (len(endpoints) == 1
            and getattr(endpoints[0], 'node_name', None) == EXPECTED_NODE
            and getattr(endpoints[0], 'node_namespace', None) == EXPECTED_NAMESPACE
            and getattr(endpoints[0], 'topic_type', None) == EXPECTED_TYPE):
        return None
    return (
        f'expected exactly one node={EXPECTED_NODE!r} '
        f'namespace={EXPECTED_NAMESPACE!r} '
        f'with type={EXPECTED_TYPE!r}; '
        f'discovered_count={len(endpoints)}; '
        f'discovered={endpoint_report(endpoints)!r}'
    )
