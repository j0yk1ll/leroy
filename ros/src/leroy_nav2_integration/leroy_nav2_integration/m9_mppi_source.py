"""M9 source-evidence check for actual MPPI's remapped diagnostic Twist.

ROS graph names are advisory metadata, NOT publisher authentication.
No hardware permission is conferred by a verified diagnostic endpoint.
"""

MPPI_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'
EXPECTED_NODE = 'controller_server'
EXPECTED_NAMESPACE = '/'
EXPECTED_TYPE = 'geometry_msgs/msg/Twist'
UNKNOWN_NODE = '_NODE_NAME_UNKNOWN_'
UNKNOWN_NAMESPACE = '_NODE_NAMESPACE_UNKNOWN_'


def inspect_mppi_source(endpoints):
    """Return (verified, pending, evidence); bad/extra publisher => neither.

    Missing graph endpoints and a lone same-type unidentified DDS endpoint
    receive a bounded discovery wait, never a positive source verdict.
    """
    evidence = [
        (getattr(e, 'node_name', None),
         getattr(e, 'node_namespace', None),
         getattr(e, 'topic_type', None))
        for e in endpoints
    ]
    if not evidence:
        return False, True, evidence
    if len(evidence) != 1:
        return False, False, evidence
    name, namespace, typ = evidence[0]
    if typ != EXPECTED_TYPE:
        return False, False, evidence
    if name == EXPECTED_NODE and namespace == EXPECTED_NAMESPACE:
        return True, False, evidence
    if (name in (EXPECTED_NODE, UNKNOWN_NODE)
            and namespace in (EXPECTED_NAMESPACE, UNKNOWN_NAMESPACE)
            and (name == UNKNOWN_NODE or namespace == UNKNOWN_NAMESPACE)):
        return False, True, evidence
    return False, False, evidence
