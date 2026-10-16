"""M14 executable SOFTWARE diagnostic command gate: pure decision core.

Unlike the M7 shadow observer, this core selects a real output
command (bounded proposal or exact zero) for a separate unconsumed ROS
DIAGNOSTIC topic. It has no ROS or hardware imports and is NOT an
independent physical safety function.

Only the ROS M14 fixture may use this core in this milestone; neither
the bridge nor physical cmd_vel consumes its output.
"""

import math

from .fault_contract import diagnostic_state
from .shadow_policy import (
    MAX_PROPOSAL_AGE_S, REARM_HEALTHY_S,
    VX_LIMIT, VY_LIMIT, WZ_LIMIT, EPS,
)

ZERO = (0.0, 0.0, 0.0)
NONZERO_EPS = 1e-4


def proposal_valid(proposal, now):
    if proposal is None:
        return 'NO_PROPOSAL'
    fields = (
        proposal.vx, proposal.vy, proposal.vz,
        proposal.wx, proposal.wy, proposal.wz,
        proposal.received_mono, now)
    if not all(math.isfinite(v) for v in fields):
        return 'INVALID_PROPOSAL'
    age = now - proposal.received_mono
    if not 0 <= age <= MAX_PROPOSAL_AGE_S:
        return 'STALE_PROPOSAL'
    if (abs(proposal.vx) > VX_LIMIT + EPS
            or abs(proposal.vy) > VY_LIMIT + EPS
            or abs(proposal.wz) > WZ_LIMIT + EPS
            or any(abs(v) > EPS for v in (
                proposal.vz, proposal.wx, proposal.wy))):
        return 'INVALID_LIMITS'
    return None


def is_nonzero(values):
    return any(abs(v) > NONZERO_EPS for v in values)


class DiagnosticCommandGate:
    """Always returns an output tuple; never reuses a stale command.

    A time jump, lost publisher identity, missing observations, or any
    invalid proposal resets the one-second synthetic recovery dwell.
    ROS graph identity is advisory; never treat 'source_ok' as proof
    of authenticated commands from a physical robot.
    """

    def __init__(self):
        self.healthy_since = None
        self.last_time = None

    def evaluate(self, snapshot, *, source_ok, graph_ok, tf_ok):
        now = snapshot.now_mono
        if (not math.isfinite(now)
                or (self.last_time is not None and now <= self.last_time)):
            self.healthy_since = None
            # Preserve the last finite timestamp on repeated/backwards
            # ticks, so a second regression cannot silently rearm.
            self.last_time = now if math.isfinite(now) else None
            return ZERO, 'BLOCKED_CLOCK'
        self.last_time = now
        if not graph_ok:
            self.healthy_since = None
            return ZERO, 'BLOCKED_GRAPH'
        if not source_ok:
            self.healthy_since = None
            return ZERO, 'BLOCKED_SOURCE'
        health = diagnostic_state(
            now_mono=now, now_ros=snapshot.now_ros,
            odom_recv=snapshot.odom_recv, odom_stamp=snapshot.odom_stamp,
            depth_recv=snapshot.depth_recv, depth_stamp=snapshot.depth_stamp,
            map_recv=snapshot.map_recv, map_valid=snapshot.map_valid)
        if health != 'READY':
            self.healthy_since = None
            return ZERO, health
        if not tf_ok:
            self.healthy_since = None
            return ZERO, 'BLOCKED_TF'
        violation = proposal_valid(snapshot.proposal, now)
        if violation:
            self.healthy_since = None
            return ZERO, 'BLOCKED_' + violation
        if self.healthy_since is None:
            self.healthy_since = now
            return ZERO, 'BLOCKED_REARM'
        if now - self.healthy_since < REARM_HEALTHY_S:
            return ZERO, 'BLOCKED_REARM'
        p = snapshot.proposal
        return (p.vx, p.vy, p.wz), 'FORWARD_DIAGNOSTIC_ONLY'
