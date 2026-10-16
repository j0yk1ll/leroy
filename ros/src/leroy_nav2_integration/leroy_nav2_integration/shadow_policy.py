"""M7 shadow-only diagnostic proposal classifier; NO motor output.

These labels describe hypothetical veto decisions against fake data.
The positive label is never a permission to move a physical robot.
No ROS imports, publishers, actuator hooks, or networking.
"""

from dataclasses import dataclass
import math

from .fault_contract import diagnostic_state


MAX_PROPOSAL_AGE_S = 0.4
REARM_HEALTHY_S = 1.0
# Fictional M5 Omni MPPI maxima, not measured Leroy limits.
VX_LIMIT = 0.15
VY_LIMIT = 0.15
WZ_LIMIT = 0.3
EPS = 1e-6


@dataclass(frozen=True)
class DiagnosticProposal:
    vx: float
    vy: float
    vz: float
    wx: float
    wy: float
    wz: float
    received_mono: float


@dataclass(frozen=True)
class ShadowSnapshot:
    now_mono: float
    now_ros: float
    odom_recv: float | None
    odom_stamp: float | None
    depth_recv: float | None
    depth_stamp: float | None
    map_recv: float | None
    map_valid: bool
    proposal: DiagnosticProposal | None


def classify(snapshot):
    """Classify a synthetic proposal WITHOUT authorizing any real movement.

    Sensor health is checked BEFORE inspecting the velocity proposal.
    Twist is unstamped in M5; receipt age is all that can be checked here.
    There is no source authenticity, TF safety, route coverage, or E-stop.
    """
    health = diagnostic_state(
        now_mono=snapshot.now_mono,
        now_ros=snapshot.now_ros,
        odom_recv=snapshot.odom_recv,
        odom_stamp=snapshot.odom_stamp,
        depth_recv=snapshot.depth_recv,
        depth_stamp=snapshot.depth_stamp,
        map_recv=snapshot.map_recv,
        map_valid=snapshot.map_valid,
    )
    if health != 'READY':
        return 'WOULD_VETO_' + health
    p = snapshot.proposal
    if p is None:
        return 'WOULD_VETO_NO_PROPOSAL'
    values = (p.vx, p.vy, p.vz, p.wx, p.wy, p.wz, p.received_mono)
    if not all(math.isfinite(v) for v in values):
        return 'WOULD_VETO_INVALID_PROPOSAL'
    age = snapshot.now_mono - p.received_mono
    if not 0.0 <= age <= MAX_PROPOSAL_AGE_S:
        return 'WOULD_VETO_STALE_PROPOSAL'
    if (abs(p.vx) > VX_LIMIT + EPS or
            abs(p.vy) > VY_LIMIT + EPS or
            abs(p.wz) > WZ_LIMIT + EPS or
            abs(p.vz) > EPS or abs(p.wx) > EPS or abs(p.wy) > EPS):
        return 'WOULD_VETO_INVALID_LIMITS'
    return 'HEALTHY_DIAGNOSTIC_ONLY'


class ShadowWindow:
    """Require sustained healthy observations after every simulated fault.

    This only emits text labels. No control code should import this as
    an actuator safety decision: it is neither a watchdog nor an interlock.
    """

    def __init__(self, recovery_s=REARM_HEALTHY_S):
        if not math.isfinite(recovery_s) or recovery_s <= 0:
            raise ValueError('Synthetic rearm window must be positive and finite')
        self._recovery_s = recovery_s
        self._healthy_since = None
        self._last_time = None

    def observe(self, snapshot):
        now = snapshot.now_mono
        if (not math.isfinite(now) or
                (self._last_time is not None and now < self._last_time)):
            self._healthy_since = None
            self._last_time = None
            return 'WOULD_VETO_CLOCK_RESET'
        self._last_time = now
        result = classify(snapshot)
        if result != 'HEALTHY_DIAGNOSTIC_ONLY':
            self._healthy_since = None
            return result
        if self._healthy_since is None:
            self._healthy_since = now
            return 'WOULD_VETO_REARM'
        if now - self._healthy_since < self._recovery_s:
            return 'WOULD_VETO_REARM'
        return 'HEALTHY_DIAGNOSTIC_ONLY'
