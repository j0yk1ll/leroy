"""M13 offline-only *thought experiment* for command authority/fault response.

Not a ROS node, actuator, interlock, deployment policy, permissions
engine, or verified safety function. No output from this file may be
used to enable robot motion or to certify a physical stop.

All "evidence" booleans are fictional scenario inputs; ROS graph
names, an M11 synthetic PASS, or operator-entered flags DO NOT
establish real authorization or independent physical safety.
"""

from dataclasses import dataclass, replace
import math
import os

from .m12_no_go_audit import VERDICT as LIVE_ROBOT_VERDICT


SCOPE = 'OFFLINE_NON_ACTUATING_DESIGN_THOUGHT_EXPERIMENT'
# Reuses M7's synthetic recovery *illustration*. NOT a physical requirement.
ILLUSTRATIVE_HOLDOFF_S = 1.0
POSITIVE_DIAGNOSTIC = 'MODEL_CONDITIONS_OBSERVED_DIAGNOSTIC_ONLY'


@dataclass(frozen=True)
class ImaginedSignals:
    """Hypothetical inputs; not sourced from any ROS or physical I/O."""

    now_mono: float
    proposal_present: bool = True
    proposal_receipt_fresh: bool = True
    proposed_source_authenticated: bool = False
    logical_owner_count: int = 0
    odom_fresh: bool = False
    depth_fresh: bool = False
    tf_healthy: bool = False
    independent_stop_evidence: bool = False
    manual_review_ack: bool = False


# Strict order: independent physical protection remains separate from
# all ROS/software checks. These labels describe MODEL hypothetical
# rejection, not a real inhibition, wheel stop or motor current state.
CHECKS = (
    ('independent_stop_evidence',
     'MODEL_WOULD_REJECT_INDEPENDENT_STOP_UNVERIFIED'),
    ('logical_owner_count',
     'MODEL_WOULD_REJECT_OWNER_COUNT'),
    ('proposed_source_authenticated',
     'MODEL_WOULD_REJECT_SOURCE_UNAUTHENTICATED'),
    ('odom_fresh',
     'MODEL_WOULD_REJECT_STALE_ODOM'),
    ('depth_fresh',
     'MODEL_WOULD_REJECT_STALE_DEPTH'),
    ('tf_healthy',
     'MODEL_WOULD_REJECT_BAD_TF'),
    ('proposal_present',
     'MODEL_WOULD_REJECT_NO_PROPOSAL'),
    ('proposal_receipt_fresh',
     'MODEL_WOULD_REJECT_STALE_PROPOSAL'),
)


def reasons(signals):
    """Return all hypothetical design rejects; no motor decision exists."""
    if not isinstance(signals, ImaginedSignals):
        raise ValueError('M13 requires typed synthetic inputs')
    if (isinstance(signals.now_mono, bool)
            or not isinstance(signals.now_mono, (float, int))
            or not math.isfinite(signals.now_mono)
            or signals.now_mono < 0):
        raise ValueError('M13 synthetic clock must be finite and >=0')
    if (type(signals.logical_owner_count) is not int
            or signals.logical_owner_count < 0):
        raise ValueError('M13 owner count must be nonnegative integer')
    for key in ('proposal_present', 'proposal_receipt_fresh',
                'proposed_source_authenticated', 'odom_fresh',
                'depth_fresh', 'tf_healthy', 'independent_stop_evidence',
                'manual_review_ack'):
        if type(getattr(signals, key)) is not bool:
            raise ValueError('M13 input must be Boolean: ' + key)
    found = []
    for key, label in CHECKS:
        if key == 'logical_owner_count':
            bad = signals.logical_owner_count != 1
        else:
            bad = not getattr(signals, key)
        if bad:
            found.append(label)
    return tuple(found)


class DesignLatch:
    """Synthetic diagnostic trace only. Always starts latched.

    A hypothetical positive diagnostic label is NEVER a motion permit:
    actual hardware protection, authentication, and timing remain open
    M12 gates no matter what Boolean values a test supplies.
    """

    def __init__(self):
        self.fault_latched = True
        self.healthy_since = None
        self.last_time = None

    def observe(self, signals):
        failures = reasons(signals)
        now = signals.now_mono
        if self.last_time is not None and now <= self.last_time:
            self.fault_latched = True
            self.healthy_since = None
            self.last_time = now
            return 'MODEL_WOULD_REJECT_NONMONOTONIC_CLOCK'
        self.last_time = now
        if failures:
            self.fault_latched = True
            self.healthy_since = None
            # Priority: missing independent stop is the first fault.
            return failures[0]
        if self.fault_latched:
            if not signals.manual_review_ack:
                return 'MODEL_WOULD_HOLD_MANUAL_REVIEW'
            self.fault_latched = False
            self.healthy_since = now
            return 'MODEL_WOULD_HOLD_RECOVERY'
        if self.healthy_since is None:
            self.fault_latched = True
            return 'MODEL_WOULD_HOLD_MANUAL_REVIEW'
        if now - self.healthy_since < ILLUSTRATIVE_HOLDOFF_S:
            return 'MODEL_WOULD_HOLD_RECOVERY'
        return POSITIVE_DIAGNOSTIC


# All fields below are deliberately imagined, including "stop
# evidence" and authentication. Source metadata from DDS is not proof.
IMAGINED_BASELINE = ImaginedSignals(
    now_mono=10.0,
    logical_owner_count=1,
    proposed_source_authenticated=True,
    odom_fresh=True,
    depth_fresh=True,
    tf_healthy=True,
    independent_stop_evidence=True,
)


def scenario_matrix():
    """Stateless fault coverage; never reads hardware or ROS graph."""
    changes = (
        ('physical_stop_unknown', {'independent_stop_evidence': False}),
        ('duplicate_command_owner', {'logical_owner_count': 2}),
        ('absent_command_owner', {'logical_owner_count': 0}),
        ('untrusted_source', {'proposed_source_authenticated': False}),
        ('stale_odom', {'odom_fresh': False}),
        ('stale_depth', {'depth_fresh': False}),
        ('tf_loss', {'tf_healthy': False}),
        ('no_proposal', {'proposal_present': False}),
        ('stale_proposal', {'proposal_receipt_fresh': False}),
    )
    matrix = []
    for name, overrides in changes:
        row = replace(IMAGINED_BASELINE, **overrides)
        checks = reasons(row)
        if not checks:
            raise RuntimeError('M13 fault case unexpectedly has no rejection')
        matrix.append((name, checks))
    return tuple(matrix)


def report():
    """No-GO always; positive trace labels confer ZERO real authority."""
    matrix = scenario_matrix()
    timeline = DesignLatch()
    first = timeline.observe(IMAGINED_BASELINE)
    second = timeline.observe(replace(
        IMAGINED_BASELINE, now_mono=10.1, manual_review_ack=True))
    third = timeline.observe(replace(IMAGINED_BASELINE, now_mono=11.2))
    if (first != 'MODEL_WOULD_HOLD_MANUAL_REVIEW'
            or second != 'MODEL_WOULD_HOLD_RECOVERY'
            or third != POSITIVE_DIAGNOSTIC):
        raise RuntimeError('M13 simulated latch contract changed')
    return {
        'verdict': LIVE_ROBOT_VERDICT,
        'scope': SCOPE,
        'scenarios': matrix,
        'synthetic_rearm_example': (first, second, third),
        'unclosed_gates': ('G01', 'G02', 'G03', 'G04', 'G05', 'G06',
                           'G07', 'G08', 'G09', 'G10', 'G11'),
    }


def main(args=None):
    """Print synthetic model outcomes ONLY, with offline environment."""
    del args
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError('M13 requires disconnected offline environment')
    result = report()
    print('M13 DESIGN: ' + result['verdict'], flush=True)
    print('M13 SCOPE: ' + result['scope'], flush=True)
    for name, faults in result['scenarios']:
        print('M13 CASE: ' + name + ' => ' + ','.join(faults), flush=True)
    print('M13 REARM EXAMPLE: ' +
          ' -> '.join(result['synthetic_rearm_example']), flush=True)
    print('M13 M12-GATES-STILL-OPEN: ' +
          ','.join(result['unclosed_gates']), flush=True)
    print(
        'PASS: M13 offline design thought experiment rejects all 9 '
        'fictional fault cases; NO motor permission, hardware '
        'safe-stop proof or release gate closure',
        flush=True,
    )


if __name__ == '__main__':
    main()
