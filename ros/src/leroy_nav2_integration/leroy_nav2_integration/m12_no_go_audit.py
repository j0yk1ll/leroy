"""M12: permanently non-authorizing offline hardware-readiness gap report.

This module has NO ROS imports, motor subscribers/publishers, network
access, device controls or hardware enable switch. It is a static
evidence-inventory utility, not an interlock, safety assessment signoff,
or replacement for robot-specific risk assessment.

It MUST never emit a GO / permit / arm decision.
"""

from dataclasses import dataclass
import os


VERDICT = 'NO_GO_LIVE_ROBOT_NOT_AUTHORIZED'
SCOPE = 'DISCONNECTED_SYNTHETIC_READINESS_REVIEW_ONLY'


@dataclass(frozen=True)
class ReleaseGate:
    code: str
    subject: str
    evidence_state: str
    reason: str
    closure: str
    source: str


# Status is deliberately NOT writable from ROS, CLI flags or YAML. An
# operator-supplied checkbox can never authorize a real motor path.
GATES = (
    ReleaseGate(
        'G01', 'Deployment policy and graph separation', 'BLOCKED',
        'Nav2 hardware.disabled.yaml has enabled: false; all M1-M11 '
        'controller output remains disconnected and diagnostics-only.',
        'Separate explicitly reviewed deployment design and graph '
        'ownership, without using synthetic tests as authority.',
        'ros/src/leroy_nav2/config/hardware.disabled.yaml'),
    ReleaseGate(
        'G02', 'Independent motor inhibit and emergency stop', 'UNVERIFIED',
        'Existing bridge software watchdogs/timeouts and capped PWM '
        'are not independent E-stop, power isolation, measured braking '
        'or verified drive safe-state evidence.',
        'Qualified physical stop architecture and independent fault '
        'verification, including loss of ROS compute, transport, '
        'drive power and operator emergency-stop availability.',
        'ros/src/leroy_bridge/README.md'),
    ReleaseGate(
        'G03', 'Command ownership and stale-input inhibition', 'BLOCKED',
        'M11 hypothetical shadow labels do not inhibit actual MPPI '
        'diagnostic proposals; 39 depth and 35 odom nonzero samples '
        'were received after the first synthetic stale-sensor veto.',
        'Specify and independently validate application command '
        'authority, freshness, arbitration, stopping and recovery '
        'requirements, distinct from independent physical protections.',
        'ros/src/leroy_nav2_integration/M11_BLACKOUT_WINDOW_METRICS.md'),
    ReleaseGate(
        'G04', 'Wheel geometry, motor signs and traction', 'UNVERIFIED',
        'Nav2 simulated wheel radius, footprint and velocity limits '
        'are fictional. Bridge PWM gain and duty cap are provisional; '
        'right encoder signs are zero in current checked-in config.',
        'Measure kinematics and mechanical envelope; verify sensors, '
        'drive directions, traction and stopping envelope by a '
        'separate qualified supervised commissioning plan.',
        'ros/src/leroy_bridge/config/leroy_bridge.yaml'),
    ReleaseGate(
        'G05', 'Metric localization, odometry and TF ownership', 'UNVERIFIED',
        'The synthetic test publishes fake odom/TF; real XRSLAM is '
        'opt-in and pose-convention validation remains gated.',
        'Validate real localization accuracy and failure detection, '
        'clock synchronization, extrinsics, single odom/TF owner '
        'and recovery when tracking is lost.',
        'ros/src/leroy_bringup/README.md'),
    ReleaseGate(
        'G06', 'Real depth and sensor coverage', 'UNVERIFIED',
        'M11 uses a 4x4 fabricated depth fixture. The physical '
        'camera extrinsics, near-field, side/rear blind spots and '
        'latency/failure distributions are not validated by it.',
        'Calibrate and assess all relevant physical sensors, '
        'environmental failure modes, acquisition timestamps and '
        'unobserved space under the intended operating envelope.',
        'ros/src/leroy_nav2_perception/README.md'),
    ReleaseGate(
        'G07', 'Footprint and inflation geometry', 'BLOCKED',
        'Synthetic 1m-square footprint and 0.65m inflation radius '
        'produce a warning against the 0.721249m circumscribed radius.',
        'Measure actual footprint and surroundings; validate costmap '
        'inflation and collision checks with physical geometry.',
        'ros/src/leroy_nav2_integration/config/m5_controller.yaml'),
    ReleaseGate(
        'G08', 'Stopping-time risk budget', 'UNVERIFIED',
        'M11 1.0983s depth and 1.4921s odom are only one-off '
        'fake-source-to-shadow-veto receipt measurements. They '
        'include neither gate nor drive/mechanical braking delay.',
        'Risk-assess the operating domain; establish required '
        'detect-inhibit-stop time and clearance with physical '
        'evidence and conservative worst-case uncertainty.',
        'ros/src/leroy_nav2_integration/M11_BLACKOUT_WINDOW_METRICS.md'),
    ReleaseGate(
        'G09', 'Real faults and recovery', 'UNVERIFIED',
        'M10/M11 fake sensor outages do not cover disconnected '
        'actuators, stalled processes, packet loss, time steps, '
        'unexpected ROS publishers, stale commands or power loss.',
        'Independent fault-injection matrix with required safe states, '
        'restart/rearm controls and observed physical outcomes.',
        'ros/src/leroy_nav2_integration/M10_ACTIVE_FAULT_MPPI_AUDIT.md'),
    ReleaseGate(
        'G10', 'Operational risk and human exposure', 'UNVERIFIED',
        'No verified intended-use envelope, bystander clearance, '
        'risk assessment or trained-operator acceptance is provided '
        'by the synthetic integration tests.',
        'Document hazard analysis, restricted commissioning '
        'environment, competent supervision, access control and '
        'appropriate safety standards for actual intended use.',
        'ros/src/leroy_nav2/README.md'),
    ReleaseGate(
        'G11', 'Reproducible physical evidence and independent review', 'UNVERIFIED',
        'Offline Colcon and fixture PASS logs do not establish live '
        'deployment qualification, test traceability or independent '
        'approval of hardware safety functions.',
        'Collect version-pinned real-system inspection and '
        'commissioning evidence; obtain relevant independent '
        'engineering and operator review before any controlled stage.',
        'ros/src/leroy_nav2_integration/M11_BLACKOUT_WINDOW_METRICS.md'),
)


def report():
    """Return a static NO-GO gap register; never a motor permission."""
    if not GATES or len({g.code for g in GATES}) != len(GATES):
        raise RuntimeError('M12 gate inventory malformed')
    if any(g.evidence_state not in ('BLOCKED', 'UNVERIFIED') for g in GATES):
        raise RuntimeError('M12 report cannot declare a gate passed')
    return {
        'verdict': VERDICT,
        'scope': SCOPE,
        'gates': [
            {
                'code': g.code,
                'area': g.subject,
                'state': g.evidence_state,
                'why': g.reason,
                'evidence_needed': g.closure,
                'source': g.source,
            }
            for g in GATES
        ],
    }


def main(args=None):
    """Offline-only printed gap report. Runs no ROS node or action."""
    del args
    expected = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in expected.items()):
        raise RuntimeError('M12 no-go report requires isolated offline fixture')
    result = report()
    print('M12 READINESS: ' + result['verdict'], flush=True)
    print('M12 SCOPE: ' + result['scope'], flush=True)
    for g in result['gates']:
        print(f"M12 GATE: {g['code']} {g['state']} {g['area']}", flush=True)
        print('  Required evidence: ' + g['evidence_needed'], flush=True)
    print(
        'PASS: M12 offline NO-GO evidence inventory generated; '
        'live robot motion is NOT authorized and no motor path exists',
        flush=True,
    )


if __name__ == '__main__':
    main()
