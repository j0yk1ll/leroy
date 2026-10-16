"""Deterministic M7 synthetic shadow-veto REPLAY, no ROS graph or motors."""

import argparse

from .shadow_policy import (
    DiagnosticProposal, ShadowSnapshot, ShadowWindow,
)


def snapshot(t, *, fault=None, proposal=None, map_valid=True):
    """Fabricate all timestamps: explicitly NOT sensor or controller evidence."""
    now = 100.0 + t
    if proposal is None:
        proposal = DiagnosticProposal(
            vx=0.04, vy=0.02, vz=0., wx=0., wy=0., wz=0.1,
            received_mono=now - 0.05)
    odom_age = 4.0 if fault in ('odom', 'both') else 0.05
    depth_age = 4.0 if fault in ('depth', 'both') else 0.05
    return ShadowSnapshot(
        now_mono=now, now_ros=500.0 + t,
        odom_recv=now - odom_age,
        odom_stamp=500.0 + t - odom_age,
        depth_recv=now - depth_age,
        depth_stamp=500.0 + t - depth_age,
        map_recv=now - 0.05,
        map_valid=map_valid,
        proposal=proposal,
    )


def run(mode):
    if mode not in ('depth', 'odom'):
        raise ValueError('Replay mode must be depth or odom')
    policy = ShadowWindow()
    sequence = [
        (0.0, None, 'WOULD_VETO_REARM'),
        (1.2, None, 'HEALTHY_DIAGNOSTIC_ONLY'),
        (10.0, mode, 'WOULD_VETO_BLOCKED_' + mode.upper()),
        (12.0, mode, 'WOULD_VETO_BLOCKED_' + mode.upper()),
        (16.0, None, 'WOULD_VETO_REARM'),
        (17.2, None, 'HEALTHY_DIAGNOSTIC_ONLY'),
    ]
    for t, fault, expected in sequence:
        got = policy.observe(snapshot(t, fault=fault))
        print(f'M7 REPLAY: t={t:0.1f} {got}', flush=True)
        if got != expected:
            raise RuntimeError(
                f'M7 replay failed at synthetic t={t}: expected {expected}, got {got}')
    # Mutations must not retain an affirmative/eligible state.
    bad = DiagnosticProposal(
        vx=0.3, vy=0., vz=0., wx=0., wy=0., wz=0.,
        received_mono=119.0)
    if policy.observe(snapshot(19.0, proposal=bad)) != 'WOULD_VETO_INVALID_LIMITS':
        raise RuntimeError('M7 replay failed to veto fictional out-of-limit candidate')
    print(
        'PASS: M7 ' + mode +
        ' synthetic shadow replay vetoed stale input and invalid proposal, '
        'then required healthy rearm; NO ROS publisher, motor gate, or '
        'hardware permission',
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('depth', 'odom'), required=True)
    args = parser.parse_args()
    run(args.mode)


if __name__ == '__main__':
    main()
