# Leroy safety middleware — foundation

This package owns the canonical non-actuating readiness and health-report policies, signed-session verification, motor authorization policy and wheel-command limiting. All callers import it directly; obsolete modules and compatibility shims have been deleted. No physical stop, firmware interlock, serial motor release, or autonomous command path has been changed.

## Safety invariants

- Remote health messages cannot assert hardware verification; the ROS adapter always records `verified=False`.
- Missing, stale, future, duplicate or conflicting evidence fails closed.
- `navigation_prerequisites_met` is informational, **never** a motion permit.
- Raw `/cmd_vel` remains disconnected from the MCU bridge.
- Keep independent physical E-stop and motor-power isolation; ROS is not a certified safety controller.

## Validation

Build the workspace and source the overlay, then run the existing `test_readiness.py` and `test_readiness_ros_adapter.py` regression suites. Mode/command-gateway integration requires explicit trust-boundary review and hardware commissioning. Never turn an observation into a motor authorization by changing a boolean.

## Actuator policy ownership

`motor_authority.py`, `authorized_session.py`, and `motor_pwm.py` are canonical modules here. The MCU bridge owns serial CDC transport, GPIO feedback and trusted observations, but imports the pure safety authority for decisions. Signing tools, ingress parsing, and unit tests use direct imports; legacy module paths have been deleted. This extraction does not lift any nonzero command inhibit or implement hardware stop feedback.
