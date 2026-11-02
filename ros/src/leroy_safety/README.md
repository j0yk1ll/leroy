# Leroy safety middleware — foundation

This package contains non-actuating, deterministic readiness and health-contract policies extracted from `leroy_bringup`. Existing imports in `leroy_bringup` remain compatibility shims. No physical stop, firmware interlock, signed command, motor release, or autonomous command path has been changed.

## Safety invariants

- Remote health messages cannot assert hardware verification; the ROS adapter always records `verified=False`.
- Missing, stale, future, duplicate or conflicting evidence fails closed.
- `navigation_prerequisites_met` is informational, **never** a motion permit.
- Raw `/cmd_vel` remains disconnected from the MCU bridge.
- Keep independent physical E-stop and motor-power isolation; ROS is not a certified safety controller.

## Validation

Build the workspace and source the overlay, then run the existing `test_readiness.py` and `test_readiness_ros_adapter.py` regression suites. Further mode/command-gateway refactors require explicit trust-boundary review and hardware commissioning. Never turn an observation into a motor authorization by changing a boolean.

## Actuator policy ownership

`motor_authority.py`, `authorized_session.py`, and `motor_pwm.py` are canonical modules here. The MCU bridge owns serial CDC transport, GPIO feedback and trusted observations, but imports the pure safety authority for decisions. Signing tools, ingress parsing, and unit tests use direct imports; legacy module paths have been deleted. This extraction does not lift any nonzero command inhibit or implement hardware stop feedback.
