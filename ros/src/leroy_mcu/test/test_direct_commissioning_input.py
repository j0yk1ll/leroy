"""Direct input tests; no device or serial access."""

from direct_commissioning_input import DirectCommissioningInput

T0 = 1_000_000_000


def test_initial_state_never_proposes_motion():
    control = DirectCommissioningInput()
    result = control.tick(T0, False)
    assert result.state == "DISARMED"
    assert result.stop_required


def test_arm_then_hold_then_release():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    assert control.policy.state == "DISARMED"
    assert control.arm_requested_ns == T0

    control.tick(T0 + 100_000_000, False)
    assert control.policy.state == "DISARMED"

    control.key_event("space", 1, T0 + 500_000_000)
    result = control.tick(T0 + 510_000_000, True)
    assert result.state == "ARMED"
    assert result.proposed_pwm == (4, 4, 4, 4)
    assert not result.stop_required

    released = control.key_event(
        "space", 0, T0 + 520_000_000
    )
    assert released.stop_required
    assert control.policy.state == "DISARMED"


def test_space_held_before_arm_does_not_arm():
    control = DirectCommissioningInput()
    control.key_event("space", 1, T0)
    control.key_event("arm", 1, T0 + 1)
    assert control.policy.state == "DISARMED"
    assert control.arm_requested_ns is None


def test_lost_release_event_inhibits():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    control.key_event("space", 1, T0 + 1)

    result = control.tick(T0 + 2, False)
    assert result.stop_required
    assert control.fault


def test_no_rearm_in_same_process():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    control.key_event("arm", 0, T0 + 1)
    control.key_event("space", 1, T0 + 2)
    control.key_event("space", 0, T0 + 3)
    control.key_event("arm", 1, T0 + 4)
    assert control.policy.state == "DISARMED"
    assert control.arm_requested_ns is None


def test_invalid_event_inhibits():
    control = DirectCommissioningInput()
    control.key_event("arm", 3, T0)
    assert control.fault
    assert control.policy.state == "DISARMED"


def test_stale_operator_tick_faults():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    control.key_event("space", 1, T0 + 1)

    result = control.tick(T0 + 200_000_000, True)
    assert result.state == "FAULT"
    assert result.stop_required


def test_arming_request_expires_without_space():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    result = control.tick(T0 + 10_100_000_000, False)
    assert control.fault
    assert result.stop_required


def test_key_repeat_cannot_activate_pending_arm():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    control.key_event("space", 2, T0 + 1)
    assert control.policy.state == "DISARMED"


def test_late_space_press_cannot_arm():
    control = DirectCommissioningInput()
    control.key_event("arm", 1, T0)
    control.key_event("space", 1, T0 + 10_100_000_000)
    assert control.fault
    assert control.policy.state != "ARMED"
