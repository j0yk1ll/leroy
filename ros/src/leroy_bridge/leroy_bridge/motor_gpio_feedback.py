"""Read-only feedback from an independent physical motor power cutoff.

Two active-high INPUTS: E-stop safety-loop healthy and motor power contactor
feedback. Hardware must remove motor power independently of software;
reading GPIO signals does NOT create that safety circuit.
Unconfigured/missing/invalid GPIO => both interlocks false.
Uses libgpiod v2 Python API when the real hardware is provisioned.
"""


class PhysicalMotorFeedback:
    def __init__(self, chip, estop_offset, motor_power_offset):
        if (type(chip) is not str or not chip.startswith("/dev/gpiochip")
                or type(estop_offset) is not int
                or type(motor_power_offset) is not int
                or estop_offset < 0 or motor_power_offset < 0
                or estop_offset == motor_power_offset):
            raise ValueError("two distinct explicit physical GPIO inputs required")
        import gpiod
        from gpiod.line import Direction, Bias
        self._gpiod = gpiod
        self._estop = estop_offset
        self._power = motor_power_offset
        self._request = gpiod.request_lines(
            chip,
            consumer="leroy-independent-power-feedback",
            config={
                (estop_offset, motor_power_offset): gpiod.LineSettings(
                    direction=Direction.INPUT,
                    bias=Bias.PULL_DOWN,
                ),
            },
        )

    def read(self):
        # Require exactly two active states from real GPIO line values.
        # Any I/O error is a deny, never a remembered healthy status.
        try:
            active = self._gpiod.line.Value.ACTIVE
            return (self._request.get_value(self._estop) == active,
                    self._request.get_value(self._power) == active)
        except Exception:
            # A Python binding/backend fault must never leave a remembered
            # "powered and E-stop released" state.
            return False, False

    def close(self):
        self._request.release()
