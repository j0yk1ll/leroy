"""ACK-supervised direct commissioning writer.

No serial device is opened here. The caller supplies the sole transport
write function and an explicitly initialized persistent sequence ledger.

No ROS topics, HMAC signing, or host/container socket relay.
"""

from leroy_mcu.motor_cdc_protocol import MotorCdcProtocol
from leroy_mcu.motor_sequence_ledger import MotorSequenceLedger


SEND_PERIOD_NS = 50_000_000


class DirectCommissioningWriter:
    def __init__(self, *, ledger, write):
        if type(ledger) is not MotorSequenceLedger:
            raise TypeError("persistent MotorSequenceLedger required")
        if not callable(write):
            raise TypeError("transport writer required")

        self.protocol = MotorCdcProtocol(ledger=ledger)
        self._write = write
        self._pending = False
        self._stopping = False
        self._startup_stop_ack = False
        self._motion_sent = False
        self._last_wheel_ns = None
        self.fault = False

    def _transmit(self, packet):
        if packet is None:
            self.fault = True
            return False

        try:
            written = self._write(packet)
        except Exception:
            self.fault = True
            self.protocol.abort("TRANSPORT_WRITE_FAILED")
            return False

        if written != len(packet):
            self.fault = True
            self.protocol.abort("PARTIAL_TRANSPORT_WRITE")
            return False

        self._pending = True
        return True

    def stop(self, *, now_ns):
        """Preempt outstanding wheel transaction with explicit STOP."""
        if self._stopping:
            return False

        self._stopping = True
        packet = self.protocol.stop(now_ns=now_ns)
        return self._transmit(packet)

    def apply(self, decision, *, now_ns):
        """Consume a safety decision; never authorize motion itself."""
        if self.protocol.expired(now_ns=now_ns):
            self.fault = True

        if self.fault or not self.protocol.healthy:
            self.stop(now_ns=now_ns)
            return False

        # A fresh process must obtain an MCU STOP ACK before
        # transmitting any wheel command, even when already armed.
        if not self._startup_stop_ack:
            self.stop(now_ns=now_ns)
            return False

        if decision.stop_required or decision.state != "ARMED":
            # Before any motion, remaining DISARMED does not
            # consume the one-use session. After motion it does.
            if self._motion_sent:
                self.stop(now_ns=now_ns)
            return False

        if self._stopping:
            return False

        pwm = decision.proposed_pwm
        if (
            type(pwm) is not tuple
            or len(pwm) != 4
            or any(type(x) is not int or abs(x) > 64 for x in pwm)
        ):
            self.fault = True
            self.stop(now_ns=now_ns)
            return False

        # Do not cause a protocol fault by sending a second wheel
        # packet before the first has been acknowledged.
        if self._pending:
            return False

        if (
            self._last_wheel_ns is not None
            and now_ns - self._last_wheel_ns < SEND_PERIOD_NS
        ):
            return False

        packet = self.protocol.calibration_wheels(
            pwm, now_ns=now_ns
        )
        if not self._transmit(packet):
            self.stop(now_ns=now_ns)
            return False

        self._motion_sent = True
        self._last_wheel_ns = now_ns
        return True

    def receive(self, line, *, now_ns):
        """Feed received MCU lines to the transaction ACK parser."""
        recognized = self.protocol.receive(line, now_ns=now_ns)
        if not recognized:
            return False

        if not self.protocol.healthy:
            self.fault = True
            self._pending = False
            return True

        self._pending = False
        if (self._stopping and not self._motion_sent
                and not self._startup_stop_ack
                and self.protocol.stop_confirmed):
            self._startup_stop_ack = True
            self._stopping = False
        return True

    @property
    def stop_confirmed(self):
        return self._stopping and self.protocol.stop_confirmed

    @property
    def healthy(self):
        return not self.fault and self.protocol.healthy
