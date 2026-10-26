"""Sequenced wheel/stop CDC protocol; all writes stay in bridge._send_command.

Tracks exactly one MCU transaction at a time. Missing/mismatched ACK or
deadline lapses latch a fault. ACK confirms MCU command acceptance, NOT
physical motion or a safety-rated stop. No USB/ROS dependencies.
"""
import re
import threading

_MAX_SEQ = 0xffffffff
_ACK = re.compile(r"ok (set_wheels_tx|stop_wheels_tx) seq=([0-9]+)\Z")
_MAX_ACK_AGE_NS = 150_000_000


class MotorCdcProtocol:
    def __init__(self):
        self._lock = threading.Lock()
        self._next_seq = 1
        self._pending = None
        self._fault = None
        self._last_stop_ack = None

    @property
    def healthy(self):
        with self._lock:
            return self._fault is None

    @property
    def stop_confirmed(self):
        with self._lock:
            return self._last_stop_ack is not None and self._pending is None

    def _make(self, kind, pwm, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            return None
        with self._lock:
            if self._pending is not None or self._next_seq > _MAX_SEQ:
                self._fault = "PENDING_TRANSACTION_OR_SEQUENCE_EXHAUSTED"
                return None
            if self._fault is not None and kind != "stop_wheels_tx":
                return None
            seq = self._next_seq
            self._next_seq += 1
            self._last_stop_ack = None
            self._pending = (kind, seq, now_ns + _MAX_ACK_AGE_NS)
        if kind == "stop_wheels_tx":
            return f"stop_wheels_tx {seq}\n".encode("ascii")
        return (f"set_wheels_tx {seq} " +
                " ".join(str(x) for x in pwm) + "\n").encode("ascii")

    def wheels(self, pwm, *, now_ns):
        if type(pwm) is not tuple or len(pwm) != 4 or any(
                type(x) is not int or abs(x) > 64 for x in pwm):
            return None
        return self._make("set_wheels_tx", pwm, now_ns)

    def stop(self, *, now_ns):
        # A stop must be possible even after a motion ACK fault, but not
        # while another unacknowledged command remains in flight.
        return self._make("stop_wheels_tx", None, now_ns)

    def receive(self, line, *, now_ns):
        match = _ACK.fullmatch(line)
        if match is None:
            return False
        kind, seq_text = match.groups()
        if len(seq_text) > 10 or str(int(seq_text)) != seq_text:
            with self._lock:
                self._fault = "MALFORMED_ACK"
            return True
        seq = int(seq_text)
        with self._lock:
            if (self._pending is None or
                    (kind, seq) != self._pending[:2] or
                    type(now_ns) is not int or
                    now_ns > self._pending[2]):
                self._fault = "MISSING_MISMATCHED_OR_STALE_ACK"
                return True
            self._pending = None
            if kind == "stop_wheels_tx":
                self._last_stop_ack = seq
        return True

    def expired(self, *, now_ns):
        with self._lock:
            if self._pending is not None and now_ns > self._pending[2]:
                self._fault = "MOTOR_ACK_TIMEOUT"
            return self._fault is not None

    def abort(self, reason="BRIDGE_STOP"):
        with self._lock:
            self._fault = reason
            self._pending = None
            self._last_stop_ack = None
