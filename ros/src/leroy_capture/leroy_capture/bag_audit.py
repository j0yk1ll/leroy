"""Stage 4D exact-write CDR audit for disconnected ROS bag replay.

This pure-Python object hashes the *same CDR bytes* handed to a bag writer.
Never re-serialize the ROS message to construct the expected digest: CDR
padding can differ between independent serialization calls. No ROS, device,
file, network or hardware access occurs here. Closing releases the writer.
"""
import hashlib

from leroy_capture.contract import CaptureError, TOPICS


class ExactWriteBagAudit:
    def __init__(self, write):
        if not callable(write):
            raise CaptureError("missing_bag_writer")
        self._write = write
        self.expected = {}
        self.seen = set()
        self.closed = False

    def write(self, topic, payload, stamp):
        if self.closed:
            raise CaptureError("bag_audit_closed")
        key = (topic, stamp)
        if topic not in TOPICS:
            raise CaptureError("bag_audit_unexpected_topic")
        if key in self.expected:
            raise CaptureError("bag_audit_duplicate_write")
        if not isinstance(payload, bytes):
            raise CaptureError("bag_audit_requires_cdr_bytes")
        digest = hashlib.sha256(payload).digest()
        try:
            self._write(topic, payload, stamp)
        except Exception:
            self.close()
            raise
        # Commit only AFTER the real writer accepts these exact bytes.
        self.expected[key] = digest

    def close(self):
        """Irreversibly detach the possibly bound rosbag2 writer reference."""
        self.closed = True
        self._write = None

    def verify_readback(self, topic, payload, stamp):
        if not self.closed:
            raise CaptureError("bag_audit_verify_before_writer_close")
        key = (topic, stamp)
        if key in self.seen:
            raise CaptureError("bag_audit_duplicate_readback:" + topic)
        if key not in self.expected:
            raise CaptureError("bag_audit_unexpected_readback:" + topic)
        if hashlib.sha256(bytes(payload)).digest() != self.expected[key]:
            raise CaptureError("bag_audit_cdr_payload_mismatch:" + topic)
        self.seen.add(key)

    def verify_complete(self):
        if not self.closed:
            raise CaptureError("bag_audit_not_closed")
        if self.seen != set(self.expected):
            missing = sorted(set(self.expected) - self.seen)
            if missing:
                raise CaptureError("bag_audit_missing_readback:" + missing[0][0])
            raise CaptureError("bag_audit_count_disagreement")
        return len(self.seen)
