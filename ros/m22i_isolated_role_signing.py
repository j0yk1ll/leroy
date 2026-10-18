"""M22I: isolated TEST-ONLY signer processes; no ROS, serial or devices.

This is an interprocess credential-ROLE rehearsal, not secure provisioning.
The local parent supervisor creates and retains BOTH secrets and can ask
the fake operator signer for an arbitrary rearm; real operator identity,
OS credential confinement and trusted source authentication are absent.
The M22D production writer remains ZERO-WHEELS-only.
"""
import multiprocessing
import os
import secrets
import time

from leroy_bridge.authorized_session import (
    SessionVerifier, sign_test_motion, sign_test_rearm,
)

SIGN_TIMEOUT_S = 3.0
CHILD_JOIN_S = 3.0


def _signer_main(pipe, role, secret, session_id):
    """Spawn target, given only ONE synthetic key by a private IPC pipe."""
    try:
        while True:
            try:
                item = pipe.recv()
            except EOFError:
                return
            if item == ("close",):
                return
            if (type(item) is not tuple or len(item) != 3
                    or type(item[0]) is not str
                    or type(item[1]) is not int
                    or type(item[2]) is not int):
                pipe.send((False, "BLOCKED_BAD_SIGN_REQUEST"))
                continue
            kind, counter, at = item
            if kind != role:
                pipe.send((False, "BLOCKED_CROSS_ROLE_SIGNING"))
                continue
            if not (0 <= counter < (1 << 64) and at > 10_000_000):
                pipe.send((False, "BLOCKED_BAD_SIGN_REQUEST"))
                continue
            try:
                if role == "motion":
                    signed = sign_test_motion(
                        key=secret, session_id=session_id,
                        sequence=counter,
                        generated_ns=at - 10_000_000,
                        expires_ns=at + 150_000_000)
                elif role == "rearm":
                    signed = sign_test_rearm(
                        key=secret, session_id=session_id,
                        counter=counter,
                        generated_ns=at - 10_000_000,
                        expires_ns=at + 150_000_000)
                else:
                    pipe.send((False, "BLOCKED_UNKNOWN_ROLE"))
                    continue
                pipe.send((True, signed))
            except (ValueError, TypeError, OverflowError):
                pipe.send((False, "BLOCKED_BAD_SIGN_REQUEST"))
    finally:
        pipe.close()


class SignerUnavailable(RuntimeError):
    pass


class IsolatedSigner:
    """One sign-only child process; no raw key is returned in responses."""

    def __init__(self, *, context, role, key, session_id):
        if role not in ("motion", "rearm"):
            raise ValueError("unsupported synthetic signer role")
        parent, child = context.Pipe(duplex=True)
        self._pipe = parent
        self._process = context.Process(
            target=_signer_main,
            args=(child, role, key, session_id),
            name="m22i-" + role + "-signer",
            daemon=True)
        self._role = role
        self._closed = False
        try:
            self._process.start()
        except BaseException:
            parent.close()
            child.close()
            raise
        child.close()

    @property
    def pid(self):
        return self._process.pid

    @property
    def alive(self):
        return not self._closed and self._process.is_alive()

    def sign(self, *, kind, counter, at_ns):
        if not self.alive:
            raise SignerUnavailable("synthetic signer unavailable")
        try:
            self._pipe.send((kind, counter, at_ns))
            if not self._pipe.poll(SIGN_TIMEOUT_S):
                raise SignerUnavailable("synthetic signer response deadline")
            success, item = self._pipe.recv()
        except (EOFError, OSError, BrokenPipeError) as exc:
            raise SignerUnavailable("synthetic signer IPC failure") from exc
        if not success:
            raise PermissionError(item)
        return item

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self._process.is_alive():
                try:
                    self._pipe.send(("close",))
                except (EOFError, OSError):
                    pass
                self._process.join(CHILD_JOIN_S)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(CHILD_JOIN_S)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(CHILD_JOIN_S)
        finally:
            self._pipe.close()


class FakeSessionSupervisor:
    """Creates synthetic ephemeral keys and separate signer processes.

    This is NOT a credential vault: the parent knows both keys. Signature
    requests are an unrestricted test API, not authenticated operator
    actions. A restart replaces BOTH keys and the verifier session.
    """

    def __init__(self):
        self._ctx = multiprocessing.get_context("spawn")
        self.verifier = None
        self.motion = None
        self.operator = None
        self._closed = False
        self.rotate_session()

    @property
    def session_id(self):
        return self.verifier.session_id

    def rotate_session(self):
        if self._closed:
            raise SignerUnavailable("supervisor stopped")
        self._stop_signers()
        motion_key = secrets.token_bytes(32)
        operator_key = secrets.token_bytes(32)
        self.verifier = SessionVerifier(
            motion_key=motion_key, operator_key=operator_key)
        session = self.verifier.session_id
        try:
            self.motion = IsolatedSigner(
                context=self._ctx, role="motion",
                key=motion_key, session_id=session)
            self.operator = IsolatedSigner(
                context=self._ctx, role="rearm",
                key=operator_key, session_id=session)
        except BaseException:
            self._stop_signers()
            raise

    def _stop_signers(self):
        for name in ("motion", "operator"):
            signer = getattr(self, name, None)
            if signer is not None:
                signer.close()
                setattr(self, name, None)

    def issue_motion(self, seq, at_ns):
        if self._closed:
            raise SignerUnavailable("supervisor stopped")
        return self.motion.sign(
            kind="motion", counter=seq, at_ns=at_ns)

    def issue_rearm(self, counter, at_ns):
        if self._closed:
            raise SignerUnavailable("supervisor stopped")
        return self.operator.sign(
            kind="rearm", counter=counter, at_ns=at_ns)

    def close(self):
        if not self._closed:
            self._closed = True
            self._stop_signers()


def isolated_role_report():
    """Read-only process identity status, never includes key material."""
    return {"parent_pid": os.getpid(), "clock": "monotonic_ns",
            "device_access": False, "credentials_production_ready": False}
