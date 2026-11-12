"""Persistent, exclusive MCU motor transaction sequence ownership.

No serial, ROS, GPIO or motor access.

Every reserved number is committed to disk before being returned.
A crash may skip numbers, but must not reuse a committed number.

A missing ledger may be initialized only for an explicitly commissioned
new sequence domain. Existing MCU transaction history must be reconciled
before using a new ledger against a still-running MCU.
"""

import fcntl
import json
import os
from pathlib import Path
import tempfile


MAX_SEQUENCE = 0xFFFFFFFF


class SequenceLedgerError(RuntimeError):
    pass


class MotorSequenceLedger:
    """One exclusive sequence owner per persistent ledger path."""

    def __init__(self, path, *, initialize=False):
        self.path = Path(path)
        self._lock_fd = None
        self._closed = False

        if not self.path.parent.is_dir():
            raise SequenceLedgerError("Ledger directory does not exist")

        lock_path = self.path.with_name(self.path.name + ".lock")

        try:
            self._lock_fd = os.open(
                lock_path,
                os.O_RDWR | os.O_CREAT,
                0o600,
            )
            fcntl.flock(
                self._lock_fd,
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )

            if self.path.exists():
                self._last = self._read()
            elif initialize:
                self._last = 0
                self._commit(0)
            else:
                raise SequenceLedgerError(
                    "Ledger missing; explicit initialization required"
                )
        except BaseException:
            self.close()
            raise

    def _read(self):
        try:
            data = json.loads(self.path.read_text())
        except (OSError, ValueError) as exc:
            raise SequenceLedgerError(
                "Unreadable or corrupt motor sequence ledger"
            ) from exc

        if (
            type(data) is not dict
            or set(data) != {"version", "last_reserved"}
            or type(data["version"]) is not int
            or data["version"] != 1
            or type(data["last_reserved"]) is not int
            or not 0 <= data["last_reserved"] <= MAX_SEQUENCE
        ):
            raise SequenceLedgerError(
                "Invalid motor sequence ledger contents"
            )

        return data["last_reserved"]

    def _commit(self, number):
        temporary = None
        try:
            fd, temporary = tempfile.mkstemp(
                prefix=self.path.name + ".tmp.",
                dir=self.path.parent,
            )

            with os.fdopen(fd, "w") as file:
                os.fchmod(file.fileno(), 0o600)
                json.dump(
                    {"version": 1, "last_reserved": number},
                    file,
                    sort_keys=True,
                )
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())

            os.replace(temporary, self.path)
            temporary = None

            directory_fd = os.open(
                self.path.parent,
                os.O_RDONLY | os.O_DIRECTORY,
            )
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass

    def reserve(self):
        if self._closed:
            raise SequenceLedgerError("Ledger is closed")

        if self._last >= MAX_SEQUENCE:
            raise SequenceLedgerError(
                "Motor transaction sequence exhausted"
            )

        number = self._last + 1

        try:
            self._commit(number)
        except OSError as exc:
            self.close()
            raise SequenceLedgerError(
                "Durable motor sequence reservation failed"
            ) from exc

        self._last = number
        return number

    def close(self):
        if self._closed:
            return

        self._closed = True

        if self._lock_fd is not None:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            finally:
                os.close(self._lock_fd)
                self._lock_fd = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
