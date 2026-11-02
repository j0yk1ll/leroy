"""Fail-closed, non-actuating aggregation of independently verified robot health.

Subsystem adapters supply *measured* evidence using record(). This class does
not synthesize validation, inspect hardware, publish ROS, or authorize wheels.
The future lifecycle supervisor must enforce separate motion interlocks.
"""
from dataclasses import dataclass
import math
import time

COMPONENTS = ("sensors", "perception", "localization", "mcu", "stop")
DEFAULT_MAX_AGE_S = 0.5


@dataclass(frozen=True)
class HealthEvidence:
    component: str
    healthy: bool
    verified: bool
    observed_ns: int
    source: str


@dataclass(frozen=True)
class RobotReadiness:
    sensors_healthy: bool
    perception_valid: bool
    localization_valid: bool
    mcu_healthy: bool
    physical_stop_verified: bool
    navigation_prerequisites_met: bool
    failure_reasons: tuple[str, ...]


class ReadinessAggregator:
    """Continuously evaluable snapshot of fresh, attributable health evidence.

    An unhealthy or unverified report replaces prior healthy evidence. Missing,
    duplicate-source, invalid, future, and stale evidence all fail closed.
    Readiness is never a motor permit, including when all five checks pass.
    """

    def __init__(self, *, max_age_s=DEFAULT_MAX_AGE_S):
        if (type(max_age_s) not in (int, float)
                or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ValueError("max_age_s must be positive and finite")
        self._max_age_ns = int(max_age_s * 1_000_000_000)
        if self._max_age_ns == 0:
            raise ValueError("max_age_s is too small")
        self._reports = {}
        self._faults = set()

    def record(self, evidence: HealthEvidence) -> None:
        if type(evidence) is not HealthEvidence:
            raise TypeError("HealthEvidence required")
        if evidence.component not in COMPONENTS:
            raise ValueError("unknown component")
        if type(evidence.healthy) is not bool or type(evidence.verified) is not bool:
            raise TypeError("health and verification must be booleans")
        if (type(evidence.observed_ns) is not int or evidence.observed_ns < 0
                or not isinstance(evidence.source, str)
                or not evidence.source.strip()):
            raise ValueError("invalid evidence provenance")
        previous = self._reports.get(evidence.component)
        if previous is not None:
            if previous.source != evidence.source:
                self._faults.add(evidence.component)
                return
            if evidence.observed_ns <= previous.observed_ns:
                self._faults.add(evidence.component)
                return
        self._reports[evidence.component] = evidence

    def reset(self) -> None:
        """Explicitly clear reports and latched provenance faults; never rearm."""
        self._reports.clear()
        self._faults.clear()

    def evaluate(self, *, now_ns: int | None = None) -> RobotReadiness:
        if now_ns is None:
            now_ns = time.monotonic_ns()
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a nonnegative integer")
        valid = {}
        reasons = []
        for component in COMPONENTS:
            report = self._reports.get(component)
            if component in self._faults:
                reason = "provenance_fault"
            elif report is None:
                reason = "missing"
            elif not 0 <= now_ns - report.observed_ns <= self._max_age_ns:
                reason = "stale_or_future"
            elif not report.verified:
                reason = "unverified"
            elif not report.healthy:
                reason = "unhealthy"
            else:
                reason = None
            valid[component] = reason is None
            if reason is not None:
                reasons.append(f"{component}:{reason}")
        return RobotReadiness(
            sensors_healthy=valid["sensors"],
            perception_valid=valid["perception"],
            localization_valid=valid["localization"],
            mcu_healthy=valid["mcu"],
            physical_stop_verified=valid["stop"],
            navigation_prerequisites_met=all(valid.values()),
            failure_reasons=tuple(reasons),
        )
