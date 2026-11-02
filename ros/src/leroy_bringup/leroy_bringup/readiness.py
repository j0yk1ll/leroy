"""Compatibility shim: canonical readiness policy lives in leroy_safety.

Readiness is observational and is never a motor permit.
"""
from leroy_safety.readiness import (  # noqa: F401
    COMPONENTS, DEFAULT_MAX_AGE_S, HealthEvidence, RobotReadiness,
    ReadinessAggregator,
)
