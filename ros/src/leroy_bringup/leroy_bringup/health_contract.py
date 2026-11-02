"""Compatibility shim: canonical health contract lives in leroy_safety."""
from leroy_safety.health_contract import (  # noqa: F401
    COMPONENTS, SCHEMA, TOPICS, OWNERS, MAX_BYTES, encode, decode,
)
