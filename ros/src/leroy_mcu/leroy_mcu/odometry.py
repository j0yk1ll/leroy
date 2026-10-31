import math
from typing import Optional


def integrate_yaw(
    yaw: float,
    previous_wz: Optional[float],
    wz: float,
    dt: float,
) -> float:
    if previous_wz is None:
        yaw += wz * dt
    else:
        yaw += 0.5 * (previous_wz + wz) * dt
    return math.atan2(math.sin(yaw), math.cos(yaw))
