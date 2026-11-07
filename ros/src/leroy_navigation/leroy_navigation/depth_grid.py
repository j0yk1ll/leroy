"""Rolling 2-D occupancy grid from observed 3-D depth rays.

No synthetic free space: a cell is cleared only by a measured depth ray
before its endpoint. Unknown cells stay -1. Uses calibrated odom z=ground.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MapSettings:
    resolution: float = 0.10
    width: int = 100
    height: int = 100
    min_obstacle_height: float = 0.08
    max_obstacle_height: float = 1.20
    max_depth: float = 3.0

    def __post_init__(self):
        v = (self.resolution, self.min_obstacle_height,
             self.max_obstacle_height, self.max_depth)
        if (not all(type(x) in (float, int) and math.isfinite(x) for x in v)
                or self.resolution <= 0 or not 0 < self.min_obstacle_height < self.max_obstacle_height
                or self.max_depth <= 0 or not 1 <= self.width <= 500
                or not 1 <= self.height <= 500):
            raise ValueError("invalid mapping configuration")


def rasterize(points, sensor_origin, robot_xy, settings=MapSettings()):
    """Return (origin_x, origin_y, occupancy) in odom coordinates.

    Points are ray endpoints already transformed into odom at acquisition time.
    A measured ground or obstacle endpoint terminates each ray. Returns
    independent new rolling snapshot, never keeps stale free cells.
    """
    if (len(sensor_origin) != 3 or len(robot_xy) != 2 or
            not all(math.isfinite(v) for v in (*sensor_origin, *robot_xy))):
        raise ValueError("invalid frame coordinates")
    ox = math.floor((robot_xy[0] - settings.width * settings.resolution / 2)
                    / settings.resolution) * settings.resolution
    oy = math.floor((robot_xy[1] - settings.height * settings.resolution / 2)
                    / settings.resolution) * settings.resolution
    def cell(x, y):
        return (math.floor((x - ox) / settings.resolution),
                math.floor((y - oy) / settings.resolution))
    def inside(c):
        return 0 <= c[0] < settings.width and 0 <= c[1] < settings.height
    def idx(c):
        return c[1] * settings.width + c[0]
    origin_cell = cell(sensor_origin[0], sensor_origin[1])
    cells = [-1] * (settings.width * settings.height)
    obstacles = set()
    for p in points:
        if len(p) != 3 or not all(math.isfinite(v) for v in p):
            continue
        dx, dy, dz = (p[i] - sensor_origin[i] for i in range(3))
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        if not 0 < distance <= settings.max_depth:
            continue
        end = cell(p[0], p[1])
        # The discrete ray includes all observed traversed cells. It does not
        # clear any unobserved space behind the depth endpoint.
        steps = max(abs(end[0]-origin_cell[0]),
                    abs(end[1]-origin_cell[1]))
        if steps > 0:
            for i in range(steps):
                c = (round(origin_cell[0] + (end[0]-origin_cell[0])*i/steps),
                     round(origin_cell[1] + (end[1]-origin_cell[1])*i/steps))
                if inside(c):
                    cells[idx(c)] = 0
        elif inside(end):
            cells[idx(end)] = 0
        if settings.min_obstacle_height <= p[2] <= settings.max_obstacle_height:
            if inside(end):
                obstacles.add(idx(end))
        elif inside(end) and p[2] < settings.min_obstacle_height:
            cells[idx(end)] = 0
        # High/uncategorized endpoints are not claimed free.
    for i in obstacles:
        cells[i] = 100
    return ox, oy, tuple(cells)
