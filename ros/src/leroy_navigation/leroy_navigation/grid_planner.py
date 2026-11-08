"""A* on ROS occupancy grid data. Unknown cells are non-traversable.

Planning is functionality, not a safety-certified obstacle monitor.
"""
from dataclasses import dataclass
import heapq
import math


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple
    occupied_threshold: int = 50
    inflation_radius_m: float = 0.30

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int
                or not 0 < self.width <= 10000 or not 0 < self.height <= 10000
                or len(self.data) != self.width * self.height
                or any(not math.isfinite(v) for v in
                       (self.resolution, self.origin_x, self.origin_y,
                        self.inflation_radius_m))
                or self.resolution <= 0 or self.inflation_radius_m < 0
                or not 1 <= self.occupied_threshold <= 100):
            raise ValueError("invalid occupancy grid")

    def cell(self, x, y):
        cx = math.floor((x - self.origin_x) / self.resolution)
        cy = math.floor((y - self.origin_y) / self.resolution)
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            raise ValueError("point outside map")
        return cx, cy

    def point(self, cell):
        return (self.origin_x + (cell[0] + 0.5) * self.resolution,
                self.origin_y + (cell[1] + 0.5) * self.resolution)

    def blocked(self):
        blocked = set()
        for i, cost in enumerate(self.data):
            if cost < 0 or cost >= self.occupied_threshold:
                blocked.add((i % self.width, i // self.width))
        cells = math.ceil(self.inflation_radius_m / self.resolution)
        if cells:
            offsets = [(dx, dy) for dx in range(-cells, cells + 1)
                       for dy in range(-cells, cells + 1)
                       if math.hypot(dx, dy) * self.resolution <=
                       self.inflation_radius_m + 1e-12]
            blocked = {(x+dx, y+dy) for x, y in blocked for dx, dy in offsets
                       if 0 <= x+dx < self.width and 0 <= y+dy < self.height}
        # Map boundaries are not assumed safe beyond the sensor horizon.
        return blocked


def plan(grid: Grid, start, goal):
    """Return list of world-coordinate waypoints, or None if unreachable."""
    src = grid.cell(*start)
    dst = grid.cell(*goal)
    forbidden = grid.blocked()
    if src in forbidden or dst in forbidden:
        return None
    if src == dst:
        return [grid.point(dst)]
    neighbors = [(dx, dy, math.hypot(dx, dy))
                 for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                 if dx or dy]
    queue = [(0.0, 0.0, src)]
    cost = {src: 0.0}
    parent = {}
    while queue:
        _, g, cell = heapq.heappop(queue)
        if g > cost.get(cell, float("inf")) + 1e-9:
            continue
        if cell == dst:
            path = [cell]
            while path[-1] != src:
                path.append(parent[path[-1]])
            path.reverse()
            return [grid.point(c) for c in path]
        x, y = cell
        for dx, dy, step in neighbors:
            nxt = (x + dx, y + dy)
            if not (0 <= nxt[0] < grid.width and 0 <= nxt[1] < grid.height):
                continue
            if nxt in forbidden:
                continue
            # No diagonal movement through the corner between obstacles.
            if dx and dy and ((x+dx, y) in forbidden or
                              (x, y+dy) in forbidden):
                continue
            ng = g + step
            if ng + 1e-9 < cost.get(nxt, float("inf")):
                parent[nxt] = cell
                cost[nxt] = ng
                heuristic = math.hypot(dst[0]-nxt[0], dst[1]-nxt[1])
                heapq.heappush(queue, (ng+heuristic, ng, nxt))
    return None
