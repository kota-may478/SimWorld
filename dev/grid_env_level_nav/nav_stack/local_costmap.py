"""Rolling local costmap around the robot (L2 crop for controller checks)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from costmap_layers import LayeredCostmap  # noqa: E402
from perception_layer import L2_LETHAL_COST  # noqa: E402

WorldXY = Tuple[float, float]


@dataclass
class RollingCostmap:
    """Robot-centered lethal grid used by the controller layer."""

    costs: np.ndarray
    origin_xy: WorldXY
    resolution_cm: float
    size_cm: float
    robot_xy: WorldXY

    @property
    def width_cells(self) -> int:
        return int(self.costs.shape[1])

    @property
    def height_cells(self) -> int:
        return int(self.costs.shape[0])

    def world_xy_to_local_grid(
        self,
        world_xy: WorldXY,
    ) -> Optional[Tuple[int, int]]:
        gx = int((world_xy[0] - self.origin_xy[0]) / self.resolution_cm)
        gy = int((world_xy[1] - self.origin_xy[1]) / self.resolution_cm)
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            return gx, gy
        return None

    def is_lethal(self, world_xy: WorldXY) -> bool:
        cell = self.world_xy_to_local_grid(world_xy)
        if cell is None:
            return False
        gx, gy = cell
        return float(self.costs[gy, gx]) >= L2_LETHAL_COST * 0.5

    def nearest_lethal_dist_cm(self, world_xy: WorldXY) -> float:
        center = self.world_xy_to_local_grid(world_xy)
        if center is None:
            return float("inf")
        cx, cy = center
        best = float("inf")
        lethal = self.costs >= L2_LETHAL_COST * 0.5
        ys, xs = np.nonzero(lethal)
        for gx, gy in zip(xs, ys):
            wx = self.origin_xy[0] + (gx + 0.5) * self.resolution_cm
            wy = self.origin_xy[1] + (gy + 0.5) * self.resolution_cm
            best = min(best, math.hypot(wx - world_xy[0], wy - world_xy[1]))
        return best


def build_local_costmap(
    layers: LayeredCostmap,
    robot_xy: WorldXY,
    *,
    size_cm: float = 600.0,
    resolution_cm: Optional[float] = None,
) -> RollingCostmap:
    """Crop merged L2 lethal cells into a rolling window centered on the robot."""
    res = resolution_cm if resolution_cm is not None else layers.resolution_cm
    half = size_cm / 2.0
    origin = (robot_xy[0] - half, robot_xy[1] - half)
    width_cells = max(1, int(math.ceil(size_cm / res)))
    height_cells = width_cells
    costs = np.zeros((height_cells, width_cells), dtype=np.float32)
    merged = layers.to_costmap2d()
    for gy in range(height_cells):
        for gx in range(width_cells):
            wx = origin[0] + (gx + 0.5) * res
            wy = origin[1] + (gy + 0.5) * res
            gcell = merged.world_xy_to_grid((wx, wy), clamp=True)
            if gcell is None:
                continue
            ggx, ggy = gcell
            costs[gy, gx] = float(merged.costs[ggy, ggx])
    return RollingCostmap(
        costs=costs,
        origin_xy=origin,
        resolution_cm=res,
        size_cm=size_cm,
        robot_xy=robot_xy,
    )
