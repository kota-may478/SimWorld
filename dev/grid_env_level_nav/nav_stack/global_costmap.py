"""Global costmap builder (L0+L1+L2 merged + planning clearance)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from costmap_layers import LayeredCostmap  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402


def build_planning_costmap(
    layers: LayeredCostmap,
    *,
    planning_clearance_cm: float,
    planning_clearance_cost: float,
) -> Costmap2D:
    """Merged map plus soft clearance cost around lethal cells for A* planning."""
    base = layers.to_costmap2d()
    radius_cells = max(0, int(math.ceil(planning_clearance_cm / base.resolution_cm)))
    if radius_cells <= 0:
        return base
    costs = base.costs.copy()
    lethal = costs >= base.lethal_cost * 0.5
    lethal_ys, lethal_xs = np.nonzero(lethal)
    if lethal_xs.size == 0:
        return base
    for cx, cy in zip(lethal_xs, lethal_ys):
        gx0 = max(0, int(cx) - radius_cells)
        gx1 = min(base.width_cells, int(cx) + radius_cells + 1)
        gy0 = max(0, int(cy) - radius_cells)
        gy1 = min(base.height_cells, int(cy) + radius_cells + 1)
        for gy in range(gy0, gy1):
            for gx in range(gx0, gx1):
                if lethal[gy, gx]:
                    continue
                dist_cm = math.hypot(gx - int(cx), gy - int(cy)) * base.resolution_cm
                if dist_cm <= planning_clearance_cm:
                    costs[gy, gx] = max(float(costs[gy, gx]), planning_clearance_cost)
    for gy in range(base.height_cells):
        for gx in range(base.width_cells):
            if lethal[gy, gx]:
                continue
            border_dist_cells = min(
                gx,
                gy,
                base.width_cells - 1 - gx,
                base.height_cells - 1 - gy,
            )
            border_dist_cm = border_dist_cells * base.resolution_cm
            if border_dist_cm <= planning_clearance_cm:
                costs[gy, gx] = max(float(costs[gy, gx]), planning_clearance_cost)
    return Costmap2D(
        costs=costs,
        origin_xy=base.origin_xy,
        resolution_cm=base.resolution_cm,
        lethal_cost=base.lethal_cost,
    )
