#!/usr/bin/env python3
"""SpotDog open-loop A* follower on a LayeredCostmap (pie_patrol compatible)."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (
    _ROOT,
    _ROOT / "dev" / "grid_env_hri",
    _ROOT / "dev" / "grid_env_10k",
    _ROOT / "dev" / "llm_material_transport",
    _THIS_DIR,
):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from costmap_layers import LayeredCostmap  # noqa: E402
from grid_env_10k_pie_patrol import robot_navigate_astar  # noqa: E402
from level_coords import local_xy_to_world  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]


def layered_costmap_to_costmap2d(layers: LayeredCostmap) -> Costmap2D:
    return layers.to_costmap2d()


def navigate_world_xy(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_xy: WorldXY,
    *,
    label: str = "",
    tolerance_cm: float = 120.0,
) -> bool:
    costmap = layered_costmap_to_costmap2d(layers)
    return robot_navigate_astar(
        ucv,
        costmap,
        goal_xy,
        tolerance_cm=tolerance_cm,
        label=label,
    )


def navigate_local_xy(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    label: str = "",
    tolerance_cm: float = 120.0,
) -> bool:
    goal_xy = local_xy_to_world(*goal_local_xy)
    return navigate_world_xy(
        ucv,
        layers,
        goal_xy,
        label=label,
        tolerance_cm=tolerance_cm,
    )
