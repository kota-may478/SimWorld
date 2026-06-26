"""Global planner server: A* replan with staged relaxation."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from costmap_layers import LayeredCostmap  # noqa: E402
from grid_env_10k_pie_patrol import plan_astar_waypoints  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402

from nav_stack.global_costmap import build_planning_costmap  # noqa: E402

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]


@dataclass(frozen=True)
class ReplanResult:
    waypoints: Optional[List[WorldXY]]
    stage: str


def safe_replan_astar(
    costmap: Costmap2D,
    start_xy: WorldXY,
    goal_xy: WorldXY,
) -> Optional[Sequence[WorldXY]]:
    try:
        result = plan_astar_waypoints(costmap, start_xy, goal_xy)
    except (ValueError, RuntimeError):
        return None
    if not result.waypoints_xy:
        return None
    return result.waypoints_xy


def replan_on_merged_layers(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    planning_clearance_cm: float,
    planning_clearance_cost: float,
    exclude_robot_self_fn=None,
    l2_seen_cells: Optional[Set[GridCell]] = None,
) -> ReplanResult:
    """Replan using merged L0+L1+L2 with clearance, then relaxed fallbacks."""
    if exclude_robot_self_fn is not None and l2_seen_cells is not None:
        exclude_robot_self_fn(layers, pos_xy, l2_seen_cells)

    costmap = build_planning_costmap(
        layers,
        planning_clearance_cm=planning_clearance_cm,
        planning_clearance_cost=planning_clearance_cost,
    )
    waypoints = safe_replan_astar(costmap, pos_xy, goal_xy)
    if waypoints is not None:
        return ReplanResult(waypoints=list(waypoints), stage="clearance_merged")

    waypoints = safe_replan_astar(layers.to_costmap2d(), pos_xy, goal_xy)
    if waypoints is not None:
        return ReplanResult(waypoints=list(waypoints), stage="tight_merged")

    merged_l01 = Costmap2D(
        costs=np.maximum(layers.l0.astype(np.float32), layers.l1.astype(np.float32)),
        origin_xy=layers.origin_xy,
        resolution_cm=layers.resolution_cm,
        lethal_cost=layers.lethal_cost,
    )
    waypoints = safe_replan_astar(merged_l01, pos_xy, goal_xy)
    if waypoints is not None:
        return ReplanResult(waypoints=list(waypoints), stage="l0_l1")

    waypoints = safe_replan_astar(layers.to_l0_costmap2d(), pos_xy, goal_xy)
    if waypoints is not None:
        return ReplanResult(waypoints=list(waypoints), stage="l0_only")
    return ReplanResult(waypoints=None, stage="failed")
