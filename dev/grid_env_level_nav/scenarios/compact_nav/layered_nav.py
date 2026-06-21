#!/usr/bin/env python3
"""L0 navigation with FusionCam-driven L2 updates and replanning."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Callable, Optional, Set, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

import grid_env_hri_simulation as geh  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402
from grid_env_10k_pie_patrol import (  # noqa: E402
    PATH_MAX_STEPS_PER_WP,
    PATH_MAX_TOTAL_STEPS,
    PATH_REPLAN_STUCK_STEPS,
    PATH_WP_REACH_TOLERANCE_CM,
    _nearest_waypoint_index_ahead,
    dist2d,
    execute_segment_command,
    get_pos2d,
    get_yaw,
    plan_astar_waypoints,
    segment_command_toward_waypoint,
)
from l2_fusion import apply_l2_from_fusion_detections, detections_summary  # noqa: E402
from level_coords import local_xy_to_world, world_xy_to_local  # noqa: E402
from pie_safety import PERCEPTION_MIN_INTERVAL_S, require_live_ucv  # noqa: E402
from viz import NavTrace  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME
PerceiveFn = Callable[[], list]


def _nearest_traversable_world_xy(
    costmap: Costmap2D,
    pos_xy: WorldXY,
    *,
    max_radius_cells: int = 10,
) -> WorldXY:
    """When the robot sits on a lethal L2 cell, pick a nearby free cell for replanning."""
    start = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if start is None:
        return pos_xy
    if costmap.is_traversable(start):
        return pos_xy
    sx, sy = start
    best: Optional[WorldXY] = None
    best_dist = float("inf")
    for r in range(1, max_radius_cells + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                gx, gy = sx + dx, sy + dy
                if gx < 0 or gy < 0 or gx >= costmap.width_cells or gy >= costmap.height_cells:
                    continue
                if not costmap.is_traversable((gx, gy)):
                    continue
                candidate = costmap.grid_to_world_xy_center((gx, gy))
                dist = dist2d(candidate, pos_xy)
                if dist < best_dist:
                    best_dist = dist
                    best = candidate
        if best is not None:
            return best
    return pos_xy


def _safe_replan_astar(costmap: Costmap2D, pos_xy: WorldXY, goal_xy: WorldXY):
    try:
        return plan_astar_waypoints(costmap, pos_xy, goal_xy)
    except ValueError:
        alt = _nearest_traversable_world_xy(costmap, pos_xy)
        if alt == pos_xy:
            raise
        try:
            return plan_astar_waypoints(costmap, alt, goal_xy)
        except ValueError:
            raise ValueError("No traversable start cell near robot for replan") from None


def navigate_layered_with_fusion(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    perceive_fn: PerceiveFn,
    robot_name: str = ROBOT_ACTOR,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = PERCEPTION_MIN_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
) -> bool:
    require_live_ucv(ucv, context="fusion layered nav")
    goal_xy = local_xy_to_world(*goal_local_xy)
    l2_seen_cells: Set[Tuple[int, int]] = set()

    start_xy = get_pos2d(ucv, robot_name)
    costmap = layers.to_costmap2d()
    plan = _safe_replan_astar(costmap, start_xy, goal_xy)
    waypoints = plan.waypoints_xy
    if trace is not None:
        trace.record_plan(waypoints, reason="initial")
        trace.record_position(world_xy_to_local(*start_xy))
    wp_index = 0
    steps_on_wp = 0
    total_steps = 0
    last_perception_t = -1e9

    print(
        f"  [FusionNav]{f' {label}' if label else ''} goal_local={goal_local_xy} "
        f"waypoints={len(waypoints)} cost={plan.total_cost:.1f}"
    )

    while total_steps < max_total_steps:
        total_steps += 1
        require_live_ucv(ucv, context=f"fusion nav step {total_steps}")
        pos_xy = get_pos2d(ucv, robot_name)
        if trace is not None:
            trace.record_position(world_xy_to_local(*pos_xy))
        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
            print(f"  [FusionNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
            if trace is not None:
                trace.arrived = True
                trace.l2_cell_count = len(l2_seen_cells)
            return True

        now = time.time()
        if now - last_perception_t >= perception_interval_s:
            detections = perceive_fn()
            if detections:
                n_cells = apply_l2_from_fusion_detections(
                    layers,
                    detections,
                    robot_xy=pos_xy,
                    robot_yaw_deg=get_yaw(ucv, robot_name),
                    known_cells=l2_seen_cells,
                )
                if n_cells > 0:
                    costmap = layers.to_costmap2d()
                    try:
                        replan = _safe_replan_astar(costmap, pos_xy, goal_xy)
                    except ValueError:
                        replan = None
                    if replan is not None and replan.waypoints_xy:
                        waypoints = replan.waypoints_xy
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                        if trace is not None:
                            trace.record_plan(waypoints, reason="l2_perception")
                            trace.l2_cell_count = len(l2_seen_cells)
                        summary = detections_summary(detections)
                        print(
                            f"  [FusionNav] L2 +{n_cells} cells detect={list(summary.keys())} "
                            f"→ replan {len(waypoints)} WP"
                        )
            last_perception_t = now

        if wp_index >= len(waypoints):
            waypoint_xy = goal_xy
        else:
            waypoint_xy = waypoints[wp_index]
            if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                wp_index += 1
                steps_on_wp = 0
                continue

        command = segment_command_toward_waypoint(
            pos_xy,
            get_yaw(ucv, robot_name),
            waypoint_xy,
        )
        if command is None:
            if wp_index >= len(waypoints):
                if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                    return True
            else:
                wp_index += 1
            steps_on_wp = 0
            continue

        execute_segment_command(ucv, command)
        steps_on_wp += 1

        if total_steps % 25 == 0:
            print(
                f"  [FusionNav] step={total_steps} dist_goal={dist2d(pos_xy, goal_xy):.0f}cm "
                f"wp={wp_index + 1}/{len(waypoints)} l2_cells={len(l2_seen_cells)}"
            )

        if steps_on_wp >= PATH_REPLAN_STUCK_STEPS:
            pos_xy = get_pos2d(ucv, robot_name)
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                return True
            costmap = layers.to_costmap2d()
            try:
                replan = _safe_replan_astar(costmap, pos_xy, goal_xy)
            except ValueError:
                replan = None
            if replan is not None and replan.waypoints_xy:
                waypoints = replan.waypoints_xy
                wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                if trace is not None:
                    trace.record_plan(waypoints, reason="stuck_replan")
            steps_on_wp = 0

        if steps_on_wp >= PATH_MAX_STEPS_PER_WP and wp_index < len(waypoints):
            wp_index += 1
            steps_on_wp = 0

    print(f"  [FusionNav] ERROR: exceeded max_total_steps={max_total_steps}")
    if trace is not None:
        trace.l2_cell_count = len(l2_seen_cells)
    return False
