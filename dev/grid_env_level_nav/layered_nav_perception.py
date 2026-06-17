#!/usr/bin/env python3
"""Layered L0+L2 navigation: replan A* while updating L2 from robot depth."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (
    _ROOT,
    _ROOT / "dev" / "grid_env_hri",
    _ROOT / "dev" / "grid_env_10k",
    _ROOT / "dev" / "grid_env_depth_perception",
    _ROOT / "dev" / "llm_material_transport",
    _THIS_DIR,
):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from grid_env_10k_pie_patrol import (  # noqa: E402
    PATH_MAX_OPEN_LOOP_MOVE_CM,
    PATH_MAX_STEPS_PER_WP,
    PATH_MAX_TOTAL_STEPS,
    PATH_REPLAN_STUCK_STEPS,
    PATH_WP_REACH_TOLERANCE_CM,
    PATH_WP_SPACING_CM,
    ROBOT_MOVE_SLICE_S,
    ROBOT_SPEED,
    ROBOT_TURN_DUR_S,
    ROTATE_THR_DEG,
    _nearest_waypoint_index_ahead,
    dist2d,
    execute_segment_command,
    get_pos2d,
    get_yaw,
    plan_astar_waypoints,
    segment_command_toward_waypoint,
)
from level_coords import local_xy_to_world  # noqa: E402
from perception_layer import EgocentricPerceptionConfig, update_l2_from_depth_image  # noqa: E402
from pie_safety import PERCEPTION_MIN_INTERVAL_S, require_live_ucv  # noqa: E402
from robot_sensor import resolve_sensor_camera_id  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME


def _fetch_depth_m(ucv: UnrealCV, camera_id: int) -> Optional[np.ndarray]:
    try:
        raw = ucv.get_image(camera_id, "depth", "png")
    except Exception:
        return None
    if raw is None:
        return None
    try:
        from io import BytesIO
        from PIL import Image

        img = Image.open(BytesIO(raw))
        arr = np.asarray(img, dtype=np.float32)
        if arr.ndim == 3:
            arr = arr[..., 0]
        if arr.max() > 50.0:
            arr = arr / 255.0 * 20.0
        return arr
    except Exception:
        return None


def update_l2_from_robot_depth(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    *,
    robot_name: str = ROBOT_ACTOR,
    camera_id: Optional[int] = None,
    config: Optional[EgocentricPerceptionConfig] = None,
) -> int:
    cam_id = camera_id if camera_id is not None else resolve_sensor_camera_id(ucv)
    depth_m = _fetch_depth_m(ucv, cam_id)
    if depth_m is None:
        return 0
    pos_xy = get_pos2d(ucv, robot_name)
    yaw_deg = get_yaw(ucv, robot_name)
    return update_l2_from_depth_image(
        depth_m,
        layers,
        robot_xy=pos_xy,
        robot_yaw_deg=yaw_deg,
        config=config,
    )


def navigate_layered_local(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    robot_name: str = ROBOT_ACTOR,
    tolerance_cm: float = 120.0,
    label: str = "",
    carry_sync_name: Optional[str] = None,
    perception_interval_s: float = PERCEPTION_MIN_INTERVAL_S,
    perception_config: Optional[EgocentricPerceptionConfig] = None,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
) -> bool:
    """Follow merged L0+L1+L2 costmap with periodic depth updates and replanning."""
    require_live_ucv(ucv, context="layered nav")
    goal_xy = local_xy_to_world(*goal_local_xy)
    camera_id = resolve_sensor_camera_id(ucv)
    cfg = perception_config or EgocentricPerceptionConfig()

    start_xy = get_pos2d(ucv, robot_name)
    costmap = layers.to_costmap2d()
    plan = plan_astar_waypoints(costmap, start_xy, goal_xy)
    waypoints = plan.waypoints_xy
    wp_index = 0
    steps_on_wp = 0
    total_steps = 0
    last_perception_t = -1e9

    print(
        f"  [LayeredNav]{f' {label}' if label else ''} "
        f"goal_local={goal_local_xy} waypoints={len(waypoints)} cost={plan.total_cost:.1f}"
    )

    while total_steps < max_total_steps:
        total_steps += 1
        require_live_ucv(ucv, context=f"layered nav step {total_steps}")
        pos_xy = get_pos2d(ucv, robot_name)
        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
            print(f"  [LayeredNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
            return True

        now = time.time()
        if now - last_perception_t >= perception_interval_s:
            n_cells = update_l2_from_robot_depth(
                ucv,
                layers,
                robot_name=robot_name,
                camera_id=camera_id,
                config=cfg,
            )
            if n_cells > 0:
                costmap = layers.to_costmap2d()
                replan = plan_astar_waypoints(costmap, pos_xy, goal_xy)
                if replan.waypoints_xy:
                    waypoints = replan.waypoints_xy
                    wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                    print(
                        f"  [LayeredNav] L2 +{n_cells} cells → replan "
                        f"{len(waypoints)} WP resume WP{wp_index + 1}"
                    )
            last_perception_t = now

        if carry_sync_name and geh.actor_exists(ucv, carry_sync_name):
            from construction_site_carry import sync_carry_pose  # noqa: WPS433

            sync_carry_pose(ucv, carry_sync_name, robot_name)

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
            pos_xy = get_pos2d(ucv, robot_name)
            print(
                f"  [LayeredNav] step={total_steps} pos=({pos_xy[0]:.0f},{pos_xy[1]:.0f}) "
                f"dist_goal={dist2d(pos_xy, goal_xy):.0f}cm wp={wp_index + 1}/{len(waypoints)}"
            )

        if steps_on_wp >= PATH_REPLAN_STUCK_STEPS:
            pos_xy = get_pos2d(ucv, robot_name)
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                return True
            costmap = layers.to_costmap2d()
            replan = plan_astar_waypoints(costmap, pos_xy, goal_xy)
            if replan.waypoints_xy:
                waypoints = replan.waypoints_xy
                wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                print(
                    f"  [LayeredNav] stuck replan @ ({pos_xy[0]:.0f},{pos_xy[1]:.0f}) "
                    f"→ {len(waypoints)} WP"
                )
            steps_on_wp = 0

        if steps_on_wp >= PATH_MAX_STEPS_PER_WP and wp_index < len(waypoints):
            wp_index += 1
            steps_on_wp = 0

    print(f"  [LayeredNav] ERROR: exceeded max_total_steps={max_total_steps}")
    return False
