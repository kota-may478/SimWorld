#!/usr/bin/env python3
"""Diagnose leg2 NavMesh planning failure (carry pose → humanoid)."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from navmesh_config import (
    NAV_PLANNING_CENTER_CLEARANCE_CM,
)
from navmesh_mission_nav import plan_navmesh_waypoints  # noqa: E402
from navmesh_obstacles import (
    fetch_actor_bounds,
    planning_clearance_exempt_actor_names as exempt_names_fn,
    setup_static_navmesh_obstacles,
)
from placement import build_registry  # noqa: E402
from surface_distance import (
    SurfaceObstacle,
    build_path_clearance_obstacles,
    center_to_aabb_surface_distance_cm,
    nearest_surface_distance_cm,
)


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _analyze_pose(
    label: str,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    *,
    ucv,
    nav_actor: str,
    prop_obstacles: List[SurfaceObstacle],
    prop_bounds_cache: Dict,
    registry,
) -> None:
    start_xyz = nq.nav_project_point(
        ucv, nav_actor, start_xy[0], start_xy[1], 6500.0
    )
    goal_xyz = nq.nav_project_point(
        ucv, nav_actor, goal_xy[0], goal_xy[1], 6500.0
    )
    print(f"\n=== {label} ===")
    print(f"start_world=({start_xy[0]:.1f},{start_xy[1]:.1f}) local={lc.world_xy_to_local(*start_xy)}")
    print(f"goal_world=({goal_xy[0]:.1f},{goal_xy[1]:.1f}) local={lc.world_xy_to_local(*goal_xy)}")
    if not start_xyz.get("ok") or not goal_xyz.get("ok"):
        print("projection failed:", start_xyz, goal_xyz)
        return
    sx = (float(start_xyz["x"]), float(start_xyz["y"]), float(start_xyz["z"]))
    gx = (float(goal_xyz["x"]), float(goal_xyz["y"]), float(goal_xyz["z"]))
    nearest_start = nearest_surface_distance_cm(start_xy, prop_obstacles)
    nearest_goal = nearest_surface_distance_cm(goal_xy, prop_obstacles)
    print(
        f"nearest_prop_center_clearance: start={nearest_start:.1f}cm goal={nearest_goal:.1f}cm "
        f"(required={NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm)"
    )
    raw = nq.nav_find_path_validated(
        ucv,
        nav_actor,
        sx,
        gx,
        agent_radius_cm=0.0,
        min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
        resample_spacing_cm=40.0,
    )
    print("nav_find_path_validated:", json.dumps(
        {k: raw.get(k) for k in ("ok", "error", "min_center_clearance_cm",
                                  "required_center_clearance_cm", "worst_obstacle_id",
                                  "worst_point_index", "corner_point_count", "output_point_count")},
        ensure_ascii=False,
    ))
    wps = plan_navmesh_waypoints(
        ucv,
        nav_actor,
        sx,
        gx,
        path_obstacles=prop_obstacles,
        prop_bounds_cache=prop_bounds_cache,
        registry=registry,
    )
    print(f"plan_navmesh_waypoints: {len(wps)} WP")
    if wps:
        mins: List[Tuple[float, str]] = []
        for i, wp in enumerate(wps):
            d = nearest_surface_distance_cm(wp, prop_obstacles)
            if d < NAV_PLANNING_CENTER_CLEARANCE_CM + 5:
                worst = min(prop_obstacles, key=lambda o: center_to_aabb_surface_distance_cm(wp, o))
                mins.append((d, worst.actor_name if hasattr(worst, "actor_name") else str(worst)))
        if mins:
            mins.sort(key=lambda x: x[0])
            print("tight WPs:", mins[:5])


def main() -> int:
    layout_id = sys.argv[1] if len(sys.argv) > 1 else "layout_01"
    registry = build_registry(layout_id=layout_id)
    ucv, _ = geh.reconnect_if_needed()
    ok_nav, nav_actor = nq.ensure_nav_query_service(ucv)
    if not ok_nav:
        print("NavQueryService unavailable")
        return 1
    bounds_cache, ok = setup_static_navmesh_obstacles(ucv, nav_actor, registry)
    if not ok:
        print("static obstacles setup failed")
        return 1
    exempt = exempt_names_fn(registry)
    prop_bounds_cache = {
        k: v for k, v in bounds_cache.items() if k not in exempt
    }
    prop_obstacles = build_path_clearance_obstacles(
        prop_bounds_cache, exempt_actor_names=exempt
    )
    human_local = registry.humanoid_local_cm
    human_xy = lc.local_xy_to_world(*human_local)
    material_xy = lc.local_xy_to_world(*registry.material_pickup_local_cm)

    # Typical post-carry pose from last E2E (world approx from metrics leg2 log)
  # local=(1734.571, 1672.892) from run_test log
    carry_local = (1745.996, 1623.017)
    carry_xy = lc.local_xy_to_world(*carry_local)

    _analyze_pose(
        "leg2_post_carry (last E2E)",
        carry_xy,
        human_xy,
        ucv=ucv,
        nav_actor=nav_actor,
        prop_obstacles=list(prop_obstacles),
        prop_bounds_cache=prop_bounds_cache,
        registry=registry,
    )
    _analyze_pose(
        "leg1_start",
        lc.local_xy_to_world(100.0, 100.0),
        material_xy,
        ucv=ucv,
        nav_actor=nav_actor,
        prop_obstacles=list(prop_obstacles),
        prop_bounds_cache=prop_bounds_cache,
        registry=registry,
    )
    # prop_011 bounds if present
    for key in sorted(prop_bounds_cache):
        if "011" in key:
            b = prop_bounds_cache[key]
            print(f"\n[prop] {key}: center=({b.cx:.0f},{b.cy:.0f}) half=({b.half_x:.0f},{b.half_y:.0f})")
            print(f"  dist carry→center={_dist(carry_xy, (b.cx, b.cy)):.0f}cm")
            print(f"  dist human→center={_dist(human_xy, (b.cx, b.cy)):.0f}cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
