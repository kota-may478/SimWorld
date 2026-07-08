#!/usr/bin/env python3
"""Deep analysis: NavFindPath clearance violations (PIE + spawned props required).

Findings (2026-07-06):
  - FindPathSync returns identical 7 corners for AgentRadius 170–220 cm.
  - AgentRadius does not carve clearance; NavModifier padding must include standoff.
  - Raw corners violate 1 m rule with 15 cm hull-only modifiers (min center 0 cm).
  - Polyline resample between corners creates additional chord violations.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from navmesh_config import (  # noqa: E402
    NAV_FINDPATH_AGENT_RADIUS_CM,
    NAV_PLANNING_CENTER_CLEARANCE_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
    PROXIMITY_EDGE_FROM_SURFACE_CM,
    SPOTDOG_BODY_RADIUS_CM,
)
from carry import pickup_standoff_xy  # noqa: E402
from navmesh_obstacles import setup_static_navmesh_obstacles  # noqa: E402
from placement import build_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402
from navmesh_mission_nav import plan_navmesh_waypoints  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    build_surface_obstacles_from_bounds,
    center_to_aabb_surface_distance_cm,
    min_clearance_on_segment_cm,
    validate_path_center_clearance,
)

WorldXY = Tuple[float, float]
SAMPLE_CM = 16.0
ROBOT = "GridEnv_SpotRobot"


def _project_local(ucv, nav: str, local_xy: Tuple[float, float]) -> Optional[Tuple[float, float, float]]:
    wx, wy = lc.local_xy_to_world(local_xy[0], local_xy[1])
    for z in (lc.NAV_PROJECT_PROBE_Z_CM, 6466.0, 6490.0, 6455.0, 6450.0):
        raw = nq.nav_project_point(ucv, nav, wx, wy, z)
        if raw.get("ok"):
            return float(raw["x"]), float(raw["y"]), float(raw["z"])
    return None


def _analyze_points(
    label: str,
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
) -> None:
    report = validate_path_center_clearance(
        points,
        obstacles,
        min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
        body_radius_cm=SPOTDOG_BODY_RADIUS_CM,
        sample_spacing_cm=SAMPLE_CM,
    )
    print(f"\n--- {label} ({len(points)} pts) ---")
    print(
        f"  center min={report.min_center_clearance_cm}cm "
        f"body_edge min={report.min_body_edge_clearance_cm}cm "
        f"worst={report.worst_obstacle_id}"
    )
    print(
        f"  wp_viol={report.violating_wp_count} seg_viol={report.violating_segment_count} "
        f"PASS={report.ok}"
    )
    if not points:
        return
    worst_pt = -1
    worst_val = 1e9
    for i, pt in enumerate(points):
        for obs in obstacles:
            d = center_to_aabb_surface_distance_cm(pt, obs)
            if d < worst_val:
                worst_val = d
                worst_pt = i
                worst_id = obs.obstacle_id
    print(f"  worst WP[{worst_pt}] center={worst_val:.1f}cm @ {worst_id}")
    if len(points) >= 2:
        worst_seg = -1
        worst_seg_val = 1e9
        for i in range(len(points) - 1):
            d = min_clearance_on_segment_cm(
                points[i], points[i + 1], obstacles, sample_spacing_cm=SAMPLE_CM
            )
            if d is not None and d < worst_seg_val:
                worst_seg_val = d
                worst_seg = i
        print(
            f"  worst seg[{worst_seg}]→[{worst_seg + 1}] center={worst_seg_val:.1f}cm"
        )


def _prop_023_detail(points: Sequence[WorldXY], obstacles: Sequence[SurfaceObstacle]) -> None:
    obs = next((o for o in obstacles if o.obstacle_id == "site20_prop_023"), None)
    if obs is None:
        return
    print(f"\n=== site20_prop_023 AABB center=({obs.cx:.0f},{obs.cy:.0f}) half=({obs.half_x:.0f},{obs.half_y:.0f}) ===")
    for i, pt in enumerate(points):
        d = center_to_aabb_surface_distance_cm(pt, obs)
        if d < 200.0:
            lx, ly = lc.world_xy_to_local(pt[0], pt[1])
            print(f"  WP[{i}] local=({lx:.0f},{ly:.0f}) center_dist={d:.1f}cm")


def main() -> int:
    print(
        f"target: center>={NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm "
        f"(body edge>={PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm) "
        f"findpath_agent={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm pad={NAV_PROP_OBSTACLE_PADDING_CM:.0f}cm"
    )
    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    if not nav:
        print("FAIL: no NavQueryService")
        return 1

    registry = build_registry(layout_id="layout_01")
    bounds_cache, ok = setup_static_navmesh_obstacles(ucv, nav, registry)
    if not ok:
        print("WARN: static obstacle setup incomplete")
    prop_cache = {k: v for k, v in bounds_cache.items() if k != registry.humanoid_actor_name}
    obstacles = build_surface_obstacles_from_bounds(prop_cache)
    print(f"obstacles={len(obstacles)} rebuild_ok={ok}")

    start = _project_local(ucv, nav, ROBOT_START_LOCAL_CM)
    material_world = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    approach_world = pickup_standoff_xy(
        material_world,
        lc.local_xy_to_world(*ROBOT_START_LOCAL_CM),
        standoff_cm=160.0,
    )
    goal = _project_local(ucv, nav, lc.world_xy_to_local(*approach_world))
    if goal is None:
        wx, wy = approach_world
        for z in (lc.NAV_PROJECT_PROBE_Z_CM, 6466.0, 6490.0, 6455.0, 6450.0):
            raw = nq.nav_project_point(ucv, nav, wx, wy, z)
            if raw.get("ok"):
                goal = (float(raw["x"]), float(raw["y"]), float(raw["z"]))
                break
    if start is None or goal is None:
        print(f"FAIL: projection start={start} approach={approach_world} goal={goal}")
        geh.release_connection(ucv)
        return 1
    print(f"start world=({start[0]:.0f},{start[1]:.0f}) goal world=({goal[0]:.0f},{goal[1]:.0f})")

    for agent in (170.0, 185.0, 200.0, 220.0):
        raw = nq.nav_find_path(ucv, nav, start, goal, agent_radius_cm=agent)
        pts = nq.path_points_xy(raw)
        print(f"\n=== NavFindPath agent={agent:.0f} ok={raw.get('ok')} corners={len(pts)} ===")
        if pts:
            _analyze_points(f"corners agent={agent:.0f}", pts, obstacles)

    for spacing in (0.0, 40.0):
        raw = nq.nav_find_path_validated(
            ucv,
            nav,
            start,
            goal,
            agent_radius_cm=NAV_FINDPATH_AGENT_RADIUS_CM,
            min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
            resample_spacing_cm=spacing,
        )
        pts = nq.path_points_xy(raw)
        print(
            f"\n=== NavFindPathValidated resample={spacing:.0f} "
            f"ok={raw.get('ok')} err={raw.get('error')} pts={len(pts)} "
            f"corners={raw.get('corner_point_count')} "
            f"min={raw.get('min_center_clearance_cm')} worst={raw.get('worst_obstacle_id')} ==="
        )
        if pts:
            _analyze_points(f"validated resample={spacing:.0f}", pts, obstacles)
            _prop_023_detail(pts, obstacles)

    mission_pts = plan_navmesh_waypoints(
        ucv,
        nav,
        start,
        goal,
        path_obstacles=obstacles,
        prop_bounds_cache=prop_cache,
        registry=registry,
    )
    print(f"\n=== plan_navmesh_waypoints (Dynamic NavMesh) === pts={len(mission_pts)}")
    if mission_pts:
        _analyze_points("mission plan", mission_pts, obstacles)
        _prop_023_detail(mission_pts, obstacles)

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
