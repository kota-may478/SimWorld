#!/usr/bin/env python3
"""Verify mission *planning* keeps >=1 m body-edge clearance from prop AABBs.

Scope: planning output only (plan_navmesh_waypoints). Execution trajectory
proximity during real movement is intentionally out of scope.

Requires: PIE Play + spawned layout_01 props + rebuilt NavQueryService.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from carry import pickup_standoff_xy  # noqa: E402
from navmesh_config import (  # noqa: E402
    NAV_FINDPATH_AGENT_RADIUS_CM,
    NAV_MODIFIER_CLEARANCE_MARGIN_CM,
    NAV_PLANNING_CENTER_CLEARANCE_CM,
    NAV_PLANNING_GOAL_PUSH_MARGIN_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
    PROXIMITY_EDGE_FROM_SURFACE_CM,
    SPOTDOG_BODY_RADIUS_CM,
    planning_goal_position_center_clearance_cm,
)
from navmesh_mission_nav import plan_navmesh_waypoints  # noqa: E402
from navmesh_obstacles import (  # noqa: E402
    fetch_actor_bounds,
    planning_clearance_exempt_actor_names,
    setup_static_navmesh_obstacles,
)
from placement import build_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    adjust_xy_for_planning_clearance,
    body_edge_to_aabb_surface_distance_cm,
    build_path_clearance_obstacles,
    center_to_aabb_surface_distance_cm,
    min_clearance_on_segment_cm,
    validate_path_center_clearance,
    validate_path_corridor_clearance,
    nearest_surface_distance_cm,
)

WorldXY = Tuple[float, float]
WorldXYZ = Tuple[float, float, float]
SAMPLE_CM = 16.0


@dataclass(frozen=True)
class LegPlanResult:
    leg_id: str
    corridor_ok: bool
    plan_ok: bool
    waypoint_count: int
    corridor_min_center_cm: Optional[float]
    corridor_min_body_edge_cm: Optional[float]
    corridor_worst_id: Optional[str]
    corridor_wp_violations: int
    corridor_seg_violations: int
    goal_center_cm: Optional[float] = None
    goal_body_edge_cm: Optional[float] = None
    goal_worst_id: Optional[str] = None


def _project_world_xyz(
    ucv,
    nav: str,
    world_xy: WorldXY,
    *,
    z_hints: Optional[Sequence[float]] = None,
) -> Optional[WorldXYZ]:
    wx, wy = world_xy
    hints: List[float] = list(z_hints or ())
    hints.extend(
        (
            lc.NAV_PROJECT_PROBE_Z_CM,
            6466.0,
            6490.0,
            6550.0,
            6455.0,
            6450.0,
        )
    )
    seen: set[float] = set()
    for z in hints:
        zf = float(z)
        if zf in seen:
            continue
        seen.add(zf)
        raw = nq.nav_project_point(ucv, nav, wx, wy, zf)
        if raw.get("ok"):
            return float(raw["x"]), float(raw["y"]), float(raw["z"])
    return None


def _project_local_xyz(
    ucv,
    nav: str,
    local_xy: Tuple[float, float],
    *,
    z_hints: Optional[Sequence[float]] = None,
) -> Optional[WorldXYZ]:
    return _project_world_xyz(
        ucv, nav, lc.local_xy_to_world(local_xy[0], local_xy[1]), z_hints=z_hints
    )


def _worst_indices(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
) -> Tuple[int, float, int, float, Optional[str], Optional[str]]:
    worst_wp = -1
    worst_wp_center = 1e9
    worst_wp_id: Optional[str] = None
    for i, pt in enumerate(points):
        for obs in obstacles:
            d = center_to_aabb_surface_distance_cm(pt, obs)
            if d < worst_wp_center:
                worst_wp_center = d
                worst_wp = i
                worst_wp_id = obs.obstacle_id

    worst_seg = -1
    worst_seg_center = 1e9
    worst_seg_id: Optional[str] = None
    for i in range(len(points) - 1):
        d = min_clearance_on_segment_cm(
            points[i],
            points[i + 1],
            obstacles,
            sample_spacing_cm=SAMPLE_CM,
        )
        if d is not None and d < worst_seg_center:
            worst_seg_center = d
            worst_seg = i
            edge, oid = _min_body_edge_on_segment(points[i], points[i + 1], obstacles)
            worst_seg_id = oid

    return worst_wp, worst_wp_center, worst_seg, worst_seg_center, worst_wp_id, worst_seg_id


def _min_body_edge_on_segment(
    start_xy: WorldXY,
    end_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
) -> Tuple[Optional[float], Optional[str]]:
    seg_len = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    if seg_len < 1e-6:
        return body_edge_to_aabb_surface_distance_cm(
            start_xy, obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
    samples = max(2, int(math.ceil(seg_len / SAMPLE_CM)))
    best: Optional[float] = None
    best_id: Optional[str] = None
    for step in range(samples + 1):
        t = step / samples
        sample_xy = (
            start_xy[0] + (end_xy[0] - start_xy[0]) * t,
            start_xy[1] + (end_xy[1] - start_xy[1]) * t,
        )
        edge, oid = body_edge_to_aabb_surface_distance_cm(
            sample_xy, obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
        if edge is None:
            continue
        if best is None or edge < best:
            best, best_id = edge, oid
    return best, best_id


def _print_violations(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    max_rows: int = 12,
) -> None:
    rows: List[Tuple[float, str, int, str]] = []
    for i, pt in enumerate(points):
        edge, oid = body_edge_to_aabb_surface_distance_cm(
            pt, obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
        center, _ = min(
            (
                (center_to_aabb_surface_distance_cm(pt, obs), obs.obstacle_id)
                for obs in obstacles
            ),
            key=lambda item: item[0],
        )
        if edge is not None and edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
            rows.append((edge, oid or "?", i, "wp"))
    for i in range(len(points) - 1):
        edge, oid = _min_body_edge_on_segment(points[i], points[i + 1], obstacles)
        if edge is not None and edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
            rows.append((edge, oid or "?", i, "seg"))
    rows.sort(key=lambda r: r[0])
    if not rows:
        print("  no body-edge violations in planned path")
        return
    print(f"  violations (worst first, up to {max_rows}):")
    for edge, oid, idx, kind in rows[:max_rows]:
        if kind == "wp":
            lx, ly = lc.world_xy_to_local(points[idx][0], points[idx][1])
            print(
                f"    WP[{idx}] body_edge={edge:.1f}cm @ {oid} "
                f"local=({lx:.0f},{ly:.0f})"
            )
        else:
            print(
                f"    seg[{idx}]→[{idx + 1}] body_edge={edge:.1f}cm @ {oid}"
            )


def _goal_clearance(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
) -> Tuple[Optional[float], Optional[float], Optional[str]]:
    if not points:
        return None, None, None
    goal_xy = points[-1]
    center, oid = nearest_surface_distance_cm(goal_xy, obstacles)
    if center is None:
        return None, None, oid
    return center, center - SPOTDOG_BODY_RADIUS_CM, oid


def _evaluate_leg(
    leg_id: str,
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
) -> LegPlanResult:
    corridor = validate_path_corridor_clearance(
        points,
        obstacles,
        min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
        body_radius_cm=SPOTDOG_BODY_RADIUS_CM,
        sample_spacing_cm=SAMPLE_CM,
    )
    goal_center, goal_edge, goal_id = _goal_clearance(points, obstacles)
    return LegPlanResult(
        leg_id=leg_id,
        corridor_ok=corridor.ok,
        plan_ok=len(points) >= 2,
        waypoint_count=len(points),
        corridor_min_center_cm=corridor.min_center_clearance_cm,
        corridor_min_body_edge_cm=corridor.min_body_edge_clearance_cm,
        corridor_worst_id=corridor.worst_obstacle_id,
        corridor_wp_violations=corridor.violating_wp_count,
        corridor_seg_violations=corridor.violating_segment_count,
        goal_center_cm=goal_center,
        goal_body_edge_cm=goal_edge,
        goal_worst_id=goal_id,
    )


def _print_leg(result: LegPlanResult, points: Sequence[WorldXY], obstacles: Sequence[SurfaceObstacle]) -> None:
    print(f"\n{'=' * 60}")
    print(f"LEG: {result.leg_id}")
    print(f"{'=' * 60}")
    if not result.plan_ok:
        print("  FAIL: no planned path returned")
        return
    print(f"  waypoints={result.waypoint_count}")
    print(
        "  planning constraint: NavFindPathValidated + modifiers use "
        f"center>={NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm "
        f"(body-edge>={PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm) on unspecified props"
    )
    print("  --- corridor (transit WPs except goal; chords not checked) ---")
    print(
        f"  min center={result.corridor_min_center_cm}cm "
        f"min body-edge={result.corridor_min_body_edge_cm}cm "
        f"worst={result.corridor_worst_id}"
    )
    print(
        f"  wp_viol={result.corridor_wp_violations} "
        f"seg_viol={result.corridor_seg_violations}"
    )
    print(
        f"  CORRIDOR PASS (WP >=1m on unspecified props): "
        f"{'YES' if result.corridor_ok else 'NO'}"
    )
    print("  --- goal (SpotDog arrival position, informational) ---")
    print(
        f"  goal center={result.goal_center_cm}cm "
        f"goal body-edge={result.goal_body_edge_cm}cm "
        f"nearest_prop={result.goal_worst_id} "
        f"(target>={planning_goal_position_center_clearance_cm():.0f}cm center)"
    )
    if not result.corridor_ok:
        _print_violations(points[:-1] if len(points) > 1 else [], obstacles)


def _plan_leg(
    ucv,
    nav: str,
    start_xyz: WorldXYZ,
    goal_xyz: WorldXYZ,
    obstacles: Sequence[SurfaceObstacle],
    prop_cache: dict,
    registry,
) -> List[WorldXY]:
    return plan_navmesh_waypoints(
        ucv,
        nav,
        start_xyz,
        goal_xyz,
        path_obstacles=obstacles,
        prop_bounds_cache=prop_cache,
        registry=registry,
    )


def main() -> int:
    layout_id = sys.argv[1] if len(sys.argv) > 1 else "layout_01"
    print("Planning verification: corridor 1m rule (execution proximity ignored)")
    print(
        f"rule: body outer edge to prop AABB surface >= "
        f"{PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm"
    )
    print(
        f"modifier_pad={NAV_PROP_OBSTACLE_PADDING_CM:.0f}cm "
        f"(standoff+margin {NAV_MODIFIER_CLEARANCE_MARGIN_CM:.0f}cm) "
        f"findpath_agent={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm"
    )

    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    if not nav:
        print("FAIL: NavQueryService not found — PIE Play required")
        geh.release_connection(ucv)
        return 1

    registry = build_registry(layout_id=layout_id)
    bounds_cache, setup_ok = setup_static_navmesh_obstacles(ucv, nav, registry)
    if not setup_ok:
        print("WARN: NavMesh obstacle setup incomplete")

    human_bounds = fetch_actor_bounds(ucv, nav, registry.humanoid_actor_name)
    if human_bounds is not None:
        bounds_cache[registry.humanoid_actor_name] = human_bounds

    prop_cache = {
        k: v for k, v in bounds_cache.items() if k != registry.humanoid_actor_name
    }
    exempt = planning_clearance_exempt_actor_names(registry)
    leg1_obstacles = build_path_clearance_obstacles(
        prop_cache, exempt_actor_names=exempt
    )
    leg2_obstacles = build_path_clearance_obstacles(
        prop_cache, exempt_actor_names=exempt
    )
    print(
        f"obstacles: leg1_props={len(leg1_obstacles)} "
        f"leg2_props={len(leg2_obstacles)} exempt={exempt} setup_ok={setup_ok}"
    )

    robot_start = _project_local_xyz(ucv, nav, ROBOT_START_LOCAL_CM)
    material_world = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    robot_world = lc.local_xy_to_world(*ROBOT_START_LOCAL_CM)
    approach_world = pickup_standoff_xy(material_world, robot_world, standoff_cm=160.0)
    approach_world, approach_ok = adjust_xy_for_planning_clearance(
        approach_world,
        leg1_obstacles,
        min_center_clearance_cm=planning_goal_position_center_clearance_cm(),
    )
    if not approach_ok:
        print("WARN: leg1 approach goal could not be shifted to planning clearance")
    leg1_goal = _project_world_xyz(
        ucv, nav, approach_world, z_hints=(lc.NAV_PROJECT_PROBE_Z_CM,)
    )
    material_bounds = fetch_actor_bounds(ucv, nav, registry.material_actor_name)
    mat_z_hints = (material_bounds.cz,) if material_bounds else ()
    leg2_start = _project_world_xyz(ucv, nav, material_world, z_hints=mat_z_hints)
    if leg2_start is None and leg1_goal is not None:
        leg2_start = leg1_goal
        print("WARN: leg2_start projection used leg1_goal fallback")
    leg2_goal = _project_local_xyz(ucv, nav, registry.humanoid_local_cm)

    missing = [
        name
        for name, val in (
            ("robot_start", robot_start),
            ("leg1_goal", leg1_goal),
            ("leg2_start", leg2_start),
            ("leg2_goal", leg2_goal),
        )
        if val is None
    ]
    if missing:
        print(f"FAIL: nav projection failed for {missing}")
        geh.release_connection(ucv)
        return 1

    assert robot_start and leg1_goal and leg2_start and leg2_goal

    leg1_pts = _plan_leg(
        ucv, nav, robot_start, leg1_goal, leg1_obstacles, prop_cache, registry
    )
    leg1_result = _evaluate_leg("leg1 robot→material approach", leg1_pts, leg1_obstacles)
    _print_leg(leg1_result, leg1_pts, leg1_obstacles)

    leg2_pts = _plan_leg(
        ucv, nav, leg2_start, leg2_goal, leg2_obstacles, prop_cache, registry
    )
    leg2_result = _evaluate_leg(
        "leg2 material→humanoid (props only; humanoid exempt)",
        leg2_pts,
        leg2_obstacles,
    )
    _print_leg(leg2_result, leg2_pts, leg2_obstacles)

    all_ok = (
        leg1_result.plan_ok
        and leg1_result.corridor_ok
        and leg2_result.plan_ok
        and leg2_result.corridor_ok
    )
    print(f"\n{'=' * 60}")
    print(
        f"OVERALL PLANNING (corridor 1m rule): {'PASS' if all_ok else 'FAIL'}"
    )
    print("(execution-time proximity not evaluated)")
    print(f"{'=' * 60}")

    geh.release_connection(ucv)
    return 0 if all_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
