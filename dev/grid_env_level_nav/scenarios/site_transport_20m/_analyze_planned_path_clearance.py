#!/usr/bin/env python3
"""Verify planned mission paths satisfy 1 m body-edge clearance from prop AABBs."""

from __future__ import annotations

import json
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
from placement import build_registry  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    body_edge_to_aabb_surface_distance_cm,
    center_to_aabb_surface_distance_cm,
    min_clearance_on_segment_cm,
    nearest_surface_distance_cm,
)

WorldXY = Tuple[float, float]
LocalXY = Tuple[float, float]

DEFAULT_ARTIFACT = Path(__file__).resolve().parent / (
    "out/site_transport_trajectory_layout_01_nav_standoff_20260706.json"
)
NAV = "BP_NavQueryService_C_0"
SAMPLE_SPACING_CM = 16.0


def _local_to_world(xy: LocalXY) -> WorldXY:
    return lc.local_xy_to_world(xy[0], xy[1])


def _fetch_live_obstacles(ucv, nav_actor: str) -> List[SurfaceObstacle]:
    reg = build_registry(layout_id="layout_01")
    out: List[SurfaceObstacle] = []
    for prop in reg.props:
        if prop.is_transport_target:
            continue
        raw = nq.get_actor_bounds(ucv, nav_actor, prop.slot_id)
        if not raw.get("ok"):
            continue
        out.append(
            SurfaceObstacle(
                obstacle_id=prop.slot_id,
                cx=float(raw["cx"]),
                cy=float(raw["cy"]),
                half_x=float(raw["half_x"]),
                half_y=float(raw["half_y"]),
            )
        )
    return out


def _min_body_edge_on_segment_cm(
    start_xy: WorldXY,
    end_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    sample_spacing_cm: float = SAMPLE_SPACING_CM,
) -> Tuple[Optional[float], Optional[str]]:
    seg_len = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    if seg_len < 1e-6:
        return body_edge_to_aabb_surface_distance_cm(
            start_xy, obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
    samples = max(2, int(math.ceil(seg_len / max(1.0, sample_spacing_cm))))
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


def _analyze_polyline(
    label: str,
    points_local: Sequence[LocalXY],
    obstacles: Sequence[SurfaceObstacle],
) -> None:
    world_pts = [_local_to_world(p) for p in points_local]
    wp_viol = 0
    wp_min = 1e9
    wp_min_id = ""
    for wx, wy in world_pts:
        edge, oid = body_edge_to_aabb_surface_distance_cm(
            (wx, wy), obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
        if edge is None:
            continue
        if edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
            wp_viol += 1
        if edge < wp_min:
            wp_min, wp_min_id = edge, oid or ""

    seg_viol = 0
    seg_min = 1e9
    seg_min_id = ""
    seg_min_idx = -1
    center_seg_min = 1e9
    for i in range(len(world_pts) - 1):
        edge, oid = _min_body_edge_on_segment_cm(
            world_pts[i], world_pts[i + 1], obstacles
        )
        center = min_clearance_on_segment_cm(
            world_pts[i],
            world_pts[i + 1],
            obstacles,
            sample_spacing_cm=SAMPLE_SPACING_CM,
        )
        if center is not None:
            center_seg_min = min(center_seg_min, center)
        if edge is None:
            continue
        if edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
            seg_viol += 1
        if edge < seg_min:
            seg_min, seg_min_id, seg_min_idx = edge, oid or "", i

    total_seg = max(0, len(world_pts) - 1)
    ok = wp_viol == 0 and seg_viol == 0
    print(f"\n=== {label} ===")
    print(f"waypoints={len(world_pts)} segments={total_seg}")
    print(
        f"WP  body-edge: min={wp_min:.1f}cm @ {wp_min_id}  "
        f"violations={wp_viol}/{len(world_pts)} "
        f"(threshold {PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm)"
    )
    print(
        f"Seg body-edge: min={seg_min:.1f}cm @ {seg_min_id} "
        f"between WP[{seg_min_idx}]→WP[{seg_min_idx + 1}]  "
        f"violations={seg_viol}/{total_seg}"
    )
    print(
        f"Seg center-to-surface: min={center_seg_min:.1f}cm "
        f"(target {NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm)"
    )
    print(f"PASS 1m body-edge rule: {'YES' if ok else 'NO'}")


def _analyze_raw_navfindpath(
    ucv,
    nav_actor: str,
    start_local: LocalXY,
    goal_local: LocalXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    label: str,
) -> None:
    sx, sy = _local_to_world(start_local)
    gx, gy = _local_to_world(goal_local)
    start_raw = nq.nav_project_point(ucv, nav_actor, sx, sy, 6466.0)
    goal_raw = nq.nav_project_point(ucv, nav_actor, gx, gy, 6466.0)
    if not start_raw.get("ok") or not goal_raw.get("ok"):
        print(f"\n=== {label} (raw NavFindPath) === projection failed")
        return
    start_xyz = (float(start_raw["x"]), float(start_raw["y"]), float(start_raw["z"]))
    goal_xyz = (float(goal_raw["x"]), float(goal_raw["y"]), float(goal_raw["z"]))
    raw = nq.nav_find_path(
        ucv,
        nav_actor,
        start_xyz,
        goal_xyz,
        agent_radius_cm=NAV_FINDPATH_AGENT_RADIUS_CM,
    )
    points = nq.path_points_xy(raw)
    if not points:
        print(f"\n=== {label} (raw NavFindPath) === FAILED: {raw}")
        return
    local_pts = [lc.world_xy_to_local(wx, wy) for wx, wy in points]
    _analyze_polyline(f"{label} (raw NavFindPath, agent={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm)", local_pts, obstacles)


def main() -> int:
    artifact = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_ARTIFACT
    if not artifact.is_file():
        print(f"missing artifact: {artifact}")
        return 1

    data = json.loads(artifact.read_text(encoding="utf-8"))
    planned = data.get("planned_paths_local_cm", [])
    traj = data.get("metrics", data).get("trajectory_local_cm", [])
    events = data.get("replan_events", [])

    print(f"artifact: {artifact.name}")
    print(
        f"rule: body outer edge to prop AABB surface >= {PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm "
        f"(body_r={SPOTDOG_BODY_RADIUS_CM:.0f}cm, center target={NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm)"
    )
    print(f"nav obstacle pad={NAV_PROP_OBSTACLE_PADDING_CM:.0f}cm findpath_agent={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm")

    ucv, _ = geh.reconnect_if_needed()
    obstacles = _fetch_live_obstacles(ucv, NAV)
    print(f"live obstacles={len(obstacles)} (GetActorBounds, unpadded AABB)")

    for i, path in enumerate(planned):
        reason = events[i]["reason"] if i < len(events) else f"leg{i + 1}"
        _analyze_polyline(f"planned [{reason}]", path, obstacles)

    if planned:
        _analyze_raw_navfindpath(
            ucv,
            NAV,
            tuple(traj[0]) if traj else tuple(planned[0][0]),
            tuple(planned[0][-1]),
            obstacles,
            label="leg1 replay",
        )

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
