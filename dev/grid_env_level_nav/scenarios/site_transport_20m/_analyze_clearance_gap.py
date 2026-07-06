#!/usr/bin/env python3
"""Compare planned vs executed clearance to fences (body-edge 1 m rule)."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import level_coords as lc  # noqa: E402
from navmesh_config import (  # noqa: E402
    NAV_PLANNING_AGENT_RADIUS_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
    NAV_ROADBLOCK_OBSTACLE_PADDING_CM,
    PROXIMITY_EDGE_FROM_SURFACE_CM,
    SPOTDOG_BODY_RADIUS_CM,
)
from placement import SiteTransportRegistry, build_registry  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    body_edge_to_aabb_surface_distance_cm,
    center_to_aabb_surface_distance_cm,
    min_clearance_on_segment_cm,
    nearest_surface_distance_cm,
)

WorldXY = Tuple[float, float]
LocalXY = Tuple[float, float]

ARTIFACT = Path(__file__).resolve().parent / (
    "out/site_transport_trajectory_layout_01_bp_walk_20260706.json"
)

# Typical BP_Roadblock_03b GetActorBounds half-extents (cm) from UE measurements.
ROADBLOCK_HALF_XY = (60.0, 30.0)
PROP_HALF_DEFAULT = (91.0, 151.0)


def _local_to_world(xy: LocalXY) -> WorldXY:
    return lc.local_xy_to_world(xy[0], xy[1])


def _load_obstacles_from_registry() -> List[SurfaceObstacle]:
    reg = build_registry(layout_id="layout_01")
    out: List[SurfaceObstacle] = []
    for prop in reg.props:
        wx, wy = _local_to_world(prop.local_xy_cm)
        if prop.cluster_id == "no_entry_roadblock":
            hx, hy = ROADBLOCK_HALF_XY
        else:
            hx, hy = PROP_HALF_DEFAULT
        out.append(
            SurfaceObstacle(
                obstacle_id=prop.slot_id,
                cx=wx,
                cy=wy,
                half_x=hx,
                half_y=hy,
            )
        )
    return out


def _load_obstacles_nav_padded(obstacles: Sequence[SurfaceObstacle], reg: SiteTransportRegistry) -> List[SurfaceObstacle]:
    """Obstacles as registered on NavMesh (half-extent padding)."""
    by_id = {p.slot_id: p for p in reg.props}
    padded: List[SurfaceObstacle] = []
    for obs in obstacles:
        prop = by_id.get(obs.obstacle_id)
        pad = NAV_PROP_OBSTACLE_PADDING_CM
        if prop is not None and prop.cluster_id == "no_entry_roadblock":
            pad += NAV_ROADBLOCK_OBSTACLE_PADDING_CM
        padded.append(
            SurfaceObstacle(
                obstacle_id=obs.obstacle_id + "_nav",
                cx=obs.cx,
                cy=obs.cy,
                half_x=obs.half_x + pad,
                half_y=obs.half_y + pad,
            )
        )
    return padded


def _roadblocks(obstacles: Sequence[SurfaceObstacle], reg: SiteTransportRegistry) -> List[SurfaceObstacle]:
    rb_ids = {p.slot_id for p in reg.props if p.cluster_id == "no_entry_roadblock"}
    return [o for o in obstacles if o.obstacle_id in rb_ids]


def _min_body_edge_on_polyline(
    points_local: Sequence[LocalXY],
    obstacles: Sequence[SurfaceObstacle],
) -> Tuple[Optional[float], Optional[str], int]:
    best: Optional[float] = None
    best_id: Optional[str] = None
    viol = 0
    for lx, ly in points_local:
        wx, wy = _local_to_world((lx, ly))
        edge, oid = body_edge_to_aabb_surface_distance_cm(
            (wx, wy), obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
        )
        if edge is not None and edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
            viol += 1
        if edge is not None and (best is None or edge < best):
            best, best_id = edge, oid
    return best, best_id, viol


def _min_center_on_polyline_segments(
    points_local: Sequence[LocalXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    sample_spacing_cm: float = 16.0,
) -> Tuple[Optional[float], Optional[str]]:
    world_pts = [_local_to_world(p) for p in points_local]
    best: Optional[float] = None
    best_id: Optional[str] = None
    for i in range(len(world_pts) - 1):
        c = min_clearance_on_segment_cm(
            world_pts[i],
            world_pts[i + 1],
            obstacles,
            sample_spacing_cm=sample_spacing_cm,
        )
        if c is None:
            continue
        if best is None or c < best:
            best = c
            _, oid = nearest_surface_distance_cm(
                (
                    (world_pts[i][0] + world_pts[i + 1][0]) * 0.5,
                    (world_pts[i][1] + world_pts[i + 1][1]) * 0.5,
                ),
                obstacles,
            )
            best_id = oid
    return best, best_id


def _simulate_bp_steps(
    points_local: Sequence[LocalXY],
    *,
    step_cm: float = 90.0,
) -> List[LocalXY]:
    """Rough BP walk: from each WP bearing, advance step_cm along chord to next WP."""
    if len(points_local) < 2:
        return list(points_local)
    out: List[LocalXY] = [points_local[0]]
    for i in range(len(points_local) - 1):
        lx0, ly0 = out[-1]
        tx, ty = points_local[i + 1]
        dx, dy = tx - lx0, ty - ly0
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            continue
        move = min(step_cm, dist)
        out.append((lx0 + dx / dist * move, ly0 + dy / dist * move))
    return out


def main() -> int:
    if not ARTIFACT.is_file():
        print(f"missing artifact: {ARTIFACT}")
        return 1
    data = json.loads(ARTIFACT.read_text(encoding="utf-8"))
    metrics = data.get("metrics", data)
    traj = metrics["trajectory_local_cm"]
    planned = data.get("planned_paths_local_cm", [])
    viol = metrics.get("violations", {})

    reg = build_registry(layout_id="layout_01")
    obstacles = _load_obstacles_from_registry()
    roadblocks = _roadblocks(obstacles, reg)

    print("=== Clearance gap analysis (layout_01 bp_walk 20260706) ===")
    print(
        f"metrics body_edge_violation_rate={viol.get('body_edge_proximity_violation_rate')} "
        f"threshold={PROXIMITY_EDGE_FROM_SURFACE_CM}cm body_r={SPOTDOG_BODY_RADIUS_CM}cm"
    )
    print(f"obstacles={len(obstacles)} roadblocks={len(roadblocks)}")
    print()

    for label, points in [("executed_trajectory", traj)]:
        edge_min, edge_id, edge_viol = _min_body_edge_on_polyline(points, obstacles)
        edge_rb_min, rb_id, _ = _min_body_edge_on_polyline(points, roadblocks)
        print(f"[{label}] samples={len(points)}")
        print(f"  min body-edge (all obs): {edge_min:.1f}cm @ {edge_id}  viol_samples={edge_viol}")
        print(f"  min body-edge (roadblocks only): {edge_rb_min:.1f}cm @ {rb_id}")

    for leg_idx, path in enumerate(planned):
        if not path:
            continue
        edge_min, edge_id, edge_viol = _min_body_edge_on_polyline(path, obstacles)
        edge_rb_min, rb_id, _ = _min_body_edge_on_polyline(path, roadblocks)
        center_seg_min, _ = _min_center_on_polyline_segments(path, obstacles)
        center_seg_rb, _ = _min_center_on_polyline_segments(path, roadblocks)
        sim = _simulate_bp_steps(path)
        sim_edge, sim_id, sim_viol = _min_body_edge_on_polyline(sim, obstacles)
        print()
        print(f"[planned_leg{leg_idx + 1}] waypoints={len(path)}")
        print(
            f"  WP min body-edge: {edge_min:.1f}cm @ {edge_id}  "
            f"viol_WPs={edge_viol} (threshold {PROXIMITY_EDGE_FROM_SURFACE_CM}cm)"
        )
        print(f"  WP min body-edge (roadblocks): {edge_rb_min:.1f}cm @ {rb_id}")
        print(
            f"  segment min center-to-surface: {center_seg_min:.1f}cm "
            f"(planning target {NAV_PLANNING_AGENT_RADIUS_CM}cm)"
        )
        print(f"  segment min center (roadblocks): {center_seg_rb:.1f}cm")
        print(
            f"  BP step simulation ({len(sim)} pts): min body-edge={sim_edge:.1f}cm @ {sim_id} "
            f"viol={sim_viol}"
        )

    # Nav padded obstacle check on first planned leg
    if planned:
        padded = _load_obstacles_nav_padded(obstacles, reg)
        c_min, _ = _min_center_on_polyline_segments(planned[0], padded)
        print()
        print(f"[nav_padded_obstacles leg1] segment min center-to-nav-box: {c_min:.1f}cm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
