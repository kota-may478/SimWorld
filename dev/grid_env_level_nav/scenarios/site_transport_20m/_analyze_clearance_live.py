#!/usr/bin/env python3
"""Fetch live GetActorBounds and analyze clearance on saved trajectory."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from navmesh_config import (  # noqa: E402
    NAV_PLANNING_AGENT_RADIUS_CM,
    PROXIMITY_EDGE_FROM_SURFACE_CM,
    SPOTDOG_BODY_RADIUS_CM,
)
from placement import build_registry  # noqa: E402
from surface_distance import (  # noqa: E402
    SurfaceObstacle,
    body_edge_to_aabb_surface_distance_cm,
    min_clearance_on_segment_cm,
)

DEFAULT_ARTIFACT = Path(__file__).resolve().parent / (
    "out/site_transport_trajectory_layout_01_nav_standoff_20260706.json"
)
NAV = "BP_NavQueryService_C_0"


def main() -> int:
    artifact = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_ARTIFACT
    data = json.loads(artifact.read_text(encoding="utf-8"))
    metrics = data["metrics"]
    traj = metrics["trajectory_local_cm"]
    planned = data.get("planned_paths_local_cm", [[]])[0]

    ucv, _ = geh.reconnect_if_needed()
    reg = build_registry(layout_id="layout_01")
    obstacles = []
    for prop in reg.props:
        raw = nq.get_actor_bounds(ucv, NAV, prop.slot_id)
        if not raw.get("ok"):
            continue
        obstacles.append(
            SurfaceObstacle(
                obstacle_id=prop.slot_id,
                cx=float(raw["cx"]),
                cy=float(raw["cy"]),
                half_x=float(raw["half_x"]),
                half_y=float(raw["half_y"]),
            )
        )
    geh.release_connection(ucv)

    rb = [o for o in obstacles if o.obstacle_id >= "site20_prop_017"]

    def analyze(label: str, points_local):
        world = [lc.local_xy_to_world(lx, ly) for lx, ly in points_local]
        min_edge = 1e9
        min_edge_id = ""
        min_center_seg = 1e9
        viol = 0
        for lx, ly in points_local:
            wx, wy = lc.local_xy_to_world(lx, ly)
            edge, oid = body_edge_to_aabb_surface_distance_cm(
                (wx, wy), obstacles, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
            )
            if edge is not None and edge < PROXIMITY_EDGE_FROM_SURFACE_CM:
                viol += 1
            if edge is not None and edge < min_edge:
                min_edge, min_edge_id = edge, oid or ""
        for i in range(len(world) - 1):
            c = min_clearance_on_segment_cm(world[i], world[i + 1], obstacles, sample_spacing_cm=16.0)
            if c is not None:
                min_center_seg = min(min_center_seg, c)
        print(f"{label}: n={len(points_local)} min_body_edge={min_edge:.1f}cm@{min_edge_id} viol={viol} min_center_seg={min_center_seg:.1f}cm (target {NAV_PLANNING_AGENT_RADIUS_CM})")

    print("=== Live GetActorBounds analysis ===")
    print(f"obstacles={len(obstacles)} roadblocks={len(rb)}")
    analyze("executed", traj)
    analyze("planned_leg1", planned)
    if rb:
        world = [lc.local_xy_to_world(lx, ly) for lx, ly in traj]
        rb_min = 1e9
        rb_id = ""
        for wx, wy in world:
            edge, oid = body_edge_to_aabb_surface_distance_cm(
                (wx, wy), rb, body_radius_cm=SPOTDOG_BODY_RADIUS_CM
            )
            if edge is not None and edge < rb_min:
                rb_min, rb_id = edge, oid or ""
        print(f"executed roadblocks_only: min_body_edge={rb_min:.1f}cm@{rb_id}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
