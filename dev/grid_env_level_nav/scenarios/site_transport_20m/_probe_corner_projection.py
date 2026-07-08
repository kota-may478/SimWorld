#!/usr/bin/env python3
"""Probe NavProjectPoint at FindPath corner XY."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from carry import pickup_standoff_xy  # noqa: E402
from navmesh_obstacles import setup_static_navmesh_obstacles  # noqa: E402
from placement import build_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    registry = build_registry(layout_id="layout_01")
    setup_static_navmesh_obstacles(ucv, nav, registry)

    material = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    approach = pickup_standoff_xy(
        material,
        lc.local_xy_to_world(*ROBOT_START_LOCAL_CM),
        160.0,
    )
    goal_local = lc.world_xy_to_local(*approach)

    start_xyz = None
    wx, wy = lc.local_xy_to_world(*ROBOT_START_LOCAL_CM)
    for z in (6450, 6466, 6490, 6565):
        raw = nq.nav_project_point(ucv, nav, wx, wy, z)
        if raw.get("ok"):
            start_xyz = (float(raw["x"]), float(raw["y"]), float(raw["z"]))
            print(f"start proj z={z} -> {start_xyz}")
            break

    gx, gy = lc.local_xy_to_world(*goal_local)
    goal_xyz = None
    for z in (6450, 6466, 6490, 6565):
        raw = nq.nav_project_point(ucv, nav, gx, gy, z)
        if raw.get("ok"):
            goal_xyz = (float(raw["x"]), float(raw["y"]), float(raw["z"]))
            print(f"goal proj z={z} -> {goal_xyz}")
            break

    if start_xyz is None or goal_xyz is None:
        print("FAIL projection")
        geh.release_connection(ucv)
        return 1

    raw = nq.nav_find_path(ucv, nav, start_xyz, goal_xyz, agent_radius_cm=170.0)
    pts = nq.path_points_xy(raw)
    print(f"corners={len(pts)} ok={raw.get('ok')}")
    z_list = [6450, 6466, 6490, 6565, 6580]
    for i, (x, y) in enumerate(pts):
        lx, ly = lc.world_xy_to_local(x, y)
        found = False
        for z in z_list:
            r = nq.nav_project_point(ucv, nav, x, y, z)
            if r.get("ok"):
                px, py = float(r["x"]), float(r["y"])
                dist = math.hypot(px - x, py - y)
                print(
                    f"  [{i}] local=({lx:.0f},{ly:.0f}) corner=({x:.0f},{y:.0f}) "
                    f"proj@z={z} dist={dist:.1f}cm pz={r.get('z')}"
                )
                found = True
                break
        if not found:
            print(f"  [{i}] local=({lx:.0f},{ly:.0f}) NO PROJECTION")

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
