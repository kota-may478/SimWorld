#!/usr/bin/env python3
"""Mission-equivalent path probe: plan leg1 corners + optional short moveto follow."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
ST_DIR = THIS_DIR / "scenarios" / "site_transport_20m"
for p in (
    THIS_DIR,
    THIS_DIR.parent / "grid_env_hri",
    THIS_DIR.parent / "grid_env_10k",
    THIS_DIR.parent / "grid_env_depth_perception",
    ST_DIR,
):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_move as nm  # noqa: E402
import nav_query as nq  # noqa: E402
from carry import pickup_standoff_xy  # noqa: E402
from grid_env_10k_pie_patrol import get_yaw  # noqa: E402
from navmesh_mission_nav import (  # noqa: E402
    _goal_xyz_for_planning,
    _start_xyz_for_robot,
    _waypoints_to_path_xyz,
    plan_navmesh_waypoints,
)
from layout_variants import build_layout_registry  # noqa: E402
from navmesh_obstacles import (  # noqa: E402
    planning_clearance_exempt_actor_names,
    setup_static_navmesh_obstacles,
)
from surface_distance import build_path_clearance_obstacles  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME


def _bearing_deg(ax: float, ay: float, bx: float, by: float) -> float:
    return math.degrees(math.atan2(by - ay, bx - ax))


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    registry = build_layout_registry(1)
    ok_nav, nav_actor = nq.ensure_nav_query_service(
        ucv,
        probe_xyz=(
            lc.local_xy_to_world(1500.0, 1500.0)[0],
            lc.local_xy_to_world(1500.0, 1500.0)[1],
            lc.NAV_PROJECT_PROBE_Z_CM,
        ),
    )
    if not ok_nav or ROBOT not in geh.actor_names(ucv):
        print("FAIL: PIE / NavQuery / robot unavailable")
        return 1

    _bounds_cache, nav_ready = setup_static_navmesh_obstacles(ucv, nav_actor, registry)
    if not nav_ready:
        print("FAIL: navmesh obstacles setup")
        return 1
    nm.nav_stop_move(ucv, ROBOT)
    ok_robot, _ = lnr.soft_reset_level_spotdog(
        ucv, (100.0, 100.0), nav_actor=nav_actor
    )
    if not ok_robot:
        print("FAIL: robot soft reset")
        return 1
    time.sleep(0.5)
    exempt = planning_clearance_exempt_actor_names(registry)
    path_obstacles = build_path_clearance_obstacles(_bounds_cache, exempt_actor_names=exempt)
    loc = ucv.get_location(ROBOT)
    robot_xy = (float(loc[0]), float(loc[1]))
    robot_local = lc.world_xy_to_local(*robot_xy)
    yaw = get_yaw(ucv, ROBOT)

    material_world = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    approach_xy = pickup_standoff_xy(material_world, robot_xy, standoff_cm=160.0)
    goal_xyz = _goal_xyz_for_planning(ucv, nav_actor, approach_xy, path_obstacles)
    start_xyz = _start_xyz_for_robot(ucv, nav_actor, robot_xy, ROBOT)
    if goal_xyz is None or start_xyz is None:
        print("FAIL: projection")
        return 1

    waypoints = plan_navmesh_waypoints(
        ucv, nav_actor, start_xyz, goal_xyz, path_obstacles=path_obstacles
    )
    if not waypoints:
        print("FAIL: no plan")
        return 1

    path_xyz = _waypoints_to_path_xyz(ucv, nav_actor, waypoints)
    print(f"robot local={robot_local} yaw={yaw:.1f}")
    print(f"material local={registry.material_pickup_local_cm}")
    print(f"approach world=({approach_xy[0]:.0f},{approach_xy[1]:.0f})")
    print(f"plan: {len(waypoints)} corners")
    for i, (wx, wy) in enumerate(waypoints[:5]):
        lx, ly = lc.world_xy_to_local(wx, wy)
        brg = _bearing_deg(robot_xy[0], robot_xy[1], wx, wy)
        print(
            f"  WP{i} local=({lx:.0f},{ly:.0f}) "
            f"bearing_from_robot={brg:.0f}° dist={math.hypot(wx-robot_xy[0], wy-robot_xy[1]):.0f}cm"
        )
    if len(waypoints) > 5:
        wx, wy = waypoints[-1]
        lx, ly = lc.world_xy_to_local(wx, wy)
        print(f"  WP{len(waypoints)-1} (last) local=({lx:.0f},{ly:.0f})")

    nm.nav_stop_move(ucv, ROBOT)
    time.sleep(0.3)
    # Follow full planned path (mission-equivalent).
    follow = nm.nav_follow_path_json(ucv, ROBOT, path_xyz)
    print(f"dispatch full path ({len(path_xyz)} pts): {json.dumps(follow)}")
    if not follow.get("ok"):
        return 1

    t0 = time.perf_counter()
    last = ""
    while time.perf_counter() - t0 < 90.0:
        st = nm.get_nav_move_status(ucv, ROBOT)
        status = str(st.get("status", ""))
        if status != last:
            loc2 = ucv.get_location(ROBOT)
            loc_l = lc.world_xy_to_local(float(loc2[0]), float(loc2[1]))
            print(
                f"  status={status} wp_index={st.get('wp_index')} "
                f"local=({loc_l[0]:.0f},{loc_l[1]:.0f}) yaw={get_yaw(ucv, ROBOT):.0f}"
            )
            last = status
        if status.lower() in ("success", "failed", "idle"):
            break
        time.sleep(0.5)

    nm.nav_stop_move(ucv, ROBOT)
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
