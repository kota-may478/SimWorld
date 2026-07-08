#!/usr/bin/env python3
"""Smoke: NavFindPathValidated + planning obstacle registration (requires PIE + rebuilt UE)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402
from level_coords import NAV_PROJECT_PROBE_Z_CM, local_xy_to_world  # noqa: E402
from navmesh_config import (  # noqa: E402
    NAV_FINDPATH_AGENT_RADIUS_CM,
    NAV_PATH_RESAMPLE_SPACING_CM,
    NAV_PLANNING_CENTER_CLEARANCE_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
)

START_LOCAL = (100.0, 100.0)
GOAL_LOCAL = (840.0, -350.0)
PROBE_PROP = "site20_prop_000"


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    nav_actor = nq.find_nav_query_actor(ucv)
    if not nav_actor:
        print("FAIL: NavQueryService actor not found")
        return 1

    if not nq.nav_validated_api_available(ucv, nav_actor):
        print(
            "SKIP: NavFindPathValidated API missing — copy ue_native/NavQueryService.* "
            "to UE project and rebuild"
        )
        geh.release_connection(ucv)
        return 0

    bounds = nq.get_actor_bounds(ucv, nav_actor, PROBE_PROP)
    if not bounds.get("ok"):
        print(f"FAIL: bounds for {PROBE_PROP}: {bounds}")
        geh.release_connection(ucv)
        return 1

    nq.nav_register_planning_obstacle(
        ucv,
        nav_actor,
        PROBE_PROP,
        float(bounds["cx"]),
        float(bounds["cy"]),
        float(bounds["half_x"]),
        float(bounds["half_y"]),
    )
    pad = NAV_PROP_OBSTACLE_PADDING_CM
    nq.nav_register_box_obstacle(
        ucv,
        nav_actor,
        f"{PROBE_PROP}_nav",
        (float(bounds["cx"]), float(bounds["cy"]), float(bounds["cz"])),
        (
            float(bounds["half_x"]) + pad,
            float(bounds["half_y"]) + pad,
            120.0,
        ),
    )
    rebuild = nq.nav_rebuild(ucv, nav_actor)
    print(f"NavRebuild: {rebuild}")

    sx, sy = local_xy_to_world(*START_LOCAL)
    gx, gy = local_xy_to_world(*GOAL_LOCAL)
    start = nq.nav_project_point(ucv, nav_actor, sx, sy, NAV_PROJECT_PROBE_Z_CM)
    goal = nq.nav_project_point(ucv, nav_actor, gx, gy, NAV_PROJECT_PROBE_Z_CM)
    if not start.get("ok") or not goal.get("ok"):
        print(f"FAIL: projection start={start} goal={goal}")
        geh.release_connection(ucv)
        return 1

    start_xyz = (float(start["x"]), float(start["y"]), float(start["z"]))
    goal_xyz = (float(goal["x"]), float(goal["y"]), float(goal["z"]))
    raw = nq.nav_find_path_validated(
        ucv,
        nav_actor,
        start_xyz,
        goal_xyz,
        agent_radius_cm=NAV_FINDPATH_AGENT_RADIUS_CM,
        min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
        resample_spacing_cm=NAV_PATH_RESAMPLE_SPACING_CM,
    )
    print(
        f"NavFindPathValidated ok={raw.get('ok')} "
        f"points={raw.get('point_count', len(raw.get('points', [])))} "
        f"corners={raw.get('corner_point_count')} "
        f"error={raw.get('error')}"
    )
    geh.release_connection(ucv)
    if not raw.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
