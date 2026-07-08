#!/usr/bin/env python3
"""Print NavRegisterBoxObstacle actual_* bounds from API response."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402
from navmesh_config import NAV_PROP_OBSTACLE_PADDING_CM  # noqa: E402

PROP = "site20_prop_019"


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    bounds = nq.get_actor_bounds(ucv, nav, PROP)
    print("prop bounds:", json.dumps(bounds))
    if not bounds.get("ok"):
        geh.release_connection(ucv)
        return 1
    pad = NAV_PROP_OBSTACLE_PADDING_CM
    half = (
        float(bounds["half_x"]) + pad,
        float(bounds["half_y"]) + pad,
        120.0,
    )
    nq.nav_clear_box_obstacles(ucv, nav)
    raw = nq.nav_register_box_obstacle(
        ucv,
        nav,
        f"{PROP}_test",
        (float(bounds["cx"]), float(bounds["cy"]), float(bounds["cz"])),
        half,
    )
    print("register:", json.dumps(raw, indent=2))
    rebuild = nq.nav_rebuild(ucv, nav)
    print("rebuild:", rebuild)
    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
