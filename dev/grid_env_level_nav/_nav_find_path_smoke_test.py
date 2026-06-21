#!/usr/bin/env python3
"""Smoke test: NavFindPath across work region (requires PIE + NavMesh)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402

START_LOCAL = (500.0, 500.0)
GOAL_LOCAL = (5000.0, 6000.0)


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    start = lc.foot_world_xyz_from_local_xy(*START_LOCAL)
    goal = lc.foot_world_xyz_from_local_xy(*GOAL_LOCAL)
    ok, name = nq.ensure_nav_query_service(ucv, probe_xyz=start)
    if not ok:
        print("FAIL: NavQueryService unavailable")
        return 1

    raw = nq.nav_find_path(ucv, name, start, goal)
    print(f"NavFindPath raw keys={list(raw.keys())}")
    if raw.get("ok"):
        n_pts = len(raw.get("points", []))
        print(f"OK: {n_pts} waypoints")
        if n_pts:
            print(f"  first={raw['points'][0]} last={raw['points'][-1]}")
        return 0

    print(f"FAIL: {json.dumps(raw, ensure_ascii=False)}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
