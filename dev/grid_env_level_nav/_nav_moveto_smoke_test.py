#!/usr/bin/env python3
"""Phase 5 Step 4 smoke: NavFollowPathJson + status polling (requires PIE)."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (
    THIS_DIR,
    THIS_DIR.parent / "grid_env_hri",
    THIS_DIR.parent / "grid_env_10k",
):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_move as nm  # noqa: E402
import nav_query as nq  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME
POLL_S = 0.5
MAX_POLLS = 60


def _project_xyz(ucv, nav_actor: str, x: float, y: float) -> tuple[float, float, float]:
    raw = nq.nav_project_point(ucv, nav_actor, x, y, lc.NAV_PROJECT_PROBE_Z_CM)
    if not raw.get("ok"):
        raise RuntimeError(f"NavProjectPoint failed @ ({x:.0f},{y:.0f}): {raw}")
    return (
        float(raw["x"]),
        float(raw["y"]),
        float(raw.get("z", lc.NAV_PROJECT_PROBE_Z_CM)),
    )


def _plan_short_path(ucv, nav_actor: str, robot: str) -> list[tuple[float, float, float]]:
    loc = ucv.get_location(robot)
    start_xy = (float(loc[0]), float(loc[1]))
    goal_xy = (start_xy[0] + 300.0, start_xy[1])
    start_xyz = _project_xyz(ucv, nav_actor, start_xy[0], start_xy[1])
    goal_xyz = _project_xyz(ucv, nav_actor, goal_xy[0], goal_xy[1])
    raw = nq.nav_find_path(ucv, nav_actor, start_xyz, goal_xyz)
    if not raw.get("ok"):
        raise RuntimeError(f"NavFindPath failed: {raw}")
    points = raw.get("points") or []
    if len(points) < 2:
        raise RuntimeError(f"path too short: {len(points)} points")
    return [(float(p["x"]), float(p["y"]), float(p["z"])) for p in points]


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok_nav, nav_actor = nq.ensure_nav_query_service(
        ucv,
        probe_xyz=(
            lc.local_xy_to_world(1500.0, 1500.0)[0],
            lc.local_xy_to_world(1500.0, 1500.0)[1],
            lc.NAV_PROJECT_PROBE_Z_CM,
        ),
    )
    if not ok_nav:
        print("FAIL: NavQueryService unavailable")
        return 1

    if ROBOT not in geh.actor_names(ucv):
        print(f"FAIL: robot {ROBOT} not in level")
        return 1

    status0 = nm.get_nav_move_status(ucv, ROBOT)
    print(f"initial status: {json.dumps(status0, separators=(',', ':'))}")
    if not nm.nav_move_api_available(ucv, ROBOT):
        print("FAIL: NavMove vbp wrappers missing (Steps 1–3 incomplete)")
        return 1

    try:
        path_xyz = _plan_short_path(ucv, nav_actor, ROBOT)
    except RuntimeError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(f"path points: {len(path_xyz)}")

    follow = nm.nav_follow_path_json(ucv, ROBOT, path_xyz)
    print(f"NavFollowPathJson: {json.dumps(follow, separators=(',', ':'))}")
    if not follow.get("ok"):
        print("FAIL: NavFollowPathJson rejected path")
        return 1

    last_status = ""
    for i in range(MAX_POLLS):
        status = nm.get_nav_move_status(ucv, ROBOT)
        st = str(status.get("status", "")).lower()
        if st != last_status:
            print(f"poll {i}: {json.dumps(status, separators=(',', ':'))}")
            last_status = st
        if st in ("success", "failed", "idle"):
            if st == "success":
                loc = ucv.get_location(ROBOT)
                print(
                    f"PASS: success @ ({loc[0]:.0f},{loc[1]:.0f}) "
                    f"after {i * POLL_S:.1f}s"
                )
                return 0
            print(f"FAIL: terminal status={st} detail={status}")
            nm.nav_stop_move(ucv, ROBOT)
            return 1
        time.sleep(POLL_S)

    print("FAIL: timeout waiting for success")
    nm.nav_stop_move(ucv, ROBOT)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
