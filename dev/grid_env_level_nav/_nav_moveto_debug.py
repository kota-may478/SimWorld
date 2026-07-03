#!/usr/bin/env python3
"""Debug NavFollowPathJson vs direct Move_Speed interaction."""

from __future__ import annotations

import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_move as nm  # noqa: E402
import nav_query as nq  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME
CTRL = "BP_SpotDogAIController_C_0"


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    _, nav_actor = nq.ensure_nav_query_service(
        ucv, probe_xyz=(0.0, 0.0, lc.NAV_PROJECT_PROBE_Z_CM)
    )
    nm.nav_stop_move(ucv, ROBOT)
    time.sleep(0.2)

    loc0 = ucv.get_location(ROBOT)
    x, y = float(loc0[0]), float(loc0[1])

    def proj(ax: float, ay: float) -> tuple[float, float, float]:
        raw = nq.nav_project_point(ucv, nav_actor, ax, ay, lc.NAV_PROJECT_PROBE_Z_CM)
        return (
            float(raw["x"]),
            float(raw["y"]),
            float(raw.get("z", lc.NAV_PROJECT_PROBE_Z_CM)),
        )

    start = proj(x, y)
    goal = proj(x + 300.0, y)
    path = nq.nav_find_path(ucv, nav_actor, start, goal)
    pts = [(float(p["x"]), float(p["y"]), float(p["z"])) for p in path["points"]]
    js = nm.path_points_to_json(pts)
    geh._ue_request(ucv, f"vbp {CTRL} NavFollowPathJson {js}", timeout_s=30.0)
    time.sleep(0.05)
    print("status after follow:", nm.get_nav_move_status(ucv, ROBOT))

    loc1 = ucv.get_location(ROBOT)
    geh._ue_request(ucv, f"vbp {ROBOT} Move_Speed 180 0.5 0", timeout_s=10.0)
    time.sleep(0.6)
    loc2 = ucv.get_location(ROBOT)
    print(
        "loc0", loc0,
        "loc1", loc1,
        "loc2", loc2,
        "delta during moving",
        float(loc2[0]) - float(loc0[0]),
        float(loc2[1]) - float(loc0[1]),
    )
    print("status after manual move:", nm.get_nav_move_status(ucv, ROBOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
