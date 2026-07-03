#!/usr/bin/env python3
"""NavFindPath check #2 — official coords vs projected (mission-equivalent)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402

START_LOCAL = (500.0, 500.0)
GOAL_LOCAL = (5000.0, 6000.0)
AGENT_RADIUS_CM = 100.0


def _proj(ucv, nav_actor: str, xyz):
    raw = nq.nav_project_point(ucv, nav_actor, xyz[0], xyz[1], lc.NAV_PROJECT_PROBE_Z_CM)
    if raw.get("ok"):
        return (
            float(raw["x"]),
            float(raw["y"]),
            float(raw.get("z", lc.NAV_PROJECT_PROBE_Z_CM)),
        ), raw
    return xyz, raw


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    try:
        start_raw = lc.foot_world_xyz_from_local_xy(*START_LOCAL)
        goal_raw = lc.foot_world_xyz_from_local_xy(*GOAL_LOCAL)
        ok, nav_actor = nq.ensure_nav_query_service(ucv, probe_xyz=start_raw)
        print(f"NavQueryService: {nav_actor!r} ok={ok}")
        print(f"START local={START_LOCAL} world={start_raw}")
        print(f"GOAL  local={GOAL_LOCAL} world={goal_raw}")

        # A: official smoke (no projection)
        path_a = nq.nav_find_path(ucv, nav_actor, start_raw, goal_raw)
        print("\n[A] official smoke (foot Z, default radius):")
        print(json.dumps(path_a, ensure_ascii=False))

        # B: projected start/goal
        start_p, start_proj = _proj(ucv, nav_actor, start_raw)
        goal_p, goal_proj = _proj(ucv, nav_actor, goal_raw)
        print("\n[B] projected start:", json.dumps(start_proj))
        print("    projected goal:", json.dumps(goal_proj))
        path_b = nq.nav_find_path(
            ucv,
            nav_actor,
            start_p,
            goal_p,
            agent_radius_cm=AGENT_RADIUS_CM,
        )
        print("[B] NavFindPathWithRadius after projection:")
        print(json.dumps({"ok": path_b.get("ok"), "n_pts": len(path_b.get("points", [])), "error": path_b.get("error")}))

        # C: interior walkable (1500,1500) like NavProjectPoint smoke
        interior = lc.foot_world_xyz_from_local_xy(1500.0, 1500.0)
        interior_p, _ = _proj(ucv, nav_actor, interior)
        goal2_p, _ = _proj(ucv, nav_actor, lc.foot_world_xyz_from_local_xy(3000.0, 3000.0))
        path_c = nq.nav_find_path(
            ucv,
            nav_actor,
            interior_p,
            goal2_p,
            agent_radius_cm=AGENT_RADIUS_CM,
        )
        print("\n[C] interior (1500,1500)→(3000,3000) projected + radius 100:")
        print(json.dumps({"ok": path_c.get("ok"), "n_pts": len(path_c.get("points", [])), "error": path_c.get("error")}))

        robot = geh.ROBOT_ACTOR_NAME
        names = geh.actor_names(ucv)
        if robot in names:
            loc = ucv.get_location(robot)
            robot_xyz = (float(loc[0]), float(loc[1]), float(loc[2]))
            robot_p, _ = _proj(ucv, nav_actor, robot_xyz)
            path_d = nq.nav_find_path(
                ucv,
                nav_actor,
                robot_p,
                goal2_p,
                agent_radius_cm=AGENT_RADIUS_CM,
            )
            print(f"\n[D] robot {robot} @ {robot_xyz} → (3000,3000):")
            print(json.dumps({"ok": path_d.get("ok"), "n_pts": len(path_d.get("points", [])), "error": path_d.get("error")}))
        else:
            print(f"\n[D] robot {robot!r} not spawned in PIE")

        passed = bool(path_b.get("ok") or path_c.get("ok"))
        return 0 if passed else 2
    finally:
        geh.release_connection(ucv)


if __name__ == "__main__":
    raise SystemExit(main())
