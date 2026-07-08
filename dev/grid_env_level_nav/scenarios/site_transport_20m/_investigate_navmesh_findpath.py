#!/usr/bin/env python3
"""Investigate why FindPathSync ignores NavModifier / AgentRadius (PIE required).

Tests (in order):
  1. NavProjectPoint on prop center: clear vs padded modifier
  2. FindPath corner count / hash: no obstacles vs full setup vs pad=15 vs pad=185
  3. AgentRadius sweep on same navmesh state
  4. Timing: FindPath immediately after NavRebuild vs after settle
"""

from __future__ import annotations

import hashlib
import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from carry import pickup_standoff_xy  # noqa: E402
from navmesh_config import (
    NAV_FINDPATH_AGENT_RADIUS_CM,
    NAV_MESH_HULL_PADDING_CM,
    NAV_PLANNING_CENTER_CLEARANCE_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
    NAV_REBUILD_SETTLE_S,
)
from navmesh_obstacles import (
    fetch_actor_bounds,
    register_box_obstacle,
    setup_static_navmesh_obstacles,
)
from placement import build_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402

WorldXY = Tuple[float, float]
PROBE_PROP = "site20_prop_019"  # worst offender in clearance analysis


def _path_fingerprint(points: Sequence[WorldXY]) -> str:
    if not points:
        return "empty"
    payload = json.dumps([[round(x, 1), round(y, 1)] for x, y in points])
    return hashlib.md5(payload.encode()).hexdigest()[:12]


def _project_local(ucv, nav: str, local_xy: Tuple[float, float]) -> Optional[Tuple[float, float, float]]:
    wx, wy = lc.local_xy_to_world(local_xy[0], local_xy[1])
    for z in (lc.NAV_PROJECT_PROBE_Z_CM, 6466.0, 6490.0, 6455.0, 6450.0):
        raw = nq.nav_project_point(ucv, nav, wx, wy, z)
        if raw.get("ok"):
            return float(raw["x"]), float(raw["y"]), float(raw["z"])
    return None


def _find_path(ucv, nav: str, start, goal, agent: float) -> Tuple[bool, int, str, List[WorldXY]]:
    raw = nq.nav_find_path(ucv, nav, start, goal, agent_radius_cm=agent)
    pts = nq.path_points_xy(raw)
    return bool(raw.get("ok")), len(pts), _path_fingerprint(pts), pts


def _project_prop_center(ucv, nav: str, prop_id: str) -> dict:
    bounds = nq.get_actor_bounds(ucv, nav, prop_id)
    if not bounds.get("ok"):
        return {"ok": False, "error": "no_bounds"}
    cx, cy = float(bounds["cx"]), float(bounds["cy"])
    cz = float(bounds.get("cz", lc.NAV_PROJECT_PROBE_Z_CM))
    raw = nq.nav_project_point(ucv, nav, cx, cy, cz)
    return {
        "ok": bool(raw.get("ok")),
        "cx": cx,
        "cy": cy,
        "projected": raw,
    }


def _register_one_prop(ucv, nav: str, prop_id: str, pad_cm: float) -> bool:
    bounds = fetch_actor_bounds(ucv, nav, prop_id)
    if bounds is None:
        return False
    nq.nav_clear_box_obstacles(ucv, nav)
    ok = register_box_obstacle(
        ucv, nav, bounds, half_extent_pad_cm=pad_cm
    )
    if not ok:
        return False
    rebuild = nq.nav_rebuild(ucv, nav)
    return bool(rebuild.get("ok"))


def main() -> int:
    print("=== NavMesh FindPath / Modifier investigation ===")
    print(
        f"config: hull={NAV_MESH_HULL_PADDING_CM:.0f}cm "
        f"modifier_pad={NAV_PROP_OBSTACLE_PADDING_CM:.0f}cm "
        f"findpath_agent={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm "
        f"planning_clearance={NAV_PLANNING_CENTER_CLEARANCE_CM:.0f}cm"
    )

    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    if not nav:
        print("FAIL: no NavQueryService (PIE Play required)")
        return 1

    registry = build_registry(layout_id="layout_01")
    start = _project_local(ucv, nav, ROBOT_START_LOCAL_CM)
    material_world = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    approach_world = pickup_standoff_xy(
        material_world,
        lc.local_xy_to_world(*ROBOT_START_LOCAL_CM),
        standoff_cm=160.0,
    )
    goal = _project_local(ucv, nav, lc.world_xy_to_local(*approach_world))
    if start is None or goal is None:
        print(f"FAIL: projection start={start} goal={goal}")
        geh.release_connection(ucv)
        return 1

    # --- Test A: projection on prop center (navmesh walkability probe) ---
    print("\n--- A: NavProjectPoint @ prop center ---")
    nq.nav_clear_box_obstacles(ucv, nav)
    nq.nav_rebuild(ucv, nav)
    time.sleep(NAV_REBUILD_SETTLE_S)
    proj_clear = _project_prop_center(ucv, nav, PROBE_PROP)
    print(f"  after clear+rebuild: {json.dumps(proj_clear, default=str)}")

    for pad in (15.0, 185.0):
        if not _register_one_prop(ucv, nav, PROBE_PROP, pad):
            print(f"  pad={pad:.0f}: register/rebuild failed")
            continue
        time.sleep(NAV_REBUILD_SETTLE_S)
        proj = _project_prop_center(ucv, nav, PROBE_PROP)
        print(f"  pad={pad:.0f}cm only {PROBE_PROP}: project_ok={proj['ok']}")

    # --- Test B: full obstacle setup, path fingerprint ---
    print("\n--- B: FindPath path fingerprint (layout leg1) ---")
    scenarios = []

    def record(label: str, agent: float = 170.0) -> None:
        ok, count, fp, _ = _find_path(ucv, nav, start, goal, agent)
        scenarios.append((label, ok, count, fp, agent))
        print(f"  {label:40s} agent={agent:5.0f} ok={ok} corners={count} fp={fp}")

    nq.nav_clear_box_obstacles(ucv, nav)
    nq.nav_rebuild(ucv, nav)
    time.sleep(NAV_REBUILD_SETTLE_S)
    record("no_obstacles")

    bounds_cache, setup_ok = setup_static_navmesh_obstacles(ucv, nav, registry)
    print(f"  full setup: props={len(bounds_cache)} rebuild_ok={setup_ok}")
    record("full_obstacles_default_pad")
    for agent in (15.0, 170.0, 185.0, 200.0, 220.0):
        record(f"full_obstacles_agent_{agent:.0f}", agent)

    # --- Test C: timing after rebuild ---
    print("\n--- C: FindPath timing after NavRebuild ---")
    nq.nav_clear_box_obstacles(ucv, nav)
    rebuild = nq.nav_rebuild(ucv, nav)
    for delay in (0.0, 0.5, 1.5, 3.0):
        if delay > 0:
            time.sleep(delay)
        ok, count, fp, _ = _find_path(ucv, nav, start, goal, 170.0)
        print(
            f"  delay={delay:.1f}s after clear+rebuild ok={ok} corners={count} fp={fp} "
            f"(rebuild_ok={rebuild.get('ok')})"
        )

    # Re-run full setup for timing with obstacles
    setup_static_navmesh_obstacles(ucv, nav, registry)
    rebuild = nq.nav_rebuild(ucv, nav)
    for delay in (0.0, NAV_REBUILD_SETTLE_S, 3.0):
        if delay > 0:
            time.sleep(delay)
        ok, count, fp, _ = _find_path(ucv, nav, start, goal, 170.0)
        print(
            f"  delay={delay:.1f}s after full setup ok={ok} corners={count} fp={fp}"
        )

    # --- Summary ---
    print("\n--- Summary ---")
    fps = {s[3] for s in scenarios}
    agents = [s for s in scenarios if s[0].startswith("full_obstacles_agent")]
    agent_fps = {s[3] for s in agents}
    print(f"  distinct path fingerprints (all scenarios): {len(fps)}")
    print(f"  distinct fingerprints (agent sweep): {len(agent_fps)}")
    if len(agent_fps) == 1 and agents:
        print(
            "  => AgentRadius 15–220 cm yields IDENTICAL geometry "
            "(query-time radius not carving corridor)"
        )
    clear_fp = next((s[3] for s in scenarios if s[0] == "no_obstacles"), None)
    full_fp = next((s[3] for s in scenarios if s[0] == "full_obstacles_default_pad"), None)
    if clear_fp == full_fp:
        print(
            "  => FindPath WITH vs WITHOUT modifiers: SAME path "
            "(modifiers not affecting navmesh used by FindPathSync)"
        )
    else:
        print("  => FindPath changes when modifiers registered (modifiers DO affect mesh)")

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
