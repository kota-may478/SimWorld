#!/usr/bin/env python3
"""Deep NavMesh root-cause probes (PIE required)."""

from __future__ import annotations

import json
import math
import sys
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
from navmesh_config import NAV_PLANNING_CENTER_CLEARANCE_CM, NAV_PROP_OBSTACLE_PADDING_CM
from navmesh_obstacles import fetch_actor_bounds, setup_static_navmesh_obstacles
from placement import build_registry  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402
from surface_distance import (
    SurfaceObstacle,
    build_surface_obstacles_from_bounds,
    center_to_aabb_surface_distance_cm,
    validate_path_center_clearance,
)

WorldXY = Tuple[float, float]
PROBE_PROPS = ("site20_prop_019", "site20_prop_023")


def _project(ucv, nav: str, wx: float, wy: float, z: float) -> dict:
    return nq.nav_project_point(ucv, nav, wx, wy, z)


def _project_local(ucv, nav: str, local_xy: Tuple[float, float]) -> Optional[Tuple[float, float, float]]:
    wx, wy = lc.local_xy_to_world(local_xy[0], local_xy[1])
    for z in (lc.NAV_PROJECT_PROBE_Z_CM, 6466.0, 6490.0, 6455.0, 6565.0):
        raw = _project(ucv, nav, wx, wy, z)
        if raw.get("ok"):
            return float(raw["x"]), float(raw["y"]), float(raw["z"])
    return None


def _ring_probe(
    ucv,
    nav: str,
    cx: float,
    cy: float,
    cz: float,
    half_x: float,
    half_y: float,
    *,
    radii_cm: Sequence[float],
) -> None:
    print(f"  ring probe center=({cx:.0f},{cy:.0f}) half=({half_x:.0f},{half_y:.0f})")
    for r in radii_cm:
        hits = 0
        samples = 8
        for i in range(samples):
            ang = 2.0 * math.pi * i / samples
            px = cx + (half_x + r) * math.cos(ang)
            py = cy + (half_y + r) * math.sin(ang)
            raw = _project(ucv, nav, px, py, cz)
            if raw.get("ok"):
                hits += 1
        print(f"    r=+{r:.0f}cm outside AABB: {hits}/{samples} project_ok")


def _corners(ucv, nav, start, goal, agent: float = 170.0) -> List[WorldXY]:
    raw = nq.nav_find_path(ucv, nav, start, goal, agent_radius_cm=agent)
    return nq.path_points_xy(raw)


def _max_corner_delta(a: Sequence[WorldXY], b: Sequence[WorldXY]) -> float:
    if len(a) != len(b):
        return float("inf")
    return max(math.hypot(ax - bx, ay - by) for (ax, ay), (bx, by) in zip(a, b))


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    if not nav:
        print("FAIL: no NavQueryService")
        return 1

    registry = build_registry(layout_id="layout_01")
    start = _project_local(ucv, nav, ROBOT_START_LOCAL_CM)
    material_world = lc.local_xy_to_world(*registry.material_pickup_local_cm)
    approach = pickup_standoff_xy(
        material_world,
        lc.local_xy_to_world(*ROBOT_START_LOCAL_CM),
        standoff_cm=160.0,
    )
    goal = _project_local(ucv, nav, lc.world_xy_to_local(*approach))
    if start is None or goal is None:
        print("FAIL: projection")
        geh.release_connection(ucv)
        return 1

    print("=== 1. Corner coordinates: clear vs full obstacles ===")
    nq.nav_clear_box_obstacles(ucv, nav)
    nq.nav_rebuild(ucv, nav)
    clear_corners = _corners(ucv, nav, start, goal)
    bounds_cache, ok = setup_static_navmesh_obstacles(ucv, nav, registry)
    full_corners = _corners(ucv, nav, start, goal)
    print(f"  clear corners={len(clear_corners)} full corners={len(full_corners)} setup_ok={ok}")
    print(f"  max corner delta (cm): {_max_corner_delta(clear_corners, full_corners):.3f}")
    for i, ((cx, cy), (fx, fy)) in enumerate(zip(clear_corners, full_corners)):
        d = math.hypot(cx - fx, cy - fy)
        lx, ly = lc.world_xy_to_local(cx, cy)
        print(f"    [{i}] local=({lx:.0f},{ly:.0f}) clear=({cx:.0f},{cy:.0f}) full=({fx:.0f},{fy:.0f}) d={d:.2f}cm")

    prop_cache = {k: v for k, v in bounds_cache.items() if k != registry.humanoid_actor_name}
    obstacles = build_surface_obstacles_from_bounds(prop_cache)
    report = validate_path_center_clearance(
        full_corners,
        obstacles,
        min_center_clearance_cm=NAV_PLANNING_CENTER_CLEARANCE_CM,
    )
    print(
        f"  full path clearance: min_center={report.min_center_clearance_cm:.1f}cm "
        f"worst={report.worst_obstacle_id} PASS={report.ok}"
    )

    print("\n=== 2. NavProjectPoint ring around violating props ===")
    for prop_id in PROBE_PROPS:
        bounds = fetch_actor_bounds(ucv, nav, prop_id)
        if bounds is None:
            print(f"  {prop_id}: no bounds")
            continue
        center_raw = _project(ucv, nav, bounds.cx, bounds.cy, bounds.cz)
        print(f"  {prop_id} center project: {center_raw}")
        _ring_probe(
            ucv,
            nav,
            bounds.cx,
            bounds.cy,
            bounds.cz,
            bounds.half_x,
            bounds.half_y,
            radii_cm=(0.0, 50.0, 100.0, 170.0, 185.0, 250.0),
        )
        pad = NAV_PROP_OBSTACLE_PADDING_CM
        _ring_probe(
            ucv,
            nav,
            bounds.cx,
            bounds.cy,
            bounds.cz,
            bounds.half_x + pad,
            bounds.half_y + pad,
            radii_cm=(0.0, 50.0, 100.0),
        )

    print("\n=== 3. Modifier effective half (expected vs projection at padded edge) ===")
    for prop_id in PROBE_PROPS:
        bounds = fetch_actor_bounds(ucv, nav, prop_id)
        if bounds is None:
            continue
        pad = NAV_PROP_OBSTACLE_PADDING_CM
        hx, hy = bounds.half_x + pad, bounds.half_y + pad
        # East edge midpoint of padded modifier box
        ex, ey = bounds.cx + hx, bounds.cy
        raw = _project(ucv, nav, ex, ey, bounds.cz)
        print(
            f"  {prop_id} padded east edge (+{hx:.0f}cm): "
            f"world=({ex:.0f},{ey:.0f}) project={raw.get('ok')} "
            f"expected: False if modifier carved"
        )
        # Just inside padded box (should not project if carved)
        ix, iy = bounds.cx + hx - 20.0, bounds.cy
        raw_in = _project(ucv, nav, ix, iy, bounds.cz)
        print(
            f"  {prop_id} inside padded box (-20cm from edge): "
            f"project={raw_in.get('ok')}"
        )

    print("\n=== 4. Spawned modifier actors in level (UnrealCV) ===")
    try:
        actors = geh._ue_request(ucv, "vget /objects", timeout_s=15.0)  # noqa: SLF001
        if isinstance(actors, str):
            lines = [ln.strip() for ln in actors.splitlines() if ln.strip()]
            nav_mods = [ln for ln in lines if "NavModifier" in ln or "nav_obs" in ln.lower()]
            print(f"  NavModifier/nav_obs actors ({len(nav_mods)}):")
            for name in sorted(nav_mods)[:40]:
                print(f"    {name}")
            if len(nav_mods) > 40:
                print(f"    ... +{len(nav_mods) - 40} more")
        else:
            print(f"  vget /objects unexpected: {type(actors)}")
    except Exception as exc:
        print(f"  vget /objects failed: {exc}")

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
