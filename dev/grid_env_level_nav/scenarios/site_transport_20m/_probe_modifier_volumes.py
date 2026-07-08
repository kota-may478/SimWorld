#!/usr/bin/env python3
"""Check NavModifierVolume transforms and navmesh Z plane."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402
from navmesh_obstacles import fetch_actor_bounds, setup_static_navmesh_obstacles  # noqa: E402
from placement import build_registry  # noqa: E402

NAV_Z = 6450.0
PROBE_PROP = "site20_prop_019"


def _loc(ucv, name: str) -> tuple:
    loc = ucv.get_location(name)
    return float(loc[0]), float(loc[1]), float(loc[2])


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    nav = nq.find_nav_query_actor(ucv)
    print(f"nav_actor={nav}")
    registry = build_registry(layout_id="layout_01")
    setup_static_navmesh_obstacles(ucv, nav, registry)

    bounds = fetch_actor_bounds(ucv, nav, PROBE_PROP)
    if bounds is None:
        print("no bounds")
        geh.release_connection(ucv)
        return 1

    print(f"\n{PROBE_PROP} bounds: center=({bounds.cx:.0f},{bounds.cy:.0f},{bounds.cz:.0f}) "
          f"half=({bounds.half_x:.0f},{bounds.half_y:.0f})")

    # Find a NavModifierVolume near prop
    objects_raw = geh._ue_request(ucv, "vget /objects", timeout_s=15.0)  # noqa: SLF001
    mod_names = []
    all_tokens: list[str] = []
    if isinstance(objects_raw, str):
        all_tokens = objects_raw.split()
        for token in all_tokens:
            if token.startswith("NavModifierVolume_"):
                mod_names.append(token)
    print(f"NavModifierVolume count={len(mod_names)}")

    # Sample first/last modifier locations
    for name in (mod_names[0], mod_names[-1]) if mod_names else []:
        try:
            x, y, z = _loc(ucv, name)
            print(f"  {name} location=({x:.0f},{y:.0f},{z:.0f})")
            b = nq.get_actor_bounds(ucv, nav, name)
            if b.get("ok"):
                print(
                    f"    GetActorBounds: center=({b['cx']:.0f},{b['cy']:.0f},{b['cz']:.0f}) "
                    f"half=({b['half_x']:.0f},{b['half_y']:.0f},{b['half_z']:.0f})"
                )
        except Exception as exc:
            print(f"  {name}: {exc}")

    # Ring at NAV_Z (floor navmesh) vs prop CZ
    cx, cy = bounds.cx, bounds.cy
    hx, hy = bounds.half_x, bounds.half_y
    for z_label, z in [("nav_floor", NAV_Z), ("prop_cz", bounds.cz)]:
        print(f"\nring @ z={z_label} ({z:.0f})")
        for r in (0, 50, 100, 200, 300):
            hits = 0
            for i in range(8):
                ang = 2 * math.pi * i / 8
                px = cx + (hx + r) * math.cos(ang)
                py = cy + (hy + r) * math.sin(ang)
                raw = nq.nav_project_point(ucv, nav, px, py, z)
                if raw.get("ok"):
                    hits += 1
            print(f"  r=+{r}cm: {hits}/8 project_ok")

    # List all NavQueryService actors
    nav_svcs = [t for t in all_tokens if "BP_NavQueryService" in t]
    print(f"\nNavQueryService actors ({len(nav_svcs)}): {nav_svcs}")

    geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
