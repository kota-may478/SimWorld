#!/usr/bin/env python3
"""Inspect NavMesh bounds vs scaffold AABB (PIE required)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
NAV = ROOT / "dev" / "grid_env_level_nav"
GEH = ROOT / "dev" / "grid_env_hri"
DEPTH = ROOT / "dev" / "grid_env_depth_perception"
for path in (str(ROOT), str(NAV), str(GEH), str(DEPTH), str(PKG)):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from ue.layout import footprint_local_cm, scaffold_xyz_to_local_cm  # noqa: E402
from ue.placement import ACTOR_PREFIX, RAMP_STEPS, all_spawn_boxes  # noqa: E402

VOLUME_NAMES = (
    "NavMeshBoundsVolume_1",
    "NavMeshBoundsVolume",
    "RecastNavMesh_0",
)
NAV_VOLUME_CENTER_Z_CM = 6680.0


def scaffold_world_aabb() -> dict:
    lx0, ly0, lx1, ly1 = footprint_local_cm()
    corners = [
        lc.local_xyz_to_world(lx0, ly0, 0.0),
        lc.local_xyz_to_world(lx0, ly1, 0.0),
        lc.local_xyz_to_world(lx1, ly0, 0.0),
        lc.local_xyz_to_world(lx1, ly1, 0.0),
        lc.local_xyz_to_world(lx0, ly0, 380.0),
        lc.local_xyz_to_world(lx1, ly1, 380.0),
    ]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    zs = [c[2] for c in corners]
    return {
        "min": [min(xs), min(ys), min(zs)],
        "max": [max(xs), max(ys), max(zs)],
        "center": [
            0.5 * (min(xs) + max(xs)),
            0.5 * (min(ys) + max(ys)),
            0.5 * (min(zs) + max(zs)),
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect NavMesh vs scaffold AABB.")
    parser.add_argument(
        "--set-blocking",
        action="store_true",
        help="enable BP_TransparentCube SetBlocking on existing ScaffoldHrc_*",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="max ScaffoldHrc actors to SetBlocking (0 = all)",
    )
    args = parser.parse_args()
    aabb = scaffold_world_aabb()
    print("scaffold_aabb", json.dumps(aabb))
    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: UnrealCV ping")
        return 1
    names = geh.actor_names(ucv)
    nav_names = [n for n in names if "Nav" in n or "Recast" in n]
    print("nav_actors", json.dumps(nav_names))
    print("scaffold_actors", sum(1 for n in names if n.startswith(f"{ACTOR_PREFIX}_")))
    if "NavMeshBoundsVolume_1" in names:
        loc = ucv.get_location("NavMeshBoundsVolume_1")
        ucv.set_location([float(loc[0]), float(loc[1]), NAV_VOLUME_CENTER_Z_CM], "NavMeshBoundsVolume_1")
        print("raised NavMeshBoundsVolume_1 z", loc[2], "->", NAV_VOLUME_CENTER_Z_CM)

    if args.set_blocking:
        scaffold = [n for n in names if n.startswith(f"{ACTOR_PREFIX}_")]
        scaffold.sort(key=lambda n: ("board" not in n and "deck" not in n, n))
        if args.limit > 0:
            scaffold = scaffold[: args.limit]
        enabled = 0
        for actor in scaffold:
            if geh.set_cube_blocking_mode(ucv, actor, blocking=True, apply_tint=True):
                enabled += 1
        print("set_blocking", enabled, "/", len(scaffold))

    ok, nav_actor = nq.ensure_nav_query_service(ucv)
    print("nav_query", ok, nav_actor)
    if not ok:
        return 1

    for name in VOLUME_NAMES:
        if name not in names:
            continue
        loc = ucv.get_location(name)
        print("loc", name, loc)
        bounds = nq.get_actor_bounds(ucv, nav_actor, name)
        print("bounds", name, json.dumps(bounds, default=str))

    mn = aabb["min"]
    mx = aabb["max"]
    dirty = nq.nav_rebuild_dirty_region(
        ucv,
        nav_actor,
        (mn[0], mn[1], mn[2]),
        (mx[0], mx[1], mx[2]),
        margin_cm=80.0,
    )
    print("dirty_rebuild", json.dumps(dirty, default=str))
    full = nq.nav_rebuild(ucv, nav_actor)
    print("full_rebuild", json.dumps(full, default=str))

    for actor in (
        f"{ACTOR_PREFIX}_deck_f2_b0_r0",
        f"{ACTOR_PREFIX}_deck_f3_b0_r0",
        f"{ACTOR_PREFIX}_ramp_L0_0",
        f"{ACTOR_PREFIX}_ramp_L1_{RAMP_STEPS - 1}",
        f"{ACTOR_PREFIX}_landing_f3_yard_r0",
        f"{ACTOR_PREFIX}_cheek_L0_s_0",
    ):
        if actor in names:
            print("bounds", actor, json.dumps(nq.get_actor_bounds(ucv, nav_actor, actor), default=str))

    site_wx, site_wy = lc.local_xy_to_world(1500.0, 1500.0)
    deck_w = lc.local_xyz_to_world(*scaffold_xyz_to_local_cm(5.0, 0.6, 0.0))
    extra = {
        "site20": nq.nav_project_point(ucv, nav_actor, site_wx, site_wy, lc.NAV_PROJECT_PROBE_Z_CM),
        "deck1": nq.nav_project_point(ucv, nav_actor, deck_w[0], deck_w[1], lc.NAV_PROJECT_PROBE_Z_CM),
        "deck2_above": nq.nav_project_point(ucv, nav_actor, deck_w[0], deck_w[1], 6660.0),
        "deck3_above": nq.nav_project_point(ucv, nav_actor, deck_w[0], deck_w[1], 6840.0),
    }
    ramps = [b for b in all_spawn_boxes() if b.kind == "ramp"]
    if ramps:
        low = ramps[0]
        high = ramps[RAMP_STEPS - 1] if len(ramps) >= RAMP_STEPS else ramps[-1]
        for key, box in (("ramp_low", low), ("ramp_high", high)):
            wx, wy, wz = lc.local_xyz_to_world(*box.local_cm)
            extra[key] = nq.nav_project_point(ucv, nav_actor, wx, wy, wz + 20.0)
    for key, payload in extra.items():
        print("nav", key, json.dumps(payload, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
