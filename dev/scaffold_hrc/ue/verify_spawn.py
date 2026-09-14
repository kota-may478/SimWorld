#!/usr/bin/env python3
"""Verify spawned scaffold actors and NavMesh projection (PIE required)."""

from __future__ import annotations

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
from paths import make_run_dir  # noqa: E402
from ue.layout import footprint_local_cm, scaffold_xyz_to_local_cm  # noqa: E402
from ue.placement import ACTOR_PREFIX  # noqa: E402

SAMPLE_ACTORS = (
    f"{ACTOR_PREFIX}_storage",
    f"{ACTOR_PREFIX}_drop_f1",
    f"{ACTOR_PREFIX}_refuge_f1",
    f"{ACTOR_PREFIX}_deck_f1_b0_r0",
    f"{ACTOR_PREFIX}_deck_f2_b0_r0",
    f"{ACTOR_PREFIX}_deck_f3_b0_r0",
    f"{ACTOR_PREFIX}_landing_f1_r0",
    f"{ACTOR_PREFIX}_landing_f2_sill_r0",
    f"{ACTOR_PREFIX}_landing_f3_exit",
    f"{ACTOR_PREFIX}_landing_f3_south_0",
    f"{ACTOR_PREFIX}_landing_f3_sill",
    f"{ACTOR_PREFIX}_post_0_0_L0",
    f"{ACTOR_PREFIX}_post_0_0_L1",
    f"{ACTOR_PREFIX}_truck",
    f"{ACTOR_PREFIX}_ramp_L0_0",
    f"{ACTOR_PREFIX}_ramp_L1_5",
    f"{ACTOR_PREFIX}_cheek_L0_s_0",
    f"{ACTOR_PREFIX}_cheek_L0_n_0",
)


def _nav_at(ucv, nav_actor: str, x_m: float, y_m: float, z_m: float) -> dict:
    lx, ly, lz = scaffold_xyz_to_local_cm(x_m, y_m, z_m)
    wx, wy, wz = lc.local_xyz_to_world(lx, ly, lz + 10.0)
    raw = nq.nav_project_point(ucv, nav_actor, wx, wy, wz)
    return {"probe_world": [wx, wy, wz], "result": raw}


def main() -> int:
    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: UnrealCV ping")
        return 1
    names = [n for n in geh.actor_names(ucv) if n.startswith(f"{ACTOR_PREFIX}_")]
    print("scaffold_actors", len(names))
    samples = {}
    for actor in SAMPLE_ACTORS:
        if actor not in names:
            samples[actor] = None
            print("missing", actor)
            continue
        loc = ucv.get_location(actor)
        samples[actor] = loc
        print("loc", actor, loc)

        leftover_long = f"{ACTOR_PREFIX}_post_0_0"
        leftover_slab = f"{ACTOR_PREFIX}_deck_f1"
        leftover_stringer = f"{ACTOR_PREFIX}_stringer_L0_outer"
        print("leftover_long_post", leftover_long in names)
        print("leftover_slab", leftover_slab in names)
        print("leftover_stringer", leftover_stringer in names)
    print("post_actors", sum(1 for n in names if "_post_" in n))

    ok, nav_actor = nq.ensure_nav_query_service(ucv)
    print("nav_actor", ok, nav_actor)
    if ok:
        for actor in (f"{ACTOR_PREFIX}_post_0_0_L0", f"{ACTOR_PREFIX}_post_0_0_L1"):
            if actor in names:
                print(
                    "bounds",
                    actor,
                    json.dumps(nq.get_actor_bounds(ucv, nav_actor, actor), default=str),
                )
    probes = {
        "deck_1f": _nav_at(ucv, nav_actor, 5.0, 0.6, 0.0) if ok else {},
        "deck_2f": _nav_at(ucv, nav_actor, 5.0, 0.6, 2.15) if ok else {},
        "deck_3f": _nav_at(ucv, nav_actor, 5.0, 0.6, 3.95) if ok else {},
        "stair_1to2": _nav_at(ucv, nav_actor, -1.4, 0.55, 0.9) if ok else {},
        "stair_2to3": _nav_at(ucv, nav_actor, -0.9, 1.85, 2.7) if ok else {},
    }
    for key, payload in probes.items():
        print("nav", key, json.dumps(payload, default=str))

    lx0, ly0, lx1, ly1 = footprint_local_cm()
    cx = 0.5 * (lx0 + lx1)
    cy = 0.5 * (ly0 + ly1)
    cam_w = lc.local_xyz_to_world(cx, cy, 1200.0)
    run_dir = make_run_dir()
    shot = run_dir / "scaffold_overview.png"
    try:
        ucv.set_camera_location(0, cam_w)
        ucv.set_camera_rotation(0, (-70.0, 0.0, 0.0))
        ucv.get_image(0, "lit", mode="file", img_path=str(shot))
        print("screenshot", shot)
    except Exception as exc:
        print("screenshot_fail", exc)
    out = run_dir / "spawn_verify.json"
    out.write_text(
        json.dumps(
            {"n_actors": len(names), "samples": samples, "probes": probes},
            indent=2,
            default=str,
        )
        + "\n",
        encoding="utf-8",
    )
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
