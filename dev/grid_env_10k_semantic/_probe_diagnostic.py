#!/usr/bin/env python3
"""Diagnose GetCollisionNum for semantic probe candidates."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
for p in (ROOT, ROOT / "dev" / "grid_env_hri", ROOT / "dev" / "grid_env_10k", THIS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k_semantic as sem  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from block_semantic_scan import parse_collision_counts  # noqa: E402

CANDIDATES = [
    ("/Game/CityDatabase/blueprints/BP_Box.BP_Box_C", "sem_probe_box"),
    (geh.CUBE_BP, "sem_probe_cube"),
]


def try_probe(ucv, bp: str, name: str, loc: tuple[float, float, float]) -> dict:
    geh.destroy_if_exists(ucv, name)
    if not geh.spawn_bp(ucv, bp, name):
        return {"error": "spawn_failed"}
    ucv.set_scale((0.2, 0.2, 0.2), name)
    ucv.set_physics(name, False)
    ucv.set_collision(name, True)
    ucv.set_movable(name, True)
    ucv.set_location(loc, name)
    time.sleep(0.1)
    try:
        ucv.tick()
    except Exception:
        pass
    time.sleep(0.1)
    raw = ucv.get_collision_num(name)
    counts = parse_collision_counts(raw)
    geh.destroy_if_exists(ucv, name)
    return {"raw": str(raw), "counts": counts}


def main() -> int:
    ucv, _ = sem.ensure_connection()
    geom = sem.compute_layer_geometry(floor_top_z_cm=geh.resolve_floor_top_z_cm(ucv))
    x, y = sem.cell_center_world_xy_cm(1, 1)
    z_low = geom.block_bottom_z_cm - geh.CUBE_SIZE_CM
    z_high = geom.block_bottom_z_cm
    locs = {
        "on_temp_floor_low": (x, y, z_low),
        "block_bottom": (x, y, z_high),
        "demo_wall": sem.cell_center_world_xy_cm(4, 4) + (z_high,),
    }
    print(f"geometry block_bottom={z_high:.1f} z_low={z_low:.1f}")
    for label, loc in locs.items():
        print(f"\n=== {label} {loc} ===")
        for bp, name in CANDIDATES:
            result = try_probe(ucv, bp, name, loc)
            print(f"  {name}: {json.dumps(result, ensure_ascii=False)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
