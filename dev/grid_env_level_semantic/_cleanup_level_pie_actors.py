#!/usr/bin/env python3
"""Remove level_sem_block_* and level_sem_verify_* actors from PIE (no spawn)."""
from __future__ import annotations

import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import grid_env_level_semantic as lvl  # noqa: E402
import level_camera_probe as lcp  # noqa: E402
from level_region import (  # noqa: E402
    default_level_region,
    subgrid_around_cell,
    world_xy_to_cell_index,
)
from spawn_fixed_height_verify import cleanup_verify_actors  # noqa: E402

DEFAULT_CENTER_XY_CM = (6300.0, 1170.0)


def main() -> int:
    if not lcp.wait_for_ue_port(30.0):
        print("ERROR: UnrealCV not reachable", file=sys.stderr)
        return 2
    ucv, _ = g10k.ensure_connection()
    cleanup_verify_actors(ucv)
    region = default_level_region()
    gx, gy = world_xy_to_cell_index(region, *DEFAULT_CENTER_XY_CM)
    subgrid = subgrid_around_cell(gx, gy, half=2, region=region)
    lvl.cleanup_level_semantic_layer(ucv, region, subgrid=subgrid)
    removed = 0
    for name in sorted(geh.actor_names(ucv)):
        if name.startswith("level_sem_block_") or name.startswith("level_sem_verify_"):
            if geh.actor_exists(ucv, name):
                geh.destroy_actor_safely(ucv, name, max_attempts=3, timeout_s=12.0)
                geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)
                removed += 1
                print(f"[Cleanup] removed stray {name}")
    print(f"[Cleanup] done (extra_stray={removed})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
