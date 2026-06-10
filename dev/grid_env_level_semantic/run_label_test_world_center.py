#!/usr/bin/env python3
"""PIE test: probe floor Z, set initial height, label + place 5×5 near world XY.

Default center (6300, 1170) cm — near camera sample
loc_cm=(6272.47, 1164.16, 6488.92).

Initial block bottom: ``BLOCK_BOTTOM_Z_CM`` (65 m). ``scan_with_height_adjust``
lowers by 0.15 m while all air; on first wall, locks at detection height + 0.15 m.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_level_semantic as lvl  # noqa: E402
import level_camera_probe as lcp  # noqa: E402
from level_region import (  # noqa: E402
    BLOCK_BOTTOM_Z_CM,
    default_level_region,
    subgrid_around_cell,
    world_xy_to_cell_index,
)

DEFAULT_CENTER_XY_CM = (6300.0, 1170.0)
DEFAULT_GRID_HALF = 2
def main() -> int:
    parser = argparse.ArgumentParser(description="Level 5×5 label test at world XY")
    parser.add_argument(
        "--center",
        default=f"{DEFAULT_CENTER_XY_CM[0]},{DEFAULT_CENTER_XY_CM[1]}",
        help="world center x,y in cm (default 6300,1170)",
    )
    parser.add_argument("--half", type=int, default=DEFAULT_GRID_HALF, help="half-size (2→5×5)")
    parser.add_argument("--wait-ue", type=float, default=120.0)
    parser.add_argument("--no-cleanup", action="store_true")
    args = parser.parse_args()

    parts = [float(p.strip()) for p in args.center.split(",")]
    if len(parts) != 2:
        raise SystemExit("--center must be x,y in cm")
    center_x, center_y = parts[0], parts[1]

    if not lcp.wait_for_ue_port(args.wait_ue):
        print("ERROR: UnrealCV not reachable", file=sys.stderr)
        return 2

    ucv, _ = lvl.ensure_connection()
    region = default_level_region()
    gx, gy = world_xy_to_cell_index(region, center_x, center_y)
    cx, cy = region.cell_center_xy_cm(gx, gy)
    subgrid = subgrid_around_cell(gx, gy, half=args.half, region=region)

    initial_z = BLOCK_BOTTOM_Z_CM
    region = default_level_region(block_bottom_z_cm=initial_z)

    print(
        f"[Init] center_xy=({center_x:.1f}, {center_y:.1f}) "
        f"block_bottom_z={initial_z:.1f}cm ({initial_z/100:.3f}m) "
        f"— height scan lowers while all air, +{lvl.HEIGHT_STEP_M:.2f}m on first wall"
    )
    print(
        f"[Subgrid] gx,gy={gx},{gy} cell_center=({cx:.1f},{cy:.1f}) "
        f"rect={subgrid}"
    )

    result = lvl.run_level_semantic_layer(
        ucv,
        region=region,
        cleanup_before=not args.no_cleanup,
        pie_subgrid=subgrid,
    )
    print(
        f"done: registry={result.registry_path} "
        f"final_z={result.geometry.block_bottom_z_cm:.1f}cm "
        f"height_steps={result.height_adjust_steps}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
