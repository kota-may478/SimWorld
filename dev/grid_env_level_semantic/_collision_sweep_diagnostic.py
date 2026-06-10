#!/usr/bin/env python3
"""Sweep block-bottom Z at verify XY — depth surface Z + semantic labels (Level PIE)."""
from __future__ import annotations

import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from level_semantic_scan import (  # noqa: E402
    classify_cell_depth,
    compute_depth_sample_cam_z_cm,
    depth_band_hits,
    surface_z_cm_at_xy,
)
from spawn_fixed_height_verify import VERIFY_CENTER_XY_CM  # noqa: E402


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    x, y = VERIFY_CENTER_XY_CM
    z0 = 6500.0
    step = 15.0
    steps = 12
    block_h = geh.CUBE_SIZE_CM
    depth_cam_z = compute_depth_sample_cam_z_cm(z0, block_h)

    print(f"center_xy_cm=({x}, {y}) z0={z0} step_cm={step} depth_cam_z={depth_cam_z}")
    surface_z = surface_z_cm_at_xy(ucv, x, y, sample_cam_z_cm=depth_cam_z)
    print(f"fixed surface_z_cm={surface_z}")
    for i in range(steps + 1):
        bottom_z = z0 - i * step
        z_lower = bottom_z - block_h
        hit0 = depth_band_hits(surface_z, bottom_z, block_h)
        hit_lo = depth_band_hits(surface_z, z_lower, block_h)
        sem, _surface_z = classify_cell_depth(
            ucv,
            x,
            y,
            z_initial_bottom_cm=bottom_z,
            block_height_cm=block_h,
            depth_sample_cam_z_cm=depth_cam_z,
        )
        print(
            f"bottom_z={bottom_z:7.1f}cm z_lower={z_lower:7.1f}cm "
            f"hit_z0={hit0} hit_zlo={hit_lo} label={sem}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
