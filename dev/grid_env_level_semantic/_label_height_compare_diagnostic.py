#!/usr/bin/env python3
"""Per-cell collision labels at two block-bottom Z heights (diagnostic)."""
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
import level_collision_probe as lcp  # noqa: E402
from level_region import default_level_region, subgrid_around_cell, world_xy_to_cell_index  # noqa: E402
from level_semantic_scan import classify_cell_collision  # noqa: E402

CENTER_XY = (6300.0, 1170.0)
HEIGHTS_CM = (6500.0, 6485.0)
BLOCK_H = geh.CUBE_SIZE_CM


def _probe_detail(
    ucv, actor: str, x: float, y: float, z: float, radius: float,
) -> dict:
    raw = lcp.parse_probe_hit(  # noqa: SLF001
        lcp._vbp_probe_hit(ucv, actor, x, y, z, radius_cm=radius),
    )
    return {
        "z": z,
        "hit": lcp.probe_point_blocks(raw),
        "raw": raw,
    }


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok, actor = lcp.ensure_collision_probe(ucv)
    if not ok:
        print("probe spawn failed")
        return 1
    region = default_level_region()
    gx, gy = world_xy_to_cell_index(region, *CENTER_XY)
    subgrid = subgrid_around_cell(gx, gy, half=2, region=region)
    gx0, gy0, gx1, gy1 = subgrid
    from level_semantic_scan import (  # noqa: WPS433
        cube_center_z_cm,
        cube_inscribed_probe_radius_cm,
        label_floor_probe_bottom_cm,
        label_wall_probe_bottom_cm,
    )

    radius = cube_inscribed_probe_radius_cm(BLOCK_H)
    print(f"subgrid={subgrid} center_r={radius}cm block_h={BLOCK_H}cm")
    print("wall: center@(z_place+2m); floor/air: center@(z_place+2m-2.30m)")
    print()
    for z0 in HEIGHTS_CM:
        z_wall = label_wall_probe_bottom_cm(z0)
        z_floor = label_floor_probe_bottom_cm(z0)
        wall_ctr = cube_center_z_cm(z_wall, BLOCK_H)
        floor_ctr = cube_center_z_cm(z_floor, BLOCK_H)
        print(
            f"=== z_place={z0:.1f}cm  "
            f"wall_ctr={wall_ctr:.1f} floor_ctr={floor_ctr:.1f} r={radius} ==="
        )
        for gx_i in range(gx0, gx1 + 1):
            row = []
            for gy_i in range(gy0, gy1 + 1):
                x, y = region.cell_center_xy_cm(gx_i, gy_i)
                sem, _ = classify_cell_collision(
                    ucv, x, y,
                    z_place_bottom_cm=z0,
                    block_height_cm=BLOCK_H,
                    probe_actor=actor,
                )
                hi = _probe_detail(ucv, actor, x, y, wall_ctr, radius)
                lo = _probe_detail(ucv, actor, x, y, floor_ctr, radius)
                row.append(
                    f"({gx_i},{gy_i}) {sem} "
                    f"[hi={hi['hit']} lo={lo['hit']}]"
                )
            print("  " + " | ".join(row))
        counts = {"wall": 0, "floor": 0, "air": 0}
        for gx_i in range(gx0, gx1 + 1):
            for gy_i in range(gy0, gy1 + 1):
                x, y = region.cell_center_xy_cm(gx_i, gy_i)
                sem, _ = classify_cell_collision(
                    ucv, x, y,
                    z_place_bottom_cm=z0,
                    block_height_cm=BLOCK_H,
                    probe_actor=actor,
                )
                counts[sem] += 1
        print(f"  totals: {counts}")
        print()
    # Z sweep at one floor cell vs one air cell
    for tag, cell in [("floor_cell", (3, 156)), ("air_cell", (3, 159))]:
        x, y = region.cell_center_xy_cm(*cell)
        print(f"--- Z sweep {tag} gx,gy={cell} xy=({x:.1f},{y:.1f}) ---")
        hits = []
        for z in range(6600, 6360, -5):
            raw = lcp.parse_probe_hit(lcp._vbp_probe_hit(ucv, actor, x, y, float(z)))  # noqa: SLF001
            if lcp.probe_point_blocks(raw):
                hits.append((z, raw))
        print(f"  probe hits (5cm steps 6600→6365): {len(hits)}")
        for z, raw in hits[:8]:
            print(f"    z={z}: {json.dumps(raw, ensure_ascii=False)}")
        if len(hits) > 8:
            print(f"    ... +{len(hits) - 8} more")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
