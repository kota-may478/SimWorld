#!/usr/bin/env python3
"""Semantic labeling for the elevated corner test (geometric AABB, PIE-safe).

Labeling rule (per cell, before blocks are placed):

1. ``z_initial`` = temp floor top + 0.15 m — intended block **bottom** height.
2. Probe at ``z_initial``: collision → **wall** (existing geometry at placement height).
3. Else probe at ``z_lower`` = ``z_initial`` − 0.30 m (one block height):
   collision → **floor**; else → **air**.

If ``z_initial`` is correct, the temp floor slab (top below ``z_initial``) cannot
produce a wall label — only a lowered probe can hit it for floor.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Literal, Tuple

BlockSemantic = Literal["wall", "floor", "air"]
BlockIndex = Tuple[int, int]

PROBE_RADIUS_CM = 10.0


@dataclass(frozen=True)
class ObstacleBox:
    """Axis-aligned obstacle volume in UE world cm."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    source: str = ""

    def contains_probe(self, x: float, y: float, z: float, *, radius_cm: float) -> bool:
        return (
            self.x_min - radius_cm <= x <= self.x_max + radius_cm
            and self.y_min - radius_cm <= y <= self.y_max + radius_cm
            and self.z_min <= z <= self.z_max
        )


def classify_semantic(*, hit_at_z_initial: bool, hit_at_z_lower: bool) -> BlockSemantic:
    if hit_at_z_initial:
        return "wall"
    if hit_at_z_lower:
        return "floor"
    return "air"


def probe_point_hits(
    x_cm: float,
    y_cm: float,
    z_cm: float,
    obstacles: Iterable[ObstacleBox],
    *,
    radius_cm: float = PROBE_RADIUS_CM,
) -> bool:
    for box in obstacles:
        if box.contains_probe(x_cm, y_cm, z_cm, radius_cm=radius_cm):
            return True
    return False


def classify_cell_at_heights(
    x_cm: float,
    y_cm: float,
    *,
    z_initial_bottom_cm: float,
    block_height_cm: float,
    obstacles: List[ObstacleBox],
) -> BlockSemantic:
    z_lower_cm = z_initial_bottom_cm - block_height_cm
    hit_initial = probe_point_hits(x_cm, y_cm, z_initial_bottom_cm, obstacles)
    hit_lower = probe_point_hits(x_cm, y_cm, z_lower_cm, obstacles)
    return classify_semantic(hit_at_z_initial=hit_initial, hit_at_z_lower=hit_lower)


def scan_region_semantics(
    ucv,
    cells: list[BlockIndex],
    *,
    cell_center_xy_cm_fn,
    z_initial_bottom_cm: float,
    block_height_cm: float,
    obstacles: List[ObstacleBox],
    progress_every: int = 10,
) -> Dict[BlockIndex, BlockSemantic]:
    del ucv
    results: Dict[BlockIndex, BlockSemantic] = {}
    total = len(cells)
    t0 = time.monotonic()
    for i, (gx, gy) in enumerate(cells, start=1):
        x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
        sem = classify_cell_at_heights(
            x_cm,
            y_cm,
            z_initial_bottom_cm=z_initial_bottom_cm,
            block_height_cm=block_height_cm,
            obstacles=obstacles,
        )
        results[(gx, gy)] = sem
        if progress_every > 0 and (i % progress_every == 0 or i == total):
            elapsed = time.monotonic() - t0
            print(
                f"[SemanticScan] {i}/{total} "
                f"last=({gx},{gy})->{sem} "
                f"z0={z_initial_bottom_cm:.1f} z-={z_initial_bottom_cm - block_height_cm:.1f} "
                f"elapsed={elapsed:.1f}s"
            )
    return results
