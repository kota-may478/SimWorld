#!/usr/bin/env python3
"""Collision-probe helpers for wall / floor / air semantic classification."""

from __future__ import annotations

import json
import time
from typing import Dict, Literal, Optional, Tuple

BlockSemantic = Literal["wall", "floor", "air"]
BlockIndex = Tuple[int, int]
WorldXYZ = Tuple[float, float, float]

PROBE_NAME = "sem_collision_probe"
PROBE_BP_PATH = "/Game/CityDatabase/blueprints/BP_Box.BP_Box_C"
PROBE_SCALE = (0.12, 0.12, 0.12)
PROBE_SETTLE_S = 0.03

SEMANTIC_COLORS: Dict[BlockSemantic, Tuple[float, float, float]] = {
    "wall": (1.0, 0.15, 0.15),
    "floor": (0.15, 0.85, 0.2),
    "air": (0.25, 0.45, 1.0),
}


def classify_semantic(*, hit_at_z0: bool, hit_at_z_low: bool) -> BlockSemantic:
    """Classify a column from probe hits at z0 and z0 - block_height."""
    if hit_at_z0:
        return "wall"
    if hit_at_z_low:
        return "floor"
    return "air"


def parse_collision_counts(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return raw_response
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if not text or text.lower().startswith("error"):
            return {}
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {}
    return {}


def collision_indicates_obstacle(counts: dict) -> bool:
    building = int(counts.get("BuildingCollision", 0) or 0)
    obj = int(counts.get("ObjectCollision", 0) or 0)
    return (building + obj) > 0


def spawn_collision_probe(ucv, location: WorldXYZ) -> None:
    objects = {str(n) for n in ucv.get_objects().tolist()}
    if PROBE_NAME not in objects:
        ucv.spawn_bp_asset(PROBE_BP_PATH, PROBE_NAME)
    ucv.set_scale(PROBE_SCALE, PROBE_NAME)
    ucv.set_physics(PROBE_NAME, False)
    ucv.set_collision(PROBE_NAME, True)
    ucv.set_movable(PROBE_NAME, True)
    ucv.set_location(location, PROBE_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), PROBE_NAME)


def destroy_collision_probe(ucv) -> None:
    objects = {str(n) for n in ucv.get_objects().tolist()}
    if PROBE_NAME in objects:
        ucv.destroy(PROBE_NAME)


def probe_hit_at(ucv, x_cm: float, y_cm: float, z_cm: float) -> bool:
    """Return True when a static obstacle overlaps the probe at (x, y, z)."""
    spawn_collision_probe(ucv, (x_cm, y_cm, z_cm))
    if PROBE_SETTLE_S > 0:
        time.sleep(PROBE_SETTLE_S)
    counts = parse_collision_counts(ucv.get_collision_num(PROBE_NAME))
    return collision_indicates_obstacle(counts)


def classify_cell_at_heights(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    block_bottom_z_cm: float,
    block_height_cm: float,
) -> BlockSemantic:
    z_low_cm = block_bottom_z_cm - block_height_cm
    hit_high = probe_hit_at(ucv, x_cm, y_cm, block_bottom_z_cm)
    hit_low = probe_hit_at(ucv, x_cm, y_cm, z_low_cm)
    return classify_semantic(hit_at_z0=hit_high, hit_at_z_low=hit_low)


def scan_region_semantics(
    ucv,
    cells: list[BlockIndex],
    *,
    cell_center_xy_cm_fn,
    block_bottom_z_cm: float,
    block_height_cm: float,
    progress_every: int = 20,
) -> Dict[BlockIndex, BlockSemantic]:
    """Scan each grid cell and return semantic labels (probe must be destroyed after)."""
    results: Dict[BlockIndex, BlockSemantic] = {}
    total = len(cells)
    t0 = time.monotonic()
    try:
        spawn_collision_probe(ucv, (0.0, 0.0, block_bottom_z_cm))
        for i, (gx, gy) in enumerate(cells, start=1):
            x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
            sem = classify_cell_at_heights(
                ucv,
                x_cm,
                y_cm,
                block_bottom_z_cm=block_bottom_z_cm,
                block_height_cm=block_height_cm,
            )
            results[(gx, gy)] = sem
            if progress_every > 0 and (i % progress_every == 0 or i == total):
                elapsed = time.monotonic() - t0
                print(
                    f"[SemanticScan] {i}/{total} "
                    f"last=({gx},{gy})->{sem} elapsed={elapsed:.1f}s"
                )
    finally:
        destroy_collision_probe(ucv)
    return results
