#!/usr/bin/env python3
"""Unit tests for registry checkpoint I/O."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from level_region import LOCKED_BLOCK_BOTTOM_Z_CM, default_level_region
from level_semantic_registry_io import (
    blocks_from_semantics,
    can_resume_registry,
    pending_cells,
    save_registry_atomic,
    semantics_from_dict,
    semantics_to_dict,
)


class TestRegistryIO(unittest.TestCase):
    def test_semantics_roundtrip(self) -> None:
        sem = {(1, 1): "floor", (2, 3): "air"}
        self.assertEqual(semantics_from_dict(semantics_to_dict(sem)), sem)

    def test_pending_cells(self) -> None:
        all_cells = [(1, 1), (1, 2), (2, 1)]
        done = {(1, 1): "air"}
        self.assertEqual(pending_cells(all_cells, done), [(1, 2), (2, 1)])

    def test_blocks_from_semantics_positions(self) -> None:
        region = default_level_region(block_bottom_z_cm=LOCKED_BLOCK_BOTTOM_Z_CM)
        sem = {(1, 1): "floor", (2, 1): "air"}
        blocks = blocks_from_semantics(
            region=region,
            block_bottom_z_cm=LOCKED_BLOCK_BOTTOM_Z_CM,
            semantics=sem,
            block_actor_name_fn=lambda gx, gy: f"b_{gx}_{gy}",
            block_bottom_to_actor_z_fn=lambda z: z,
            mode_for_semantic_fn=lambda s: "F" if s == "floor" else "T",
        )
        self.assertEqual(len(blocks), 2)
        x1, y1, z1 = blocks["b_1_1"]["world_cm"]
        cx, cy = region.cell_center_xy_cm(1, 1)
        self.assertAlmostEqual(x1, cx)
        self.assertAlmostEqual(y1, cy)
        self.assertAlmostEqual(z1, LOCKED_BLOCK_BOTTOM_Z_CM)

    def test_resume_match(self) -> None:
        region = default_level_region(block_bottom_z_cm=LOCKED_BLOCK_BOTTOM_Z_CM)
        data = {
            "status": "in_progress",
            "block_bottom_z_cm": LOCKED_BLOCK_BOTTOM_Z_CM,
            "region": {
                "grid_nx": region.grid_nx,
                "grid_ny": region.grid_ny,
                "grid_origin_xy_cm": list(region.grid_origin_xy_cm),
                "subgrid": None,
            },
            "semantics": {"001_001": "air"},
        }
        self.assertTrue(
            can_resume_registry(
                data,
                region=region,
                block_bottom_z_cm=LOCKED_BLOCK_BOTTOM_Z_CM,
                subgrid=None,
            )
        )

    def test_atomic_save(self) -> None:
        path = Path(__file__).resolve().parent / ".test_registry_tmp.json"
        try:
            save_registry_atomic(path, {"status": "in_progress", "n": 1})
            self.assertEqual(json.loads(path.read_text())["n"], 1)
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
