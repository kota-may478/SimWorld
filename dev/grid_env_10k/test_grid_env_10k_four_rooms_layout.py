"""Unit tests for four-room layout (no UE)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k_four_rooms_layout as layout  # noqa: E402


class TestFourRoomsLayout(unittest.TestCase):
    def setUp(self) -> None:
        self.room_layout = layout.build_four_rooms_layout()

    def test_door_width_three_cells(self) -> None:
        self.assertEqual(layout.DOOR_WIDTH_CELLS, 3)
        self.assertEqual(len(layout.SW_SE_DOOR_GY), 3)
        self.assertEqual(len(layout.SW_NW_DOOR_GX), 3)
        self.assertEqual(len(layout.SE_NE_DOOR_GX), 3)

    def test_pillar_is_solid(self) -> None:
        self.assertIn(layout.PILLAR_CELL, self.room_layout.wall_cells)

    def test_entity_goal_solid_in_ue_not_costmap(self) -> None:
        self.assertIn(layout.ENTITY_GOAL_CELL, self.room_layout.ue_solid_cells)
        self.assertNotIn(layout.ENTITY_GOAL_CELL, self.room_layout.costmap_lethal_cells)

    def test_layout_validation_passes(self) -> None:
        errors = layout.validate_room_adjacency(self.room_layout)
        self.assertEqual(errors, [], msg="\n".join(errors))

    def test_sw_cannot_reach_ne_directly(self) -> None:
        rooms = layout.bfs_room_reachability(self.room_layout, layout.ROBOT_START_CELL)
        self.assertIn("SE", rooms)
        self.assertIn("NW", rooms)
        self.assertIn("NE", rooms)
        self.assertTrue(
            layout.path_exists(self.room_layout, layout.ROBOT_START_CELL, (19, 20))
        )

    def test_outer_corners_on_wall(self) -> None:
        for corner in ((1, 1), (30, 1), (1, 30), (30, 30)):
            self.assertIn(corner, self.room_layout.wall_cells)


if __name__ == "__main__":
    unittest.main()
