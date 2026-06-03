"""Unit tests for UE helpers (no live SimWorld required)."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock

import grid_env_hri_simulation as geh


class TestUeRequest(unittest.TestCase):
    def test_ue_request_does_not_patch_simplequeue(self) -> None:
        ucv = MagicMock()
        lock = MagicMock()
        lock.__enter__.return_value = None
        lock.__exit__.return_value = None
        ucv.lock = lock
        ucv.client.request.return_value = "1 2 3"

        result = geh._ue_request(ucv, "vget /object/foo/location")

        self.assertEqual(result, "1 2 3")
        ucv.client.request.assert_called_once_with("vget /object/foo/location")

    def test_set_cube_blocking_mode_issues_vbp(self) -> None:
        ucv = MagicMock()
        lock = MagicMock()
        lock.__enter__.return_value = None
        lock.__exit__.return_value = None
        ucv.lock = lock
        ucv.client.request.return_value = "ok"

        geh.set_cube_blocking_mode(ucv, "demo_solid_00", blocking=False)

        ucv.client.request.assert_called_with("vbp demo_solid_00 SetBlocking False")
        ucv.set_collision.assert_called_once_with("demo_solid_00", False)

    def test_actor_exists_uses_location_not_object_list(self) -> None:
        ucv = MagicMock()
        lock = MagicMock()
        lock.__enter__.return_value = None
        lock.__exit__.return_value = None
        ucv.lock = lock
        ucv.client.request.return_value = "0 0 100"

        self.assertTrue(geh.actor_exists(ucv, "grid_floor_main"))
        ucv.client.request.assert_called_once_with("vget /object/grid_floor_main/location")


class TestCleanupActorNames(unittest.TestCase):
    def test_extra_cleanup_includes_toggle_cube(self) -> None:
        extras = geh.grid_env_extra_cleanup_actor_names()
        self.assertIn(geh.SINGLE_TOGGLE_CUBE_NAME, extras)

    def test_extra_cleanup_merges_optional_ids(self) -> None:
        extras = geh.grid_env_extra_cleanup_actor_names(extra_ids=["foo", "foo"])
        self.assertIn("foo", extras)
        self.assertEqual(extras.count("foo"), 1)


class TestCubeFloorPlacement(unittest.TestCase):
    def test_bottom_pivot_z_on_floor(self) -> None:
        orig = geh.CUBE_PIVOT_AT_CENTER
        geh.CUBE_PIVOT_AT_CENTER = False
        try:
            self.assertAlmostEqual(geh.cube_actor_z_on_floor_cm(), 100.5)
            x, y, z = geh.cube_actor_location_on_floor_cm(550.0, 550.0)
            self.assertEqual((x, y), (550.0, 550.0))
            self.assertAlmostEqual(z, 100.5)
        finally:
            geh.CUBE_PIVOT_AT_CENTER = orig

    def test_center_pivot_z_on_floor(self) -> None:
        orig = geh.CUBE_PIVOT_AT_CENTER
        geh.CUBE_PIVOT_AT_CENTER = True
        try:
            self.assertAlmostEqual(geh.cube_actor_z_on_floor_cm(), 115.5)
        finally:
            geh.CUBE_PIVOT_AT_CENTER = orig

    def test_physics_drop_raises_above_floor(self) -> None:
        orig_center = geh.CUBE_PIVOT_AT_CENTER
        orig_drop = geh.CUBE_SPAWN_ABOVE_FLOOR_CM
        geh.CUBE_PIVOT_AT_CENTER = False
        geh.CUBE_SPAWN_ABOVE_FLOOR_CM = 5.0
        try:
            _x, _y, z = geh.cube_actor_location_physics_drop_cm(0.0, 0.0)
            self.assertAlmostEqual(z, 105.5)
        finally:
            geh.CUBE_PIVOT_AT_CENTER = orig_center
            geh.CUBE_SPAWN_ABOVE_FLOOR_CM = orig_drop


if __name__ == "__main__":
    unittest.main()
