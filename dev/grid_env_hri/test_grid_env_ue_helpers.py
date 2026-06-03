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


if __name__ == "__main__":
    unittest.main()
