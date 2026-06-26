#!/usr/bin/env python3
"""Unit tests for nav_pose_query (batch pose + movement token)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from metrics import NavTimingAccumulator  # noqa: E402
from nav_pose_query import (  # noqa: E402
    POSE_STALE_KEY,
    _parse_pose2d_payload,
    fetch_nav_pose,
    fetch_robot_pose2d,
    init_pose_cache,
    invalidate_robot_pose,
)


class NavPoseQueryTest(unittest.TestCase):
    def test_parse_pose2d_json(self) -> None:
        parsed = _parse_pose2d_payload('{"x": 100.0, "y": 200.0, "yaw": 45.0}')
        self.assertEqual(parsed, ((100.0, 200.0), 45.0))

    def test_movement_token_skips_ue(self) -> None:
        cache: dict = {}
        init_pose_cache(cache)
        cache["xy"] = (1.0, 2.0)
        cache["yaw"] = 90.0
        cache[POSE_STALE_KEY] = False
        ucv = MagicMock()
        timing = NavTimingAccumulator()
        xy, yaw, _ = fetch_nav_pose(ucv, "robot", timing, cache, force=False)
        self.assertEqual(xy, (1.0, 2.0))
        self.assertEqual(yaw, 90.0)
        self.assertEqual(timing.pose_cache_hits, 1)
        ucv.client.request.assert_not_called()

    def test_invalidate_forces_fetch_split(self) -> None:
        cache: dict = {}
        init_pose_cache(cache)
        cache["xy"] = (1.0, 2.0)
        cache["yaw"] = 90.0
        cache[POSE_STALE_KEY] = False
        ucv = MagicMock()
        ucv.client.request.side_effect = RuntimeError("no vbp")
        timing = NavTimingAccumulator()

        def _pos2d(_ucv, _name):
            return (3.0, 4.0)

        def _yaw(_ucv, _name):
            return 180.0

        import nav_pose_query as npq  # noqa: WPS433

        orig_pos = npq.get_pos2d
        orig_yaw = npq.get_yaw
        npq.get_pos2d = _pos2d
        npq.get_yaw = _yaw
        try:
            invalidate_robot_pose(cache, reason="motion")
            xy, yaw, _ = fetch_nav_pose(ucv, "robot", timing, cache, force=False)
        finally:
            npq.get_pos2d = orig_pos
            npq.get_yaw = orig_yaw

        self.assertEqual(xy, (3.0, 4.0))
        self.assertEqual(yaw, 180.0)
        self.assertEqual(timing.pose_batch_split_fetches, 1)
        self.assertFalse(cache[POSE_STALE_KEY])

    def test_vbp_batch_fetch(self) -> None:
        ucv = MagicMock()
        ucv.client.request.return_value = '{"x": 10.0, "y": 20.0, "yaw": 30.0}'
        timing = NavTimingAccumulator()
        xy, yaw, _, mode = fetch_robot_pose2d(ucv, "GridEnv_SpotRobot", timing)
        self.assertEqual(xy, (10.0, 20.0))
        self.assertEqual(yaw, 30.0)
        self.assertEqual(mode, "vbp")
        self.assertEqual(timing.pose_batch_vbp_fetches, 1)


if __name__ == "__main__":
    unittest.main()
