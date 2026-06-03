"""Unit tests for demo-cube passage verdict logic (no SimWorld)."""

from __future__ import annotations

import unittest

import grid_env_hri_simulation as geh


class TestPassageJudge(unittest.TestCase):
    def test_pass_through_ok(self) -> None:
        ok, msg = geh.judge_passage_trial(
            expects_pass_through=True,
            goal_distance_cm=300.0,
            progress_cm=280.0,
            min_dist_to_obstacle_cm=20.0,
            max_object_collision=0,
            crossed_obstacle=True,
        )
        self.assertTrue(ok, msg)

    def test_pass_through_fail_no_cross(self) -> None:
        ok, _ = geh.judge_passage_trial(
            expects_pass_through=True,
            goal_distance_cm=300.0,
            progress_cm=280.0,
            min_dist_to_obstacle_cm=20.0,
            max_object_collision=0,
            crossed_obstacle=False,
        )
        self.assertFalse(ok)

    def test_blocked_ok_stopped_before_plane(self) -> None:
        ok, msg = geh.judge_passage_trial(
            expects_pass_through=False,
            goal_distance_cm=300.0,
            progress_cm=120.0,
            min_dist_to_obstacle_cm=80.0,
            max_object_collision=0,
            crossed_obstacle=False,
        )
        self.assertTrue(ok, msg)

    def test_blocked_fail_crossed_plane(self) -> None:
        ok, msg = geh.judge_passage_trial(
            expects_pass_through=False,
            goal_distance_cm=300.0,
            progress_cm=297.0,
            min_dist_to_obstacle_cm=47.0,
            max_object_collision=0,
            crossed_obstacle=True,
        )
        self.assertFalse(ok)
        self.assertIn("SetBlocking", msg)

    def test_progress_along_segment(self) -> None:
        start = (0.0, 0.0)
        goal = (300.0, 0.0)
        final = (250.0, 10.0)
        progress = geh.passage_progress_along_segment(start, goal, final)
        self.assertAlmostEqual(progress, 250.0, places=1)

    def test_crossed_obstacle_plane_eastbound(self) -> None:
        start = (650.0, 800.0)
        goal = (950.0, 800.0)
        obstacle = (800.0, 800.0)
        self.assertTrue(
            geh.crossed_obstacle_plane(start, goal, obstacle, (947.0, 800.0))
        )
        self.assertFalse(
            geh.crossed_obstacle_plane(start, goal, obstacle, (700.0, 800.0))
        )


if __name__ == "__main__":
    unittest.main()
