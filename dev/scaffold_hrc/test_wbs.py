#!/usr/bin/env python3
"""Unit tests for schedule-bound WBS lifetime (no UE)."""

from __future__ import annotations

import unittest

from wbs.clock import build_stage1_wbs


class WbsClockTest(unittest.TestCase):
    def test_stage1_has_supply_then_erect_per_floor(self) -> None:
        clock = build_stage1_wbs()
        ids = [t.task_id for t in clock.tasks]
        self.assertEqual(
            ids,
            [
                "A_F1_supply",
                "A_F1_erect",
                "A_F2_supply",
                "A_F2_erect",
                "A_F3_supply",
                "A_F3_erect",
            ],
        )

    def test_only_current_task_is_active(self) -> None:
        clock = build_stage1_wbs()
        self.assertTrue(clock.is_active("A_F1_supply"))
        self.assertFalse(clock.is_active("A_F1_erect"))
        self.assertEqual(clock.active_zone_id(), "SCAFFOLD_A_F1")
        self.assertTrue(clock.constrains_floor(1))
        self.assertFalse(clock.constrains_floor(2))

    def test_completion_expires_and_advances(self) -> None:
        clock = build_stage1_wbs()
        nxt = clock.complete_current()
        self.assertTrue(nxt.is_active("A_F1_erect"))
        self.assertFalse(nxt.is_active("A_F1_supply"))
        nxt = nxt.complete_current()
        self.assertTrue(nxt.is_active("A_F2_supply"))
        self.assertEqual(nxt.active_zone_id(), "SCAFFOLD_A_F2")

    def test_utterance_binds_to_active_zone_not_invented_clock(self) -> None:
        clock = build_stage1_wbs()
        bound = clock.bind_utterance_zone("SCAFFOLD_A_F1")
        self.assertEqual(bound.task_id, "A_F1_supply")
        self.assertGreater(bound.t_end, bound.t_start)
        with self.assertRaises(ValueError):
            clock.bind_utterance_zone("SCAFFOLD_A_F3")

    def test_clock_is_frozen(self) -> None:
        clock = build_stage1_wbs()
        with self.assertRaises(Exception):
            clock.cursor = 1  # type: ignore[misc]


if __name__ == "__main__":
    unittest.main()
