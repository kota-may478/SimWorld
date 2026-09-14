#!/usr/bin/env python3
"""Condition-card catalog sanity (no oracle)."""

from __future__ import annotations

import unittest

from fronts.condition_cards import condition_cards


class ConditionCardsTest(unittest.TestCase):
    def test_six_cards_cover_both_families(self) -> None:
        cards = condition_cards()
        self.assertEqual(len(cards), 6)
        families = {c.family for c in cards}
        self.assertEqual(families, {"language", "safety_search"})
        self.assertTrue(any("b3_keyword" in c.methods for c in cards))
        self.assertTrue(any("b4_discrete_mode" in c.methods for c in cards))
        self.assertTrue(any("safeopt" in c.methods for c in cards))
        self.assertTrue(all("proposed" in c.methods for c in cards))


if __name__ == "__main__":
    unittest.main()
