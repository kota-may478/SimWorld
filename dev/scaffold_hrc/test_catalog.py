#!/usr/bin/env python3
"""Catalog parse / apply tests (no network, no model weights)."""

from __future__ import annotations

import unittest

from fronts.catalog import (
    apply_catalog_label,
    parse_catalog_label,
    parse_from_options,
    pref_to_catalog_id,
)
from fronts.gold_language import gold_items
from fronts.language import Preference, ground_utterance, has_relative_cue
from fronts.staged_ground import cell_from_alpha_beta_labels


class CatalogTest(unittest.TestCase):
    def test_parse_picks_longest_label(self) -> None:
        self.assertEqual(parse_catalog_label("Label: safe_slow"), "safe_slow")
        self.assertEqual(parse_catalog_label("the answer is a_bit_slower."), "a_bit_slower")
        self.assertIsNone(parse_catalog_label("hello world"))

    def test_apply_relative_from_center(self) -> None:
        start = Preference(0.5, 0.5)
        pref = apply_catalog_label("a_bit_slower", start)
        self.assertAlmostEqual(pref.alpha, 0.5)
        self.assertAlmostEqual(pref.beta, 0.75)
        self.assertEqual(pref.kind, "relative")

    def test_apply_efficiency_opens_on_relative_slow(self) -> None:
        start = Preference(0.0, 0.5)
        pref = apply_catalog_label("a_bit_slower", start)
        self.assertAlmostEqual(pref.alpha, 0.25)
        self.assertAlmostEqual(pref.beta, 0.75)

    def test_generator_is_not_preempted_by_keywords(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                if "Kind:" in prompt:
                    return "absolute"
                if "Alpha:" in prompt:
                    return "efficiency"
                if "Beta:" in prompt:
                    return "middle"
                return "none"

        pref = ground_utterance("please go slow", generator=_Gen())
        self.assertEqual(pref.cell, "efficient")
        self.assertEqual(pref.notes, "catalog-efficient")

    def test_staged_absolute_alpha_beta_combine(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                if "Kind:" in prompt:
                    return "absolute"
                if "Alpha:" in prompt:
                    return "safety"
                if "Beta:" in prompt:
                    return "slow"
                return "none"

        pref = ground_utterance("please go slow", generator=_Gen())
        self.assertEqual(pref.cell, "safe_slow")
        self.assertAlmostEqual(pref.alpha, 1.0)
        self.assertAlmostEqual(pref.beta, 1.0)

    def test_staged_absolute_efficiency_ignores_beta_cell(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                if "Kind:" in prompt:
                    return "absolute"
                if "Alpha:" in prompt:
                    return "efficiency"
                if "Beta:" in prompt:
                    return "distance"
                return "none"

        pref = ground_utterance("急いで", generator=_Gen())
        self.assertEqual(pref.cell, "efficient")
        self.assertAlmostEqual(pref.alpha, 0.0)

    def test_staged_relative_two_questions(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                if "Kind:" in prompt:
                    return "relative"
                if "Direction:" in prompt:
                    return "slower"
                if "Strength:" in prompt:
                    return "a_bit"
                return "none"

        pref = ground_utterance("もう少しゆっくり", generator=_Gen())
        self.assertEqual(pref.notes, "catalog-a_bit_slower")
        self.assertAlmostEqual(pref.beta, 0.75)

    def test_staged_none_keeps_state(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                _ = prompt
                return "none"

        start = Preference(0.75, 0.25)
        pref = ground_utterance("how is the weather today", state=start, generator=_Gen())
        self.assertEqual(pref.kind, "unchanged")
        self.assertAlmostEqual(pref.alpha, 0.75)

    def test_gold_set_is_closed_and_nonempty(self) -> None:
        items = gold_items()
        self.assertGreaterEqual(len(items), 30)
        ids = {item.gold_id for item in items}
        self.assertIn("efficient", ids)
        self.assertIn("a_bit_slower", ids)
        self.assertIn("unchanged", ids)

    def test_parse_from_options_uses_first_hit_on_first_line(self) -> None:
        self.assertEqual(
            parse_from_options("safety", ("efficiency", "balanced", "safety")),
            "safety",
        )
        self.assertEqual(
            parse_from_options("safety\nefficiency", ("efficiency", "balanced", "safety")),
            "safety",
        )

    def test_alpha_beta_labels_cover_three_by_three(self) -> None:
        self.assertEqual(cell_from_alpha_beta_labels("efficiency", "distance"), "efficient")
        self.assertEqual(cell_from_alpha_beta_labels("efficiency", "slow"), "efficient")
        self.assertEqual(cell_from_alpha_beta_labels("balanced", "distance"), "normal_distance")
        self.assertEqual(cell_from_alpha_beta_labels("balanced", "middle"), "normal")
        self.assertEqual(cell_from_alpha_beta_labels("balanced", "slow"), "normal_slow")
        self.assertEqual(cell_from_alpha_beta_labels("safety", "distance"), "safe_distance")
        self.assertEqual(cell_from_alpha_beta_labels("safety", "middle"), "safe")
        self.assertEqual(cell_from_alpha_beta_labels("safety", "slow"), "safe_slow")

    def test_pref_to_catalog_roundtrip(self) -> None:
        start = Preference(0.5, 0.5)
        pref = apply_catalog_label("much_farther", start)
        self.assertEqual(pref_to_catalog_id(pref, start), "much_farther")

    def test_relative_cue_requires_degree_words(self) -> None:
        self.assertFalse(has_relative_cue("please go slow"))
        self.assertFalse(has_relative_cue("減速して"))
        self.assertTrue(has_relative_cue("もう少しゆっくり"))
        self.assertTrue(has_relative_cue("a bit slower"))
        self.assertTrue(has_relative_cue("much farther"))

    def test_apply_relative_skips_shared_theta(self) -> None:
        from constraints.pareto import EvaluatedTheta, Theta
        from fronts.ab_map import LEVELS

        shared = EvaluatedTheta(
            Theta(0.43, 1.6), tt=1171.0, t_ssm=0.43, si_min=1.2, completed=True, min_sep_m=1.58
        )
        nxt = EvaluatedTheta(
            Theta(0.42, 1.6), tt=1188.0, t_ssm=0.42, si_min=1.2, completed=True, min_sep_m=1.57
        )
        table = {(a, b): shared for a in LEVELS for b in LEVELS}
        table[(1.0, 0.5)] = nxt
        start = Preference(1.0, 0.0)
        pref = apply_catalog_label("a_bit_slower", start, theta_table=table)
        self.assertAlmostEqual(pref.beta, 0.5)

    def test_staged_relative_without_cue_becomes_absolute(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                if "Kind:" in prompt:
                    return "relative"
                if "Alpha:" in prompt:
                    return "safety"
                if "Beta:" in prompt:
                    return "slow"
                if "Direction:" in prompt:
                    return "slower"
                if "Strength:" in prompt:
                    return "a_bit"
                return "none"

        pref = ground_utterance("please go slow", generator=_Gen())
        self.assertEqual(pref.cell, "safe_slow")
        self.assertEqual(pref.kind, "absolute")


if __name__ == "__main__":
    unittest.main()
