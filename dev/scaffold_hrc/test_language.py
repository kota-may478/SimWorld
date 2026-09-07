#!/usr/bin/env python3
"""Unit tests for 3x3 language grounding (no network)."""

from __future__ import annotations

import unittest

from fronts.language import (
    Preference,
    cell_for_levels,
    ground_utterance,
    has_preference_lexicon,
    parse_relative,
)


class LanguageGroundTest(unittest.TestCase):
    def test_relative_slower_and_farther(self) -> None:
        slow = parse_relative("もう少しゆっくり")
        far = parse_relative("もう少し離れて")
        self.assertIsNotNone(slow)
        self.assertIsNotNone(far)
        assert slow is not None and far is not None
        self.assertEqual(slow[:2], (0, 1))
        self.assertEqual(far[:2], (0, -1))

    def test_relative_stronger_step(self) -> None:
        mild = parse_relative("もう少し慎重に")
        strong = parse_relative("かなり慎重に")
        self.assertIsNotNone(mild)
        self.assertIsNotNone(strong)
        assert mild is not None and strong is not None
        self.assertEqual(mild[0], 1)
        self.assertEqual(strong[0], 2)

    def test_reset_returns_center(self) -> None:
        pref = ground_utterance("いつも通り", state=Preference(1.0, 1.0))
        self.assertAlmostEqual(pref.alpha, 0.5)
        self.assertAlmostEqual(pref.beta, 0.5)
        self.assertEqual(pref.kind, "reset")

    def test_absolute_keywords_hit_three_by_three(self) -> None:
        hurry = ground_utterance("急いで")
        slow = ground_utterance("ゆっくり動いて")
        far = ground_utterance("距離を取って")
        self.assertEqual((hurry.alpha, hurry.beta), (0.0, 0.5))
        self.assertEqual((slow.alpha, slow.beta), (1.0, 1.0))
        self.assertEqual((far.alpha, far.beta), (1.0, 0.0))
        self.assertEqual(cell_for_levels(1.0, 1.0), "safe_slow")

    def test_relative_applies_to_state(self) -> None:
        pref = ground_utterance(
            "もう少し急いで",
            state=Preference(0.75, 0.5),
        )
        self.assertAlmostEqual(pref.alpha, 0.5)
        self.assertAlmostEqual(pref.beta, 0.5)
        self.assertEqual(pref.kind, "relative")

    def test_english_go_slow(self) -> None:
        pref = ground_utterance("please go slow")
        self.assertAlmostEqual(pref.alpha, 1.0)
        self.assertAlmostEqual(pref.beta, 1.0)

    def test_english_much_slower_is_relative(self) -> None:
        parsed = parse_relative("much slower")
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed[:2], (0, 2))
        pref = ground_utterance("much slower", state=Preference(0.5, 0.5))
        self.assertEqual(pref.kind, "relative")
        self.assertEqual(pref.notes, "catalog-much_slower")

    def test_preference_gate_blocks_thanks_even_if_embed_is_confident(self) -> None:
        class _Emb:
            def encode(self, texts: list[str]) -> list[list[float]]:
                out = []
                for item in texts:
                    if item in ("ありがとう", "感謝しています", "thanks a lot"):
                        out.append([1.0, 0.0])
                    else:
                        out.append([0.0, 1.0])
                return out

        start = Preference(0.5, 0.5)
        pref = ground_utterance("ありがとう", state=start, embedder=_Emb())
        self.assertEqual(pref.kind, "unchanged")
        self.assertFalse(has_preference_lexicon("ありがとう"))

    def test_preference_lexicon_allows_paraphrase_embed(self) -> None:
        class _Emb:
            def encode(self, texts: list[str]) -> list[list[float]]:
                out = [[0.0, 1.0] for _ in texts]
                out[0] = [1.0, 0.0]
                out[1] = [1.0, 0.0]
                return out

        pref = ground_utterance("finish as fast as you can", embedder=_Emb())
        self.assertEqual(pref.cell, "efficient")
        self.assertTrue(pref.notes.startswith("embed:efficient:"))

    def test_embed_rejects_ood_below_threshold(self) -> None:
        class _Emb:
            def encode(self, texts: list[str]) -> list[list[float]]:
                out = [[1.0, 0.0] for _ in texts]
                out[0] = [0.0, 1.0]
                return out

        start = Preference(0.5, 0.5)
        pref = ground_utterance("今日の天気は？", state=start, embedder=_Emb())
        self.assertEqual(pref.kind, "unchanged")
        self.assertAlmostEqual(pref.alpha, 0.5)

    def test_llm_json_is_used_when_keywords_miss(self) -> None:
        class _Gen:
            def generate(self, prompt: str) -> str:
                _ = prompt
                return '{"kind": "absolute", "alpha": 1.0, "beta": 0.0}'

        pref = ground_utterance("prioritize standoff please", generator=_Gen())
        self.assertAlmostEqual(pref.alpha, 1.0)
        self.assertAlmostEqual(pref.beta, 0.0)
        self.assertEqual(pref.kind, "absolute")


if __name__ == "__main__":
    unittest.main()
