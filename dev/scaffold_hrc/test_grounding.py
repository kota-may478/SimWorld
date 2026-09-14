#!/usr/bin/env python3
"""Unit tests for preference-to-theta grounding (no LLM)."""

from __future__ import annotations

import unittest

from constraints.pareto import EvaluatedTheta, Theta, synthetic_front
from fronts.evaluate import OracleEvaluator, measure_t_ref, opt_config
from fronts.grounding import (
    b1_direct,
    b2_clip_box,
    b3_keyword,
    b4_discrete_mode,
    b5_simulate_reject,
    front_thetas,
    proposed,
)
from fronts.safe_bo import best_safe_incumbent
from fronts.space import HALLUCINATED_THETA, REF_THETA, ThetaBox
from oracle.simulate import OracleConfig


class GroundingMapTest(unittest.TestCase):
    def test_b1_keeps_hallucinated_numbers(self) -> None:
        raw = HALLUCINATED_THETA
        out = b1_direct(raw)
        self.assertEqual(out.theta, raw)
        self.assertEqual(out.method, "b1_direct")

    def test_b2_clips_to_the_box(self) -> None:
        box = ThetaBox()
        out = b2_clip_box(HALLUCINATED_THETA, box)
        self.assertLessEqual(out.theta.vmax_mps, box.vmax_hi + 1e-9)
        self.assertLessEqual(out.theta.dmin_m, box.dmin_hi + 1e-9)

    def test_b3_keyword_maps_safe_and_fast(self) -> None:
        box = ThetaBox()
        safe = b3_keyword("please be more cautious", box)
        fast = b3_keyword("hurry up", box)
        self.assertGreater(safe.theta.dmin_m, fast.theta.dmin_m)
        self.assertLess(safe.theta.vmax_mps, fast.theta.vmax_mps)

    def test_b4_modes_are_box_corners_or_mid(self) -> None:
        box = ThetaBox()
        eff = b4_discrete_mode("efficient", box)
        safe = b4_discrete_mode("conservative", box)
        self.assertAlmostEqual(eff.theta.dmin_m, box.dmin_lo)
        self.assertAlmostEqual(safe.theta.vmax_mps, box.vmax_lo)

    def test_proposed_from_text_picks_slow_cell(self) -> None:
        from fronts.grounding import proposed_from_text

        rows = (
            EvaluatedTheta(
                Theta(1.0, 0.35),
                tt=50.0,
                t_ssm=1.0,
                si_min=1.2,
                completed=True,
                min_sep_m=0.4,
            ),
            EvaluatedTheta(
                Theta(0.2, 1.6),
                tt=200.0,
                t_ssm=0.2,
                si_min=1.2,
                completed=True,
                min_sep_m=1.6,
            ),
        )
        out = proposed_from_text("ゆっくり動いて", rows)
        self.assertAlmostEqual(out.alpha, 1.0)
        self.assertAlmostEqual(out.beta, 1.0)
        self.assertAlmostEqual(out.theta.vmax_mps, 0.2)

    def test_proposed_ignores_hallucinated_theta(self) -> None:
        front = synthetic_front()
        out = proposed(0.0, front, theta_llm=HALLUCINATED_THETA)
        self.assertEqual(out.method, "proposed")
        self.assertEqual(out.theta, front[0])


class GroundingOracleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        config = opt_config(
            OracleConfig(
                dt_s=0.25,
                timeout_s=480.0,
                erect_s=0.25,
                truck_load_s=0.25,
                drop_place_s=0.25,
                assembler_pickup_s=0.25,
                sockets_per_floor=2,
                record_trace=False,
            )
        )
        cls.evaluator = OracleEvaluator(
            config=config, t_ref_s=measure_t_ref(config, REF_THETA)
        )

    def test_b5_returns_an_evaluated_theta(self) -> None:
        out = b5_simulate_reject(Theta(vmax_mps=1.0, dmin_m=0.35), self.evaluator, n_retry=2)
        self.assertEqual(out.method, "b5_simulate_reject")
        self.assertIsNotNone(out.si_min)

    def test_proposed_front_thetas_come_from_iso_pareto(self) -> None:
        rows = [
            self.evaluator.evaluate(Theta(vmax_mps=1.0, dmin_m=0.35)),
            self.evaluator.evaluate(Theta(vmax_mps=0.35, dmin_m=1.40)),
        ]
        front = front_thetas(rows)
        self.assertGreaterEqual(len(front), 1)
        picked = proposed(1.0, front)
        self.assertIn(picked.theta, front)

    def test_compare_methods_lists_proposed_baselines_and_safeopt(self) -> None:
        from fronts.grounding import compare_methods

        front = synthetic_front()
        compared = compare_methods(
            alpha=0.0,
            front=front,
            evaluator=self.evaluator,
            score_all=False,
        )
        names = [item.method for item in compared]
        self.assertEqual(
            names,
            [
                "proposed",
                "b1_direct",
                "b2_clip_box",
                "b3_keyword",
                "b4_discrete_mode",
                "b5_simulate_reject",
                "safeopt",
            ],
        )
        self.assertEqual(compared[0].theta, front[0])

    def test_b5_retracts_linearly_from_the_original_clip(self) -> None:
        seen: list[Theta] = []
        box = ThetaBox()

        class _Probe:
            def evaluate(self, theta: Theta) -> EvaluatedTheta:
                seen.append(theta)
                return EvaluatedTheta(
                    theta, tt=1.0, t_ssm=0.2, si_min=0.5, completed=True
                )

        raw = Theta(vmax_mps=1.0, dmin_m=0.35)
        b5_simulate_reject(raw, _Probe(), n_retry=4)  # type: ignore[arg-type]
        self.assertEqual(len(seen), 5)
        self.assertAlmostEqual(seen[0].dmin_m, 0.35)
        self.assertAlmostEqual(seen[0].vmax_mps, 1.0)
        self.assertAlmostEqual(seen[2].dmin_m, 0.35 + 0.5 * (box.dmin_hi - 0.35))
        self.assertAlmostEqual(
            seen[2].vmax_mps, 1.0 + 0.5 * (box.vmax_lo - 1.0)
        )
        self.assertAlmostEqual(seen[-1].dmin_m, box.dmin_hi)
        self.assertAlmostEqual(seen[-1].vmax_mps, box.vmax_lo)

    def test_safeopt_incumbent_picks_lowest_tt_under_limit(self) -> None:
        rows = (
            EvaluatedTheta(
                Theta(0.4, 1.40), tt=1.2, t_ssm=0.01, si_min=1.2, completed=True
            ),
            EvaluatedTheta(
                Theta(1.0, 0.35), tt=0.7, t_ssm=0.04, si_min=1.1, completed=True
            ),
            EvaluatedTheta(
                Theta(0.9, 0.50), tt=0.6, t_ssm=0.20, si_min=1.1, completed=True
            ),
        )
        best = best_safe_incumbent(rows, d_lim=0.95)
        self.assertAlmostEqual(best.tt, 0.6)
        self.assertAlmostEqual(best.theta.vmax_mps, 0.9)


if __name__ == "__main__":
    unittest.main()
