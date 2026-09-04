"""Scalarization: maximize J = Jeff - w * Jsafe for several penalty weights."""

from __future__ import annotations

import random
from typing import List

from constraints.pareto import EvaluatedTheta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox


def run_weighted_sum(
    evaluator: OracleEvaluator,
    *,
    weights: tuple[float, ...] = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0),
    restarts: int = 3,
    steps: int = 6,
    seed: int = 19,
    box: ThetaBox | None = None,
) -> List[EvaluatedTheta]:
    space = box or ThetaBox()
    rng = random.Random(seed)
    rows: list[EvaluatedTheta] = []
    for weight in weights:
        for _ in range(restarts):
            theta = space.from_unit(rng.random(), rng.random())
            best = evaluator.evaluate(theta)
            rows.append(best)
            best_j = best.jeff - weight * best.jsafe
            for _step in range(steps):
                u, v = space.to_unit(best.theta)
                cand = space.from_unit(
                    min(1.0, max(0.0, u + rng.uniform(-0.18, 0.18))),
                    min(1.0, max(0.0, v + rng.uniform(-0.18, 0.18))),
                )
                row = evaluator.evaluate(cand)
                rows.append(row)
                j = row.jeff - weight * row.jsafe
                if j > best_j:
                    best = row
                    best_j = j
    return rows
