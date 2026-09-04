"""Epsilon-constraint: maximize Jeff subject to Jsafe <= eps, for several eps."""

from __future__ import annotations

import random
from typing import List

from constraints.pareto import EvaluatedTheta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox


def run_epsilon_constraint(
    evaluator: OracleEvaluator,
    *,
    epsilons: tuple[float, ...] = (0.0, 0.01, 0.03, 0.08, 0.2),
    n_try: int = 8,
    seed: int = 23,
    box: ThetaBox | None = None,
) -> List[EvaluatedTheta]:
    space = box or ThetaBox()
    rng = random.Random(seed)
    rows: list[EvaluatedTheta] = []
    for eps in epsilons:
        best: EvaluatedTheta | None = None
        for _ in range(n_try):
            theta = space.from_unit(rng.random(), rng.random())
            row = evaluator.evaluate(theta)
            rows.append(row)
            if row.jsafe > eps:
                continue
            if best is None or row.jeff > best.jeff:
                best = row
    return rows
