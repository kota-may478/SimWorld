"""Latin-hypercube style sampling of the theta box."""

from __future__ import annotations

import random
from typing import List

from constraints.pareto import EvaluatedTheta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox


def run_lhs(
    evaluator: OracleEvaluator,
    *,
    n_samples: int = 24,
    seed: int = 7,
    box: ThetaBox | None = None,
) -> List[EvaluatedTheta]:
    if n_samples < 2:
        raise ValueError("lhs needs at least 2 samples")
    space = box or ThetaBox()
    rng = random.Random(seed)
    du = [i / n_samples for i in range(n_samples)]
    dv = [i / n_samples for i in range(n_samples)]
    rng.shuffle(du)
    rng.shuffle(dv)
    rows: list[EvaluatedTheta] = []
    for u0, v0 in zip(du, dv):
        u = u0 + rng.random() / n_samples
        v = v0 + rng.random() / n_samples
        rows.append(evaluator.evaluate(space.from_unit(u, v)))
    return rows
