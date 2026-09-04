"""Uniform grid sweep of (d_min, v_max)."""

from __future__ import annotations

from typing import List

from constraints.pareto import EvaluatedTheta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox


def run_grid(
    evaluator: OracleEvaluator,
    *,
    n_dmin: int = 6,
    n_vmax: int = 6,
    box: ThetaBox | None = None,
) -> List[EvaluatedTheta]:
    space = box or ThetaBox()
    if n_dmin < 2 or n_vmax < 2:
        raise ValueError("grid needs at least 2x2")
    rows: list[EvaluatedTheta] = []
    for i in range(n_dmin):
        u = i / (n_dmin - 1)
        for j in range(n_vmax):
            v = j / (n_vmax - 1)
            theta = space.from_unit(u, v)
            rows.append(evaluator.evaluate(theta))
    return rows
