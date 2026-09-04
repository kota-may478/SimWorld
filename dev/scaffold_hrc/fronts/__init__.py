"""Pareto-front discovery methods. The kinematic oracle stays in oracle/."""

from __future__ import annotations

from fronts.epsilon_constraint import run_epsilon_constraint
from fronts.grid_sweep import run_grid
from fronts.lhs_sample import run_lhs
from fronts.nsga2 import run_nsga2
from fronts.safe_bo import run_safe_bo
from fronts.weighted_sum import run_weighted_sum

METHODS = {
    "grid": run_grid,
    "lhs": run_lhs,
    "nsga2": run_nsga2,
    "weighted_sum": run_weighted_sum,
    "epsilon_constraint": run_epsilon_constraint,
    "safe_bo": run_safe_bo,
}
