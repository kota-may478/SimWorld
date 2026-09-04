"""Geometric projection onto a (d_min, v_max) Pareto front."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class Theta:
    dmin_m: float
    vmax_mps: float


@dataclass(frozen=True)
class EvaluatedTheta:
    theta: Theta
    jeff: float
    jsafe: float
    completed: bool


def synthetic_front(*, n: int = 9) -> Tuple[Theta, ...]:
    """Monotone front: more separation, lower speed. vmax cap is 1.0 m/s."""
    if n < 2:
        raise ValueError("front needs at least two points")
    points: list[Theta] = []
    for i in range(n):
        a = i / (n - 1)
        dmin = 0.35 + a * (1.60 - 0.35)
        vmax = 1.00 - a * (1.00 - 0.20)
        points.append(Theta(dmin_m=dmin, vmax_mps=vmax))
    return tuple(points)


def project(theta_llm: Theta, alpha: float, front: Sequence[Theta]) -> Theta:
    """Map α onto the front. θ_LLM is accepted then discarded so a hallucinated
    numeric anchor cannot leave P_WBS; the discrete index is α only.
    """
    if not front:
        raise ValueError("empty Pareto front")
    _ = theta_llm
    alpha_clamped = min(1.0, max(0.0, float(alpha)))
    last = len(front) - 1
    idx = int(round(alpha_clamped * last))
    return front[idx]


def nondominated(rows: Sequence[EvaluatedTheta]) -> Tuple[EvaluatedTheta, ...]:
    """Keep θ that no other sample beats on both Jeff (max) and Jsafe (min)."""
    kept: list[EvaluatedTheta] = []
    for a in rows:
        dominated = False
        for b in rows:
            if b.theta == a.theta and b.jeff == a.jeff and b.jsafe == a.jsafe:
                continue
            if b.jeff >= a.jeff and b.jsafe <= a.jsafe and (
                b.jeff > a.jeff or b.jsafe < a.jsafe
            ):
                dominated = True
                break
        if not dominated:
            kept.append(a)
    return tuple(sorted(kept, key=lambda row: row.theta.dmin_m))
