"""Geometric projection onto an ISO-feasible (TT, v_max, min_sep) Pareto front."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class Theta:
    vmax_mps: float
    dmin_m: float

    def as_dict(self) -> dict[str, float]:
        return {"vmax_mps": self.vmax_mps, "dmin_m": self.dmin_m}


@dataclass(frozen=True)
class EvaluatedTheta:
    theta: Theta
    tt: float
    t_ssm: float
    si_min: float
    completed: bool
    mission_s: float = 0.0
    scaffold_safe_s: float = 0.0
    scaffold_unsafe_s: float = 0.0
    min_sep_m: float = 0.0

    @property
    def iso_feasible(self) -> bool:
        return self.completed and self.si_min + 1e-9 >= 1.0

    @property
    def jeff(self) -> float:
        """NSGA/BO compatibility: maximize jeff ≡ minimize TT."""
        return -self.tt

    @property
    def jsafe(self) -> float:
        """NSGA/BO compatibility: minimize jsafe ≡ minimize v_max."""
        return self.theta.vmax_mps

    @property
    def jdist(self) -> float:
        """NSGA compatibility: minimize jdist ≡ maximize min separation."""
        return -self.min_sep_m


def synthetic_front(*, n: int = 9) -> Tuple[Theta, ...]:
    """Monotone front: lower speed, larger ranging keep-out. vmax cap is 1.0 m/s."""
    if n < 2:
        raise ValueError("front needs at least two points")
    points: list[Theta] = []
    for i in range(n):
        a = i / (n - 1)
        vmax = 1.00 - a * (1.00 - 0.20)
        dmin = 0.35 + a * (1.60 - 0.35)
        points.append(Theta(vmax_mps=vmax, dmin_m=dmin))
    return tuple(points)


def project(theta_llm: Theta, alpha: float, front: Sequence[Theta]) -> Theta:
    """Map α onto the front. θ_LLM is accepted then discarded so a hallucinated
    numeric anchor cannot leave P_WBS; the discrete index is α only.
    α=0 is the efficient end (lowest TT), α=1 is the conservative end.
    """
    if not front:
        raise ValueError("empty Pareto front")
    _ = theta_llm
    alpha_clamped = min(1.0, max(0.0, float(alpha)))
    last = len(front) - 1
    idx = int(round(alpha_clamped * last))
    return front[idx]


def nondominated(rows: Sequence[EvaluatedTheta]) -> Tuple[EvaluatedTheta, ...]:
    """Keep θ that no other sample beats on TT (min), v_max (min), min_sep (max)."""
    kept: list[EvaluatedTheta] = []
    for a in rows:
        dominated = False
        for b in rows:
            if b.theta == a.theta and b.tt == a.tt and b.min_sep_m == a.min_sep_m:
                continue
            b_better = (
                b.tt <= a.tt
                and b.theta.vmax_mps <= a.theta.vmax_mps
                and b.min_sep_m >= a.min_sep_m
            )
            b_strict = (
                b.tt < a.tt
                or b.theta.vmax_mps < a.theta.vmax_mps
                or b.min_sep_m > a.min_sep_m
            )
            if b_better and b_strict:
                dominated = True
                break
        if not dominated:
            kept.append(a)
    return tuple(sorted(kept, key=lambda row: row.tt))


def iso_pareto(rows: Sequence[EvaluatedTheta]) -> Tuple[EvaluatedTheta, ...]:
    """Non-dominated ISO-feasible points, ordered by increasing TT for α."""
    feasible = tuple(row for row in rows if row.iso_feasible)
    return nondominated(feasible)
