"""NSGA-II on (max Jeff, min Jsafe) over the theta box."""

from __future__ import annotations

import random
from typing import List, Sequence, Tuple

from constraints.pareto import EvaluatedTheta, Theta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox

Individual = Tuple[float, float]


def _dominates(a: EvaluatedTheta, b: EvaluatedTheta) -> bool:
    better = a.jeff >= b.jeff and a.jsafe <= b.jsafe
    strict = a.jeff > b.jeff or a.jsafe < b.jsafe
    return better and strict


def _ranks(pop: Sequence[EvaluatedTheta]) -> List[int]:
    n = len(pop)
    ranks = [0] * n
    remaining = list(range(n))
    front_idx = 0
    while remaining:
        current: list[int] = []
        for i in remaining:
            dominated = False
            for j in remaining:
                if i != j and _dominates(pop[j], pop[i]):
                    dominated = True
                    break
            if not dominated:
                current.append(i)
        for i in current:
            ranks[i] = front_idx
        remaining = [i for i in remaining if i not in current]
        front_idx += 1
    return ranks


def _crowding(pop: Sequence[EvaluatedTheta], ranks: Sequence[int]) -> List[float]:
    n = len(pop)
    crowd = [0.0] * n
    by_rank: dict[int, list[int]] = {}
    for i, rank in enumerate(ranks):
        by_rank.setdefault(rank, []).append(i)
    for members in by_rank.values():
        if len(members) <= 2:
            for i in members:
                crowd[i] = float("inf")
            continue
        for getter in (
            lambda k: pop[k].jeff,
            lambda k: -pop[k].jsafe,
        ):
            ordered = sorted(members, key=getter)
            crowd[ordered[0]] = float("inf")
            crowd[ordered[-1]] = float("inf")
            lo = getter(ordered[0])
            hi = getter(ordered[-1])
            span = hi - lo if hi != lo else 1.0
            for a, b, c in zip(ordered[:-2], ordered[1:-1], ordered[2:]):
                crowd[b] += abs(getter(c) - getter(a)) / span
    return crowd


def _tournament(
    rng: random.Random,
    pop: Sequence[EvaluatedTheta],
    ranks: Sequence[int],
    crowd: Sequence[float],
) -> EvaluatedTheta:
    i, j = rng.randrange(len(pop)), rng.randrange(len(pop))
    if ranks[i] < ranks[j]:
        return pop[i]
    if ranks[j] < ranks[i]:
        return pop[j]
    return pop[i] if crowd[i] >= crowd[j] else pop[j]


def _sbx(
    rng: random.Random,
    a: Theta,
    b: Theta,
    box: ThetaBox,
    eta: float = 12.0,
) -> Tuple[Theta, Theta]:
    def mix(x: float, y: float, lo: float, hi: float) -> Tuple[float, float]:
        if rng.random() > 0.9:
            return x, y
        u = rng.random()
        if u <= 0.5:
            beta = (2.0 * u) ** (1.0 / (eta + 1.0))
        else:
            beta = (1.0 / (2.0 * (1.0 - u))) ** (1.0 / (eta + 1.0))
        c1 = 0.5 * ((x + y) - beta * (y - x))
        c2 = 0.5 * ((x + y) + beta * (y - x))
        return min(hi, max(lo, c1)), min(hi, max(lo, c2))

    d0, d1 = mix(a.dmin_m, b.dmin_m, box.dmin_lo, box.dmin_hi)
    v0, v1 = mix(a.vmax_mps, b.vmax_mps, box.vmax_lo, box.vmax_hi)
    return Theta(d0, v0), Theta(d1, v1)


def _mutate(rng: random.Random, theta: Theta, box: ThetaBox, eta: float = 16.0) -> Theta:
    def poly(x: float, lo: float, hi: float) -> float:
        if rng.random() > 0.2:
            return x
        u = rng.random()
        if u < 0.5:
            delta = (2.0 * u) ** (1.0 / (eta + 1.0)) - 1.0
        else:
            delta = 1.0 - (2.0 * (1.0 - u)) ** (1.0 / (eta + 1.0))
        return min(hi, max(lo, x + delta * (hi - lo)))

    return Theta(
        dmin_m=poly(theta.dmin_m, box.dmin_lo, box.dmin_hi),
        vmax_mps=poly(theta.vmax_mps, box.vmax_lo, box.vmax_hi),
    )


def run_nsga2(
    evaluator: OracleEvaluator,
    *,
    pop_size: int = 12,
    n_gen: int = 5,
    seed: int = 11,
    box: ThetaBox | None = None,
) -> List[EvaluatedTheta]:
    if pop_size < 4 or n_gen < 1:
        raise ValueError("nsga2 needs pop_size>=4 and n_gen>=1")
    space = box or ThetaBox()
    rng = random.Random(seed)
    pop = [
        evaluator.evaluate(space.from_unit(rng.random(), rng.random()))
        for _ in range(pop_size)
    ]
    history: list[EvaluatedTheta] = list(pop)
    for _ in range(n_gen):
        ranks = _ranks(pop)
        crowd = _crowding(pop, ranks)
        children: list[EvaluatedTheta] = []
        while len(children) < pop_size:
            pa = _tournament(rng, pop, ranks, crowd)
            pb = _tournament(rng, pop, ranks, crowd)
            c1, c2 = _sbx(rng, pa.theta, pb.theta, space)
            children.append(evaluator.evaluate(_mutate(rng, c1, space)))
            if len(children) < pop_size:
                children.append(evaluator.evaluate(_mutate(rng, c2, space)))
        history.extend(children)
        merged = pop + children
        ranks = _ranks(merged)
        crowd = _crowding(merged, ranks)
        order = sorted(range(len(merged)), key=lambda i: (ranks[i], -crowd[i]))
        pop = [merged[i] for i in order[:pop_size]]
    return history
