"""Safe UCB Bayesian optimization: max Jeff s.t. Jsafe <= d_lim.

Two independent RBF GPs on a discrete candidate grid. New queries are
taken only from the predicted safe set (upper confidence of Jsafe).
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import math

from constraints.pareto import EvaluatedTheta, Theta
from fronts.evaluate import OracleEvaluator
from fronts.space import ThetaBox

Vec = Tuple[float, float]


def _rbf(a: Sequence[Vec], b: Sequence[Vec], length: float, var: float) -> list[list[float]]:
    out: list[list[float]] = []
    inv = 1.0 / (2.0 * length * length)
    for x in a:
        row: list[float] = []
        for y in b:
            d2 = (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2
            row.append(var * math.exp(-d2 * inv))
        out.append(row)
    return out


def _cholesky(a: list[list[float]]) -> list[list[float]]:
    n = len(a)
    l = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            acc = a[i][j] - sum(l[i][k] * l[j][k] for k in range(j))
            if i == j:
                l[i][j] = math.sqrt(max(acc, 1e-12))
            else:
                l[i][j] = acc / l[j][j]
    return l


def _solve_lower(l: list[list[float]], b: Sequence[float]) -> list[float]:
    y = [0.0] * len(b)
    for i, bi in enumerate(b):
        y[i] = (bi - sum(l[i][j] * y[j] for j in range(i))) / l[i][i]
    return y


def _solve_upper(l: list[list[float]], y: Sequence[float]) -> list[float]:
    n = len(y)
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (y[i] - sum(l[j][i] * x[j] for j in range(i + 1, n))) / l[i][i]
    return x


class _GP:
    def __init__(self, length: float = 0.35, var: float = 1.0, noise: float = 1e-4) -> None:
        self.length = length
        self.var = var
        self.noise = noise
        self.x: list[Vec] = []
        self.alpha: list[float] = []
        self.chol: list[list[float]] = []

    def fit(self, xs: Sequence[Vec], ys: Sequence[float]) -> None:
        self.x = list(xs)
        k = _rbf(self.x, self.x, self.length, self.var)
        for i in range(len(k)):
            k[i][i] += self.noise
        self.chol = _cholesky(k)
        self.alpha = _solve_upper(self.chol, _solve_lower(self.chol, ys))

    def predict(self, xs: Sequence[Vec]) -> Tuple[list[float], list[float]]:
        k_s = _rbf(xs, self.x, self.length, self.var)
        mean = [sum(row[i] * self.alpha[i] for i in range(len(self.alpha))) for row in k_s]
        k_ss = _rbf(xs, xs, self.length, self.var)
        var: list[float] = []
        for i, row in enumerate(k_s):
            v = _solve_lower(self.chol, row)
            var.append(max(1e-9, k_ss[i][i] - sum(a * a for a in v)))
        return mean, var


def _candidates(box: ThetaBox, n_dmin: int, n_vmax: int) -> list[Theta]:
    pts: list[Theta] = []
    for i in range(n_dmin):
        for j in range(n_vmax):
            u = i / max(n_dmin - 1, 1)
            v = j / max(n_vmax - 1, 1)
            pts.append(box.from_unit(u, v))
    return pts


def _near_safe(unit: Vec, safe_units: Sequence[Vec], radius: float = 0.18) -> bool:
    r2 = radius * radius
    for other in safe_units:
        du = unit[0] - other[0]
        dv = unit[1] - other[1]
        if du * du + dv * dv <= r2:
            return True
    return False


def run_safe_bo(
    evaluator: OracleEvaluator,
    *,
    n_iter: int = 16,
    d_lim: float = 0.05,
    beta: float = 1.5,
    n_dmin: int = 9,
    n_vmax: int = 7,
    box: ThetaBox | None = None,
    seed_theta: Theta | None = None,
    densify: bool = True,
) -> List[EvaluatedTheta]:
    space = box or ThetaBox()
    seeds = [
        seed_theta or Theta(dmin_m=1.45, vmax_mps=0.45),
        Theta(dmin_m=1.60, vmax_mps=0.30),
        Theta(dmin_m=1.35, vmax_mps=0.70),
        Theta(dmin_m=1.20, vmax_mps=0.50),
    ]
    cands = _candidates(space, n_dmin, n_vmax)
    rows: list[EvaluatedTheta] = []
    queried: set[tuple[float, float]] = set()
    for seed in seeds:
        row = evaluator.evaluate(space.clip(seed))
        key = (round(row.theta.dmin_m, 5), round(row.theta.vmax_mps, 5))
        if key in queried:
            continue
        rows.append(row)
        queried.add(key)

    gp_eff = _GP()
    gp_safe = _GP()

    for _ in range(n_iter):
        xs = [space.to_unit(r.theta) for r in rows]
        gp_eff.fit(xs, [r.jeff for r in rows])
        gp_safe.fit(xs, [r.jsafe for r in rows])
        units = [space.to_unit(th) for th in cands]
        mu_e, var_e = gp_eff.predict(units)
        mu_s, var_s = gp_safe.predict(units)
        safe_units = [
            space.to_unit(r.theta) for r in rows if r.jsafe <= d_lim
        ]
        best_i = -1
        best_acq = -1e18
        for i, theta in enumerate(cands):
            key = (round(theta.dmin_m, 5), round(theta.vmax_mps, 5))
            if key in queried:
                continue
            u_safe = mu_s[i] + beta * math.sqrt(var_s[i])
            expandable = _near_safe(units[i], safe_units)
            if u_safe > d_lim and not expandable:
                continue
            acq = mu_e[i] + beta * math.sqrt(var_e[i])
            if expandable:
                acq += 0.25 * math.sqrt(var_s[i])
            if acq > best_acq:
                best_acq = acq
                best_i = i
        if best_i < 0:
            break
        nxt = evaluator.evaluate(cands[best_i])
        rows.append(nxt)
        queried.add(
            (round(nxt.theta.dmin_m, 5), round(nxt.theta.vmax_mps, 5))
        )

    if densify:
        safe_units = [space.to_unit(r.theta) for r in rows if r.jsafe <= d_lim]
        xs = [space.to_unit(r.theta) for r in rows]
        gp_safe.fit(xs, [r.jsafe for r in rows])
        units = [space.to_unit(th) for th in cands]
        mu_s, var_s = gp_safe.predict(units)
        for i, theta in enumerate(cands):
            key = (round(theta.dmin_m, 5), round(theta.vmax_mps, 5))
            if key in queried:
                continue
            u_safe = mu_s[i] + beta * math.sqrt(var_s[i])
            if u_safe > d_lim and not _near_safe(units[i], safe_units):
                continue
            nxt = evaluator.evaluate(theta)
            rows.append(nxt)
            queried.add(key)
    return rows
