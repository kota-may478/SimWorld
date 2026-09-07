"""5×5 weighted-sum map on a 3-objective ISO Pareto set.

α is efficiency vs safety. β is speed vs distance inside safety.
λ(α,β) = (1-α, αβ, α(1-β)) on min-max normalized (TT, v_max, -S_min).
"""

from __future__ import annotations

from typing import Mapping, Optional, Sequence, Tuple

from constraints.pareto import EvaluatedTheta, iso_pareto

LEVELS: Tuple[float, ...] = (0.0, 0.25, 0.5, 0.75, 1.0)
AbKey = Tuple[float, float]
Weight3 = Tuple[float, float, float]


def snap_level(value: float) -> float:
    return min(LEVELS, key=lambda level: abs(level - float(value)))


def mix_weights(alpha: float, beta: float) -> Weight3:
    a = snap_level(alpha)
    b = snap_level(beta)
    return (1.0 - a, a * b, a * (1.0 - b))


def _span(values: Sequence[float]) -> Tuple[float, float]:
    lo = min(values)
    hi = max(values)
    if hi - lo < 1e-12:
        return lo, lo + 1.0
    return lo, hi


def _normalized(
    rows: Sequence[EvaluatedTheta],
) -> Tuple[Tuple[EvaluatedTheta, Weight3], ...]:
    tts = [row.tt for row in rows]
    vmaxs = [row.theta.vmax_mps for row in rows]
    seps = [row.min_sep_m for row in rows]
    t_lo, t_hi = _span(tts)
    v_lo, v_hi = _span(vmaxs)
    s_lo, s_hi = _span(seps)
    packed: list[Tuple[EvaluatedTheta, Weight3]] = []
    for row in rows:
        f_tt = (row.tt - t_lo) / (t_hi - t_lo)
        f_vmax = (row.theta.vmax_mps - v_lo) / (v_hi - v_lo)
        f_sep = (s_hi - row.min_sep_m) / (s_hi - s_lo)
        packed.append((row, (f_tt, f_vmax, f_sep)))
    return tuple(packed)


def mix_weights_raw(alpha: float, beta: float) -> Weight3:
    a = min(1.0, max(0.0, float(alpha)))
    b = min(1.0, max(0.0, float(beta)))
    return (1.0 - a, a * b, a * (1.0 - b))


def pick_by_lambda(
    rows: Sequence[EvaluatedTheta],
    weights: Weight3,
) -> EvaluatedTheta:
    pool = iso_pareto(tuple(rows))
    if not pool:
        raise ValueError("empty ISO Pareto front")
    ranked = _normalized(pool)

    def score(item: Tuple[EvaluatedTheta, Weight3]) -> Tuple[float, float, float]:
        _row, feats = item
        cost = (
            weights[0] * feats[0]
            + weights[1] * feats[1]
            + weights[2] * feats[2]
        )
        return (cost, _row.tt, -_row.min_sep_m)

    return min(ranked, key=score)[0]


def pick_weighted(
    rows: Sequence[EvaluatedTheta],
    alpha: float,
    beta: float,
) -> EvaluatedTheta:
    return pick_by_lambda(rows, mix_weights(alpha, beta))


def enumerate_table(
    rows: Sequence[EvaluatedTheta],
) -> dict[AbKey, EvaluatedTheta]:
    table: dict[AbKey, EvaluatedTheta] = {}
    for alpha in LEVELS:
        for beta in LEVELS:
            table[(alpha, beta)] = pick_weighted(rows, alpha, beta)
    return table


def dump_table(table: Mapping[AbKey, EvaluatedTheta]) -> list[dict]:
    rows: list[dict] = []
    for alpha, beta in sorted(table):
        picked = table[(alpha, beta)]
        rows.append(
            {
                "alpha": alpha,
                "beta": beta,
                **picked.theta.as_dict(),
                "tt": picked.tt,
                "min_sep_m": picked.min_sep_m,
                "si_min": picked.si_min,
            }
        )
    return rows


def theta_key(row: EvaluatedTheta) -> Tuple[float, float]:
    return (round(row.theta.vmax_mps, 4), round(row.theta.dmin_m, 4))


def step_preference(
    alpha: float,
    beta: float,
    *,
    d_alpha: int,
    d_beta: int,
    theta_table: Optional[Mapping[AbKey, EvaluatedTheta]] = None,
) -> Tuple[float, float]:
    a = snap_level(alpha)
    b = snap_level(beta)
    da = int(d_alpha)
    db = int(d_beta)
    if theta_table is not None:
        return _step_distinct_theta(a, b, d_alpha=da, d_beta=db, table=theta_table)
    if a <= 1e-12 and db != 0 and da == 0:
        da = 1
    ia = LEVELS.index(a)
    ib = LEVELS.index(b)
    ia = max(0, min(len(LEVELS) - 1, ia + da))
    ib = max(0, min(len(LEVELS) - 1, ib + db))
    return LEVELS[ia], LEVELS[ib]


def _row_theta_keys(
    ia: int, table: Mapping[AbKey, EvaluatedTheta]
) -> set[Tuple[float, float]]:
    return {theta_key(table[(LEVELS[ia], LEVELS[j])]) for j in range(len(LEVELS))}


def _step_distinct_theta(
    alpha: float,
    beta: float,
    *,
    d_alpha: int,
    d_beta: int,
    table: Mapping[AbKey, EvaluatedTheta],
) -> Tuple[float, float]:
    hops = abs(d_alpha) + abs(d_beta)
    if hops == 0:
        return alpha, beta
    sign_a = 0 if d_alpha == 0 else (1 if d_alpha > 0 else -1)
    sign_b = 0 if d_beta == 0 else (1 if d_beta > 0 else -1)
    ia = LEVELS.index(alpha)
    ib = LEVELS.index(beta)
    last = len(LEVELS) - 1
    current = theta_key(table[(LEVELS[ia], LEVELS[ib])])

    def in_range(index: int) -> bool:
        return 0 <= index <= last

    for _ in range(hops):
        opened = False
        if (
            sign_b != 0
            and sign_a == 0
            and ia < last
            and len(_row_theta_keys(ia, table)) == 1
        ):
            ia += 1
            nxt = theta_key(table[(LEVELS[ia], LEVELS[ib])])
            if nxt != current:
                current = nxt
                opened = True
        if opened:
            continue
        found = False
        offset = 1
        while offset <= last:
            na = ia if sign_a == 0 else ia + sign_a * offset
            nb = ib if sign_b == 0 else ib + sign_b * offset
            if in_range(na) and in_range(nb) and (na, nb) != (ia, ib):
                nxt = theta_key(table[(LEVELS[na], LEVELS[nb])])
                if nxt != current:
                    ia, ib = na, nb
                    current = nxt
                    found = True
                    break
                offset += 1
                continue
            break
        if not found:
            break
    return LEVELS[ia], LEVELS[ib]
