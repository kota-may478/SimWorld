"""Dimensionless Jeff and Jsafe. Jsafe is a penalty, not a hard constraint."""

from __future__ import annotations

from dataclasses import dataclass

from oracle.simulate import OracleResult

W_TCR = 1.0
W_TT = 1.0
W_SAFE = 1.0


@dataclass(frozen=True)
class ObjectiveBreakdown:
    tcr: float
    tt: float
    jeff: float
    jsafe: float
    j: float
    n_filled: int
    n_sockets: int
    makespan_s: float
    violation_s: float


def score(
    result: OracleResult,
    *,
    w_tcr: float = W_TCR,
    w_tt: float = W_TT,
    w_safe: float = W_SAFE,
    t_ref_s: float | None = None,
) -> ObjectiveBreakdown:
    """Jeff = w1 TCR - w2 TT ; J = Jeff - w3 Jsafe.

    TCR is filled sockets / total.
    TT = makespan / T_ref (may exceed 1 if slower than the reference run).
    Jsafe = T_viol / T_ref (may exceed 1 if the run is stuck near people).
    Maximize J; Jsafe is only a penalty when w3 > 0.
    """
    t_ref = t_ref_s if t_ref_s is not None else result.timeout_s
    if t_ref <= 0.0:
        raise ValueError("t_ref_s must be positive")
    tcr = result.n_filled / max(result.n_sockets, 1)
    tt = result.makespan_s / t_ref
    jsafe = result.violation_s / t_ref
    jeff = w_tcr * tcr - w_tt * tt
    return ObjectiveBreakdown(
        tcr=tcr,
        tt=tt,
        jeff=jeff,
        jsafe=jsafe,
        j=jeff - w_safe * jsafe,
        n_filled=result.n_filled,
        n_sockets=result.n_sockets,
        makespan_s=result.makespan_s,
        violation_s=result.violation_s,
    )
