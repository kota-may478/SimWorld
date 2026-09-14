"""Minimize TT [s] and v_max [m/s]. Keep-out is a constraint, not an objective.

TT is the full mission clock (yard, corridor, stairs, scaffold, waits, and
hand assembly). Scaffold-only dwell remains in ``scaffold_time_s`` for
diagnostics. Feasible means completed and S > S_p while keep-out is on
(ISO SI = S/S_p).
"""

from __future__ import annotations

from dataclasses import dataclass

from oracle.simulate import OracleResult

ISO_SI_MIN = 1.0


@dataclass(frozen=True)
class ObjectiveBreakdown:
    tcr: float
    tt: float
    t_ssm: float
    si_min: float
    iso_feasible: bool
    n_filled: int
    n_sockets: int
    makespan_s: float
    ssm_s: float
    violation_s: float
    mission_s: float = 0.0
    scaffold_safe_s: float = 0.0
    scaffold_unsafe_s: float = 0.0
    scaffold_time_s: float = 0.0


def score(
    result: OracleResult,
    *,
    t_ref_s: float | None = None,
    vmax_mps: float = 0.0,
) -> ObjectiveBreakdown:
    """TT = full mission time in seconds. Second Pareto objective is v_max.

    t_ref_s is ignored. t_ssm is filled with vmax_mps so callers that still
    read the field as the second minimize-objective keep working.
    """
    _ = t_ref_s
    tcr = result.n_filled / max(result.n_sockets, 1)
    tt = result.makespan_s
    iso_feasible = result.completed and result.si_min + 1e-9 >= ISO_SI_MIN
    return ObjectiveBreakdown(
        tcr=tcr,
        tt=tt,
        t_ssm=vmax_mps,
        si_min=result.si_min,
        iso_feasible=iso_feasible,
        n_filled=result.n_filled,
        n_sockets=result.n_sockets,
        makespan_s=result.makespan_s,
        ssm_s=result.ssm_s,
        violation_s=result.violation_s,
        mission_s=result.makespan_s,
        scaffold_safe_s=result.scaffold_safe_s,
        scaffold_unsafe_s=result.scaffold_unsafe_s,
        scaffold_time_s=result.scaffold_time_s,
    )
