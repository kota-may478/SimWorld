"""Map a preference or raw θ onto an executable controller setting."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from constraints.pareto import EvaluatedTheta, Theta, iso_pareto, project
from fronts.ab_map import enumerate_table, pick_weighted, snap_level
from fronts.evaluate import OracleEvaluator
from fronts.language import Preference, ground_utterance
from fronts.safe_bo import best_safe_incumbent, run_safe_bo
from fronts.space import HALLUCINATED_THETA, ThetaBox


@dataclass(frozen=True)
class GroundingResult:
    theta: Theta
    method: str
    iso_feasible: bool
    tt: Optional[float] = None
    t_ssm: Optional[float] = None
    si_min: Optional[float] = None
    notes: str = ""
    alpha: Optional[float] = None
    beta: Optional[float] = None
    cell: str = ""
    min_sep_m: Optional[float] = None


def b1_direct(theta_raw: Theta) -> GroundingResult:
    """Use the numeric θ as given. May leave the box or the ISO set."""
    return GroundingResult(theta=theta_raw, method="b1_direct", iso_feasible=False)


def b2_clip_box(theta_raw: Theta, box: ThetaBox | None = None) -> GroundingResult:
    space = box or ThetaBox()
    return GroundingResult(
        theta=space.clip(theta_raw),
        method="b2_clip_box",
        iso_feasible=False,
        notes="clipped to ThetaBox only",
    )


def b3_keyword(text: str, box: ThetaBox | None = None) -> GroundingResult:
    """Fixed rule table from preference words. Not a Pareto point."""
    space = box or ThetaBox()
    blob = text.lower()
    if any(token in blob for token in ("safe", "slow", "cautious", "慎重", "安全")):
        theta = space.conservative()
        note = "keyword: conservative"
    elif any(token in blob for token in ("fast", "hurry", "急", "早く")):
        theta = space.aggressive()
        note = "keyword: aggressive"
    else:
        theta = space.midpoint()
        note = "keyword: balanced default"
    return GroundingResult(
        theta=space.clip(theta),
        method="b3_keyword",
        iso_feasible=False,
        notes=note,
    )


def b4_discrete_mode(mode: str, box: ThetaBox | None = None) -> GroundingResult:
    """Three named operating modes inside the box."""
    space = box or ThetaBox()
    key = mode.strip().lower()
    if key in ("efficient", "fast", "alpha0"):
        theta = space.aggressive()
    elif key in ("conservative", "safe", "alpha1"):
        theta = space.conservative()
    else:
        theta = space.midpoint()
    return GroundingResult(
        theta=space.clip(theta),
        method="b4_discrete_mode",
        iso_feasible=False,
        notes=key,
    )


def b5_simulate_reject(
    theta_raw: Theta,
    evaluator: OracleEvaluator,
    *,
    box: ThetaBox | None = None,
    n_retry: int = 4,
) -> GroundingResult:
    """Clip, evaluate, and retract toward a conservative seed until ISO-feasible."""
    space = box or ThetaBox()
    conservative = space.conservative()
    original = space.clip(theta_raw)
    last: EvaluatedTheta | None = None
    cand = original
    for step in range(n_retry + 1):
        if step == 0:
            cand = original
        else:
            u = step / n_retry
            cand = space.clip(
                Theta(
                    vmax_mps=original.vmax_mps
                    + u * (conservative.vmax_mps - original.vmax_mps),
                    dmin_m=original.dmin_m
                    + u * (conservative.dmin_m - original.dmin_m),
                )
            )
        last = evaluator.evaluate(cand)
        if last.iso_feasible:
            return GroundingResult(
                theta=cand,
                method="b5_simulate_reject",
                iso_feasible=True,
                tt=last.tt,
                t_ssm=last.t_ssm,
                si_min=last.si_min,
                notes=f"accepted after {step} retractions",
            )
    assert last is not None
    return GroundingResult(
        theta=cand,
        method="b5_simulate_reject",
        iso_feasible=False,
        tt=last.tt,
        t_ssm=last.t_ssm,
        si_min=last.si_min,
        notes="no ISO-feasible retraction",
    )


def proposed(
    alpha: float,
    front: Sequence[Theta],
    *,
    beta: float = 0.5,
    theta_llm: Theta | None = None,
    rows: Sequence[EvaluatedTheta] | None = None,
) -> GroundingResult:
    """Select θ* from P. Hallucinated numeric θ is discarded.

    If `rows` (evaluated ISO points) are given, use the 5×5 weighted sum.
    Otherwise index a 1-D Theta list by α (legacy tests / B comparisons).
    """
    if not front and not rows:
        raise ValueError("ISO Pareto front P is empty; cannot project alpha")
    _ = theta_llm or HALLUCINATED_THETA
    a = snap_level(alpha)
    b = snap_level(beta)
    if rows:
        picked = pick_weighted(rows, a, b)
        return GroundingResult(
            theta=picked.theta,
            method="proposed",
            iso_feasible=True,
            tt=picked.tt,
            t_ssm=picked.t_ssm,
            si_min=picked.si_min,
            notes="weighted sum on ISO front",
            alpha=a,
            beta=b,
            min_sep_m=picked.min_sep_m,
        )
    if not front:
        raise ValueError("ISO Pareto front P is empty; cannot project alpha")
    theta = project(Theta(vmax_mps=a, dmin_m=b), alpha=a, front=front)
    return GroundingResult(
        theta=theta,
        method="proposed",
        iso_feasible=True,
        notes="legacy TT index on Theta front",
        alpha=a,
        beta=b,
    )


def proposed_from_text(
    text: str,
    rows: Sequence[EvaluatedTheta],
    *,
    state: Preference | None = None,
    embedder: object | None = None,
    generator: object | None = None,
) -> GroundingResult:
    table = enumerate_table(tuple(rows))
    pref = ground_utterance(
        text,
        state=state,
        embedder=embedder,
        generator=generator,
        theta_table=table,
    )
    picked = pick_weighted(rows, pref.alpha, pref.beta)
    return GroundingResult(
        theta=picked.theta,
        method="proposed",
        iso_feasible=True,
        tt=picked.tt,
        t_ssm=picked.t_ssm,
        si_min=picked.si_min,
        notes=pref.notes or pref.kind,
        alpha=pref.alpha,
        beta=pref.beta,
        cell=pref.cell,
        min_sep_m=picked.min_sep_m,
    )


def safeopt_search(
    evaluator: OracleEvaluator,
    *,
    box: ThetaBox | None = None,
    d_lim: float = 0.55,
    n_iter: int = 16,
    n_vmax: int = 9,
    n_dmin: int = 7,
    densify: bool = False,
) -> GroundingResult:
    """Apply SafeOpt: min TT s.t. v_max <= d_lim. Returns the incumbent."""
    space = box or ThetaBox()
    rows = run_safe_bo(
        evaluator,
        n_iter=n_iter,
        d_lim=d_lim,
        n_vmax=n_vmax,
        n_dmin=n_dmin,
        box=space,
        densify=densify,
    )
    best = best_safe_incumbent(rows, d_lim=d_lim)
    return GroundingResult(
        theta=best.theta,
        method="safeopt",
        iso_feasible=best.iso_feasible,
        tt=best.tt,
        t_ssm=best.t_ssm,
        si_min=best.si_min,
        notes=f"min TT s.t. T_SSM<={d_lim}; n_eval={len(rows)}",
    )


def front_thetas(rows: Sequence[EvaluatedTheta]) -> tuple[Theta, ...]:
    return tuple(row.theta for row in iso_pareto(rows))


def score_grounding(
    item: GroundingResult,
    evaluator: OracleEvaluator,
) -> GroundingResult:
    """Fill TT / T_SSM / SI_min by running the oracle. Skip if already scored."""
    if item.tt is not None and item.t_ssm is not None and item.si_min is not None:
        return item
    row = evaluator.evaluate(item.theta)
    return GroundingResult(
        theta=item.theta,
        method=item.method,
        iso_feasible=row.iso_feasible,
        tt=row.tt,
        t_ssm=row.t_ssm,
        si_min=row.si_min,
        notes=item.notes,
    )


def compare_methods(
    *,
    alpha: float,
    front: Sequence[Theta],
    evaluator: OracleEvaluator,
    hallucinated: Theta | None = None,
    preference_text: str = "please be more cautious",
    mode: str = "balanced",
    score_all: bool = True,
    include_safeopt: bool = True,
    safeopt_n_iter: int = 8,
    safeopt_n_vmax: int = 6,
    safeopt_n_dmin: int = 5,
    safeopt_d_lim: float = 0.55,
    beta: float = 0.5,
    rows: Sequence[EvaluatedTheta] | None = None,
) -> tuple[GroundingResult, ...]:
    """Proposed (α, β) vs B1–B5 and SafeOpt."""
    raw = hallucinated or HALLUCINATED_THETA
    items = [
        proposed(alpha, front, beta=beta, theta_llm=raw, rows=rows),
        b1_direct(raw),
        b2_clip_box(raw),
        b3_keyword(preference_text),
        b4_discrete_mode(mode),
        b5_simulate_reject(raw, evaluator),
    ]
    if include_safeopt:
        if score_all:
            items.append(
                safeopt_search(
                    evaluator,
                    d_lim=safeopt_d_lim,
                    n_iter=safeopt_n_iter,
                    n_vmax=safeopt_n_vmax,
                    n_dmin=safeopt_n_dmin,
                )
            )
        else:
            items.append(
                GroundingResult(
                    theta=ThetaBox().conservative(),
                    method="safeopt",
                    iso_feasible=False,
                    notes="deferred; score_all=False",
                )
            )
    if not score_all:
        return tuple(items)
    scored: list[GroundingResult] = []
    for item in items:
        if item.method == "b1_direct":
            scored.append(item)
            continue
        scored.append(score_grounding(item, evaluator))
    return tuple(scored)
