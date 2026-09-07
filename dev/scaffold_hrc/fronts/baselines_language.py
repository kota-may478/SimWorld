"""Language-to-preference baselines used in the closed comparison."""

from __future__ import annotations

import json
import re
from typing import Optional, Sequence

from constraints.pareto import EvaluatedTheta, Theta
from fronts.ab_map import mix_weights_raw, pick_by_lambda, pick_weighted, snap_level
from fronts.catalog import apply_catalog_label
from fronts.language import Preference, _keyword_absolute, ground_utterance
from fronts.space import ThetaBox

_NUM = re.compile(
    r"vmax[^0-9]*([0-9]*\.?[0-9]+).*?dmin[^0-9]*([0-9]*\.?[0-9]+)",
    re.IGNORECASE | re.DOTALL,
)
_LAMBDA = re.compile(
    r"\[?\s*([0-9]*\.?[0-9]+)\s*,\s*([0-9]*\.?[0-9]+)\s*,\s*([0-9]*\.?[0-9]+)\s*\]?"
)
_AB = re.compile(
    r"alpha[^0-9]*([0-9]*\.?[0-9]+).*?beta[^0-9]*([0-9]*\.?[0-9]+)",
    re.IGNORECASE | re.DOTALL,
)


def keyword_pref(text: str, state: Preference) -> Preference:
    return ground_utterance(text, state=state)


def embed_pref(text: str, state: Preference, embedder: object) -> Preference:
    return ground_utterance(text, state=state, embedder=embedder)


def discrete_3mode(text: str, state: Preference) -> Preference:
    pref = keyword_pref(text, state)
    if pref.kind == "relative":
        alpha = snap_level(pref.alpha)
        if alpha <= 0.25:
            return Preference(0.0, 0.5, kind="absolute", cell="efficient", notes="3mode")
        if alpha >= 0.75:
            return Preference(1.0, 0.5, kind="absolute", cell="safe", notes="3mode")
        return Preference(0.5, 0.5, kind="absolute", cell="normal", notes="3mode")
    if pref.kind in ("reset", "unchanged"):
        return Preference(0.5, 0.5, kind=pref.kind, cell="normal", notes="3mode")
    alpha = 0.0 if pref.alpha < 0.25 else (1.0 if pref.alpha > 0.75 else 0.5)
    cell = "efficient" if alpha == 0.0 else ("safe" if alpha == 1.0 else "normal")
    return Preference(alpha, 0.5, kind="absolute", cell=cell, notes="3mode")


def alpha_1d(text: str, state: Preference) -> Preference:
    pref = keyword_pref(text, state)
    return Preference(
        pref.alpha,
        0.5,
        kind=pref.kind,
        cell=pref.cell,
        notes="alpha1d",
    )


def speed_scale(text: str, state: Preference) -> Preference:
    blob = text.lower()
    d_alpha = 0
    d_beta = 0
    if any(token in blob for token in ("ゆっくり", "slow", "減速")):
        d_beta = 2 if any(t in blob for t in ("かなり", "もっと", "much")) else 1
    elif any(token in blob for token in ("急い", "fast", "hurry", "早く")):
        d_alpha = -2 if any(t in blob for t in ("かなり", "もっと", "much")) else -1
    elif any(token in blob for token in ("離れ", "距離", "far", "distance")):
        d_beta = -2 if any(t in blob for t in ("かなり", "もっと", "much")) else -1
    elif any(token in blob for token in ("慎重", "安全", "careful", "cautious")):
        d_alpha = 2 if any(t in blob for t in ("かなり", "もっと", "much")) else 1
    else:
        keyed = _keyword_absolute(text)
        if keyed is not None:
            return keyed
        return Preference(
            state.alpha,
            state.beta,
            kind="unchanged",
            cell=state.cell or "normal",
            notes="speed-scale-miss",
        )
    return apply_catalog_label(
        {
            (0, 1): "a_bit_slower",
            (0, 2): "much_slower",
            (-1, 0): "a_bit_faster",
            (-2, 0): "much_faster",
            (0, -1): "a_bit_farther",
            (0, -2): "much_farther",
            (1, 0): "a_bit_safer",
            (2, 0): "much_safer",
        }[(d_alpha, d_beta)],
        state,
    )


def _first_json(raw: str) -> Optional[dict]:
    match = re.search(r"\{.*\}", raw, re.DOTALL)
    if match is None:
        return None
    try:
        payload = json.loads(match.group(0))
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def qwen_direct_theta_pref(
    raw: str,
    state: Preference,
    rows: Sequence[EvaluatedTheta],
) -> Preference:
    box = ThetaBox()
    vmax = None
    dmin = None
    payload = _first_json(raw)
    if payload is not None:
        vmax = payload.get("vmax") or payload.get("vmax_mps")
        dmin = payload.get("dmin") or payload.get("dmin_m")
    if vmax is None or dmin is None:
        match = _NUM.search(raw)
        if match is not None:
            vmax, dmin = match.group(1), match.group(2)
    if vmax is None or dmin is None:
        return Preference(
            state.alpha,
            state.beta,
            kind="unchanged",
            cell="normal",
            notes="direct-theta-miss",
        )
    theta = box.clip(Theta(vmax_mps=float(vmax), dmin_m=float(dmin)))
    nearest = min(
        rows,
        key=lambda row: (row.theta.vmax_mps - theta.vmax_mps) ** 2
        + (row.theta.dmin_m - theta.dmin_m) ** 2,
    )
    picked = pick_weighted((nearest,), 0.0, 0.5)
    _ = picked
    return Preference(
        0.5,
        0.5,
        kind="absolute",
        cell="normal",
        notes=f"direct-theta:{theta.vmax_mps:.3f},{theta.dmin_m:.3f}",
    )


def nearest_iso_theta(
    vmax: float,
    dmin: float,
    rows: Sequence[EvaluatedTheta],
) -> EvaluatedTheta:
    box = ThetaBox()
    theta = box.clip(Theta(vmax_mps=vmax, dmin_m=dmin))
    return min(
        rows,
        key=lambda row: (row.theta.vmax_mps - theta.vmax_mps) ** 2
        + (row.theta.dmin_m - theta.dmin_m) ** 2,
    )


def parse_direct_theta(raw: str) -> Optional[Theta]:
    payload = _first_json(raw)
    if payload is not None:
        vmax = payload.get("vmax") or payload.get("vmax_mps")
        dmin = payload.get("dmin") or payload.get("dmin_m")
        if vmax is not None and dmin is not None:
            try:
                return Theta(vmax_mps=float(vmax), dmin_m=float(dmin))
            except (TypeError, ValueError):
                return None
    match = _NUM.search(raw)
    if match is None:
        return None
    try:
        return Theta(vmax_mps=float(match.group(1)), dmin_m=float(match.group(2)))
    except ValueError:
        return None


def parse_continuous_ab(raw: str) -> Optional[tuple[float, float]]:
    payload = _first_json(raw)
    if payload is not None and "alpha" in payload and "beta" in payload:
        try:
            return float(payload["alpha"]), float(payload["beta"])
        except (TypeError, ValueError):
            return None
    match = _AB.search(raw)
    if match is None:
        return None
    try:
        return float(match.group(1)), float(match.group(2))
    except ValueError:
        return None


def parse_lambda3(raw: str) -> Optional[tuple[float, float, float]]:
    payload = _first_json(raw)
    if payload is not None:
        vals = payload.get("lambda") or payload.get("weights")
        if isinstance(vals, list) and len(vals) == 3:
            try:
                return float(vals[0]), float(vals[1]), float(vals[2])
            except (TypeError, ValueError):
                return None
        if all(k in payload for k in ("w_tt", "w_vmax", "w_sep")):
            try:
                return (
                    float(payload["w_tt"]),
                    float(payload["w_vmax"]),
                    float(payload["w_sep"]),
                )
            except (TypeError, ValueError):
                return None
    match = _LAMBDA.search(raw)
    if match is None:
        return None
    try:
        return float(match.group(1)), float(match.group(2)), float(match.group(3))
    except ValueError:
        return None


def pick_continuous_ab(
    rows: Sequence[EvaluatedTheta],
    alpha: float,
    beta: float,
) -> EvaluatedTheta:
    return pick_by_lambda(rows, mix_weights_raw(alpha, beta))
