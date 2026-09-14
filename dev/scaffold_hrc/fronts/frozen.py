"""Frozen ISO-front index for UE5 validation (no search)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

from constraints.pareto import EvaluatedTheta, Theta
from fronts.ab_map import AbKey

TABLE_PATH = Path(__file__).resolve().parent.parent / "data" / "frozen_theta_table.json"


def load_frozen(path: Path | None = None) -> dict:
    return json.loads((path or TABLE_PATH).read_text(encoding="utf-8"))


def theta_table(payload: Mapping | None = None) -> dict[AbKey, EvaluatedTheta]:
    blob = payload or load_frozen()
    table: dict[AbKey, EvaluatedTheta] = {}
    for cell in blob["cells"]:
        table[(float(cell["alpha"]), float(cell["beta"]))] = EvaluatedTheta(
            Theta(vmax_mps=cell["vmax_mps"], dmin_m=cell["dmin_m"]),
            tt=cell["tt_s"],
            t_ssm=cell["vmax_mps"],
            si_min=cell["si_min"],
            completed=True,
            min_sep_m=cell["smin_m"],
        )
    return table


def named_cells(payload: Mapping | None = None) -> tuple[dict, ...]:
    blob = payload or load_frozen()
    return tuple(blob["named"])
