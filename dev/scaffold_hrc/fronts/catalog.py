"""Closed 3x3 absolute + 5x5 relative label catalog for LLM extraction."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

from fronts.ab_map import AbKey, step_preference
from fronts.language import Preference, cell_for_levels
from constraints.pareto import EvaluatedTheta

CATALOG_PATH = Path(__file__).resolve().parent.parent / "data" / "preference_catalog.json"


@dataclass(frozen=True)
class CatalogEntry:
    label: str
    kind: str
    alpha: Optional[float] = None
    beta: Optional[float] = None
    d_alpha: int = 0
    d_beta: int = 0
    gloss: str = ""


def load_catalog(path: Path | None = None) -> tuple[CatalogEntry, ...]:
    payload = json.loads((path or CATALOG_PATH).read_text(encoding="utf-8"))
    entries: list[CatalogEntry] = []
    for item in payload["absolute"]:
        entries.append(
            CatalogEntry(
                label=item["id"],
                kind="absolute",
                alpha=float(item["alpha"]),
                beta=float(item["beta"]),
                gloss=item.get("gloss", ""),
            )
        )
    for item in payload["relative"]:
        entries.append(
            CatalogEntry(
                label=item["id"],
                kind="relative",
                d_alpha=int(item["d_alpha"]),
                d_beta=int(item["d_beta"]),
                gloss=item.get("gloss", ""),
            )
        )
    for item in payload["special"]:
        entries.append(
            CatalogEntry(
                label=item["id"],
                kind=str(item["kind"]),
                gloss=item.get("gloss", ""),
            )
        )
    return tuple(entries)


def catalog_labels(entries: tuple[CatalogEntry, ...] | None = None) -> tuple[str, ...]:
    packed = entries or load_catalog()
    return tuple(sorted((item.label for item in packed), key=len, reverse=True))


def parse_from_options(raw: str, options: Sequence[str]) -> Optional[str]:
    blob = raw.strip().strip("`\"'")
    if not blob:
        return None
    line = blob.splitlines()[0]
    best: Optional[str] = None
    best_pos = len(line) + 1
    for label in options:
        match = re.search(
            rf"(?i)(?<![A-Za-z0-9_]){re.escape(label)}(?![A-Za-z0-9_])",
            line,
        )
        if match is not None and match.start() < best_pos:
            best = label
            best_pos = match.start()
    return best


def parse_catalog_label(
    raw: str,
    entries: tuple[CatalogEntry, ...] | None = None,
) -> Optional[str]:
    packed = entries or load_catalog()
    blob = raw.strip().strip("`\"'")
    for label in catalog_labels(packed):
        if re.search(rf"(?i)(?<![A-Za-z0-9_]){re.escape(label)}(?![A-Za-z0-9_])", blob):
            return label
    return None


def entry_for(label: str, entries: tuple[CatalogEntry, ...] | None = None) -> CatalogEntry:
    packed = entries or load_catalog()
    for item in packed:
        if item.label == label:
            return item
    raise KeyError(label)


def apply_catalog_label(
    label: str,
    state: Preference,
    entries: tuple[CatalogEntry, ...] | None = None,
    theta_table: Mapping[AbKey, EvaluatedTheta] | None = None,
) -> Preference:
    item = entry_for(label, entries)
    if item.kind == "reset":
        return Preference(0.5, 0.5, kind="reset", cell="normal", notes="catalog-reset")
    if item.kind == "unchanged":
        return Preference(
            state.alpha,
            state.beta,
            kind="unchanged",
            cell=cell_for_levels(state.alpha, state.beta),
            notes="catalog-unchanged",
        )
    if item.kind == "relative":
        alpha, beta = step_preference(
            state.alpha,
            state.beta,
            d_alpha=item.d_alpha,
            d_beta=item.d_beta,
            theta_table=theta_table,
        )
        return Preference(
            alpha,
            beta,
            kind="relative",
            cell=cell_for_levels(alpha, beta),
            notes=f"catalog-{item.label}",
        )
    assert item.alpha is not None and item.beta is not None
    return Preference(
        item.alpha,
        item.beta,
        kind="absolute",
        cell=item.label,
        notes=f"catalog-{item.label}",
    )


RELATIVE_DELTA_LABELS = {
    (0, 1): "a_bit_slower",
    (0, 2): "much_slower",
    (-1, 0): "a_bit_faster",
    (-2, 0): "much_faster",
    (0, -1): "a_bit_farther",
    (0, -2): "much_farther",
    (1, 0): "a_bit_safer",
    (2, 0): "much_safer",
    (0, 0): "unchanged",
}


def relative_steps_to_label(d_alpha: int, d_beta: int) -> str:
    return RELATIVE_DELTA_LABELS.get((int(d_alpha), int(d_beta)), "")


def pref_to_catalog_id(pref: Preference, start: Preference) -> str:
    note = pref.notes
    if note.startswith("catalog-"):
        return note.split("catalog-", 1)[1]
    if pref.kind == "reset":
        return "reset"
    if pref.kind == "unchanged":
        return "unchanged"
    if pref.kind == "absolute" and pref.cell:
        return pref.cell
    from fronts.ab_map import LEVELS, snap_level

    ia0 = LEVELS.index(snap_level(start.alpha))
    ib0 = LEVELS.index(snap_level(start.beta))
    ia1 = LEVELS.index(snap_level(pref.alpha))
    ib1 = LEVELS.index(snap_level(pref.beta))
    delta = (ia1 - ia0, ib1 - ib0)
    return RELATIVE_DELTA_LABELS.get(delta, pref.cell or "unchanged")


def catalog_prompt_block(entries: tuple[CatalogEntry, ...] | None = None) -> str:
    packed = entries or load_catalog()
    lines = ["Allowed labels (reply with exactly one label, nothing else):"]
    for item in packed:
        lines.append(f"- {item.label}: {item.gloss}")
    return "\n".join(lines)
