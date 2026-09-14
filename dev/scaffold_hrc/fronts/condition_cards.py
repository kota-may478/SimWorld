"""Stage-1 paper condition cards: language (B3/B4) + SafeOpt vs proposed.

Utterances are a small closed set that still spans efficient / safe / distance /
relative, matching the two contrast axes requested for UE validation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ConditionCard:
    card_id: str
    family: str  # language | safety_search
    text: str
    start_alpha: float
    start_beta: float
    methods: Tuple[str, ...]
    notes: str


def condition_cards() -> tuple[ConditionCard, ...]:
    lang = ("proposed", "b3_keyword", "b4_discrete_mode")
    return (
        ConditionCard(
            "L1_efficient",
            "language",
            "急いで",
            0.5,
            0.5,
            lang,
            "absolute efficient vs keyword / 3-mode",
        ),
        ConditionCard(
            "L2_safe",
            "language",
            "慎重に",
            0.5,
            0.5,
            lang,
            "absolute safe vs keyword / 3-mode",
        ),
        ConditionCard(
            "L3_distance",
            "language",
            "離れて作業して",
            0.5,
            0.5,
            lang,
            "distance preference; B3 collapses to conservative",
        ),
        ConditionCard(
            "L4_relative_slower",
            "language",
            "もう少しゆっくり",
            0.5,
            0.5,
            ("proposed", "b4_discrete_mode"),
            "relative 5x5 hop; B4 has no relative axis",
        ),
        ConditionCard(
            "S1_safeopt_vs_normal",
            "safety_search",
            "",
            0.5,
            0.5,
            ("proposed", "safeopt"),
            "SafeOpt incumbent vs proposed normal cell",
        ),
        ConditionCard(
            "S2_safeopt_vs_safe",
            "safety_search",
            "",
            1.0,
            0.5,
            ("proposed", "safeopt"),
            "SafeOpt incumbent vs proposed safe cell",
        ),
    )
