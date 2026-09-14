"""Closed gold utterances for 3x3 absolute / 5x5 relative catalog mapping.

English-only evaluation set: the original English items from the closed gold
set (Japanese counterparts are omitted from scoring).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class GoldItem:
    uid: str
    text: str
    gold_id: str
    accept_ids: Tuple[str, ...]
    start_alpha: float = 0.5
    start_beta: float = 0.5
    family: str = "absolute"


def gold_items() -> tuple[GoldItem, ...]:
    items: list[GoldItem] = []

    def add(
        uid: str,
        text: str,
        gold_id: str,
        *,
        family: str = "absolute",
        accept: Tuple[str, ...] = (),
        a: float = 0.5,
        b: float = 0.5,
    ) -> None:
        accept_ids = accept if accept else (gold_id,)
        items.append(
            GoldItem(
                uid=uid,
                text=text,
                gold_id=gold_id,
                accept_ids=accept_ids,
                start_alpha=a,
                start_beta=b,
                family=family,
            )
        )

    add("eff_en1", "hurry up", "efficient")
    add("eff_en2", "be efficient", "efficient")
    add("eff_en3", "finish as fast as you can", "efficient")

    add("nd_en1", "keep space, otherwise normal", "normal_distance")
    add("nd_en2", "stay a bit away but otherwise as usual", "normal_distance")

    add("nm_en1", "normal please", "normal")
    add("nm_en2", "balanced setting", "normal")

    add("ns_en1", "unhurried, otherwise normal", "normal_slow")
    add("ns_en2", "no rush, but keep the usual style", "normal_slow")

    add("sd_en1", "stay back", "safe_distance")
    add("sd_en2", "keep more distance", "safe_distance")
    add("sd_en3", "give the worker a wider berth", "safe_distance")

    add("sf_en1", "be careful", "safe")
    add("sf_en2", "cautiously", "safe")
    add("sf_en3", "prioritize safety", "safe")

    add("ss_en1", "please go slow", "safe_slow")
    add("ss_en2", "move slowly", "safe_slow")
    add("ss_en3", "reduce your speed", "safe_slow")

    add("rs_en1", "reset", "reset", family="special")
    add("rs_en2", "as usual", "reset", family="special")

    add("rel_slow3", "a bit slower", "a_bit_slower", family="relative")
    add("rel_slow4", "a little slower please", "a_bit_slower", family="relative")
    add("rel_slow7", "much slower", "much_slower", family="relative")
    add("rel_slow8", "a lot slower", "much_slower", family="relative")

    add("rel_fast3", "a bit faster", "a_bit_faster", family="relative")
    add("rel_fast6", "much faster", "much_faster", family="relative")

    add("rel_far3", "a bit farther", "a_bit_farther", family="relative")
    add("rel_far4", "keep a little more distance", "a_bit_farther", family="relative")
    add("rel_far7", "much farther", "much_farther", family="relative")

    add("rel_safe3", "a bit more careful", "a_bit_safer", family="relative")
    add("rel_safe6", "much more cautious", "much_safer", family="relative")

    add("ood_en1", "what time is lunch", "unchanged", family="ood")
    add("ood_en2", "tell me a joke", "unchanged", family="ood")

    add(
        "cmp_en1",
        "go slow and stay far away",
        "safe_slow",
        family="composite",
        accept=("safe_slow", "safe_distance", "safe"),
    )

    return tuple(items)
