"""Closed gold utterances for 3x3 absolute / 5x5 relative catalog mapping."""

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

    add("eff_jp1", "急いで", "efficient")
    add("eff_jp2", "早く終わらせて", "efficient")
    add("eff_jp3", "効率よく進めて", "efficient")
    add("eff_jp4", "時間を優先して", "efficient")
    add("eff_en1", "hurry up", "efficient")
    add("eff_en2", "be efficient", "efficient")
    add("eff_en3", "finish as fast as you can", "efficient")

    add("nd_jp1", "普通でいいが近づきすぎない", "normal_distance")
    add("nd_jp2", "いつも通りで距離だけ取って", "normal_distance")
    add("nd_en1", "keep space, otherwise normal", "normal_distance")
    add("nd_en2", "stay a bit away but otherwise as usual", "normal_distance")

    add("nm_jp1", "普通に", "normal")
    add("nm_jp2", "バランスよく", "normal")
    add("nm_en1", "normal please", "normal")
    add("nm_en2", "balanced setting", "normal")

    add("ns_jp1", "普通だが急がない", "normal_slow")
    add("ns_jp2", "急がず普通に", "normal_slow")
    add("ns_en1", "unhurried, otherwise normal", "normal_slow")
    add("ns_en2", "no rush, but keep the usual style", "normal_slow")

    add("sd_jp1", "離れて作業して", "safe_distance")
    add("sd_jp2", "距離を取って", "safe_distance")
    add("sd_jp3", "人からもっと離れて", "safe_distance")
    add("sd_en1", "stay back", "safe_distance")
    add("sd_en2", "keep more distance", "safe_distance")
    add("sd_en3", "give the worker a wider berth", "safe_distance")

    add("sf_jp1", "慎重に", "safe")
    add("sf_jp2", "安全に", "safe")
    add("sf_jp3", "危なくないように", "safe")
    add("sf_en1", "be careful", "safe")
    add("sf_en2", "cautiously", "safe")
    add("sf_en3", "prioritize safety", "safe")

    add("ss_jp1", "ゆっくり動いて", "safe_slow")
    add("ss_jp2", "減速して", "safe_slow")
    add("ss_jp3", "速度を落として", "safe_slow")
    add("ss_en1", "please go slow", "safe_slow")
    add("ss_en2", "move slowly", "safe_slow")
    add("ss_en3", "reduce your speed", "safe_slow")

    add("rs_jp1", "リセット", "reset", family="special")
    add("rs_jp2", "いつも通り", "reset", family="special")
    add("rs_en1", "reset", "reset", family="special")
    add("rs_en2", "as usual", "reset", family="special")

    add("rel_slow1", "もう少しゆっくり", "a_bit_slower", family="relative")
    add("rel_slow2", "もうすこし遅く", "a_bit_slower", family="relative")
    add("rel_slow3", "a bit slower", "a_bit_slower", family="relative")
    add("rel_slow4", "a little slower please", "a_bit_slower", family="relative")
    add("rel_slow5", "かなりゆっくり", "much_slower", family="relative")
    add("rel_slow6", "もっと遅く", "much_slower", family="relative")
    add("rel_slow7", "much slower", "much_slower", family="relative")
    add("rel_slow8", "a lot slower", "much_slower", family="relative")

    add("rel_fast1", "もう少し急いで", "a_bit_faster", family="relative")
    add("rel_fast2", "もう少し早く", "a_bit_faster", family="relative")
    add("rel_fast3", "a bit faster", "a_bit_faster", family="relative")
    add("rel_fast4", "かなり急いで", "much_faster", family="relative")
    add("rel_fast5", "もっと早く", "much_faster", family="relative")
    add("rel_fast6", "much faster", "much_faster", family="relative")

    add("rel_far1", "もう少し離れて", "a_bit_farther", family="relative")
    add("rel_far2", "もう少し距離を取って", "a_bit_farther", family="relative")
    add("rel_far3", "a bit farther", "a_bit_farther", family="relative")
    add("rel_far4", "keep a little more distance", "a_bit_farther", family="relative")
    add("rel_far5", "かなり離れて", "much_farther", family="relative")
    add("rel_far6", "もっと離れて", "much_farther", family="relative")
    add("rel_far7", "much farther", "much_farther", family="relative")

    add("rel_safe1", "もう少し慎重に", "a_bit_safer", family="relative")
    add("rel_safe2", "もう少し安全に", "a_bit_safer", family="relative")
    add("rel_safe3", "a bit more careful", "a_bit_safer", family="relative")
    add("rel_safe4", "かなり慎重に", "much_safer", family="relative")
    add("rel_safe5", "もっと安全に", "much_safer", family="relative")
    add("rel_safe6", "much more cautious", "much_safer", family="relative")

    add(
        "rel_from_eff",
        "もう少しゆっくり",
        "a_bit_slower",
        family="relative",
        a=0.0,
        b=0.5,
    )
    add(
        "rel_from_safe",
        "もう少し急いで",
        "a_bit_faster",
        family="relative",
        a=1.0,
        b=0.5,
    )

    add("ood_jp1", "今日の天気は？", "unchanged", family="ood")
    add("ood_jp2", "ありがとう", "unchanged", family="ood")
    add("ood_jp3", "足場を組み立てて", "unchanged", family="ood")
    add("ood_en1", "what time is lunch", "unchanged", family="ood")
    add("ood_en2", "tell me a joke", "unchanged", family="ood")

    add(
        "cmp_jp1",
        "ゆっくり離れて",
        "safe_slow",
        family="composite",
        accept=("safe_slow", "safe_distance", "safe"),
    )
    add(
        "cmp_jp2",
        "急いでいいが人には近づくな",
        "normal_distance",
        family="composite",
        accept=("normal_distance", "safe_distance", "efficient"),
    )
    add(
        "cmp_en1",
        "go slow and stay far away",
        "safe_slow",
        family="composite",
        accept=("safe_slow", "safe_distance", "safe"),
    )

    return tuple(items)
