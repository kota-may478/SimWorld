"""Bare-ground erect sequence shared by oracle and UE.

1F is ground (z = 0). Each lift's posts carry the *next* working deck:
  建地 of this lift
  → 1F 布/床 with the first lift only
  → 交差筋違 (braces)
  → 上側の布 / 横架材 of the deck at the top of this lift (2F, then 3F)
  → その階の床
  → 階段塔の建地, then 踏み板 (zigzag AABB; climb onto that deck)
    then the next lift's posts, delivered up the stair onto that deck.

Do not stand the next lift's posts before that deck exists. After the
2F deck and its stair are in, Spot and the assembler work from 2F.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from scene.geometry import STAGE1_GEOM, ScaffoldGeom
from scene.scaffold_grammar import Module, N_WORKING_BAYS, build_scaffold


@dataclass(frozen=True)
class ErectItem:
    item_id: str
    kind: str
    floor: int
    module: Module
    # After this item is placed, Spot may treat this floor as walkable / climbable.
    unlocks_deck: bool = False
    unlocks_stair_lift: Optional[int] = None


def _floor_of_module(module: Module, geom: ScaffoldGeom) -> int:
    if module.kind == "stair_tread":
        # tread_L{lift}_t — stair serves access toward lift+2 floor
        try:
            lift = int(module.module_id.split("_")[1].replace("L", ""))
        except (IndexError, ValueError):
            lift = 0
        return lift + 1
    z = module.z_m
    if module.z_is_center:
        z = module.z_m
    # posts sit at lift base; attribute to upper floor of that lift
    if module.kind == "post":
        lift = int(round(module.z_m / max(geom.lift_m, 1e-6)))
        return min(geom.n_floors, lift + 1)
    if module.kind == "brace":
        lift = int(round((module.z_m - 0.5 * geom.lift_m) / max(geom.lift_m, 1e-6)))
        return min(geom.n_floors, max(1, lift + 1))
    # ledgers / boards / transoms use floor z
    for floor in range(1, geom.n_floors + 1):
        if abs(module.z_m - geom.floor_z_m(floor)) < 0.25:
            return floor
    return 1


def build_erect_sequence(geom: ScaffoldGeom) -> Tuple[ErectItem, ...]:
    """Return install tasks in bare-ground assembly order."""
    spec = build_scaffold(geom)
    by_id = {m.module_id: m for m in spec.modules}
    items: list[ErectItem] = []
    n_lifts = geom.n_floors - 1

    def add(module_id: str, *, unlocks_deck: bool = False, unlocks_stair_lift: Optional[int] = None) -> None:
        module = by_id[module_id]
        items.append(
            ErectItem(
                item_id=module_id,
                kind=module.kind,
                floor=_floor_of_module(module, geom),
                module=module,
                unlocks_deck=unlocks_deck,
                unlocks_stair_lift=unlocks_stair_lift,
            )
        )

    for lift in range(n_lifts):
        # 1F = ground. This lift's posts carry deck_floor (2F, then 3F).
        deck_floor = lift + 2
        # 1) 建地 of this lift (working deck + stair-bay posts, not the ramp tower)
        for mid in sorted(by_id):
            if (
                mid.startswith("post_")
                and mid.endswith(f"_L{lift}")
                and not mid.startswith("post_ramp_")
            ):
                add(mid)
        # Ground 1F 布/床 only with the first lift
        if lift == 0:
            for mid in sorted(by_id):
                if mid.startswith("ledger_f1_") or mid.startswith("transom_f1_"):
                    add(mid)
            for mid in sorted(by_id):
                if mid.startswith("boardslot_f1_"):
                    add(mid, unlocks_deck=True)
        # 2) 交差筋違 of this lift
        for mid in sorted(by_id):
            if mid.startswith(f"brace_f{lift}_"):
                add(mid)
        # 3) 上側の布 / 横架材 of the deck at the top of this lift
        for mid in sorted(by_id):
            if mid.startswith(f"ledger_f{deck_floor}_") or mid.startswith(
                f"transom_f{deck_floor}_"
            ):
                add(mid)
        # 4) その階の床 — laid from below before anyone climbs
        for mid in sorted(by_id):
            if mid.startswith(f"boardslot_f{deck_floor}_"):
                add(mid, unlocks_deck=True)
        # 5) 階段塔の建地, then 踏み板 — climb onto the deck that already exists
        for mid in sorted(by_id):
            if mid.startswith("post_ramp_") and mid.endswith(f"_L{lift}"):
                add(mid)
        tread_ids = sorted(
            mid for mid in by_id if mid.startswith(f"tread_L{lift}_")
        )
        for i, mid in enumerate(tread_ids):
            add(
                mid,
                unlocks_stair_lift=lift if i + 1 == len(tread_ids) else None,
            )

    return tuple(items)


def limit_erect_sequence(
    sequence: Tuple[ErectItem, ...],
    *,
    max_items: Optional[int] = None,
    max_floors: Optional[int] = None,
    boards_per_floor: Optional[int] = None,
) -> Tuple[ErectItem, ...]:
    """Optional thinning for Pareto search / smoke tests."""
    out: list[ErectItem] = []
    boards_on_floor: dict[int, int] = {}
    for item in sequence:
        if max_floors is not None and item.floor > max_floors:
            continue
        if item.kind == "board" and boards_per_floor is not None:
            n = boards_on_floor.get(item.floor, 0)
            if n >= boards_per_floor:
                continue
            boards_on_floor[item.floor] = n + 1
        out.append(item)
        if max_items is not None and len(out) >= max_items:
            break
    return tuple(out)


def prefix_enables_keepout_sampling(sequence: Tuple[ErectItem, ...]) -> bool:
    """True if at least one item is delivered after a deck board has unlocked."""
    unlocked = False
    for item in sequence:
        if unlocked:
            return True
        if item.unlocks_deck:
            unlocked = True
    return False


def keepout_sample_item_count(sequence: Tuple[ErectItem, ...]) -> int:
    """Shortest prefix that includes one keep-out delivery after first deck unlock."""
    for index, item in enumerate(sequence):
        if item.unlocks_deck:
            if index + 1 >= len(sequence):
                raise ValueError("sequence ends on the first deck unlock")
            return index + 2
    raise ValueError("sequence has no deck unlock")


@dataclass(frozen=True)
class UeProtocolLimits:
    max_items: Optional[int]
    max_floors: Optional[int]
    boards_per_floor: Optional[int]
    require_smin: bool


def ue_protocol_limits(
    protocol: str,
    geom: Optional[ScaffoldGeom] = None,
) -> UeProtocolLimits:
    """Numeric thinning for UE replay protocols.

    smoke: 3 posts (no S_min). smin: first keep-out leg. full: all members.
    """
    if protocol == "smoke":
        return UeProtocolLimits(3, 1, 2, False)
    if protocol == "smin":
        used = geom if geom is not None else STAGE1_GEOM
        n = keepout_sample_item_count(build_erect_sequence(used))
        return UeProtocolLimits(n, None, None, True)
    if protocol == "full":
        return UeProtocolLimits(None, None, None, True)
    raise ValueError(f"unknown UE protocol {protocol!r}")


def select_erect_sequence(
    protocol: str,
    geom: Optional[ScaffoldGeom] = None,
) -> Tuple[ErectItem, ...]:
    used = geom if geom is not None else STAGE1_GEOM
    limits = ue_protocol_limits(protocol, used)
    return limit_erect_sequence(
        build_erect_sequence(used),
        max_items=limits.max_items,
        max_floors=limits.max_floors,
        boards_per_floor=limits.boards_per_floor,
    )


def bay_count() -> int:
    return N_WORKING_BAYS
