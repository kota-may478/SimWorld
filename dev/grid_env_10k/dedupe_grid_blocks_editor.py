#!/usr/bin/env python3
"""Remove duplicate block_* / grid_floor_main actors in the open level (Editor-only).

World Partition: unloaded actors are invisible to get_all_level_actors().
This script finds duplicates via ActorDesc, loads them, then destroys extras.

Run: Tools -> Execute Python Script (PIE must be stopped).
"""

from __future__ import annotations

import time

import unreal

FLOOR_NAME = "grid_floor_main"
BLOCK_PREFIX = "block_"

# grid_env_10k bake layout (cm); margin for WP cell bounds
GRID_BOX_MIN_CM = (-500.0, -500.0, 0.0)
GRID_BOX_MAX_CM = (3500.0, 3500.0, 500.0)
LOAD_SETTLE_S = 2.0


def _editor_actors() -> unreal.EditorActorSubsystem:
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _grid_bounds_box() -> unreal.Box:
    return unreal.Box(
        unreal.Vector(*GRID_BOX_MIN_CM),
        unreal.Vector(*GRID_BOX_MAX_CM),
    )


def _is_target_label(label: str) -> bool:
    return label == FLOOR_NAME or label.startswith(BLOCK_PREFIX)


def _collect_descs_by_label() -> dict[str, list[unreal.ActorDesc]]:
    wp = unreal.WorldPartitionBlueprintLibrary
    box = _grid_bounds_box()
    descs = wp.get_intersecting_actor_descs(box)
    if descs is None:
        descs = wp.get_actor_descs()
    if descs is None:
        unreal.log_error("[dedupe_grid] WorldPartitionBlueprintLibrary returned no ActorDesc")
        return {}

    by_label: dict[str, list[unreal.ActorDesc]] = {}
    for desc in descs:
        label = str(desc.label)
        if _is_target_label(label):
            by_label.setdefault(label, []).append(desc)

    unreal.log(
        f"[dedupe_grid] ActorDesc in grid box: "
        f"{sum(len(v) for v in by_label.values())} actors, "
        f"{len(by_label)} unique labels"
    )
    return by_label


def _load_guids(guids: list[unreal.Guid]) -> None:
    if not guids:
        return
    unreal.log(f"[dedupe_grid] loading {len(guids)} actor(s) from World Partition ...")
    unreal.WorldPartitionBlueprintLibrary.load_actors(guids)
    time.sleep(LOAD_SETTLE_S)


def _dedupe_loaded_actors() -> tuple[int, int, dict[str, list[unreal.Actor]]]:
    subsys = _editor_actors()
    by_label: dict[str, list[unreal.Actor]] = {}

    for actor in subsys.get_all_level_actors():
        label = actor.get_actor_label()
        if _is_target_label(label):
            by_label.setdefault(label, []).append(actor)

    removed = 0
    dup_label_count = 0
    for label in sorted(by_label):
        actors = by_label[label]
        if len(actors) <= 1:
            continue
        dup_label_count += 1
        for extra in actors[1:]:
            subsys.destroy_actor(extra)
            removed += 1
        by_label[label] = actors[:1]

    return removed, dup_label_count, by_label


def main() -> None:
    desc_by_label = _collect_descs_by_label()
    if not desc_by_label:
        unreal.log_warning(
            "[dedupe_grid] no block/floor ActorDesc found — "
            "try Window -> World Partition -> load grid region manually, then re-run"
        )
        return

    dup_desc_labels = [lbl for lbl, items in desc_by_label.items() if len(items) > 1]
    unreal.log(
        f"[dedupe_grid] descriptor duplicates: {len(dup_desc_labels)} label(s), "
        f"{sum(len(desc_by_label[l]) - 1 for l in dup_desc_labels)} extra instance(s)"
    )
    if dup_desc_labels[:10]:
        unreal.log(f"[dedupe_grid] sample dup labels: {dup_desc_labels[:10]}")

    if not dup_desc_labels:
        unreal.log("[dedupe_grid] no duplicates in ActorDesc — nothing to do")
        return

    guids_to_load: list[unreal.Guid] = []
    for label in dup_desc_labels:
        for desc in desc_by_label[label]:
            guids_to_load.append(desc.guid)
    _load_guids(guids_to_load)

    removed, dup_label_count, by_label = _dedupe_loaded_actors()
    floor_n = len(by_label.get(FLOOR_NAME, []))
    block_labels = [k for k in by_label if k.startswith(BLOCK_PREFIX)]

    unreal.log(
        f"[dedupe_grid] removed={removed} loaded duplicate actor(s) "
        f"({dup_label_count} labels)"
    )
    unreal.log(
        f"[dedupe_grid] remaining loaded: floor={floor_n}, "
        f"unique block labels={len(block_labels)}"
    )

    if removed > 0:
        unreal.EditorLevelLibrary.save_current_level()
        unreal.log("[dedupe_grid] saved current level")
    else:
        unreal.log_warning(
            "[dedupe_grid] duplicates seen in ActorDesc but none removed after load — "
            "load grid region in World Partition window, then re-run"
        )


if __name__ == "__main__":
    main()
