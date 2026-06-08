#!/usr/bin/env python3
"""Load (and optionally pin) all grid_floor_main + block_* in the open level.

World Partition keeps most actors Unloaded until a region is loaded.
Run in Editor with PIE stopped after opening grid_100x100.

Env:
  GRID_PIN_ACTORS=1   (default) pin after load so WP is less likely to unload them
  GRID_PIN_ACTORS=0   load only, no pin
"""

from __future__ import annotations

import os
import time

import unreal

FLOOR_NAME = "grid_floor_main"
BLOCK_PREFIX = "block_"

GRID_BOX_MIN_CM = (-500.0, -500.0, 0.0)
GRID_BOX_MAX_CM = (3500.0, 3500.0, 500.0)
LOAD_SETTLE_S = 3.0

PIN_AFTER_LOAD = os.environ.get("GRID_PIN_ACTORS", "1").strip().lower() not in (
    "0",
    "false",
    "no",
)


def _is_target_label(label: str) -> bool:
    return label == FLOOR_NAME or label.startswith(BLOCK_PREFIX)


def main() -> None:
    wp = unreal.WorldPartitionBlueprintLibrary
    box = unreal.Box(
        unreal.Vector(*GRID_BOX_MIN_CM),
        unreal.Vector(*GRID_BOX_MAX_CM),
    )

    descs = wp.get_intersecting_actor_descs(box)
    if descs is None:
        descs = wp.get_actor_descs()
    if descs is None:
        unreal.log_error("[load_grid] no ActorDesc — is grid_100x100 a World Partition map?")
        return

    guids: list[unreal.Guid] = []
    labels = 0
    for desc in descs:
        label = str(desc.label)
        if _is_target_label(label):
            guids.append(desc.guid)
            labels += 1

    unreal.log(f"[load_grid] loading {len(guids)} actor(s) ({labels} descriptors) ...")
    wp.load_actors(guids)
    time.sleep(LOAD_SETTLE_S)

    if PIN_AFTER_LOAD:
        unreal.log(f"[load_grid] pinning {len(guids)} actor(s) ...")
        wp.pin_actors(guids)

    subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    loaded = 0
    for actor in subsys.get_all_level_actors():
        label = actor.get_actor_label()
        if _is_target_label(label):
            loaded += 1

    unreal.log(
        f"[load_grid] done: EditorActorSubsystem sees {loaded} loaded "
        f"(pin={PIN_AFTER_LOAD})"
    )
    unreal.log(
        "[load_grid] tip: Outliner footer should show more '(loaded)' — "
        "if still many Unloaded, use World Partition window -> Load Region on full grid"
    )


if __name__ == "__main__":
    main()
