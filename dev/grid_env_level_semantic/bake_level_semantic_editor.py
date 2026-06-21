#!/usr/bin/env python3
"""Bake level_sem_block_* from registry into the open UE level (Editor-only).

Run from UE Editor after PIE produced `.level_semantic_registry.json`:
  Tools -> Execute Python Script -> this file

Then File -> Save Current Level As -> /Game/Maps/Level_semantic
(or uncomment SAVE_AS below if your UE build supports it).
"""

from __future__ import annotations

import json
from pathlib import Path

import unreal

CUBE_BP = "/Game/CustomAssets/BP_TransparentCube.BP_TransparentCube_C"
BLOCK_PREFIX = "level_sem_block"
REGISTRY_PATH = Path(__file__).resolve().parent / ".level_semantic_registry.json"
CUBE_HALF_CM = 15.0
CUBE_PIVOT_AT_CENTER = False


def _editor_actors() -> unreal.EditorActorSubsystem:
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _resolve_actor_class(path: str) -> unreal.Class:
    asset = unreal.load_asset(path)
    if asset is None:
        raise RuntimeError(f"Blueprint not found: {path}")
    if isinstance(asset, unreal.Blueprint):
        actor_class = asset.generated_class()
    elif isinstance(asset, unreal.BlueprintGeneratedClass):
        actor_class = asset
    else:
        actor_class = asset
    if actor_class is None:
        raise RuntimeError(f"No actor class for {path}")
    return actor_class


def _set_actor_label(actor: unreal.Actor, label: str) -> None:
    subsys = _editor_actors()
    if hasattr(subsys, "set_actor_label"):
        subsys.set_actor_label(actor, label)
    elif hasattr(actor, "set_actor_label"):
        actor.set_actor_label(label)


def _destroy_actor_by_label(label: str) -> None:
    subsys = _editor_actors()
    for actor in subsys.get_all_level_actors():
        if actor.get_actor_label() == label:
            subsys.destroy_actor(actor)


def _block_bottom_to_actor_z(bottom_z_cm: float) -> float:
    if CUBE_PIVOT_AT_CENTER:
        return bottom_z_cm + CUBE_HALF_CM
    return bottom_z_cm


def _load_registry() -> dict:
    if not REGISTRY_PATH.is_file():
        raise FileNotFoundError(f"Registry not found: {REGISTRY_PATH}")
    return json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))


def main() -> None:
    data = _load_registry()
    blocks = data.get("blocks") or {}
    if not blocks:
        raise RuntimeError("Registry has no blocks — run PIE placement first")

    cube_class = _resolve_actor_class(CUBE_BP)
    spawned = 0
    failed = 0

    for name, rec in sorted(blocks.items()):
        _destroy_actor_by_label(name)
        world = rec.get("world_cm") or [0.0, 0.0, 0.0]
        bottom_z = float(rec.get("block_bottom_z_cm", data.get("block_bottom_z_cm", 0.0)))
        actor_z = _block_bottom_to_actor_z(bottom_z)
        loc = unreal.Vector(float(world[0]), float(world[1]), actor_z)
        try:
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                cube_class,
                loc,
                unreal.Rotator(0.0, 0.0, 0.0),
            )
            if actor is None:
                raise RuntimeError("spawn returned None")
            _set_actor_label(actor, name)
            spawned += 1
        except Exception as exc:
            failed += 1
            if failed <= 5:
                unreal.log_error(f"[bake_level_sem] {name}: {exc}")

    unreal.log(
        f"[bake_level_sem] DONE spawned={spawned} failed={failed} "
        f"from {REGISTRY_PATH.name}"
    )
    unreal.log(
        "[bake_level_sem] Next: File -> Save Current Level As -> "
        "/Game/Maps/Level_semantic"
    )


if __name__ == "__main__":
    main()
