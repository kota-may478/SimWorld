#!/usr/bin/env python3
"""Bake grid_100x100 into the open UE level (Editor-only, no UnrealCV).

Run from UE Editor:
  Tools -> Execute Python Script -> select this file

Requires:
  - Level open (e.g. /Game/Maps/grid_100x100)
  - BP_Floor_30x30, BP_TransparentCube in Content/CustomAssets (pakchunk9002)
  - Python Editor Script Plugin enabled
  - BP_TransparentCube class default = No Collision (SetBlocking False / state F)

Dry run (5×5 = 25 blocks, same as run_phase1_spawn grid_n=5):
  Default DRY_RUN_GRID_N=5 below. Full 100×100: set OS env BLOCK_SPAWN_DRY_RUN_N=0
  before launching Editor, or change the default to 0.

Lighting (dark level): spawns grid_bake_sun + grid_bake_skylight above the floor.
Tune with BAKE_SUN_INTENSITY / BAKE_SKY_INTENSITY (see below).
"""

from __future__ import annotations

import os

import unreal

# Match grid_env_hri / grid_env_10k (UE cm, map origin bottom-left)
FLOOR_BP = "/Game/CustomAssets/BP_Floor_30x30.BP_Floor_30x30_C"
CUBE_BP = "/Game/CustomAssets/BP_TransparentCube.BP_TransparentCube_C"
FLOOR_ACTOR_NAME = "grid_floor_main"
SUN_ACTOR_NAME = "grid_bake_sun"
SKY_ACTOR_NAME = "grid_bake_skylight"
BLOCK_PREFIX = "block"
GRID_N = int(os.environ.get("BLOCK_GRID_N", "100"))
# BLOCK_SPAWN_DRY_RUN_N = grid side length (5 → 5×5 blocks). 0 = full GRID_N.
DRY_RUN_GRID_N = int(os.environ.get("BLOCK_SPAWN_DRY_RUN_N", "5"))
grid_n = DRY_RUN_GRID_N if DRY_RUN_GRID_N > 0 else GRID_N

FLOOR_SIZE_M = 30.0
CUBE_SIZE_M = 0.3
MAP_ORIGIN_XY_CM = (0.0, 0.0)
FLOOR_TOP_Z_CM = 100.0
FLOOR_ACTOR_Z_CM = FLOOR_TOP_Z_CM
CUBE_ON_FLOOR_EPS_CM = 0.5
CUBE_HALF_CM = CUBE_SIZE_M * 50.0  # 15 cm

# UE5 directional light: lux (raise if still dark, e.g. 50000)
BAKE_SUN_INTENSITY = float(os.environ.get("BAKE_SUN_INTENSITY", "25000"))
BAKE_SKY_INTENSITY = float(os.environ.get("BAKE_SKY_INTENSITY", "2.0"))
BAKE_ADD_LIGHTING = os.environ.get("BAKE_ADD_LIGHTING", "1").strip().lower() not in (
    "0",
    "false",
    "no",
)


def _editor_actors() -> unreal.EditorActorSubsystem:
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _resolve_actor_class(path: str) -> unreal.Class:
    """load_asset on *_C paths returns BlueprintGeneratedClass, not Blueprint."""
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
        return
    if hasattr(actor, "set_actor_label"):
        actor.set_actor_label(label)
        return
    unreal.log_warning(f"[bake_grid] could not set actor label {label!r}")


def _destroy_actor_by_label(label: str) -> None:
    subsys = _editor_actors()
    for actor in subsys.get_all_level_actors():
        if actor.get_actor_label() == label:
            subsys.destroy_actor(actor)
            unreal.log(f"[bake_grid] removed existing {label!r}")


def _spawn_bp(actor_class: unreal.Class, name: str, location: unreal.Vector) -> unreal.Actor:
    _destroy_actor_by_label(name)
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        actor_class,
        location,
        unreal.Rotator(0.0, 0.0, 0.0),
    )
    if actor is None:
        raise RuntimeError(f"spawn failed for {name!r}")
    _set_actor_label(actor, name)
    loc = actor.get_actor_location()
    unreal.log(f"[bake_grid] spawned {name!r} @ ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
    return actor


def _configure_directional_light(actor: unreal.Actor, intensity: float) -> None:
    comp = actor.get_component_by_class(unreal.DirectionalLightComponent)
    if comp is None:
        return
    comp.set_mobility(unreal.ComponentMobility.MOBILE)
    if hasattr(comp, "set_intensity"):
        comp.set_intensity(intensity)
    else:
        comp.set_editor_property("intensity", intensity)
    comp.set_editor_property("cast_shadows", True)


def _configure_skylight(actor: unreal.Actor, intensity: float) -> None:
    comp = actor.get_component_by_class(unreal.SkyLightComponent)
    if comp is None:
        return
    comp.set_mobility(unreal.ComponentMobility.MOBILE)
    if hasattr(comp, "set_intensity"):
        comp.set_intensity(intensity)
    else:
        comp.set_editor_property("intensity", intensity)
    if hasattr(comp, "recapture_sky"):
        comp.recapture_sky()


def _ensure_bake_lighting(look_at: unreal.Vector) -> None:
    """Add sun + skylight above the grid (safe to re-run)."""
    subsys = _editor_actors()
    _destroy_actor_by_label(SUN_ACTOR_NAME)
    _destroy_actor_by_label(SKY_ACTOR_NAME)

    sun_height_cm = max(FLOOR_SIZE_M * 100.0 * grid_n, 3000.0)
    sun_loc = unreal.Vector(look_at.x, look_at.y, look_at.z + sun_height_cm)
    sun_rot = unreal.Rotator(-55.0, 45.0, 0.0)

    sun = subsys.spawn_actor_from_class(
        unreal.DirectionalLight.static_class(),
        sun_loc,
        sun_rot,
    )
    if sun is None:
        unreal.log_warning("[bake_grid] failed to spawn directional light")
        return
    _set_actor_label(sun, SUN_ACTOR_NAME)
    _configure_directional_light(sun, BAKE_SUN_INTENSITY)

    sky_loc = unreal.Vector(look_at.x, look_at.y, look_at.z + 500.0)
    sky = subsys.spawn_actor_from_class(
        unreal.SkyLight.static_class(),
        sky_loc,
        unreal.Rotator(0.0, 0.0, 0.0),
    )
    if sky is not None:
        _set_actor_label(sky, SKY_ACTOR_NAME)
        _configure_skylight(sky, BAKE_SKY_INTENSITY)

    unreal.log(
        f"[bake_grid] lighting: {SUN_ACTOR_NAME} intensity={BAKE_SUN_INTENSITY:g}, "
        f"{SKY_ACTOR_NAME} intensity={BAKE_SKY_INTENSITY:g}"
    )


def _cube_xy_cm(gx: int, gy: int) -> tuple[float, float]:
    """1-indexed (gx, gy) -> world XY cm (grid_env_10k)."""
    col = gx - 1
    row = gy - 1
    x = MAP_ORIGIN_XY_CM[0] + (col + 0.5) * CUBE_SIZE_M * 100.0
    y = MAP_ORIGIN_XY_CM[1] + (row + 0.5) * CUBE_SIZE_M * 100.0
    return x, y


def main() -> None:
    unreal.log(
        f"[bake_grid] grid_n={grid_n} "
        f"(dry={DRY_RUN_GRID_N > 0}, blocks={grid_n * grid_n})"
    )

    floor_class = _resolve_actor_class(FLOOR_BP)
    cube_class = _resolve_actor_class(CUBE_BP)

    floor_loc = unreal.Vector(
        MAP_ORIGIN_XY_CM[0] + FLOOR_SIZE_M * 50.0,
        MAP_ORIGIN_XY_CM[1] + FLOOR_SIZE_M * 50.0,
        FLOOR_ACTOR_Z_CM,
    )

    if BAKE_ADD_LIGHTING:
        _ensure_bake_lighting(floor_loc)

    _spawn_bp(floor_class, FLOOR_ACTOR_NAME, floor_loc)
    unreal.log(f"[bake_grid] floor spawned: {FLOOR_ACTOR_NAME}")

    spawned = 0
    failed = 0
    for gy in range(1, grid_n + 1):
        for gx in range(1, grid_n + 1):
            name = f"{BLOCK_PREFIX}_{gx:03d}_{gy:03d}"
            x, y = _cube_xy_cm(gx, gy)
            z = FLOOR_TOP_Z_CM + CUBE_ON_FLOOR_EPS_CM + CUBE_HALF_CM
            try:
                _spawn_bp(
                    cube_class,
                    name,
                    unreal.Vector(x, y, z),
                )
                spawned += 1
            except Exception as exc:
                failed += 1
                if failed <= 5:
                    unreal.log_error(f"[bake_grid] {name}: {exc}")

        if gy % 10 == 0 or gy == grid_n:
            unreal.log(f"[bake_grid] progress row {gy}/{grid_n} spawned={spawned} failed={failed}")

    unreal.log(f"[bake_grid] DONE spawned={spawned} failed={failed} expected={grid_n * grid_n}")
    unreal.EditorLevelLibrary.save_current_level()


if __name__ == "__main__":
    main()
