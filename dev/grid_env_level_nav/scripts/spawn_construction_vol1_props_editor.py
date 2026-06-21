#!/usr/bin/env python3
"""UE Editor: spawn Construction VOL.1 props on NavMesh (Editor mode, not PIE).

For PIE runtime spawn use spawn_construction_vol1_props_pie.py from WSL instead.

Run: Tools -> Execute Python Script -> this file
"""

from __future__ import annotations

import math
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
_nav_dir = os.path.dirname(_script_dir)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
if _nav_dir not in sys.path:
    sys.path.insert(0, _nav_dir)

import unreal

from level_prop_blueprint_utils import OUT_DIR
from prop_catalog import discover_entries_from_content_dir

PROP_LABEL_PREFIX = "prop_vol1"
MARGIN_LOCAL_CM = 900.0
SPACING_CM = 580.0
FOOT_Z_OFFSET_CM = 5.0
REGION_SIZE_X_CM = 7000.0
REGION_SIZE_Y_CM = 7900.0
REGION_ORIGIN_XY = (-1000.0, -2200.0)
FLOOR_REF_Z_CM = 6440.0


def _local_to_world(lx: float, ly: float) -> tuple[float, float]:
    return REGION_ORIGIN_XY[0] + lx, REGION_ORIGIN_XY[1] + ly


def _grid_centers(count: int) -> list[tuple[float, float]]:
    margin = MARGIN_LOCAL_CM
    usable_x = REGION_SIZE_X_CM - 2.0 * margin
    cols = max(1, int(math.floor(usable_x / SPACING_CM)))
    rows = max(1, int(math.ceil(count / cols)))
    points: list[tuple[float, float]] = []
    for row in range(rows):
        for col in range(cols):
            if len(points) >= count:
                return points
            lx = margin + (col + 0.5) * SPACING_CM
            ly = margin + (row + 0.5) * SPACING_CM
            points.append((lx, ly))
    return points


def _resolve_actor_class(bp_path: str) -> unreal.Class:
    asset = unreal.load_asset(bp_path)
    if asset is None:
        raise RuntimeError(f"missing blueprint: {bp_path}")
    if isinstance(asset, unreal.Blueprint):
        actor_class = asset.generated_class()
    elif isinstance(asset, unreal.BlueprintGeneratedClass):
        actor_class = asset
    else:
        actor_class = asset
    if actor_class is None:
        raise RuntimeError(f"no generated class: {bp_path}")
    return actor_class


def _project_to_nav(world: unreal.World, wx: float, wy: float) -> unreal.Vector | None:
    nav_sys = unreal.NavigationSystemV1.get_navigation_system(world)
    if nav_sys is None:
        return None
    probe = unreal.Vector(wx, wy, FLOOR_REF_Z_CM + 10.0)
    extent = unreal.Vector(80.0, 80.0, 120.0)
    hit = nav_sys.project_point_to_navigation(probe, None, extent)
    if hit is None:
        return None
    return unreal.Vector(hit.x, hit.y, hit.z + FOOT_Z_OFFSET_CM)


def _destroy_existing(subsys: unreal.EditorActorSubsystem) -> int:
    removed = 0
    for actor in subsys.get_all_level_actors():
        label = actor.get_actor_label()
        if label.startswith(PROP_LABEL_PREFIX):
            subsys.destroy_actor(actor)
            removed += 1
    return removed


def main() -> None:
    entries = discover_entries_from_content_dir()
    if not entries:
        unreal.log_error("[PropSpawnEditor] no generated BPs found")
        return

    world = unreal.EditorLevelLibrary.get_editor_world()
    subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    removed = _destroy_existing(subsys)
    unreal.log(f"[PropSpawnEditor] removed {removed} existing props")

    centers = _grid_centers(len(entries))
    spawned = 0
    failed = 0
    cand_idx = 0
    for idx, entry in enumerate(entries):
        label = f"{PROP_LABEL_PREFIX}_{idx:03d}"
        location = None
        while cand_idx < len(centers):
            lx, ly = centers[cand_idx]
            cand_idx += 1
            wx, wy = _local_to_world(lx, ly)
            location = _project_to_nav(world, wx, wy)
            if location is not None:
                break
        if location is None:
            unreal.log_error(f"[PropSpawnEditor] no nav point for {entry.bp_name}")
            failed += 1
            continue

        actor_class = _resolve_actor_class(entry.bp_path)
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            actor_class,
            location,
            unreal.Rotator(0.0, 0.0, 0.0),
        )
        if actor is None:
            unreal.log_error(f"[PropSpawnEditor] spawn failed {entry.bp_name}")
            failed += 1
            continue
        subsys.set_actor_label(actor, label)
        spawned += 1
        unreal.log(
            f"[PropSpawnEditor] OK {label} <- {entry.bp_name} "
            f"@ ({location.x:.0f}, {location.y:.0f}, {location.z:.0f})"
        )

    unreal.log(f"[PropSpawnEditor] done spawned={spawned} failed={failed}")


if __name__ == "__main__":
    main()
