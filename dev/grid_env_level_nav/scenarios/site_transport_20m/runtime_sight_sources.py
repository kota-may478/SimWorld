#!/usr/bin/env python3
"""Runtime AI Sight source registration for PIE-spawned site20 actors."""

from __future__ import annotations

import os
from pathlib import Path

from simworld.communicator.unrealcv import UnrealCV

_UE_REGISTER_SITE20_SIGHT_SOURCES = r"""
import unreal

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is not None:
    sight = unreal.AISense_Sight.static_class()
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    for actor in actors:
        name = actor.get_name()
        if not name.startswith("site20_"):
            continue
        comp = actor.get_component_by_class(unreal.AIPerceptionStimuliSourceComponent)
        if comp is not None:
            comp.set_editor_property("auto_register_as_source", True)
            comp.set_editor_property("register_as_source_for_senses", [sight])
            if hasattr(comp, "register_for_sense"):
                comp.register_for_sense(sight)
            if hasattr(comp, "register_component"):
                comp.register_component()
        unreal.AIPerceptionSystem.register_perception_stimuli_source(world, sight, actor)

    for actor in actors:
        if "SpotDogAIController" not in actor.get_name():
            continue
        pcomp = actor.get_component_by_class(unreal.AIPerceptionComponent)
        if pcomp is not None and hasattr(pcomp, "request_stimuli_listener_update"):
            pcomp.request_stimuli_listener_update()
"""


def _windows_path_to_wsl(path: str) -> Path:
    normalized = path.replace("\\", "/")
    if len(normalized) >= 3 and normalized[1:3] == ":/":
        drive = normalized[0].lower()
        return Path("/mnt") / drive / normalized[3:]
    return Path(normalized)


def ensure_runtime_site20_sight_sources(ucv: UnrealCV) -> bool:
    """Register spawned site20 actors with UE's AI Perception system.

    The generated prop BPs can contain an ``AIPerceptionStimuliSourceComponent`` whose
    saved defaults are incomplete. Registering the live PIE actors is non-destructive
    and makes ``GetCurrentlyPerceivedActors`` observe props immediately.
    """
    windows_path = os.environ.get(
        "SIMWORLD_UE_RUNTIME_SIGHT_SCRIPT",
        "C:/Temp/simworld_runtime_site20_sight_sources.py",
    )
    local_path = _windows_path_to_wsl(windows_path)
    try:
        local_path.parent.mkdir(parents=True, exist_ok=True)
        local_path.write_text(_UE_REGISTER_SITE20_SIGHT_SOURCES, encoding="utf-8")
    except OSError as exc:
        print(f"[Site20Sight] failed to write UE Python script: {exc}")
        return False

    command = f"vrun py exec(open(r'{windows_path}', encoding='utf-8').read())"
    try:
        raw = ucv.client.request(command, timeout=60)
    except (ConnectionError, OSError, RuntimeError, ValueError) as exc:
        print(f"[Site20Sight] runtime source registration failed: {exc}")
        return False
    ok = str(raw).strip().lower() == "ok"
    print(f"[Site20Sight] runtime source registration {'OK' if ok else raw}")
    return ok
