#!/usr/bin/env python3
"""Optional: compile TrafficSystem BPs only (never BP_AgentBase).

WARNING: compile_blueprint can crash UE on broken parent refs.
Prefer verify_traffic_system_editor.py first.
"""
from __future__ import annotations

import os

import unreal

# Do NOT compile BP_AgentBase here — SpotDog chain owns that; recompile can crash Editor.
BP_PATHS = [
    "/Game/TrafficSystem/Pedestrian/Base_Pedestrian",
    "/Game/TrafficSystem/Pedestrian/Base_User_Agent",
]

MIN_AGENT_BYTES = 100_000


def _agent_has_generated_class() -> bool:
    content = unreal.Paths.project_content_dir()
    agent_uasset = os.path.join(content, "Agent", "BP_AgentBase.uasset")
    if not os.path.isfile(agent_uasset):
        unreal.log_error("[TrafficSystem] missing BP_AgentBase.uasset")
        return False
    if os.path.getsize(agent_uasset) < MIN_AGENT_BYTES:
        unreal.log_error("[TrafficSystem] BP_AgentBase looks like cooked stub")
        return False
    asset = unreal.load_asset("/Game/Agent/BP_AgentBase")
    if not isinstance(asset, unreal.Blueprint):
        return False
    ok = asset.generated_class() is not None
    if not ok:
        unreal.log_error(
            "[TrafficSystem] BP_AgentBase has no generated class — "
            "run compile_robot_dog_editor.py first (not this script)"
        )
    return ok


def _compile_blueprint(asset: unreal.Blueprint) -> bool:
    if asset is None:
        return False
    gen_before = asset.generated_class()
    if gen_before is not None:
        unreal.log(
            f"[TrafficSystem] skip compile (already has {gen_before.get_name()})"
        )
        return True
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(asset)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(asset)
        else:
            unreal.log_error("[TrafficSystem] no compile API")
            return False
    except Exception as exc:  # noqa: BLE001
        unreal.log_error(f"[TrafficSystem] compile exception: {exc}")
        return False
    return asset.generated_class() is not None


def _compile_one(path: str) -> bool:
    asset = unreal.load_asset(path)
    if asset is None:
        unreal.log_error(f"[TrafficSystem] load failed: {path}")
        return False
    if not isinstance(asset, unreal.Blueprint):
        return True
    ok = _compile_blueprint(asset)
    unreal.EditorAssetLibrary.save_asset(path, only_if_is_dirty=False)
    gen = asset.generated_class()
    unreal.log(
        f"[TrafficSystem] {'OK' if ok else 'FAIL'} {path} -> "
        f"{gen.get_name() if gen else 'NO_CLASS'}"
    )
    return ok


def main() -> None:
    unreal.log("[TrafficSystem] compile (TrafficSystem only — Agent untouched)")
    if not _agent_has_generated_class():
        return
    ok_all = True
    for path in BP_PATHS:
        if not _compile_one(path):
            ok_all = False
            break
    if ok_all:
        unreal.log("[TrafficSystem] compile done")
    else:
        unreal.log_error(
            "[TrafficSystem] compile stopped — use verify_traffic_system_editor.py "
            "or compile Base_User_Agent manually in Blueprint Editor"
        )


if __name__ == "__main__":
    main()
