#!/usr/bin/env python3
"""Preflight TrafficSystem assets in Editor WITHOUT loading Blueprints.

NEVER call unreal.load_asset() or generated_class() on TrafficSystem BPs here.
Those APIs invoke BlueprintEditorLibrary internally and can crash UE (ACCESS_VIOLATION)
when parent refs are broken.

Runtime proof: WSL run_humanoid_spawn_test.py while PIE is playing.
"""
from __future__ import annotations

import os

import unreal

CONTENT_CHECKS = (
    ("Agent/BP_AgentBase.uasset", 100_000),
    ("TrafficSystem/Pedestrian/Base_User_Agent.uasset", 50_000),
    ("TrafficSystem/Pedestrian/Base_Pedestrian.uasset", 50_000),
    ("TrafficSystem/Pedestrian/input/IMC_Demo.uasset", 1_000),
)

ASSET_PATHS = (
    "/Game/Agent/BP_AgentBase",
    "/Game/TrafficSystem/Pedestrian/Base_Pedestrian",
    "/Game/TrafficSystem/Pedestrian/Base_User_Agent",
)

MIN_TRAFFIC_FILES = 30


def _content_dir() -> str:
    return unreal.Paths.project_content_dir()


def _check_files_on_disk() -> bool:
    content = _content_dir()
    ok = True
    for rel, min_bytes in CONTENT_CHECKS:
        path = os.path.join(content, rel.replace("/", os.sep))
        if not os.path.isfile(path):
            unreal.log_error(f"[TrafficSystem] missing file: {path}")
            ok = False
            continue
        size = os.path.getsize(path)
        if size < min_bytes:
            unreal.log_error(
                f"[TrafficSystem] too small ({size} B, need >={min_bytes}): {path}"
            )
            ok = False
        else:
            unreal.log(f"[TrafficSystem] file OK ({size} B): {rel}")
    traffic_root = os.path.join(content, "TrafficSystem")
    file_count = 0
    for _root, _dirs, files in os.walk(traffic_root):
        file_count += len(files)
    if file_count < MIN_TRAFFIC_FILES:
        unreal.log_error(
            f"[TrafficSystem] TrafficSystem has only {file_count} files "
            f"(expected >={MIN_TRAFFIC_FILES}). Re-run install_traffic_system_editor.sh "
            "with Editor closed."
        )
        ok = False
    else:
        unreal.log(f"[TrafficSystem] TrafficSystem file count OK: {file_count}")
    return ok


def _check_asset_registry() -> bool:
    ok = True
    for asset_path in ASSET_PATHS:
        exists = unreal.EditorAssetLibrary.does_asset_exist(asset_path)
        unreal.log(
            f"[TrafficSystem] registry {'OK' if exists else 'FAIL'}: {asset_path}"
        )
        if not exists:
            ok = False
    return ok


def main() -> None:
    unreal.log(
        "[TrafficSystem] preflight (no Blueprint load — safe for Editor Python)"
    )
    ok = _check_files_on_disk() and _check_asset_registry()
    if ok:
        unreal.log(
            "[TrafficSystem] preflight OK — PIE Play on grid_100x100, then WSL:\n"
            "  python dev/grid_env_10k/run_humanoid_spawn_test.py"
        )
    else:
        unreal.log_error(
            "[TrafficSystem] preflight FAILED — close Editor, run:\n"
            "  bash dev/grid_env_10k/scripts/install_traffic_system_editor.sh\n"
            "Do NOT open Base_User_Agent in Blueprint Editor until preflight passes."
        )


if __name__ == "__main__":
    main()
