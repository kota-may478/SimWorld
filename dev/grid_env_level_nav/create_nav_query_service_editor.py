#!/usr/bin/env python3
"""UE Editor: create / compile BP_NavQueryService.

Run in UE: Tools → Execute Python Script → this file.

Requires ``ANavQueryService`` in SimWorld Source (see ``ue_native/INSTALL_NATIVE.md``).
"""
from __future__ import annotations

import unreal

ASSET_DIR = "/Game/CustomAssets"
ASSET_NAME = "BP_NavQueryService"
ASSET_PATH = f"{ASSET_DIR}/{ASSET_NAME}"
NATIVE_CLASS_PATH = "/Script/SimWorld.NavQueryService"

BLUEPRINT_MANUAL_STEPS = """
=== BP_NavQueryService (native parent missing) ===
1. Copy ue_native/NavQueryService.* into SimWorld Source and Rebuild (INSTALL_NATIVE.md).
2. Re-run this script, or create Blueprint parent = NavQueryService manually.
3. Compile, Save. Place actor in Level with label NavQueryService.
4. PIE test (WSL):
   python dev/grid_env_level_nav/_nav_project_point_smoke_test.py
"""


def _resolve_parent_class() -> type:
    native = unreal.load_class(None, NATIVE_CLASS_PATH)
    if native is not None:
        unreal.log(f"[NavQuery] native parent: {NATIVE_CLASS_PATH}")
        return native
    unreal.log_warning(
        "[NavQuery] native class not found — copy C++ per ue_native/INSTALL_NATIVE.md"
    )
    return unreal.Actor


def _compile_blueprint(asset: unreal.Blueprint) -> bool:
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(asset)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(asset)
        else:
            unreal.log_error("[NavQuery] no compile API")
            return False
    except Exception as exc:  # noqa: BLE001
        unreal.log_error(f"[NavQuery] compile failed: {exc}")
        return False
    return asset.generated_class() is not None


def main() -> None:
    if not unreal.EditorAssetLibrary.does_directory_exist(ASSET_DIR):
        unreal.EditorAssetLibrary.make_directory(ASSET_DIR)

    parent = _resolve_parent_class()
    if parent is unreal.Actor:
        unreal.log_error(BLUEPRINT_MANUAL_STEPS)
        return

    asset = unreal.EditorAssetLibrary.load_asset(ASSET_PATH)
    if asset is None:
        tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.BlueprintFactory()
        factory.set_editor_property("parent_class", parent)
        asset = tools.create_asset(ASSET_NAME, ASSET_DIR, unreal.Blueprint, factory)
        if asset is None:
            unreal.log_error("[NavQuery] create_asset failed")
            return
        unreal.log(f"[NavQuery] created {ASSET_PATH}")
    else:
        unreal.log(f"[NavQuery] exists: {ASSET_PATH}")

    if isinstance(asset, unreal.Blueprint):
        ok = _compile_blueprint(asset)
        unreal.EditorAssetLibrary.save_asset(ASSET_PATH, only_if_is_dirty=False)
        unreal.log(f"[NavQuery] compile={'OK' if ok else 'FAIL'} saved {ASSET_PATH}")

    unreal.log(
        "[NavQuery] next: drag BP into Level, Actor Label = NavQueryService, Save Level"
    )


if __name__ == "__main__":
    main()
