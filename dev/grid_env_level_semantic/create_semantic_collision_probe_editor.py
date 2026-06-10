#!/usr/bin/env python3
"""UE Editor: create / compile BP_SemanticCollisionProbe for Approach C labeling.

Run in UE: Tools → Execute Python Script → this file.

If SimWorld already ships ``ASemanticCollisionProbe`` (see ``ue_native/``), the
Blueprint parent is set automatically. Otherwise create the Blueprint function
``ProbePointHit`` manually as logged below, then compile and save.
"""
from __future__ import annotations

import unreal

ASSET_DIR = "/Game/CustomAssets"
ASSET_NAME = "BP_SemanticCollisionProbe"
ASSET_PATH = f"{ASSET_DIR}/{ASSET_NAME}"
NATIVE_CLASS_PATH = "/Script/SimWorld.SemanticCollisionProbe"

BLUEPRINT_MANUAL_STEPS = """
=== BP_SemanticCollisionProbe (Blueprint-only parent = Actor) ===
1. Open BP_SemanticCollisionProbe.
2. Add function ProbePointHit(X:float, Y:float, Z:float) -> Return: String.
3. In the graph:
   a. Make Vector(X, Y, Z).
   b. Sphere Overlap Actors
      - World Context Object: self
      - Sphere Pos: vector from (a)
      - Sphere Radius: 12.0
      - Object Types: WorldStatic + WorldDynamic
      - Actor Class Filter: Actor
      - Actors to Ignore: self
   c. Length on Out Actors array -> hit_count.
   d. Branch: hit_count > 0
      - True: return {"hit":true,"building":hit_count,"object":0}
      - False: return {"hit":false,"building":0,"object":0}
   (Use Make Literal String or Format Text; UnrealCV expects JSON text.)
4. Compile, Save. PIE test:
   vbp level_sem_collision_probe ProbePointHit 6285 1185 6873.5
"""


def _resolve_parent_class() -> type:
    native = unreal.load_class(None, NATIVE_CLASS_PATH)
    if native is not None:
        unreal.log(f"[SemanticProbe] native parent: {NATIVE_CLASS_PATH}")
        return native
    unreal.log_warning(
        "[SemanticProbe] native class not found — parent=Actor; "
        "add ProbePointHit in Blueprint (see log)."
    )
    return unreal.Actor


def _compile_blueprint(asset: unreal.Blueprint) -> bool:
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(asset)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(asset)
        else:
            unreal.log_error("[SemanticProbe] no compile API")
            return False
    except Exception as exc:  # noqa: BLE001
        unreal.log_error(f"[SemanticProbe] compile failed: {exc}")
        return False
    return asset.generated_class() is not None


def main() -> None:
    if not unreal.EditorAssetLibrary.does_directory_exist(ASSET_DIR):
        unreal.EditorAssetLibrary.make_directory(ASSET_DIR)

    parent = _resolve_parent_class()
    asset = unreal.EditorAssetLibrary.load_asset(ASSET_PATH)

    if asset is None:
        tools = unreal.AssetToolsHelpers.get_asset_tools()
        factory = unreal.BlueprintFactory()
        factory.set_editor_property("parent_class", parent)
        asset = tools.create_asset(ASSET_NAME, ASSET_DIR, unreal.Blueprint, factory)
        if asset is None:
            unreal.log_error("[SemanticProbe] create_asset failed")
            return
        unreal.log(f"[SemanticProbe] created {ASSET_PATH}")
    else:
        unreal.log(f"[SemanticProbe] exists: {ASSET_PATH}")

    if isinstance(asset, unreal.Blueprint):
        ok = _compile_blueprint(asset)
        unreal.EditorAssetLibrary.save_asset(ASSET_PATH, only_if_is_dirty=False)
        unreal.log(f"[SemanticProbe] compile={'OK' if ok else 'FAIL'} saved {ASSET_PATH}")

    if parent is unreal.Actor:
        unreal.log_warning(BLUEPRINT_MANUAL_STEPS)
    else:
        unreal.log(
            "[SemanticProbe] native ProbePointHit ready — spawn in PIE: "
            f"{ASSET_PATH}_C as level_sem_collision_probe"
        )


if __name__ == "__main__":
    main()
