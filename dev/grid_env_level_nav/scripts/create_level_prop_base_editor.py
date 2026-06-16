#!/usr/bin/env python3
"""UE Editor: create BP_LevelProp_Base (Tier 0) — variables-only parent.

Run in UE: Tools → Execute Python Script → this file.

UE 5.3: Base must NOT contain PropMesh. Each Generated child adds its own
local PropMesh (see generate_construction_vol1_level_props_editor.py).
"""
from __future__ import annotations

import unreal

BASE_DIR = "/Game/SimWorld/LevelProps/Base"
BASE_NAME = "BP_LevelProp_Base"
BASE_PATH = f"{BASE_DIR}/{BASE_NAME}"

MANUAL_VARIABLE_STEPS = """
=== BP_LevelProp_Base: add variables (one-time, ~3 min) ===
1. Content Browser → SimWorld/LevelProps/Base → double-click BP_LevelProp_Base
2. My Blueprint panel → + Variable (each):
   - PropTypeId        (String)
   - SemanticTags      (String)   default: obstacle
   - FootprintRadiusCm (Float)    default: 50
   - SpawnFootOffsetZCm (Float)   default: 5
3. Do NOT add PropMesh here (children add their own mesh component).
4. Compile → Save
"""


def _ensure_dir(path: str) -> None:
    if not unreal.EditorAssetLibrary.does_directory_exist(path):
        unreal.EditorAssetLibrary.make_directory(path)
        unreal.log(f"[LevelPropBase] mkdir {path}")


def _compile_blueprint(asset: unreal.Blueprint) -> bool:
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(asset)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(asset)
        else:
            return False
    except Exception as exc:  # noqa: BLE001
        unreal.log_error(f"[LevelPropBase] compile failed: {exc}")
        return False
    return asset.generated_class() is not None


def _add_static_mesh_root(bp: unreal.Blueprint) -> None:
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if not handles:
        unreal.log_error("[LevelPropBase] no subobject handles on new BP")
        return
    root_handle = handles[0]
    params = unreal.AddNewSubobjectParams(
        parent_handle=root_handle,
        new_class=unreal.StaticMeshComponent,
        blueprint_context=bp,
    )
    sub_handle, fail_reason = subsystem.add_new_subobject(params)
    if fail_reason and not str(fail_reason).isspace():
        unreal.log_warning(f"[LevelPropBase] add mesh component: {fail_reason}")
    if sub_handle is None:
        unreal.log_error("[LevelPropBase] failed to add StaticMeshComponent")
        return
    subsystem.rename_subobject(sub_handle, unreal.Text("PropMesh"))
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    comp = flib.get_object(flib.get_data(sub_handle))
    if comp is not None:
        comp.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        comp.set_collision_profile_name(unreal.Name("BlockAll"))


def main() -> None:
    _ensure_dir("/Game/SimWorld")
    _ensure_dir("/Game/SimWorld/LevelProps")
    _ensure_dir(BASE_DIR)

    existing = unreal.EditorAssetLibrary.load_asset(BASE_PATH)
    if existing is not None:
        unreal.log(f"[LevelPropBase] already exists: {BASE_PATH}")
        unreal.log(MANUAL_VARIABLE_STEPS)
        return

    tools = unreal.AssetToolsHelpers.get_asset_tools()
    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", unreal.Actor)
    bp = tools.create_asset(BASE_NAME, BASE_DIR, unreal.Blueprint, factory)
    if bp is None:
        unreal.log_error("[LevelPropBase] create_asset failed")
        return

    ok = _compile_blueprint(bp)
    unreal.EditorAssetLibrary.save_loaded_asset(bp)
    unreal.log(f"[LevelPropBase] created {BASE_PATH} compile={'OK' if ok else 'FAIL'}")
    unreal.log(MANUAL_VARIABLE_STEPS)


if __name__ == "__main__":
    main()
