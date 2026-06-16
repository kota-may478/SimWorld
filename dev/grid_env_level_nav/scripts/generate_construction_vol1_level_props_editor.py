#!/usr/bin/env python3
"""UE Editor: Tier-1 child BPs for Construction VOL.1 static meshes (73 SM_*).

Children are created directly under BP_LevelProp_Base with a local PropMesh.
Run rebuild_generated_level_props_editor.py if a prior Actor+reparent run left
broken assets (checkered thumbnails, missing Components tab).

- Reads:  /Game/Construction_VOL1/Meshes/SM_*
- Writes: /Game/SimWorld/LevelProps/Generated/Construction_VOL1/BP_<name>
"""
from __future__ import annotations

import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

from level_prop_blueprint_utils import (
    BASE_BP_PATH,
    MESH_DIR,
    OUT_DIR,
    PIPELINE_VERSION,
    assign_prop_mesh_to_child_blueprint,
    blueprint_has_compile_errors,
    compile_blueprint,
    ensure_base_variables_only,
    save_blueprint_package,
    set_prop_variable_defaults,
)

DRY_RUN = False
FORCE_RECREATE = False
VERBOSE_FIRST_FAILURE = True
# All 73 SM_* meshes (including Z* debug meshes) are generated.
EXCLUDE_MESH_NAMES: frozenset[str] = frozenset()


def _ensure_dir(path: str) -> None:
    if not unreal.EditorAssetLibrary.does_directory_exist(path):
        unreal.EditorAssetLibrary.make_directory(path)


def _resolve_parent_class() -> type:
    asset = unreal.load_asset(BASE_BP_PATH)
    if isinstance(asset, unreal.Blueprint):
        gen = asset.generated_class()
        if gen is not None:
            unreal.log(f"[GenProps] parent = {BASE_BP_PATH}")
            return gen
    unreal.log_warning(
        f"[GenProps] {BASE_BP_PATH} missing — parent=Actor. "
        "Run create_level_prop_base_editor.py first."
    )
    return unreal.Actor


def _asset_data_load_path(data: unreal.AssetData) -> str:
    asset_name = str(data.asset_name)
    package_name = str(data.package_name)
    if package_name.endswith(f".{asset_name}"):
        return package_name
    return f"{package_name}.{asset_name}"


def _mesh_asset_paths() -> list[str]:
    paths: list[str] = []
    seen: set[str] = set()

    registry = unreal.AssetRegistryHelpers.get_asset_registry()
    assets = registry.get_assets_by_path(MESH_DIR, recursive=False)
    if assets:
        for data in assets:
            name = str(data.asset_name)
            if not name.startswith("SM_"):
                continue
            if name in EXCLUDE_MESH_NAMES:
                continue
            if str(data.asset_class_path.asset_name) != "StaticMesh":
                continue
            load_path = _asset_data_load_path(data)
            if load_path not in seen:
                seen.add(load_path)
                paths.append(load_path)
        return sorted(paths)

    for listed in unreal.EditorAssetLibrary.list_assets(
        MESH_DIR, recursive=False, include_folder=False
    ):
        base = listed.split(".", 1)[0]
        asset_name = base.rsplit("/", 1)[-1]
        if not asset_name.startswith("SM_"):
            continue
        if asset_name in EXCLUDE_MESH_NAMES:
            continue
        asset_data = unreal.EditorAssetLibrary.find_asset_data(base)
        if asset_data is None:
            continue
        if str(asset_data.asset_class_path.asset_name) != "StaticMesh":
            continue
        load_path = _asset_data_load_path(asset_data)
        if load_path not in seen:
            seen.add(load_path)
            paths.append(load_path)
    return sorted(paths)


def _bp_name_from_mesh_path(mesh_path: str) -> str:
    asset_name = mesh_path.rsplit("/", 1)[-1].split(".", 1)[0]
    if asset_name.startswith("SM_"):
        asset_name = asset_name[3:]
    return f"BP_{asset_name}"


def _prop_type_id_from_mesh_path(mesh_path: str) -> str:
    asset_name = mesh_path.rsplit("/", 1)[-1].split(".", 1)[0]
    if asset_name.startswith("SM_"):
        asset_name = asset_name[3:]
    return asset_name.lower()


def _create_child_bp(
    bp_name: str,
    mesh_path: str,
    parent_class: type,
    prop_type_id: str,
    *,
    verbose: bool = False,
) -> bool:
    out_path = f"{OUT_DIR}/{bp_name}"
    if unreal.EditorAssetLibrary.does_asset_exist(out_path):
        if not FORCE_RECREATE:
            unreal.log(f"[GenProps] skip exists: {out_path}")
            return False
        if not unreal.EditorAssetLibrary.delete_asset(out_path):
            unreal.log_error(f"[GenProps] failed to delete existing asset: {out_path}")
            return False

    if DRY_RUN:
        unreal.log(f"[GenProps] dry-run would create {out_path} ← {mesh_path}")
        return True

    sm = unreal.load_asset(mesh_path)
    if sm is None:
        unreal.log_error(f"[GenProps] mesh not found: {mesh_path}")
        return False

    if parent_class in (None, unreal.Actor):
        unreal.log_error(
            "[GenProps] BP_LevelProp_Base missing — run create_level_prop_base_editor.py"
        )
        return False

    tools = unreal.AssetToolsHelpers.get_asset_tools()
    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", parent_class)
    bp = tools.create_asset(bp_name, OUT_DIR, unreal.Blueprint, factory)
    if bp is None:
        unreal.log_error(f"[GenProps] create failed: {out_path}")
        return False

    if not assign_prop_mesh_to_child_blueprint(
        bp, sm, log_tag="GenProps", verbose=verbose
    ):
        unreal.log_error(f"[GenProps] failed to assign PropMesh on {out_path}")
        unreal.EditorAssetLibrary.delete_asset(out_path)
        return False

    set_prop_variable_defaults(bp, prop_type_id)
    ok = compile_blueprint(bp)
    if blueprint_has_compile_errors(bp):
        unreal.log_error(f"[GenProps] compile errors on {out_path}")
        ok = False
    if not ok:
        unreal.log(f"[GenProps] FAIL {out_path} ← {mesh_path}")
        return False
    if not save_blueprint_package(bp, out_path, log_tag="GenProps"):
        if unreal.EditorAssetLibrary.does_asset_exist(out_path):
            unreal.EditorAssetLibrary.delete_asset(out_path)
        return False
    unreal.log(f"[GenProps] OK {out_path} ← {mesh_path}")
    return True


def main() -> None:
    unreal.log(f"[GenProps] pipeline={PIPELINE_VERSION}")
    _ensure_dir("/Game/SimWorld/LevelProps/Generated")
    _ensure_dir(OUT_DIR)
    ensure_base_variables_only(log_tag="GenProps")

    parent = _resolve_parent_class()
    meshes = _mesh_asset_paths()
    unreal.log(f"[GenProps] found {len(meshes)} static meshes under {MESH_DIR}")

    created = 0
    skipped = 0
    failed = 0
    for index, mesh_path in enumerate(meshes):
        bp_name = _bp_name_from_mesh_path(mesh_path)
        out_path = f"{OUT_DIR}/{bp_name}"
        if unreal.EditorAssetLibrary.does_asset_exist(out_path) and not FORCE_RECREATE:
            skipped += 1
            continue
        prop_id = _prop_type_id_from_mesh_path(mesh_path)
        verbose = VERBOSE_FIRST_FAILURE and failed == 0 and created == 0
        if _create_child_bp(bp_name, mesh_path, parent, prop_id, verbose=verbose):
            created += 1
        else:
            failed += 1

    unreal.log(
        f"[GenProps] done: created={created} skipped={skipped} failed={failed} "
        f"(vendor BPs under Construction_VOL1/Blueprints untouched)"
    )


if __name__ == "__main__":
    main()
