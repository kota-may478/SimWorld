#!/usr/bin/env python3
"""UE Editor: enable Render CustomDepth on PropMesh for all Generated LevelProps.

Run in UE (PIE must be stopped):
  Tools → Execute Python Script → this file

Targets: /Game/SimWorld/LevelProps/Generated/Construction_VOL1/BP_*
"""
from __future__ import annotations

import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

from level_prop_blueprint_utils import (  # noqa: E402
    OUT_DIR,
    _get_component_object,
    _get_scs_node_template,
    _get_scs_nodes,
    _is_inherited_subobject,
    _is_static_mesh_subobject,
    _subobject_var_name,
    blueprint_has_compile_errors,
    compile_blueprint,
    editor_is_in_play_mode,
    save_blueprint_package,
)

CUSTOM_DEPTH_STENCIL_VALUE = 1
LOG_TAG = "CustomDepthProps"


def _set_custom_depth_on_component(comp: unreal.Object) -> bool:
    if comp is None or comp.get_class() != unreal.StaticMeshComponent.static_class():
        return False
    changed = False
    for prop in ("render_custom_depth", "b_render_custom_depth", "RenderCustomDepth"):
        try:
            before = comp.get_editor_property(prop)
            if not before:
                comp.set_editor_property(prop, True)
                changed = True
            break
        except Exception:  # noqa: BLE001
            continue
    for prop in ("custom_depth_stencil_value", "CustomDepthStencilValue"):
        try:
            current = int(comp.get_editor_property(prop))
            if current != CUSTOM_DEPTH_STENCIL_VALUE:
                comp.set_editor_property(prop, CUSTOM_DEPTH_STENCIL_VALUE)
                changed = True
            break
        except Exception:  # noqa: BLE001
            continue
    return changed


def _apply_on_blueprint(bp: unreal.Blueprint) -> bool:
    changed = False
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "PropMesh" and not _is_static_mesh_subobject(subdata):
            continue
        comp = _get_component_object(subdata, bp)
        if _set_custom_depth_on_component(comp):
            changed = True
        elif comp is not None and comp.get_class() == unreal.StaticMeshComponent.static_class():
            changed = True

    try:
        scs = bp.get_editor_property("simple_construction_script")
    except Exception:  # noqa: BLE001
        scs = None
    if scs is not None:
        for node in _get_scs_nodes(scs):
            template = _get_scs_node_template(node)
            if template is None:
                continue
            if template.get_class() != unreal.StaticMeshComponent.static_class():
                continue
            if _set_custom_depth_on_component(template):
                changed = True

    if not changed:
        return False
    ok = compile_blueprint(bp)
    if blueprint_has_compile_errors(bp):
        unreal.log_error(f"[{LOG_TAG}] compile errors on {bp.get_name()}")
        return False
    package_path = bp.get_path_name().split(".", 1)[0]
    return save_blueprint_package(bp, package_path, log_tag=LOG_TAG) and ok


def main() -> None:
    if editor_is_in_play_mode():
        unreal.log_error(
            f"[{LOG_TAG}] Editor is in Play mode. Stop PIE, then re-run this script."
        )
        return

    registry = unreal.AssetRegistryHelpers.get_asset_registry()
    assets = registry.get_assets_by_path(OUT_DIR, recursive=False)
    bp_paths: list[str] = []
    for data in assets:
        name = str(data.asset_name)
        if not name.startswith("BP_"):
            continue
        if str(data.asset_class_path.asset_name) != "Blueprint":
            continue
        package = str(data.package_name)
        bp_paths.append(f"{package}.{name}")
    bp_paths.sort()

    unreal.log(f"[{LOG_TAG}] found {len(bp_paths)} blueprints under {OUT_DIR}")

    updated = 0
    skipped = 0
    failed = 0
    for load_path in bp_paths:
        asset = unreal.load_asset(load_path)
        if not isinstance(asset, unreal.Blueprint):
            skipped += 1
            continue
        try:
            if _apply_on_blueprint(asset):
                updated += 1
                unreal.log(f"[{LOG_TAG}] OK {load_path}")
            else:
                skipped += 1
        except Exception as exc:  # noqa: BLE001
            failed += 1
            unreal.log_error(f"[{LOG_TAG}] FAIL {load_path}: {exc}")

    unreal.log(
        f"[{LOG_TAG}] done: updated={updated} skipped={skipped} failed={failed} "
        f"stencil={CUSTOM_DEPTH_STENCIL_VALUE}"
    )


if __name__ == "__main__":
    main()
