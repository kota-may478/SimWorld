#!/usr/bin/env python3
"""UE Editor: add AI Perception Stimuli Source (Sight) to LevelProp + Humanoid BPs.

Run in UE (PIE must be stopped):
  Tools → Execute Python Script → this file

Default: patch BP_LevelProp_Base (73 child props inherit) + Humanoid Base_User_Agent.

Optional env SIMWORLD_SIGHT_STIMULI_MODE:
  base_and_extras  — default; parent prop BP + humanoid BP
  props_only       — BP_LevelProp_Base only (no humanoid)
  all_generated    — each BP_* under Construction_VOL1 (73) + humanoid BP
"""
from __future__ import annotations

import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

from level_prop_blueprint_utils import (  # noqa: E402
    BASE_BP_PATH,
    OUT_DIR,
    _get_component_object,
    _subobject_var_name,
    blueprint_has_compile_errors,
    compile_blueprint,
    editor_is_in_play_mode,
    save_blueprint_package,
)

LOG_TAG = "SightStimuliProps"
STIMULI_VAR_NAME = "AIPerceptionStimuliSource"

# site20_humanoid spawns from this BP (see grid_env_hri_simulation.HUMAN_BP).
HUMANOID_BP_PACKAGES: tuple[str, ...] = (
    "/Game/TrafficSystem/Pedestrian/Base_User_Agent",
)

MODE = os.environ.get("SIMWORLD_SIGHT_STIMULI_MODE", "base_and_extras").strip().lower()


def _stimuli_component_class() -> unreal.Class:
    return unreal.AIPerceptionStimuliSourceComponent.static_class()


def _sight_sense_class() -> unreal.Class:
    return unreal.AISense_Sight.static_class()


def _is_stimuli_component(comp: unreal.Object | None) -> bool:
    return comp is not None and comp.get_class() == _stimuli_component_class()


def _find_parent_handle_for_new_component(handles: list, flib: object) -> object:
    bp_handle = handles[0]
    scene_root = None
    any_scene = None
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        if hasattr(flib, "is_scene_component") and flib.is_scene_component(subdata):
            any_scene = any_scene or handle
            if hasattr(flib, "is_root_component") and flib.is_root_component(subdata):
                scene_root = handle
    return scene_root or any_scene or bp_handle


def _configure_stimuli_component(comp: unreal.Object) -> bool:
    changed = False
    for prop in ("auto_register_as_source", "b_auto_register_as_source"):
        try:
            if not bool(comp.get_editor_property(prop)):
                comp.set_editor_property(prop, True)
                changed = True
            break
        except Exception:  # noqa: BLE001
            continue

    sight = _sight_sense_class()
    for method_name in ("register_as_source_for_sense", "register_for_sense"):
        method = getattr(comp, method_name, None)
        if callable(method):
            try:
                method(sight)
                changed = True
            except Exception:  # noqa: BLE001
                pass

    for prop in ("register_as_source_for_senses", "RegisterAsSourceForSenses"):
        try:
            current = comp.get_editor_property(prop)
            if sight not in list(current or []):
                comp.set_editor_property(prop, [sight])
                changed = True
            break
        except Exception:  # noqa: BLE001
            continue
    return changed


def _find_or_add_stimuli_component(bp: unreal.Blueprint) -> tuple[unreal.Object | None, bool]:
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    stimuli_cls = _stimuli_component_class()

    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        comp = _get_component_object(subdata, bp)
        if var_name == STIMULI_VAR_NAME or _is_stimuli_component(comp):
            return comp, False

    parent_handle = _find_parent_handle_for_new_component(handles, flib)
    params = unreal.AddNewSubobjectParams(
        parent_handle=parent_handle,
        new_class=unreal.AIPerceptionStimuliSourceComponent,
        blueprint_context=bp,
    )
    sub_handle, fail_reason = subsystem.add_new_subobject(params)
    if sub_handle is None:
        reason = "" if fail_reason is None else str(fail_reason)
        unreal.log_error(
            f"[{LOG_TAG}] add_new_subobject failed on {bp.get_name()}: {reason or 'null handle'}"
        )
        return None, False

    subsystem.attach_subobject(parent_handle, sub_handle)
    subsystem.rename_subobject(sub_handle, unreal.Text(STIMULI_VAR_NAME))
    subdata = flib.get_data(sub_handle)
    comp = _get_component_object(subdata, bp)
    if comp is None or comp.get_class() != stimuli_cls:
        unreal.log_error(f"[{LOG_TAG}] stimuli component missing after add on {bp.get_name()}")
        return None, False
    return comp, True


def _apply_on_blueprint(bp: unreal.Blueprint) -> bool:
    comp, added = _find_or_add_stimuli_component(bp)
    if comp is None:
        return False
    changed = added or _configure_stimuli_component(comp)
    if not changed:
        return False

    ok = compile_blueprint(bp)
    if blueprint_has_compile_errors(bp):
        unreal.log_error(f"[{LOG_TAG}] compile errors on {bp.get_name()}")
        return False
    package_path = bp.get_path_name().split(".", 1)[0]
    return save_blueprint_package(bp, package_path, log_tag=LOG_TAG) and ok


def _list_generated_blueprint_paths() -> list[str]:
    registry = unreal.AssetRegistryHelpers.get_asset_registry()
    assets = registry.get_assets_by_path(OUT_DIR, recursive=False)
    paths: list[str] = []
    for data in assets:
        name = str(data.asset_name)
        if not name.startswith("BP_"):
            continue
        if str(data.asset_class_path.asset_name) != "Blueprint":
            continue
        package = str(data.package_name)
        paths.append(f"{package}.{name}")
    paths.sort()
    return paths


def _blueprint_load_path(package_path: str) -> str | None:
    """Resolve package path to Blueprint load path (Package.AssetName)."""
    if not unreal.EditorAssetLibrary.does_asset_exist(package_path):
        return None
    asset_name = package_path.rsplit("/", 1)[-1]
    return f"{package_path}.{asset_name}"


def _discover_humanoid_blueprint_paths() -> list[str]:
    found: list[str] = []
    for package_path in HUMANOID_BP_PACKAGES:
        load_path = _blueprint_load_path(package_path)
        if load_path is None:
            unreal.log_warning(
                f"[{LOG_TAG}] humanoid BP not found (skip): {package_path} "
                f"— mount TrafficSystem pak or copy Base_User_Agent"
            )
            continue
        found.append(load_path)
    return found


def _resolve_target_paths() -> list[str]:
    paths: list[str] = []

    if MODE in ("all_generated",):
        paths.extend(_list_generated_blueprint_paths())
    elif MODE in ("props_only", "base"):
        paths.append(BASE_BP_PATH)
    elif MODE in ("base_and_extras", "default", ""):
        paths.append(BASE_BP_PATH)
    else:
        unreal.log_warning(
            f"[{LOG_TAG}] unknown SIMWORLD_SIGHT_STIMULI_MODE={MODE!r}; "
            f"using base_and_extras"
        )
        paths.append(BASE_BP_PATH)

    if MODE not in ("props_only", "base"):
        for extra in _discover_humanoid_blueprint_paths():
            if extra not in paths:
                paths.append(extra)

    return paths


def main() -> None:
    if editor_is_in_play_mode():
        unreal.log_error(
            f"[{LOG_TAG}] Editor is in Play mode. Stop PIE, then re-run this script."
        )
        return

    target_paths = _resolve_target_paths()
    unreal.log(f"[{LOG_TAG}] mode={MODE} targets={len(target_paths)}")
    for path in target_paths:
        unreal.log(f"[{LOG_TAG}]   target: {path}")

    updated = 0
    skipped = 0
    failed = 0
    for load_path in target_paths:
        asset = unreal.load_asset(load_path)
        if not isinstance(asset, unreal.Blueprint):
            unreal.log_warning(f"[{LOG_TAG}] skip (not Blueprint): {load_path}")
            skipped += 1
            continue
        try:
            if _apply_on_blueprint(asset):
                updated += 1
                unreal.log(f"[{LOG_TAG}] OK {load_path}")
            else:
                skipped += 1
                unreal.log(f"[{LOG_TAG}] skip (already configured): {load_path}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            unreal.log_error(f"[{LOG_TAG}] FAIL {load_path}: {exc}")

    unreal.log(
        f"[{LOG_TAG}] done: mode={MODE} updated={updated} skipped={skipped} failed={failed}"
    )


if __name__ == "__main__":
    main()
