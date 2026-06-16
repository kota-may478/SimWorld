"""Shared UE Editor helpers for SimWorld LevelProp blueprints (UE 5.3 safe).

Working pattern:

1. BP_LevelProp_Base: variables only (no StaticMesh on parent)
2. Create each child directly parented to BP_LevelProp_Base
3. Add local PropMesh under inherited DefaultSceneRoot (attach_subobject)

Do NOT create as Actor then reparent — that duplicates DefaultSceneRoot and
triggers Blueprint compiler ICE (broken thumbnails / data-only BPs).
"""
from __future__ import annotations

import unreal

BASE_BP_PATH = "/Game/SimWorld/LevelProps/Base/BP_LevelProp_Base.BP_LevelProp_Base"
BASE_BP_PACKAGE = "/Game/SimWorld/LevelProps/Base/BP_LevelProp_Base"
MESH_DIR = "/Game/Construction_VOL1/Meshes"
OUT_DIR = "/Game/SimWorld/LevelProps/Generated/Construction_VOL1"


def compile_blueprint(bp: unreal.Blueprint) -> bool:
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(bp)
    except Exception as exc:  # noqa: BLE001
        unreal.log_error(f"[LevelPropBP] compile failed: {exc}")
        return False
    if bp.generated_class() is None:
        return False
    return not blueprint_has_compile_errors(bp)


def blueprint_has_compile_errors(bp: unreal.Blueprint) -> bool:
    bel = getattr(unreal, "BlueprintEditorLibrary", None)
    if bel is None:
        return False
    for method_name in (
        "does_blueprint_have_compile_errors",
        "has_blueprint_compile_errors",
    ):
        checker = getattr(bel, method_name, None)
        if callable(checker):
            try:
                return bool(checker(bp))
            except Exception:  # noqa: BLE001
                pass
    return False


def _fail_reason_text(fail_reason: object) -> str:
    if fail_reason is None:
        return ""
    if hasattr(fail_reason, "is_empty"):
        try:
            return "" if fail_reason.is_empty() else str(fail_reason)
        except Exception:  # noqa: BLE001
            pass
    return str(fail_reason)


def _subobject_var_name(subdata: unreal.SubobjectData) -> str:
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    if hasattr(flib, "get_variable_name"):
        return str(flib.get_variable_name(subdata))
    return ""


def _is_static_mesh_subobject(subdata: unreal.SubobjectData) -> bool:
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    obj = flib.get_object(subdata)
    return obj is not None and obj.get_class() == unreal.StaticMeshComponent.static_class()


def _is_inherited_subobject(subdata: unreal.SubobjectData) -> bool:
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    if hasattr(flib, "is_inherited_component"):
        return bool(flib.is_inherited_component(subdata))
    return False


def _get_component_object(
    subdata: unreal.SubobjectData,
    bp: unreal.Blueprint,
) -> unreal.Object | None:
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    comp = flib.get_object(subdata)
    if comp is None and hasattr(flib, "get_object_for_blueprint"):
        comp = flib.get_object_for_blueprint(subdata, bp)
    return comp


def _apply_static_mesh(comp: unreal.Object, sm: unreal.StaticMesh | None) -> None:
    smc = unreal.StaticMeshComponent.cast(comp)
    target = smc if smc is not None else comp
    if hasattr(target, "set_static_mesh"):
        try:
            target.set_static_mesh(sm)
        except Exception:  # noqa: BLE001
            pass
    for prop in ("static_mesh", "StaticMesh"):
        try:
            target.set_editor_property(prop, sm)
        except Exception:  # noqa: BLE001
            pass
    if sm is not None:
        target.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        target.set_collision_profile_name(unreal.Name("BlockAll"))


def _find_parent_handle_for_new_mesh(handles: list, flib: object) -> object:
    """Prefer DefaultSceneRoot; fall back to blueprint root handle."""
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


def _log_subobject_layout(bp: unreal.Blueprint, log_tag: str) -> None:
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    unreal.log(f"[{log_tag}] subobjects on {bp.get_name()}: {len(handles)}")
    for idx, handle in enumerate(handles):
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        inherited = _is_inherited_subobject(subdata)
        is_comp = flib.is_component(subdata) if hasattr(flib, "is_component") else False
        unreal.log(
            f"[{log_tag}]   [{idx}] name={var_name or '?'} inherited={inherited} component={is_comp}"
        )


def _try_mesh_via_simple_construction_script(
    bp: unreal.Blueprint,
    sm: unreal.StaticMesh,
    log_tag: str,
) -> bool:
    try:
        scs = bp.get_editor_property("simple_construction_script")
    except Exception:  # noqa: BLE001
        return False
    if scs is None:
        return False

    nodes: list = []
    if hasattr(scs, "get_all_nodes"):
        try:
            nodes = list(scs.get_all_nodes())
        except Exception:  # noqa: BLE001
            nodes = []
    if not nodes and hasattr(scs, "get_editor_property"):
        try:
            nodes = list(scs.get_editor_property("all_nodes") or [])
        except Exception:  # noqa: BLE001
            nodes = []

    for node in nodes:
        if node is None:
            continue
        template = None
        if hasattr(node, "component_template"):
            template = node.component_template
        if template is None and hasattr(node, "get_editor_property"):
            try:
                template = node.get_editor_property("component_template")
            except Exception:  # noqa: BLE001
                template = None
        if template is None:
            continue
        if template.get_class() != unreal.StaticMeshComponent.static_class():
            continue
        _apply_static_mesh(template, sm)
        unreal.log(f"[{log_tag}] set mesh via SCS on {bp.get_name()}")
        return True
    return False


def add_local_prop_mesh_to_blueprint(
    bp: unreal.Blueprint,
    sm: unreal.StaticMesh,
    log_tag: str = "LevelPropBP",
    *,
    verbose: bool = False,
) -> bool:
    """Add or update a local PropMesh StaticMeshComponent on a blueprint asset."""
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if not handles:
        unreal.log_error(f"[{log_tag}] no subobject handles on {bp.get_name()}")
        return False

    if verbose:
        _log_subobject_layout(bp, log_tag)

    for handle in handles:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "PropMesh" and not _is_static_mesh_subobject(subdata):
            continue
        if _is_inherited_subobject(subdata):
            comp = _get_component_object(subdata, bp)
            if comp is not None and comp.get_class() == unreal.StaticMeshComponent.static_class():
                _apply_static_mesh(comp, sm)
                unreal.log(f"[{log_tag}] set inherited PropMesh override on {bp.get_name()}")
                return True
            continue
        comp = _get_component_object(subdata, bp)
        if comp is None or comp.get_class() != unreal.StaticMeshComponent.static_class():
            continue
        _apply_static_mesh(comp, sm)
        unreal.log(f"[{log_tag}] updated local PropMesh on {bp.get_name()}")
        return True

    parent_handle = _find_parent_handle_for_new_mesh(handles, flib)
    params = unreal.AddNewSubobjectParams(
        parent_handle=parent_handle,
        new_class=unreal.StaticMeshComponent,
        blueprint_context=bp,
    )
    sub_handle, fail_reason = subsystem.add_new_subobject(params)
    reason = _fail_reason_text(fail_reason)
    if sub_handle is None or reason:
        unreal.log_error(
            f"[{log_tag}] add_new_subobject failed on {bp.get_name()}: "
            f"{reason or 'null handle'}"
        )
        if _try_mesh_via_simple_construction_script(bp, sm, log_tag):
            return True
        if verbose:
            _log_subobject_layout(bp, log_tag)
        return False

    attach_ok = subsystem.attach_subobject(parent_handle, sub_handle)
    if not attach_ok:
        unreal.log_warning(f"[{log_tag}] attach_subobject returned false on {bp.get_name()}")

    subsystem.rename_subobject(sub_handle, unreal.Text("PropMesh"))
    subdata = flib.get_data(sub_handle)
    comp = _get_component_object(subdata, bp)
    if comp is None:
        unreal.log_error(f"[{log_tag}] PropMesh object missing after add on {bp.get_name()}")
        return _try_mesh_via_simple_construction_script(bp, sm, log_tag)

    _apply_static_mesh(comp, sm)
    unreal.log(f"[{log_tag}] added local PropMesh on {bp.get_name()}")
    return True


PIPELINE_VERSION = "v4-direct-parent-scs"


def editor_is_in_play_mode() -> bool:
    """True when PIE/Simulate is active (asset save to disk is blocked)."""
    try:
        subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        if subsystem is not None:
            for attr in ("is_play_in_editor_active", "is_play_in_editor_running"):
                fn = getattr(subsystem, attr, None)
                if callable(fn):
                    return bool(fn())
    except Exception:  # noqa: BLE001
        pass
    return False


def save_blueprint_package(
    bp: unreal.Blueprint,
    package_path: str,
    *,
    log_tag: str = "LevelPropBP",
) -> bool:
    """Persist a blueprint package to disk. Returns False during PIE or on save failure."""
    if editor_is_in_play_mode():
        unreal.log_error(
            f"[{log_tag}] cannot save {package_path}: Editor is in Play mode (PIE). "
            "Stop Play, then re-run this script."
        )
        return False
    unreal.EditorAssetLibrary.save_loaded_asset(bp)
    saved = unreal.EditorAssetLibrary.save_asset(package_path, only_if_is_dirty=False)
    if not saved:
        unreal.log_error(f"[{log_tag}] save_asset returned false for {package_path}")
        return False
    if not unreal.EditorAssetLibrary.does_asset_exist(package_path):
        unreal.log_error(f"[{log_tag}] asset not registered after save: {package_path}")
        return False
    return True


def _get_scs_nodes(scs: object) -> list:
    if hasattr(scs, "get_all_nodes"):
        try:
            return list(scs.get_all_nodes())
        except Exception:  # noqa: BLE001
            pass
    if hasattr(scs, "get_editor_property"):
        try:
            return list(scs.get_editor_property("all_nodes") or [])
        except Exception:  # noqa: BLE001
            pass
    return []


def _get_scs_default_root_node(scs: object) -> object | None:
    for method_name in ("get_default_scene_root_node", "get_root_nodes"):
        method = getattr(scs, method_name, None)
        if not callable(method):
            continue
        try:
            result = method()
            if result is None:
                continue
            if isinstance(result, (list, tuple)):
                return result[0] if result else None
            return result
        except Exception:  # noqa: BLE001
            continue
    for prop in ("default_scene_root_node", "DefaultSceneRootNode"):
        if hasattr(scs, "get_editor_property"):
            try:
                node = scs.get_editor_property(prop)
                if node is not None:
                    return node
            except Exception:  # noqa: BLE001
                pass
    nodes = _get_scs_nodes(scs)
    return nodes[0] if nodes else None


def _get_scs_node_template(node: object) -> object | None:
    if node is None:
        return None
    if hasattr(node, "component_template"):
        template = node.component_template
        if template is not None:
            return template
    if hasattr(node, "get_editor_property"):
        try:
            return node.get_editor_property("component_template")
        except Exception:  # noqa: BLE001
            return None
    return None


def add_prop_mesh_via_scs_create(
    bp: unreal.Blueprint,
    sm: unreal.StaticMesh,
    log_tag: str = "LevelPropBP",
) -> bool:
    """Add PropMesh through Simple Construction Script (reliable on UE 5.3)."""
    try:
        scs = bp.get_editor_property("simple_construction_script")
    except Exception:  # noqa: BLE001
        scs = None
    if scs is None:
        return False

    if _try_mesh_via_simple_construction_script(bp, sm, log_tag):
        return True

    root_node = _get_scs_default_root_node(scs)
    new_node = None
    if hasattr(scs, "create_node"):
        try:
            new_node = scs.create_node(unreal.StaticMeshComponent)
        except Exception:  # noqa: BLE001
            new_node = None

    if new_node is None:
        return False

    if root_node is not None:
        if hasattr(new_node, "set_parent"):
            try:
                new_node.set_parent(root_node)
            except Exception:  # noqa: BLE001
                pass
        elif hasattr(scs, "add_node"):
            try:
                scs.add_node(new_node, root_node, False)
            except Exception:  # noqa: BLE001
                try:
                    scs.add_node(new_node)
                except Exception:  # noqa: BLE001
                    pass

    for prop in ("variable_name", "VariableName", "component_variable_name"):
        if hasattr(new_node, "set_editor_property"):
            try:
                new_node.set_editor_property(prop, unreal.Name("PropMesh"))
                break
            except Exception:  # noqa: BLE001
                pass

    template = _get_scs_node_template(new_node)
    if template is None:
        return False
    _apply_static_mesh(template, sm)
    unreal.log(f"[{log_tag}] added PropMesh via SCS on {bp.get_name()}")
    return True


def strip_local_default_scene_roots(bp: unreal.Blueprint, log_tag: str = "LevelPropBP") -> int:
    """Delete non-inherited DefaultSceneRoot components (cause parent/child ICE)."""
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if not handles:
        return 0

    root_handle = handles[0]
    deleted = 0
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "DefaultSceneRoot":
            continue
        if _is_inherited_subobject(subdata):
            continue
        count = subsystem.delete_subobject(root_handle, handle, bp_context=bp)
        if count > 0:
            deleted += int(count)
            unreal.log(
                f"[{log_tag}] deleted local DefaultSceneRoot from {bp.get_name()}"
            )
    return deleted


def prepare_child_level_prop_blueprint(
    bp: unreal.Blueprint,
    log_tag: str = "LevelPropBP",
    *,
    verbose: bool = False,
) -> None:
    compile_blueprint(bp)
    strip_local_default_scene_roots(bp, log_tag=log_tag)
    remove_duplicate_local_default_scene_root(bp, log_tag=log_tag)
    if verbose:
        _log_subobject_layout(bp, log_tag)


def assign_prop_mesh_to_child_blueprint(
    bp: unreal.Blueprint,
    sm: unreal.StaticMesh,
    log_tag: str = "LevelPropBP",
    *,
    verbose: bool = False,
) -> bool:
    prepare_child_level_prop_blueprint(bp, log_tag=log_tag, verbose=verbose)
    if add_prop_mesh_via_scs_create(bp, sm, log_tag=log_tag):
        return True
    if add_local_prop_mesh_to_blueprint(bp, sm, log_tag=log_tag, verbose=verbose):
        return True
    if verbose:
        _log_subobject_layout(bp, log_tag)
    return False


def set_prop_variable_defaults(bp: unreal.Blueprint, prop_type_id: str) -> None:
    try:
        gen_class = bp.generated_class()
        cdo = unreal.get_default_object(gen_class)
        if cdo is None:
            return
        for var_name in ("PropTypeId", "prop_type_id"):
            if hasattr(cdo, var_name):
                cdo.set_editor_property(var_name, prop_type_id)
                break
        for var_name in ("SemanticTags", "semantic_tags"):
            if hasattr(cdo, var_name):
                cdo.set_editor_property(var_name, "obstacle")
                break
    except Exception:  # noqa: BLE001
        pass


def _strip_mesh_from_simple_construction_script(
    bp: unreal.Blueprint,
    log_tag: str,
) -> int:
    try:
        scs = bp.get_editor_property("simple_construction_script")
    except Exception:  # noqa: BLE001
        return 0
    if scs is None:
        return 0

    nodes: list = []
    if hasattr(scs, "get_all_nodes"):
        try:
            nodes = list(scs.get_all_nodes())
        except Exception:  # noqa: BLE001
            nodes = []
    if not nodes and hasattr(scs, "get_editor_property"):
        try:
            nodes = list(scs.get_editor_property("all_nodes") or [])
        except Exception:  # noqa: BLE001
            nodes = []

    cleared = 0
    for node in nodes:
        if node is None:
            continue
        template = None
        if hasattr(node, "component_template"):
            template = node.component_template
        if template is None and hasattr(node, "get_editor_property"):
            try:
                template = node.get_editor_property("component_template")
            except Exception:  # noqa: BLE001
                template = None
        if template is None:
            continue
        if template.get_class() != unreal.StaticMeshComponent.static_class():
            continue
        _apply_static_mesh(template, None)  # type: ignore[arg-type]
        cleared += 1
        unreal.log(f"[{log_tag}] cleared StaticMesh from SCS on {bp.get_name()}")
    return cleared


def remove_duplicate_local_default_scene_root(
    bp: unreal.Blueprint,
    log_tag: str = "LevelPropBP",
) -> bool:
    """Remove a local DefaultSceneRoot when an inherited one already exists."""
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if len(handles) < 2:
        return False

    root_handle = handles[0]
    inherited_scene_roots: list[object] = []
    local_scene_roots: list[object] = []
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "DefaultSceneRoot":
            continue
        if _is_inherited_subobject(subdata):
            inherited_scene_roots.append(handle)
        else:
            local_scene_roots.append(handle)

    if not inherited_scene_roots or not local_scene_roots:
        return False

    deleted = 0
    for handle in local_scene_roots:
        count = subsystem.delete_subobject(root_handle, handle, bp_context=bp)
        if count > 0:
            deleted += int(count)
            unreal.log(
                f"[{log_tag}] removed duplicate local DefaultSceneRoot from {bp.get_name()}"
            )
    return deleted > 0


def strip_prop_mesh_component(bp: unreal.Blueprint, log_tag: str = "LevelPropBP") -> bool:
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if not handles:
        return False

    root_handle = handles[0]
    deleted = 0
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "PropMesh" and not _is_static_mesh_subobject(subdata):
            continue
        if _is_inherited_subobject(subdata):
            comp = _get_component_object(subdata, bp)
            if comp is not None and comp.get_class() == unreal.StaticMeshComponent.static_class():
                _apply_static_mesh(comp, None)  # type: ignore[arg-type]
                deleted += 1
                unreal.log(
                    f"[{log_tag}] cleared inherited mesh override on "
                    f"{var_name or 'StaticMeshComponent'} in {bp.get_name()}"
                )
            continue
        count = subsystem.delete_subobject(root_handle, handle, bp_context=bp)
        if count > 0:
            deleted += int(count)
            unreal.log(
                f"[{log_tag}] deleted {var_name or 'StaticMeshComponent'} from {bp.get_name()}"
            )

    scs_cleared = _strip_mesh_from_simple_construction_script(bp, log_tag)
    duplicate_removed = remove_duplicate_local_default_scene_root(bp, log_tag=log_tag)
    changed = deleted > 0 or scs_cleared > 0 or duplicate_removed
    if changed:
        compile_blueprint(bp)
        unreal.EditorAssetLibrary.save_loaded_asset(bp)
    return changed


def delete_generated_child_blueprints(log_tag: str = "LevelPropBP") -> int:
    package_paths: set[str] = set()
    listed = unreal.EditorAssetLibrary.list_assets(
        OUT_DIR, recursive=False, include_folder=False
    )
    for listed_path in listed:
        package_paths.add(listed_path.split(".", 1)[0])

    registry = unreal.AssetRegistryHelpers.get_asset_registry()
    for data in registry.get_assets_by_path(OUT_DIR, recursive=False):
        asset_name = str(data.asset_name)
        if not asset_name.startswith("BP_"):
            continue
        if str(data.asset_class_path.asset_name) != "Blueprint":
            continue
        package_paths.add(str(data.package_name))

    deleted = 0
    for package_path in sorted(package_paths):
        if not unreal.EditorAssetLibrary.does_asset_exist(package_path):
            unreal.log_warning(f"[{log_tag}] skip missing: {package_path}")
            continue
        if unreal.EditorAssetLibrary.delete_asset(package_path):
            deleted += 1
            unreal.log(f"[{log_tag}] deleted {package_path}")
        else:
            unreal.log_error(f"[{log_tag}] delete failed: {package_path}")
    return deleted


def reload_base_blueprint(log_tag: str = "LevelPropBP") -> unreal.Blueprint | None:
    if unreal.EditorAssetLibrary.does_asset_exist(BASE_BP_PACKAGE):
        try:
            unreal.EditorAssetLibrary.reload_asset(BASE_BP_PACKAGE)
        except Exception:  # noqa: BLE001
            pass
    asset = unreal.load_asset(BASE_BP_PATH)
    if isinstance(asset, unreal.Blueprint):
        unreal.log(f"[{log_tag}] loaded base blueprint {BASE_BP_PATH}")
        return asset
    unreal.log_error(f"[{log_tag}] base missing: {BASE_BP_PATH}")
    return None


def ensure_base_variables_only(log_tag: str = "LevelPropBP") -> bool:
    asset = reload_base_blueprint(log_tag=log_tag)
    if asset is None:
        return False
    if strip_prop_mesh_component(asset, log_tag=log_tag):
        unreal.log(f"[{log_tag}] BP_LevelProp_Base cleaned (no mesh components)")
    else:
        unreal.log(f"[{log_tag}] BP_LevelProp_Base already variables-only")
    ok = compile_blueprint(asset)
    unreal.EditorAssetLibrary.save_loaded_asset(asset)
    if not ok:
        unreal.log_error(f"[{log_tag}] BP_LevelProp_Base compile errors remain")
    return ok
