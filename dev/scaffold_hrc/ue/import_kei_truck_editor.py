"""UE Editor: import the Suzuki Carry FBX and wrap it in BP_KeiTruck.

Stop Play (PIE) first. During PIE, UE will not write .uasset files to disk.

Run: Tools → Execute Python Script → this file
  C:\\UEProjects\\SimWorld\\Scripts\\import_kei_truck_editor.py

After import, assign paint materials with
  C:\\UEProjects\\SimWorld\\Scripts\\assign_kei_truck_materials_editor.py
"""
from __future__ import annotations

import os

import unreal

FBX_PATH = r"C:\UEProjects\SimWorld\RawImport\suzuki_carry_kei_truck_1993.fbx"
DEST_DIR = "/Game/SimWorld/Props/KeiTruck"
MESH_NAME = "SM_SuzukiCarry1993"
BP_NAME = "BP_KeiTruck"
MESH_PATH = f"{DEST_DIR}/{MESH_NAME}"
BP_PATH = f"{DEST_DIR}/{BP_NAME}"
SPAWN_CLASS = f"{BP_PATH}.{BP_NAME}_C"
OK_MARKER = r"C:\UEProjects\SimWorld\RawImport\kei_truck_import_ok.txt"
LOG = "[KeiTruck]"
# Typical 5th-gen Carry length is ~3.3 m. Used only for a log warning.
EXPECTED_LENGTH_CM = 330.0


def _log(msg: str) -> None:
    unreal.log(f"{LOG} {msg}")


def _err(msg: str) -> None:
    unreal.log_error(f"{LOG} {msg}")


def editor_is_in_play_mode() -> bool:
    try:
        subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        if subsystem is None:
            return False
        for attr in ("is_play_in_editor_active", "is_play_in_editor_running"):
            fn = getattr(subsystem, attr, None)
            if callable(fn) and bool(fn()):
                return True
    except Exception as exc:  # noqa: BLE001
        _log(f"play-mode check skipped: {exc}")
    return False


def _ensure_dir(path: str) -> None:
    if not unreal.EditorAssetLibrary.does_directory_exist(path):
        unreal.EditorAssetLibrary.make_directory(path)
        _log(f"mkdir {path}")


def _compile_blueprint(bp: unreal.Blueprint) -> bool:
    try:
        if hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(bp)
        elif hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(bp)
        else:
            return False
    except Exception as exc:  # noqa: BLE001
        _err(f"compile failed: {exc}")
        return False
    return bp.generated_class() is not None


def _longest_cm(sm: unreal.StaticMesh) -> float:
    bounds = sm.get_bounds()
    extent = bounds.box_extent
    return 2.0 * max(abs(extent.x), abs(extent.y), abs(extent.z))


def _import_static_mesh() -> unreal.StaticMesh | None:
    existing = unreal.load_asset(MESH_PATH)
    if isinstance(existing, unreal.StaticMesh):
        _log(f"mesh already exists {MESH_PATH}")
        return existing
    if not os.path.isfile(FBX_PATH):
        _err(f"FBX not found: {FBX_PATH}")
        return None

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", FBX_PATH)
    task.set_editor_property("destination_path", DEST_DIR)
    task.set_editor_property("destination_name", MESH_NAME)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("automated", True)
    task.set_editor_property("save", True)

    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh", True)
    options.set_editor_property("import_textures", True)
    options.set_editor_property("import_materials", True)
    options.set_editor_property("import_as_skeletal", False)
    options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    sm_data = options.static_mesh_import_data
    sm_data.set_editor_property("combine_meshes", True)
    sm_data.set_editor_property("auto_generate_collision", True)
    sm_data.set_editor_property("convert_scene", True)
    sm_data.set_editor_property("convert_scene_unit", True)
    sm_data.set_editor_property("force_front_x_axis", False)
    task.set_editor_property("options", options)

    _log(f"importing {FBX_PATH}")
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    mesh = unreal.load_asset(MESH_PATH)
    if isinstance(mesh, unreal.StaticMesh):
        return mesh
    imported = list(task.imported_object_paths or [])
    _log(f"imported_object_paths={imported}")
    for path in imported:
        asset = unreal.load_asset(path)
        if isinstance(asset, unreal.StaticMesh):
            if path.split(".", 1)[0] != MESH_PATH:
                renamed = unreal.EditorAssetLibrary.rename_asset(path.split(".", 1)[0], MESH_PATH)
                _log(f"rename mesh -> {MESH_PATH} ok={renamed}")
                mesh = unreal.load_asset(MESH_PATH)
                if isinstance(mesh, unreal.StaticMesh):
                    return mesh
            return asset
    _err("FBX import produced no StaticMesh")
    return None


def _add_mesh_component(bp: unreal.Blueprint, sm: unreal.StaticMesh) -> bool:
    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    if not handles:
        _err("no subobject handles on new BP")
        return False
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    parent_handle = handles[0]
    for handle in handles[1:]:
        subdata = flib.get_data(handle)
        if hasattr(flib, "is_scene_component") and flib.is_scene_component(subdata):
            parent_handle = handle
            break
    params = unreal.AddNewSubobjectParams(
        parent_handle=parent_handle,
        new_class=unreal.StaticMeshComponent,
        blueprint_context=bp,
    )
    sub_handle, fail_reason = subsystem.add_new_subobject(params)
    if sub_handle is None:
        _err(f"add StaticMeshComponent failed: {fail_reason}")
        return False
    subsystem.attach_subobject(parent_handle, sub_handle)
    subsystem.rename_subobject(sub_handle, unreal.Text("TruckMesh"))
    comp = flib.get_object(flib.get_data(sub_handle))
    if comp is None:
        _err("TruckMesh object missing after add")
        return False
    smc = unreal.StaticMeshComponent.cast(comp)
    target = smc if smc is not None else comp
    target.set_static_mesh(sm)
    target.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    target.set_collision_profile_name(unreal.Name("BlockAll"))
    return True


def _create_or_update_blueprint(sm: unreal.StaticMesh) -> unreal.Blueprint | None:
    existing = unreal.load_asset(BP_PATH)
    if isinstance(existing, unreal.Blueprint):
        _log(f"blueprint already exists {BP_PATH}")
        ok = _compile_blueprint(existing)
        unreal.EditorAssetLibrary.save_asset(BP_PATH, only_if_is_dirty=False)
        _log(f"recompiled existing BP ok={ok}")
        return existing

    tools = unreal.AssetToolsHelpers.get_asset_tools()
    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", unreal.Actor)
    bp = tools.create_asset(BP_NAME, DEST_DIR, unreal.Blueprint, factory)
    if bp is None:
        _err("create_asset BP_KeiTruck failed")
        return None
    if not _add_mesh_component(bp, sm):
        return None
    if not _compile_blueprint(bp):
        _err("BP_KeiTruck compile failed")
        return None
    unreal.EditorAssetLibrary.save_loaded_asset(bp)
    saved = unreal.EditorAssetLibrary.save_asset(BP_PATH, only_if_is_dirty=False)
    if not saved:
        _err("save_asset returned false for BP_KeiTruck")
        return None
    return bp


def _write_marker(length_cm: float, compile_ok: bool) -> None:
    lines = (
        f"spawn_class={SPAWN_CLASS}\n"
        f"mesh={MESH_PATH}\n"
        f"length_cm={length_cm:.1f}\n"
        f"compile_ok={compile_ok}\n"
    )
    with open(OK_MARKER, "w", encoding="utf-8") as handle:
        handle.write(lines)
    _log(f"wrote {OK_MARKER}")


def main() -> None:
    if editor_is_in_play_mode():
        _err(
            "ABORT: Editor is in Play mode (PIE). "
            "Stop Play, then Tools → Execute Python Script → this file."
        )
        return
    if not os.path.isfile(FBX_PATH):
        _err(f"FBX not found: {FBX_PATH}")
        return

    _ensure_dir("/Game/SimWorld")
    _ensure_dir("/Game/SimWorld/Props")
    _ensure_dir(DEST_DIR)

    sm = _import_static_mesh()
    if sm is None:
        return
    length_cm = _longest_cm(sm)
    _log(f"mesh longest axis {length_cm:.1f} cm (Carry is about {EXPECTED_LENGTH_CM:.0f} cm)")
    if length_cm < 80.0:
        _err("mesh is far too small — likely a unit mismatch. Tell the WSL agent.")
    elif length_cm > 2000.0:
        _err("mesh is far too large — likely a unit mismatch. Tell the WSL agent.")

    bp = _create_or_update_blueprint(sm)
    if bp is None:
        return
    compile_ok = bp.generated_class() is not None
    _write_marker(length_cm, compile_ok)
    _log(f"done spawn class {SPAWN_CLASS} compile_ok={compile_ok}")
    _log("Next: Play on /Game/Maps/Level, then spawn from WSL with --truck-only")


if __name__ == "__main__":
    main()
