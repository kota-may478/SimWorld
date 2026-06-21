#!/usr/bin/env python3
"""UE Editor: create the 3 debug-mesh child BPs excluded from the first rebuild.

IMPORTANT: Stop Play (PIE) before running. During PIE, UE logs
"The Editor is currently in a play mode" and assets are NOT written to disk.

Run in UE: Tools → Execute Python Script → this file

Meshes: SM_ZBackdrop_01, SM_ZPlane_01a, SM_ZSphere
"""
from __future__ import annotations

import importlib
import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

import generate_construction_vol1_level_props_editor as gen
import level_prop_blueprint_utils as utils

# UE Editor caches imported modules for the session — reload so new helpers
# (editor_is_in_play_mode, save_blueprint_package) are always visible.
importlib.reload(utils)
importlib.reload(gen)

from generate_construction_vol1_level_props_editor import (  # noqa: E402
    _bp_name_from_mesh_path,
    _create_child_bp,
    _ensure_dir,
    _prop_type_id_from_mesh_path,
    _resolve_parent_class,
)
from level_prop_blueprint_utils import editor_is_in_play_mode  # noqa: E402

MISSING_MESH_PATHS = (
    "/Game/Construction_VOL1/Meshes/SM_ZBackdrop_01.SM_ZBackdrop_01",
    "/Game/Construction_VOL1/Meshes/SM_ZPlane_01a.SM_ZPlane_01a",
    "/Game/Construction_VOL1/Meshes/SM_ZSphere.SM_ZSphere",
)


def main() -> None:
    if editor_is_in_play_mode():
        unreal.log_error(
            "[GenMissing] ABORT: Editor is in Play mode (PIE). "
            "Stop Play, then re-run this script. "
            "PIE中に実行するとメモリ上だけ作成され .uasset は保存されません。"
        )
        return

    unreal.log("[GenMissing] creating 3 previously excluded Construction VOL.1 props")
    _ensure_dir("/Game/SimWorld/LevelProps/Generated")
    _ensure_dir("/Game/SimWorld/LevelProps/Generated/Construction_VOL1")
    parent = _resolve_parent_class()
    created = 0
    skipped = 0
    failed = 0
    for mesh_path in MISSING_MESH_PATHS:
        bp_name = _bp_name_from_mesh_path(mesh_path)
        out_path = f"/Game/SimWorld/LevelProps/Generated/Construction_VOL1/{bp_name}"
        package_dot = f"{out_path}.{bp_name}"
        if unreal.EditorAssetLibrary.does_asset_exist(out_path):
            existing = unreal.load_asset(package_dot)
            if existing is not None and utils.save_blueprint_package(
                existing, out_path, log_tag="GenMissing"
            ):
                unreal.log(f"[GenMissing] saved existing: {out_path}")
                skipped += 1
                continue
            unreal.log_warning(
                f"[GenMissing] ghost/unsaved asset — delete and recreate: {out_path}"
            )
            unreal.EditorAssetLibrary.delete_asset(out_path)
        prop_id = _prop_type_id_from_mesh_path(mesh_path)
        if _create_child_bp(bp_name, mesh_path, parent, prop_id, verbose=True):
            created += 1
        else:
            failed += 1
    unreal.log(f"[GenMissing] done: created={created} skipped={skipped} failed={failed}")
    if created > 0:
        unreal.log(
            "[GenMissing] verify in Content Browser: "
            "SimWorld/LevelProps/Generated/Construction_VOL1/BP_Z*"
        )


if __name__ == "__main__":
    main()
