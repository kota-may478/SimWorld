#!/usr/bin/env python3
"""UE Editor: diagnose one Generated LevelProp BP (default: BP_Barrel_01).

Run in Output Log and share lines starting with [DiagProps].
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
    OUT_DIR,
    _get_component_object,
    _is_inherited_subobject,
    _log_subobject_layout,
    _subobject_var_name,
    blueprint_has_compile_errors,
    compile_blueprint,
    reload_base_blueprint,
)

TARGET_BP = f"{OUT_DIR}/BP_Barrel_01.BP_Barrel_01"


def main() -> None:
    unreal.log("[DiagProps] === diagnose LevelProp blueprints ===")

    base = reload_base_blueprint(log_tag="DiagProps")
    if base is not None:
        _log_subobject_layout(base, "DiagProps")

    bp = unreal.load_asset(TARGET_BP)
    if not isinstance(bp, unreal.Blueprint):
        unreal.log_error(f"[DiagProps] missing target: {TARGET_BP}")
        return

    _log_subobject_layout(bp, "DiagProps")

    subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    flib = unreal.SubobjectDataBlueprintFunctionLibrary
    handles = subsystem.k2_gather_subobject_data_for_blueprint(bp)
    for handle in handles:
        subdata = flib.get_data(handle)
        var_name = _subobject_var_name(subdata)
        if var_name != "PropMesh":
            continue
        comp = _get_component_object(subdata, bp)
        mesh = None
        if comp is not None:
            try:
                mesh = comp.get_editor_property("static_mesh")
            except Exception:  # noqa: BLE001
                mesh = None
        unreal.log(
            f"[DiagProps] PropMesh inherited={_is_inherited_subobject(subdata)} "
            f"mesh={mesh}"
        )

    gen = bp.generated_class()
    if gen is not None:
        cdo = unreal.get_default_object(gen)
        if cdo is not None:
            for var_name in ("PropTypeId", "SemanticTags"):
                if hasattr(cdo, var_name):
                    unreal.log(
                        f"[DiagProps] CDO {var_name}={cdo.get_editor_property(var_name)}"
                    )

    compile_blueprint(bp)
    unreal.log(
        f"[DiagProps] compile_errors={blueprint_has_compile_errors(bp)} "
        f"generated_class={bp.generated_class() is not None}"
    )
    unreal.log("[DiagProps] === end ===")


if __name__ == "__main__":
    main()
