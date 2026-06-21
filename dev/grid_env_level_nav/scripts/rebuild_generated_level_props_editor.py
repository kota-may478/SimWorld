#!/usr/bin/env python3
"""UE Editor: One-shot rebuild for broken Generated LevelProp BPs (UE 5.3).

Use when child BPs show checkered thumbnails, compiler ICE in SimWorld.log,
or "data-only blueprint" without Components / PropMesh.

Steps performed:
1. Clean BP_LevelProp_Base (strip mesh + duplicate DefaultSceneRoot)
2. Delete all /Generated/Construction_VOL1/BP_* children
3. Regenerate children directly under BP_LevelProp_Base + local PropMesh
"""
from __future__ import annotations

import os
import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import unreal

import importlib

import generate_construction_vol1_level_props_editor as gen
import level_prop_blueprint_utils as utils

importlib.reload(utils)
importlib.reload(gen)

from level_prop_blueprint_utils import (
    PIPELINE_VERSION,
    delete_generated_child_blueprints,
    ensure_base_variables_only,
)


def main() -> None:
    unreal.log(
        f"[RebuildProps] === start full rebuild pipeline={PIPELINE_VERSION} ==="
    )

    if not ensure_base_variables_only(log_tag="RebuildProps"):
        unreal.log_error("[RebuildProps] abort: BP_LevelProp_Base missing")
        return

    deleted = delete_generated_child_blueprints(log_tag="RebuildProps")
    unreal.log(f"[RebuildProps] deleted {deleted} existing generated child BPs")

    gen.VERBOSE_FIRST_FAILURE = True
    gen.FORCE_RECREATE = True
    gen.main()

    unreal.log(
        "[RebuildProps] === done === reopen BP_Barrel_01: expect Components tab, "
        "PropMesh -> SM_Barrel_01, barrel thumbnail"
    )


if __name__ == "__main__":
    main()
