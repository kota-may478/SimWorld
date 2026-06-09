#!/usr/bin/env python3
"""Compile SpotDog blueprint chain in UE Editor (safe — skips broken assets)."""
from __future__ import annotations

import os

import unreal

BP_PATHS = [
    "/Game/CityDatabase/blueprints/BPI_Objects",
    "/Game/InteractableAsset/Blueprints/BP_InteractableAssetBase",
    "/Game/Agent/BP_AgentBase",
    "/Game/Robot_Dog/Blueprint/BP_SpotRobot",
    "/Game/Robot_Dog/Blueprint/BP_SpotRobot_Child",
]

MIN_AGENT_BYTES = 100_000


def _agent_editor_ready() -> bool:
    content = unreal.Paths.project_content_dir()
    agent_uasset = os.path.join(content, "Agent", "BP_AgentBase.uasset")
    if not os.path.isfile(agent_uasset):
        unreal.log_error(
            "[Robot_Dog] missing Content/Agent/BP_AgentBase.uasset — "
            "close Editor and run finalize_agent_for_editor.sh"
        )
        return False
    size = os.path.getsize(agent_uasset)
    if size < MIN_AGENT_BYTES:
        unreal.log_error(
            f"[Robot_Dog] Content/Agent/BP_AgentBase.uasset is only {size} bytes "
            "(cooked). Close Editor and run finalize_agent_for_editor.sh"
        )
        return False
    return True


def _compile_blueprint(asset: unreal.Blueprint) -> bool:
    if asset is None:
        return False
    try:
        if hasattr(unreal, "BlueprintEditorLibrary"):
            unreal.BlueprintEditorLibrary.compile_blueprint(asset)
        elif hasattr(unreal, "KismetEditorUtilities"):
            unreal.KismetEditorUtilities.compile_blueprint(asset)
        else:
            unreal.log_error("[Robot_Dog] no compile API on this UE build")
            return False
    except Exception as exc:  # noqa: BLE001 — UE may raise native errors
        unreal.log_error(f"[Robot_Dog] compile_blueprint exception: {exc}")
        return False
    gen = asset.generated_class()
    return gen is not None


def _compile_one(path: str) -> bool:
    asset = unreal.load_asset(path)
    if asset is None:
        unreal.log_error(f"[Robot_Dog] load failed: {path}")
        return False
    if not isinstance(asset, unreal.Blueprint):
        unreal.log_warning(f"[Robot_Dog] skip (not Blueprint): {path}")
        return True

    gen_before = asset.generated_class()
    if gen_before is None and path in (
        "/Game/Robot_Dog/Blueprint/BP_SpotRobot",
        "/Game/Robot_Dog/Blueprint/BP_SpotRobot_Child",
    ):
        unreal.log_error(
            f"[Robot_Dog] skip compile (no generated class, parent chain broken): {path}"
        )
        return False

    ok = _compile_blueprint(asset)
    unreal.EditorAssetLibrary.save_asset(path, only_if_is_dirty=False)
    gen = asset.generated_class()
    unreal.log(
        f"[Robot_Dog] {'OK' if ok else 'FAIL'} {path} -> "
        f"{gen.get_name() if gen else 'NO_CLASS'}"
    )
    return ok


def main() -> None:
    if not _agent_editor_ready():
        unreal.log_error("[Robot_Dog] abort — fix Content/Agent before compiling")
        return

    ok = True
    for path in BP_PATHS:
        if not _compile_one(path):
            ok = False
            if path == "/Game/Agent/BP_AgentBase":
                unreal.log_error(
                    "[Robot_Dog] BP_AgentBase failed — skipping SpotDog compile"
                )
                break

    if ok:
        unreal.log("[Robot_Dog] compile chain done — start PIE, then WSL patrol")
    else:
        unreal.log_error(
            "[Robot_Dog] compile incomplete. Check Output Log; do not re-run "
            "until Content/Agent is editor-ready (finalize_agent_for_editor.sh)."
        )


if __name__ == "__main__":
    main()
