"""Create editor-only BP_InteractableAssetBase stub (PIE 前に Editor で実行)."""
from __future__ import annotations

import unreal

ASSET_DIR = "/Game/InteractableAsset/Blueprints"
ASSET_NAME = "BP_InteractableAssetBase"
ASSET_PATH = f"{ASSET_DIR}/{ASSET_NAME}"


def main() -> None:
  if unreal.EditorAssetLibrary.does_asset_exist(ASSET_PATH):
    unreal.log(f"[InteractableStub] exists: {ASSET_PATH}")
    return

  tools = unreal.AssetToolsHelpers.get_asset_tools()
  factory = unreal.BlueprintFactory()
  factory.set_editor_property("parent_class", unreal.Actor)

  asset = tools.create_asset(
    ASSET_NAME,
    ASSET_DIR,
    unreal.Blueprint,
    factory,
  )
  if asset is None:
    unreal.log_error("[InteractableStub] create_asset failed")
    return

  unreal.EditorAssetLibrary.save_loaded_asset(asset)
  unreal.log(f"[InteractableStub] created {ASSET_PATH}")


if __name__ == "__main__":
  main()
