#!/usr/bin/env python3
"""Mount SimWorldServer runtime paks in UE Editor (best-effort).

UE 5.3 Editor Python has no PakFileFunctionLibrary. This script tries the
engine console command `Mount`. If that fails, rebuild UnrealCV (MountPak command)
and use mount_simworld_runtime_paks_pie.py from WSL during PIE.

Run before PIE:
  Tools -> Execute Python Script -> this file
"""

from __future__ import annotations

import os

import unreal

PAK_NAMES = (
    "pakchunk1000-Windows.pak",
    "pakchunk0-Windows.pak",
)

DEFAULT_PAK_SRC_WIN = r"C:\SimWorldServer\SimWorld\Content\Paks"
VERIFY_ASSET = "/Game/Robot_Dog/Blueprint/BP_SpotRobot"


def _normalize(path: str) -> str:
    return os.path.normpath(path.replace("/", os.sep))


def _pak_source_dirs() -> list[str]:
    dirs: list[str] = []
    env_src = os.environ.get("SIMWORLD_PAK_SRC", "").strip()
    if env_src:
        dirs.append(_normalize(env_src))
    dirs.append(_normalize(DEFAULT_PAK_SRC_WIN))
    try:
        content = unreal.Paths.convert_relative_path_to_full(
            unreal.Paths.project_content_dir()
        )
        dirs.append(_normalize(os.path.join(content, "Paks")))
    except Exception:
        pass
    seen: set[str] = set()
    return [d for d in dirs if d and not (d in seen or seen.add(d))]  # type: ignore[func-returns-value]


def _resolve_pak_path(name: str) -> str | None:
    for base in _pak_source_dirs():
        candidate = _normalize(os.path.join(base, name))
        if os.path.isfile(candidate):
            unreal.log(f"[Pak] resolved {name} -> {candidate}")
            return candidate
    return None


def _editor_world():
    try:
        subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        if subsystem is not None:
            return subsystem.get_editor_world()
    except Exception:
        pass
    try:
        return unreal.EditorLevelLibrary.get_editor_world()
    except Exception:
        return None


def _try_console_mount(pak_path: str) -> None:
    world = _editor_world()
    if world is None:
        unreal.log_warning("[Pak] no editor world for console Mount")
        return
    for cmd in (f'Mount "{pak_path}"', f"Mount {pak_path}"):
        unreal.log(f"[Pak] console: {cmd}")
        unreal.SystemLibrary.execute_console_command(world, cmd, None)


def _verify_assets() -> bool:
    asset = unreal.load_asset(VERIFY_ASSET)
    ok = asset is not None
    unreal.log(f"[Pak] verify {VERIFY_ASSET}: {'OK' if ok else 'FAIL'}")
    return ok


def main() -> None:
    unreal.log(f"[Pak] search dirs: {_pak_source_dirs()}")
    if _verify_assets():
        unreal.log("[Pak] already loadable — skip mount")
        return

    missing = False
    for name in PAK_NAMES:
        path = _resolve_pak_path(name)
        if path is None:
            unreal.log_error(f"[Pak] not found: {name}")
            missing = True
            continue
        _try_console_mount(path)

    if _verify_assets():
        unreal.log("[Pak] done — BP_SpotRobot loadable. Start PIE, run WSL patrol.")
        return

    unreal.log_error(
        "[Pak] console Mount did not expose BP_SpotRobot.\n"
        "  1. Close Editor\n"
        "  2. Rebuild UnrealCV plugin (MountPak added to ObjectHandler.cpp)\n"
        "  3. Open Editor -> PIE on grid_100x100\n"
        "  4. WSL: python dev/grid_env_10k/scripts/mount_simworld_runtime_paks_pie.py\n"
        "  5. python dev/grid_env_10k/grid_env_10k_pie_patrol.py"
    )
    if missing:
        unreal.log_error(f"[Pak] expected paks under {DEFAULT_PAK_SRC_WIN}")


if __name__ == "__main__":
    main()
