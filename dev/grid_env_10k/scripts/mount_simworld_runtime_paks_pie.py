#!/usr/bin/env python3
"""Mount SimWorldServer paks via UnrealCV during PIE (after UnrealCV plugin rebuild)."""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for p in (ROOT, ROOT / "dev" / "grid_env_10k", ROOT / "dev" / "grid_env_hri"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402

PAK_NAMES = (
    "pakchunk1000-Windows.pak",
    "pakchunk0-Windows.pak",
)
PAK_SRC = os.environ.get(
    "SIMWORLD_PAK_SRC",
    r"C:\SimWorldServer\SimWorld\Content\Paks",
)
_PAKS_MOUNTED = False
_ROBOT_PROBE_OK = False


def _mount_one_pak(ucv, path: str, name: str) -> bool:
    path = path.replace("/", "\\")
    for cmd in (
        f"vset /action/mount_pak {path}",
        f'vrun Mount "{path}"',
    ):
        res = geh._ue_request(ucv, cmd, timeout_s=120.0)
        text = str(res).strip() if res is not None else ""
        if text.lower().startswith("error") or "no handler" in text.lower():
            print(f"[Pak] try {cmd[:48]}... -> {text}")
            continue
        print(f"[Pak] OK {name}: {text}")
        return True
    print(f"[Pak] FAIL {name}: all mount methods failed")
    return False


def mount_paks(ucv, *, force: bool = False) -> bool:
    global _PAKS_MOUNTED
    if _PAKS_MOUNTED and not force:
        print("[Pak] skip remount (already mounted this session)")
        return True
    ok_all = True
    for name in PAK_NAMES:
        path = os.path.join(PAK_SRC, name)
        if not _mount_one_pak(ucv, path, name):
            ok_all = False
    if ok_all:
        _PAKS_MOUNTED = True
    return ok_all


def probe_robot_spawn(ucv, *, force: bool = False) -> bool:
    global _ROBOT_PROBE_OK
    if _ROBOT_PROBE_OK and not force:
        print("[Robot] skip probe (already OK this session)")
        return True
    probe_name = "__GridEnv_SpotRobot_probe__"
    geh.destroy_if_exists(ucv, probe_name)
    ok = geh.spawn_bp(ucv, geh.ROBOT_BP, probe_name, timeout_s=60.0)
    if ok:
        geh.destroy_if_exists(ucv, probe_name)
        _ROBOT_PROBE_OK = True
    return ok


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    if not ucv.client.isconnected():
        print("UnrealCV not connected — start PIE first.")
        return 1
    if not mount_paks(ucv):
        print(
            "Mount failed. Rebuild UnrealCV plugin in UE (ObjectHandler MountPak), "
            "restart Editor, PIE again."
        )
        return 1
    probe = probe_robot_spawn(ucv)
    print(f"robot_bp probe: {'OK' if probe else 'FAIL'}")
    return 0 if probe else 1


if __name__ == "__main__":
    raise SystemExit(main())
