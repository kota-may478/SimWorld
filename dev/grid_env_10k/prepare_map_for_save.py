#!/usr/bin/env python3
"""マップ保存前: ログ相当の監査 → ランタイム Actor 削除 → 再監査。

SimWorld を起動した状態で実行する（empty.umap 等、Phase A 済みのレベル）。
環境変数 WAIT_UE_PORT_S（既定 180）で 127.0.0.1:9000 を待つ。
"""

from __future__ import annotations

import os
import socket
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k as g10k  # noqa: E402


def wait_for_ue_port(timeout_s: float) -> bool:
    host, port = "127.0.0.1", 9000
    deadline = time.monotonic() + timeout_s
    print(f"[prepare_map_for_save] waiting for {host}:{port} (up to {timeout_s:.0f}s) ...")
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2.0):
                print(f"[prepare_map_for_save] OK {host}:{port}")
                return True
        except OSError:
            time.sleep(2.0)
    print(f"[prepare_map_for_save] TIMEOUT {host}:{port}", file=sys.stderr)
    return False


def main() -> int:
    wait_s = float(os.environ.get("WAIT_UE_PORT_S", "180"))
    if not wait_for_ue_port(wait_s):
        print(
            "[prepare_map_for_save] Start SimWorld on Windows first, e.g.:\n"
            "  .\\SimWorld.exe -windowed /Game/Maps/empty.umap\n"
            "Then re-run: python prepare_map_for_save.py",
            file=sys.stderr,
        )
        return 2

    ucv, _comm = g10k.ensure_connection()
    after = g10k.prepare_runtime_actors_for_map_save(ucv)

    ok = True
    if not after.get("floor_present"):
        print("[prepare_map_for_save] FAIL: grid_floor_main missing", file=sys.stderr)
        ok = False
    block_count = int(after.get("block_count", 0))
    expected = int(after.get("expected_blocks", g10k.BLOCK_GRID_N ** 2))
    if block_count != expected:
        print(
            f"[prepare_map_for_save] FAIL: blocks {block_count}/{expected} "
            "(Phase A not loaded or SimWorld was restarted without blocks)",
            file=sys.stderr,
        )
        ok = False
    if after.get("humanoids") or after.get("robots") or after.get("demos"):
        print("[prepare_map_for_save] FAIL: runtime actors still present", file=sys.stderr)
        ok = False

    if ok:
        print(
            "[prepare_map_for_save] SUCCESS — level ready for UE Editor Step C.\n"
            "  Next: quit SimWorld.exe, open UE project, Save As grid_100x100"
        )
        return 0

    print(
        "[prepare_map_for_save] See SAVE_MAP_GRID_100x100.md. "
        "If blocks missing, re-run Phase A on empty.umap with SimWorld running.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
