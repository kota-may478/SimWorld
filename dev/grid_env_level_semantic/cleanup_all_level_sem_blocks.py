#!/usr/bin/env python3
"""Remove all live level_sem_block_* actors from PIE (gentle destroy)."""
from __future__ import annotations

import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import grid_env_level_semantic as lvl  # noqa: E402
import level_camera_probe as lcp  # noqa: E402

PREFIX = lvl.BLOCK_ACTOR_PREFIX
BATCH_PAUSE_EVERY = 8


def main() -> int:
    if not lcp.wait_for_ue_port(60.0):
        print("ERROR: UnrealCV not reachable", file=sys.stderr)
        return 2
    ucv, _ = g10k.ensure_connection()
    live = sorted(
        n for n in geh.actor_names(ucv) if n.startswith(f"{PREFIX}_")
    )
    print(f"[CleanupAll] removing {len(live)} {PREFIX}_* actors ...")
    if not live:
        print("[CleanupAll] done (nothing to remove)")
        return 0
    for i, name in enumerate(reversed(live), start=1):
        lvl._gentle_destroy_level_actor(ucv, name)
        if i % BATCH_PAUSE_EVERY == 0:
            time.sleep(lvl.DESTROY_BATCH_PAUSE_S)
    lvl.prepare_spawn_session(ucv)
    settle = lvl._scaled_cleanup_settle_s(len(live))
    print(f"[CleanupAll] settle {settle:.1f}s ...")
    time.sleep(settle)
    print("[CleanupAll] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
