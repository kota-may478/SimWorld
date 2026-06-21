#!/usr/bin/env python3
"""Quick Humanoid on-floor spawn test (no SpotDog navigation)."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT, ROOT / "dev" / "grid_env_10k", ROOT / "dev" / "grid_env_hri"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_10k_pie_patrol as patrol  # noqa: E402

_scripts = Path(__file__).resolve().parent / "scripts"
if str(_scripts) not in sys.path:
    sys.path.insert(0, str(_scripts))
from mount_simworld_runtime_paks_pie import mount_paks  # noqa: E402


def main() -> int:
    ucv, communicator = g10k.ensure_connection()
    if not mount_paks(ucv):
        print("[Pak] mount failed — SpotDog/Humanoid BPs may be missing")
    patrol.prepare_pie_rerun(ucv)
    name = patrol.spawn_humanoid_at(communicator, ucv, patrol.HUMAN_CELL)
    ok, _, xy, feet_z = patrol.verify_humanoid_after_patrol(
        ucv, communicator, patrol.HUMAN_CELL, name
    )
    print(f"humanoid={name} on_floor={ok} xy={xy} feet_z={feet_z}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
