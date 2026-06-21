#!/usr/bin/env python3
"""Try humanoid BP candidates after mounting paks."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for p in (ROOT, ROOT / "dev" / "grid_env_10k", ROOT / "dev" / "grid_env_hri"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from mount_simworld_runtime_paks_pie import mount_paks  # noqa: E402

CANDIDATES = (
    geh.HUMAN_BP,
    "/Game/TrafficSystem/Pedestrian/Base_Pedestrian.Base_Pedestrian_C",
    "/Game/Human_Avatar/DefaultCharacter/Blueprint/BP_Default_Character.BP_Default_Character_C",
)


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    mount_paks(ucv)
    for bp in CANDIDATES:
        name = "__probe_human__"
        geh.destroy_actor_safely(ucv, name)
        ok = geh.spawn_bp(ucv, bp, name, timeout_s=45.0)
        print(f"{'OK' if ok else 'FAIL'} {bp}")
        if ok:
            geh.destroy_actor_safely(ucv, name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
