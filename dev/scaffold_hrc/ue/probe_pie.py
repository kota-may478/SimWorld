#!/usr/bin/env python3
"""Print UnrealCV liveness and current map actors (PIE required)."""

from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
for path in (str(ROOT), str(PKG), str(ROOT / "dev" / "grid_env_hri")):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402


def main() -> int:
    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: UnrealCV ping")
        return 1
    version = ucv.client.request("vget /version")
    print("version", version)
    names = geh.actor_names(ucv)
    print("n_actors", len(names))
    for name in names:
        if "Level" in name or "Spot" in name or "Scaffold" in name or "Nav" in name:
            print(" ", name)
    print("has_spot", any("Spot" in n or "GridEnv" in n for n in names))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
