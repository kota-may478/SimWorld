#!/usr/bin/env python3
"""Mount each SimWorld pak and probe humanoid BP (diagnostic)."""
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
from mount_simworld_runtime_paks_pie import _mount_one_pak  # noqa: E402

PAK_SRC = os.environ.get(
    "SIMWORLD_PAK_SRC",
    r"C:\SimWorldServer\SimWorld\Content\Paks",
)
PROBE_NAME = "__probe_human_all__"


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    names = sorted(
        p.name
        for p in Path(PAK_SRC.replace("\\", "/")).glob("pakchunk*.pak")
        if p.is_file()
    )
    # Path on WSL can't read Windows paks dir — use fixed list from Windows listing
    if not names:
        names = [
            "pakchunk0-Windows.pak",
            "pakchunk1000-Windows.pak",
            "pakchunk1001-Windows.pak",
            "pakchunk1001optional-Windows.pak",
            "pakchunk2005-Windows.pak",
            "pakchunk2007-Windows.pak",
            "pakchunk9001-Windows.pak",
            "pakchunk9001optional-Windows.pak",
            "pakchunk9002-Windows.pak",
        ]
    for name in names:
        path = os.path.join(PAK_SRC, name)
        mounted = _mount_one_pak(ucv, path, name)
        if not mounted:
            print(f"SKIP mount {name}")
            continue
        geh.destroy_actor_safely(ucv, PROBE_NAME)
        ok = geh.spawn_bp(ucv, geh.HUMAN_BP, PROBE_NAME, timeout_s=30.0)
        print(f"{'HUMANOID_OK' if ok else 'humanoid_fail'} after {name}")
        if ok:
            geh.destroy_actor_safely(ucv, PROBE_NAME)
            return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
