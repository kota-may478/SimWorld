#!/usr/bin/env python3
"""Collect UE5 layout + climb evidence for paper condition cards (PIE required)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
NAV = ROOT / "dev" / "grid_env_level_nav"
GEH = ROOT / "dev" / "grid_env_hri"
DEPTH = ROOT / "dev" / "grid_env_depth_perception"
for path in (str(ROOT), str(NAV), str(GEH), str(DEPTH), str(PKG)):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from scene.field import STORAGE_XY_M, RAMP_RUN_M, ramp_yard_x_m  # noqa: E402
from ue.layout import STAGING_LOCAL_XY_CM  # noqa: E402
from ue.placement import ACTOR_PREFIX, CARRY_MAX_M, all_spawn_boxes  # noqa: E402
from paths import make_run_dir  # noqa: E402


def main() -> int:
    cards_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    out_dir = make_run_dir()
    boxes = all_spawn_boxes()
    extents = []
    for box in boxes:
        if box.kind == "marker":
            continue
        # native cube 0.30 m
        extent = tuple(s * 0.30 for s in box.scale)
        extents.append({"actor": box.actor, "kind": box.kind, "extent_m": extent})
    oversize = [e for e in extents if max(e["extent_m"]) > CARRY_MAX_M + 1e-6 and e["kind"] != "brace"]
    payload = {
        "staging_local_cm": list(STAGING_LOCAL_XY_CM),
        "storage_xy_m": list(STORAGE_XY_M),
        "ramp_yard_x_m": ramp_yard_x_m(),
        "ramp_run_m": RAMP_RUN_M,
        "n_spawn_boxes": len(boxes),
        "n_cheeks": sum(1 for b in boxes if "cheek_" in b.actor),
        "oversize_non_brace": oversize,
        "condition_cards": None,
        "ue_ping": False,
        "scaffold_actors": 0,
    }
    if cards_path and cards_path.is_file():
        payload["condition_cards"] = json.loads(cards_path.read_text(encoding="utf-8"))

    try:
        ucv, _ = ue_client_guard.prepare_ue_connection()
        if geh._ping_ucv(ucv):  # noqa: SLF001
            payload["ue_ping"] = True
            names = [n for n in geh.actor_names(ucv) if n.startswith(f"{ACTOR_PREFIX}_")]
            payload["scaffold_actors"] = len(names)
            payload["has_truck"] = f"{ACTOR_PREFIX}_truck" in names
            payload["has_both_cheeks"] = (
                f"{ACTOR_PREFIX}_cheek_L0_s_0" in names
                and f"{ACTOR_PREFIX}_cheek_L0_n_0" in names
            )
    except Exception as exc:
        payload["ue_error"] = str(exc)

    out = out_dir / "ue_field_snapshot.json"
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(out)
    print(json.dumps({k: payload[k] for k in ("ue_ping", "scaffold_actors", "n_spawn_boxes", "n_cheeks")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
