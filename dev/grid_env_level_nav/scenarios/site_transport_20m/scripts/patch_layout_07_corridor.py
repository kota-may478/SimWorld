#!/usr/bin/env python3
"""Widen leg2 corridor for layout_07 by nudging SW props off the return path."""

from __future__ import annotations

import json
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[3]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

from paths import site_transport_registry_path  # noqa: E402
from zones import sw_cluster_rect_from_points  # noqa: E402

REGISTRY_PATH = site_transport_registry_path("layout_07")

NUDGES = {
    "site20_prop_000": (270.0, 760.0),  # portapotty — off y≈900 chokepoint
    "site20_prop_004": (600.0, 860.0),  # watertank — opens gap near (388,957)
}


def main() -> None:
    data = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
    sw_points: list[tuple[float, float]] = []
    for prop in data["props"]:
        slot = prop["slot_id"]
        if slot in NUDGES:
            prop["local_xy_cm"] = list(NUDGES[slot])
        if prop.get("cluster_id") in {"facilities_sw", "equipment_sw"}:
            xy = prop["local_xy_cm"]
            sw_points.append((float(xy[0]), float(xy[1])))

    sw_rect = sw_cluster_rect_from_points(sw_points)
    for zone in data["forbidden_zones"]:
        if zone["zone_id"] == "sw_prop_cluster":
            zone["rect_local_cm"] = list(sw_rect)
            zone["note"] = (
                "SW quadrant prop cluster (auto bounds; layout_07 corridor patch)"
            )

    REGISTRY_PATH.write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"Patched {REGISTRY_PATH}")
    print(f"sw_prop_cluster rect={list(sw_rect)}")


if __name__ == "__main__":
    main()
