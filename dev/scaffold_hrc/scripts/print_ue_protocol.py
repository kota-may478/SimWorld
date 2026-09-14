#!/usr/bin/env python3
"""Print UE erect-protocol lengths (no PIE)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from scene.erect_plan import keepout_sample_item_count, select_erect_sequence
from scene.geometry import STAGE1_GEOM
from scene.erect_plan import build_erect_sequence

full = build_erect_sequence(STAGE1_GEOM)
print(
    json.dumps(
        {
            "smoke": len(select_erect_sequence("smoke")),
            "smin": len(select_erect_sequence("smin")),
            "full": len(select_erect_sequence("full")),
            "first_keepout_count": keepout_sample_item_count(full),
            "smin_kinds": sorted({i.kind for i in select_erect_sequence("smin")}),
        }
    )
)
