#!/usr/bin/env python3
"""Zone ID → grid cells for L1 semantic layer (Room D closure, etc.)."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from work_region import DEFAULT_RESOLUTION_CM, local_rect_to_cells

GridCell = Tuple[int, int]


@dataclass
class ZoneDefinition:
    cells: List[GridCell]
    default_cost: float = 1.0
    closed_cost: float = 1.0e9
    note: str = ""

    def cell_set(self) -> set[GridCell]:
        return set(self.cells)


@dataclass
class ZoneRegistry:
    zones: Dict[str, ZoneDefinition] = field(default_factory=dict)
    resolution_cm: float = DEFAULT_RESOLUTION_CM

    @classmethod
    def load(cls, path: Path | str) -> ZoneRegistry:
        path = Path(path)
        raw = json.loads(path.read_text(encoding="utf-8"))
        resolution_cm = float(raw.get("resolution_cm", DEFAULT_RESOLUTION_CM))
        zones: Dict[str, ZoneDefinition] = {}
        zone_blob = raw.get("zones", raw)
        skip_keys = {"resolution_cm", "version", "note", "zones"}
        for zone_id, body in zone_blob.items():
            if zone_id in skip_keys:
                continue
            if not isinstance(body, dict) or "cells" not in body:
                continue
            cells = [tuple(int(c) for c in pair) for pair in body["cells"]]
            zones[zone_id] = ZoneDefinition(
                cells=cells,
                default_cost=float(body.get("default_cost", 1.0)),
                closed_cost=float(body.get("closed_cost", body.get("default_cost", 1.0e9))),
                note=str(body.get("note", "")),
            )
        return cls(zones=zones, resolution_cm=resolution_cm)

    def save(self, path: Path | str) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "resolution_cm": self.resolution_cm,
            "zones": {
                zid: {
                    "cells": [list(c) for c in zdef.cells],
                    "default_cost": zdef.default_cost,
                    "closed_cost": zdef.closed_cost,
                    "note": zdef.note,
                }
                for zid, zdef in sorted(self.zones.items())
            },
        }
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def add_rect_zone(
        self,
        zone_id: str,
        lx0: float,
        ly0: float,
        lx1: float,
        ly1: float,
        *,
        default_cost: float = 1.0,
        closed_cost: float = 1.0e9,
        note: str = "",
    ) -> ZoneDefinition:
        cells = local_rect_to_cells(lx0, ly0, lx1, ly1, self.resolution_cm)
        zdef = ZoneDefinition(
            cells=cells,
            default_cost=default_cost,
            closed_cost=closed_cost,
            note=note,
        )
        self.zones[zone_id] = zdef
        return zdef

    def get(self, zone_id: str) -> ZoneDefinition:
        if zone_id not in self.zones:
            raise KeyError(f"unknown zone: {zone_id!r}")
        return self.zones[zone_id]

    def cells_for(self, zone_id: str) -> Iterable[GridCell]:
        return self.get(zone_id).cells
