#!/usr/bin/env python3
"""
Zone label catalog: name → region in local/world UE coordinates.

Cells are derived at runtime for the active L0 resolution (no pre-baked gx/gy).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, Iterable, List, Literal, Optional, Tuple, Union

from level_coords import local_xy_to_world, world_xy_to_local
from work_region import local_rect_to_cells

GridCell = Tuple[int, int]
RectLocal = Tuple[float, float, float, float]  # lx0, ly0, lx1, ly1 cm
RectWorld = Tuple[float, float, float, float]  # wx0, wy0, wx1, wy1 cm


class ZoneKind(str, Enum):
    RECT_LOCAL = "rect_local"
    RECT_WORLD = "rect_world"
    CELLS = "cells"  # legacy: explicit gx/gy list


@dataclass
class ZoneRegion:
    """Geometry before grid rasterization."""

    kind: ZoneKind
    rect_local: Optional[RectLocal] = None
    rect_world: Optional[RectWorld] = None
    cells: Optional[List[GridCell]] = None

    def to_cells(self, resolution_cm: float) -> List[GridCell]:
        if self.kind == ZoneKind.CELLS and self.cells is not None:
            return list(self.cells)
        if self.kind == ZoneKind.RECT_LOCAL and self.rect_local is not None:
            lx0, ly0, lx1, ly1 = self.rect_local
            return local_rect_to_cells(lx0, ly0, lx1, ly1, resolution_cm)
        if self.kind == ZoneKind.RECT_WORLD and self.rect_world is not None:
            wx0, wy0, wx1, wy1 = self.rect_world
            lx0, ly0 = world_xy_to_local(min(wx0, wx1), min(wy0, wy1))
            lx1, ly1 = world_xy_to_local(max(wx0, wx1), max(wy0, wy1))
            return local_rect_to_cells(lx0, ly0, lx1, ly1, resolution_cm)
        return []


@dataclass
class ZoneCatalogEntry:
    zone_id: str
    region: ZoneRegion
    default_cost: float = 1.0
    closed_cost: float = 1.0e9
    note: str = ""
    tags: List[str] = field(default_factory=list)

    def cells_at(self, resolution_cm: float) -> List[GridCell]:
        return self.region.to_cells(resolution_cm)


@dataclass
class ZoneCatalog:
    """Picklist: RoomA, RoomB, AreaA, … → UE coordinates (local or world rects)."""

    entries: Dict[str, ZoneCatalogEntry] = field(default_factory=dict)
    catalog_note: str = ""

    @classmethod
    def load(cls, path: Path | str) -> ZoneCatalog:
        path = Path(path)
        raw = json.loads(path.read_text(encoding="utf-8"))
        catalog = cls(catalog_note=str(raw.get("note", "")))
        zones = raw.get("zones", raw)
        skip = {"version", "note", "zones", "resolution_cm"}
        for zone_id, body in zones.items():
            if zone_id in skip or not isinstance(body, dict):
                continue
            region = _parse_region(body)
            catalog.entries[zone_id] = ZoneCatalogEntry(
                zone_id=zone_id,
                region=region,
                default_cost=float(body.get("default_cost", 1.0)),
                closed_cost=float(body.get("closed_cost", body.get("default_cost", 1.0e9))),
                note=str(body.get("note", "")),
                tags=list(body.get("tags", [])),
            )
        return catalog

    def save(self, path: Path | str) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 2,
            "note": self.catalog_note,
            "zones": {
                zid: _entry_to_json(entry)
                for zid, entry in sorted(self.entries.items())
            },
        }
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def add_rect_local(
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
        tags: Optional[List[str]] = None,
    ) -> ZoneCatalogEntry:
        entry = ZoneCatalogEntry(
            zone_id=zone_id,
            region=ZoneRegion(
                kind=ZoneKind.RECT_LOCAL,
                rect_local=(lx0, ly0, lx1, ly1),
            ),
            default_cost=default_cost,
            closed_cost=closed_cost,
            note=note,
            tags=tags or [],
        )
        self.entries[zone_id] = entry
        return entry

    def add_rect_world(
        self,
        zone_id: str,
        wx0: float,
        wy0: float,
        wx1: float,
        wy1: float,
        **kwargs,
    ) -> ZoneCatalogEntry:
        entry = ZoneCatalogEntry(
            zone_id=zone_id,
            region=ZoneRegion(
                kind=ZoneKind.RECT_WORLD,
                rect_world=(wx0, wy0, wx1, wy1),
            ),
            default_cost=kwargs.get("default_cost", 1.0),
            closed_cost=kwargs.get("closed_cost", 1.0e9),
            note=kwargs.get("note", ""),
            tags=kwargs.get("tags", []),
        )
        self.entries[zone_id] = entry
        return entry

    def list_zones(self) -> List[str]:
        return sorted(self.entries.keys())

    def picklist_table(self, resolution_cm: float) -> List[dict]:
        """Human-readable summary for verification."""
        rows: List[dict] = []
        for zid in self.list_zones():
            e = self.entries[zid]
            row: dict = {
                "zone_id": zid,
                "kind": e.region.kind.value,
                "default_cost": e.default_cost,
                "closed_cost": e.closed_cost,
                "note": e.note,
                "tags": e.tags,
                "cell_count": len(e.cells_at(resolution_cm)),
            }
            if e.region.rect_local is not None:
                row["local_xy_cm"] = list(e.region.rect_local)
            if e.region.rect_world is not None:
                row["world_xy_cm"] = list(e.region.rect_world)
            rows.append(row)
        return rows


def _parse_region(body: dict) -> ZoneRegion:
    kind_str = body.get("kind", "")
    if kind_str == ZoneKind.RECT_LOCAL.value or "local_xy_cm" in body:
        r = body["local_xy_cm"]
        if len(r) == 2:
            (lx0, ly0), (lx1, ly1) = r
        else:
            lx0, ly0, lx1, ly1 = r
        return ZoneRegion(kind=ZoneKind.RECT_LOCAL, rect_local=(lx0, ly0, lx1, ly1))
    if kind_str == ZoneKind.RECT_WORLD.value or "world_xy_cm" in body:
        r = body["world_xy_cm"]
        if len(r) == 2:
            (wx0, wy0), (wx1, wy1) = r
        else:
            wx0, wy0, wx1, wy1 = r
        return ZoneRegion(kind=ZoneKind.RECT_WORLD, rect_world=(wx0, wy0, wx1, wy1))
    # legacy explicit cells
    if "cells" in body:
        cells = [tuple(int(c) for c in pair) for pair in body["cells"]]
        return ZoneRegion(kind=ZoneKind.CELLS, cells=cells)
    raise ValueError(f"zone entry missing geometry: {body!r}")


def _entry_to_json(entry: ZoneCatalogEntry) -> dict:
    out: dict = {
        "kind": entry.region.kind.value,
        "default_cost": entry.default_cost,
        "closed_cost": entry.closed_cost,
        "note": entry.note,
    }
    if entry.tags:
        out["tags"] = entry.tags
    if entry.region.rect_local is not None:
        lx0, ly0, lx1, ly1 = entry.region.rect_local
        out["local_xy_cm"] = [[lx0, ly0], [lx1, ly1]]
    elif entry.region.rect_world is not None:
        wx0, wy0, wx1, wy1 = entry.region.rect_world
        out["world_xy_cm"] = [[wx0, wy0], [wx1, wy1]]
    elif entry.region.cells is not None:
        out["cells"] = [list(c) for c in entry.region.cells]
    return out


def catalog_to_zone_registry(catalog: ZoneCatalog, resolution_cm: float):
    """Build runtime ZoneRegistry (cells materialized) for LayeredCostmap."""
    from zone_registry import ZoneDefinition, ZoneRegistry

    reg = ZoneRegistry(resolution_cm=resolution_cm)
    for zid, entry in catalog.entries.items():
        cells = entry.cells_at(resolution_cm)
        reg.zones[zid] = ZoneDefinition(
            cells=cells,
            default_cost=entry.default_cost,
            closed_cost=entry.closed_cost,
            note=entry.note,
        )
    return reg
