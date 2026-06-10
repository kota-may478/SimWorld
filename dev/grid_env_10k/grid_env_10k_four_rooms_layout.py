#!/usr/bin/env python3
"""Four-room grid layout inside (1,1)-(30,30) with pillar at (10,10).

Rooms (interior walkable cells, excluding partition lines):
  SW — corner (1,1) on outer wall
  SE — corner (30,1)
  NW — corner (1,30)
  NE — corner (30,30)

Connectivity (doors only):
  SW <-> SE, SW <-> NW, SE <-> NE
  NE is reachable from SE only (not from NW directly).

Door width: 90 cm = 3 blocks (30 cm each).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterator, List, Optional, Set, Tuple

BlockIndex = Tuple[int, int]

REGION_GX0 = 1
REGION_GY0 = 1
REGION_GX1 = 30
REGION_GY1 = 30

PILLAR_CELL: BlockIndex = (10, 10)
PARTITION_GX = 10
PARTITION_GY = 10

DOOR_WIDTH_CM = 90.0
BLOCK_SIZE_CM = 30.0
DOOR_WIDTH_CELLS = int(round(DOOR_WIDTH_CM / BLOCK_SIZE_CM))

# Doors centred on each internal wall segment (3 cells = 90 cm).
SW_SE_DOOR_GY: Tuple[int, ...] = (5, 6, 7)
SW_NW_DOOR_GX: Tuple[int, ...] = (5, 6, 7)
SE_NE_DOOR_GX: Tuple[int, ...] = (18, 19, 20)

ENTITY_GOAL_CELL: BlockIndex = (20, 20)
ROBOT_START_CELL: BlockIndex = (5, 5)

RoomId = str

REQUIRED_ROOM_ADJACENCY: Set[Tuple[RoomId, RoomId]] = {
    ("SW", "SE"),
    ("SE", "SW"),
    ("SW", "NW"),
    ("NW", "SW"),
    ("SE", "NE"),
    ("NE", "SE"),
}



@dataclass(frozen=True)
class FourRoomsLayout:
    """Blocking sets for UE (solid) vs costmap (walls + pillar only)."""

    region_cells: frozenset[BlockIndex]
    wall_cells: frozenset[BlockIndex]
    entity_cells: frozenset[BlockIndex]
    ue_solid_cells: frozenset[BlockIndex]
    costmap_lethal_cells: frozenset[BlockIndex]
    walkable_cells: frozenset[BlockIndex]


def iter_rectangle_indices(
    gx0: int,
    gy0: int,
    gx1: int,
    gy1: int,
) -> Iterator[BlockIndex]:
    lo_gx, hi_gx = (gx0, gx1) if gx0 <= gx1 else (gx1, gx0)
    lo_gy, hi_gy = (gy0, gy1) if gy0 <= gy1 else (gy1, gy0)
    for gx in range(lo_gx, hi_gx + 1):
        for gy in range(lo_gy, hi_gy + 1):
            yield gx, gy


def iter_rectangle_perimeter(
    gx0: int,
    gy0: int,
    gx1: int,
    gy1: int,
) -> Iterator[BlockIndex]:
    lo_gx, hi_gx = (gx0, gx1) if gx0 <= gx1 else (gx1, gx0)
    lo_gy, hi_gy = (gy0, gy1) if gy0 <= gy1 else (gy1, gy0)
    for gx in range(lo_gx, hi_gx + 1):
        yield gx, lo_gy
        if hi_gy != lo_gy:
            yield gx, hi_gy
    for gy in range(lo_gy + 1, hi_gy):
        yield lo_gx, gy
        if hi_gx != lo_gx:
            yield hi_gx, gy


def _door_cells_on_vertical(gx: int, gy_values: Tuple[int, ...]) -> Set[BlockIndex]:
    return {(gx, gy) for gy in gy_values}


def _door_cells_on_horizontal(gy: int, gx_values: Tuple[int, ...]) -> Set[BlockIndex]:
    return {(gx, gy) for gx in gx_values}


def build_wall_cells() -> Set[BlockIndex]:
    """Outer box + internal cross walls with door gaps (pillar handled separately)."""
    walls: Set[BlockIndex] = set(iter_rectangle_perimeter(REGION_GX0, REGION_GY0, REGION_GX1, REGION_GY1))

    # Vertical partition gx=10, south segment (SW | SE).
    for gy in range(REGION_GY0 + 1, PARTITION_GY):
        walls.add((PARTITION_GX, gy))
    walls -= _door_cells_on_vertical(PARTITION_GX, SW_SE_DOOR_GY)

    # Vertical partition gx=10, north segment (NW | NE) — no door.
    for gy in range(PARTITION_GY + 1, REGION_GY1):
        walls.add((PARTITION_GX, gy))

    # Horizontal partition gy=10, west segment (SW | NW).
    for gx in range(REGION_GX0 + 1, PARTITION_GX):
        walls.add((gx, PARTITION_GY))
    walls -= _door_cells_on_horizontal(PARTITION_GY, SW_NW_DOOR_GX)

    # Horizontal partition gy=10, east segment (SE | NE).
    for gx in range(PARTITION_GX + 1, REGION_GX1):
        walls.add((gx, PARTITION_GY))
    walls -= _door_cells_on_horizontal(PARTITION_GY, SE_NE_DOOR_GX)

    return walls


def room_of_cell(gx: int, gy: int) -> Optional[RoomId]:
    if gx == PARTITION_GX or gy == PARTITION_GY or (gx, gy) == PILLAR_CELL:
        return None
    if REGION_GX0 < gx < PARTITION_GX and REGION_GY0 < gy < PARTITION_GY:
        return "SW"
    if PARTITION_GX < gx < REGION_GX1 and REGION_GY0 < gy < PARTITION_GY:
        return "SE"
    if REGION_GX0 < gx < PARTITION_GX and PARTITION_GY < gy < REGION_GY1:
        return "NW"
    if PARTITION_GX < gx < REGION_GX1 and PARTITION_GY < gy < REGION_GY1:
        return "NE"
    return None


def build_four_rooms_layout(
    *,
    entity_cell: BlockIndex = ENTITY_GOAL_CELL,
) -> FourRoomsLayout:
    region_cells = frozenset(
        iter_rectangle_indices(REGION_GX0, REGION_GY0, REGION_GX1, REGION_GY1)
    )
    wall_cells = frozenset(build_wall_cells() | {PILLAR_CELL})
    entity_cells = frozenset({entity_cell})
    ue_solid_cells = frozenset(set(wall_cells) | set(entity_cells))
    costmap_lethal_cells = wall_cells
    walkable_cells = frozenset(
        cell for cell in region_cells if cell not in ue_solid_cells
    )
    return FourRoomsLayout(
        region_cells=region_cells,
        wall_cells=wall_cells,
        entity_cells=entity_cells,
        ue_solid_cells=ue_solid_cells,
        costmap_lethal_cells=costmap_lethal_cells,
        walkable_cells=walkable_cells,
    )


def _neighbors4(gx: int, gy: int) -> List[BlockIndex]:
    return [(gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)]


def bfs_room_reachability(layout: FourRoomsLayout, start: BlockIndex) -> Set[RoomId]:
    if start not in layout.walkable_cells:
        return set()
    seen: Set[BlockIndex] = {start}
    rooms: Set[RoomId] = set()
    start_room = room_of_cell(*start)
    if start_room:
        rooms.add(start_room)
    queue: deque[BlockIndex] = deque([start])
    while queue:
        gx, gy = queue.popleft()
        for nx, ny in _neighbors4(gx, gy):
            cell = (nx, ny)
            if cell not in layout.walkable_cells or cell in seen:
                continue
            seen.add(cell)
            room = room_of_cell(nx, ny)
            if room:
                rooms.add(room)
            queue.append(cell)
    return rooms


def path_exists(layout: FourRoomsLayout, start: BlockIndex, goal: BlockIndex) -> bool:
    if start not in layout.walkable_cells or goal not in layout.walkable_cells:
        return False
    if start == goal:
        return True
    seen: Set[BlockIndex] = {start}
    queue: deque[BlockIndex] = deque([start])
    while queue:
        gx, gy = queue.popleft()
        for nx, ny in _neighbors4(gx, gy):
            cell = (nx, ny)
            if cell not in layout.walkable_cells or cell in seen:
                continue
            if cell == goal:
                return True
            seen.add(cell)
            queue.append(cell)
    return False


def validate_room_adjacency(layout: FourRoomsLayout) -> List[str]:
    """Return list of violation messages (empty if OK)."""
    errors: List[str] = []
    probes: Dict[RoomId, BlockIndex] = {
        "SW": (3, 3),
        "SE": (20, 3),
        "NW": (3, 20),
        "NE": (19, 20),
    }
    reachable_from: Dict[RoomId, Set[RoomId]] = {}
    for room, cell in probes.items():
        if cell not in layout.walkable_cells:
            errors.append(f"probe cell for {room} not walkable: {cell}")
            continue
        reachable_from[room] = bfs_room_reachability(layout, cell)

    for required_src, required_dst in REQUIRED_ROOM_ADJACENCY:
        if required_src in reachable_from and required_dst not in reachable_from[required_src]:
            errors.append(f"missing required adjacency {required_src} -> {required_dst}")

    # North segment of gx=10 (NW|NE) must stay fully walled — no NW<->NE door.
    for gy in range(PARTITION_GY + 1, REGION_GY1):
        if (PARTITION_GX, gy) not in layout.wall_cells:
            errors.append(f"missing north partition wall at ({PARTITION_GX}, {gy})")

    if not path_exists(layout, ROBOT_START_CELL, (19, 20)):
        errors.append(
            f"no path from start {ROBOT_START_CELL} to NE approach (19, 20)"
        )

    door_span_cm = DOOR_WIDTH_CELLS * BLOCK_SIZE_CM
    if not math.isclose(door_span_cm, DOOR_WIDTH_CM, rel_tol=0.0, abs_tol=1e-6):
        errors.append(f"door width mismatch: {door_span_cm} cm != {DOOR_WIDTH_CM} cm")

    return errors
