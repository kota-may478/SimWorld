#!/usr/bin/env python3
"""grid_100x100 隅の小領域: 仮床 + wall/floor/air ラベル付きブロック敷き詰めデモ。

前提:
  - UE Editor で grid_100x100 を開き PIE 実行中
  - WSL: conda activate simworld

座標:
  - ブロック下面 = 配置高度の定義（CUBE_PIVOT_AT_CENTER=0 想定）
  - 仮床上面とブロック下面の隙間 = BLOCK_GAP_ABOVE_FLOOR_M (0.15 m)
  - 敷き詰め領域: 隅 10×10 (gx,gy = 1..10)
  - 仮床: その内側 6×6 (gx,gy = 1..6) — 残りは仮床外（air 想定）
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Literal, Optional, Tuple

BlockIndex = Tuple[int, int]
BlockMode = Literal["T", "F"]

def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
THIS_DIR = Path(__file__).resolve().parent
for p in (ROOT, GEH_DIR, G10K_DIR, THIS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from block_semantic_scan import (  # noqa: E402
    BlockSemantic,
    SEMANTIC_COLORS,
    scan_region_semantics,
)
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

# ---- 領域（隅: 仮床あり / なし を同一矩形に含む） ----
FILL_GX0, FILL_GY0 = 1, 1
FILL_GX1, FILL_GY1 = 10, 10
TEMP_FLOOR_GX0, TEMP_FLOOR_GY0 = 1, 1
TEMP_FLOOR_GX1, TEMP_FLOOR_GY1 = 6, 6

# ---- 高度 ----
BLOCK_GAP_ABOVE_FLOOR_M = 0.15
BLOCK_GAP_ABOVE_FLOOR_CM = BLOCK_GAP_ABOVE_FLOOR_M * 100.0
EXISTING_BLOCK_CLEARANCE_CM = 200.0
TEMP_FLOOR_THICKNESS_CM = 20.0

# ---- Actor 名 ----
TEMP_FLOOR_ACTOR = "sem_temp_floor"
DEMO_WALL_ACTOR = "sem_demo_wall"
BLOCK_ACTOR_PREFIX = "sem_block"
REGISTRY_PATH = THIS_DIR / ".semantic_layer_registry.json"

MAP_LAUNCH_ARG = "/Game/Maps/grid_100x100.umap"
BOX_BP = "/Game/CityDatabase/blueprints/BP_Box.BP_Box_C"


@dataclass(frozen=True)
class LayerGeometry:
    """Computed Z heights for the elevated demo layer [cm]."""

    existing_block_top_z_cm: float
    temp_floor_top_z_cm: float
    block_bottom_z_cm: float


@dataclass
class SemanticBlockRecord:
    gx: int
    gy: int
    semantic: BlockSemantic
    mode: BlockMode
    block_bottom_z_cm: float
    actor_name: str
    world_cm: Tuple[float, float, float]
    on_temp_floor: bool


@dataclass(frozen=True)
class SemanticLayerResult:
    geometry: LayerGeometry
    semantics: Dict[BlockIndex, BlockSemantic]
    blocks: Dict[str, SemanticBlockRecord]
    registry_path: Path


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


def cell_on_temp_floor(gx: int, gy: int) -> bool:
    return (
        TEMP_FLOOR_GX0 <= gx <= TEMP_FLOOR_GX1
        and TEMP_FLOOR_GY0 <= gy <= TEMP_FLOOR_GY1
    )


def block_actor_name(gx: int, gy: int) -> str:
    return f"{BLOCK_ACTOR_PREFIX}_{gx:03d}_{gy:03d}"


def cell_center_world_xy_cm(gx: int, gy: int) -> Tuple[float, float]:
    col = gx - 1
    row = gy - 1
    ox, oy = geh.MAP_ORIGIN_XY_CM
    x = ox + col * geh.CUBE_SIZE_CM + geh.CUBE_HALF_CM
    y = oy + row * geh.CUBE_SIZE_CM + geh.CUBE_HALF_CM
    return x, y


def compute_layer_geometry(*, floor_top_z_cm: Optional[float] = None) -> LayerGeometry:
    """Auto elevation: existing block top + clearance → temp floor → block gap."""
    top = geh.FLOOR_TOP_Z_CM if floor_top_z_cm is None else floor_top_z_cm
    existing_block_top = top + geh.CUBE_ON_FLOOR_EPS_CM + geh.CUBE_SIZE_CM
    temp_floor_top = existing_block_top + EXISTING_BLOCK_CLEARANCE_CM
    block_bottom = temp_floor_top + BLOCK_GAP_ABOVE_FLOOR_CM
    return LayerGeometry(
        existing_block_top_z_cm=existing_block_top,
        temp_floor_top_z_cm=temp_floor_top,
        block_bottom_z_cm=block_bottom,
    )


def block_bottom_to_actor_z(block_bottom_z_cm: float) -> float:
    if geh.CUBE_PIVOT_AT_CENTER:
        return block_bottom_z_cm + geh.CUBE_HALF_CM
    return block_bottom_z_cm


def actor_z_to_block_bottom(actor_z_cm: float) -> float:
    if geh.CUBE_PIVOT_AT_CENTER:
        return actor_z_cm - geh.CUBE_HALF_CM
    return actor_z_cm


def ensure_connection() -> Tuple[UnrealCV, object]:
    return g10k.ensure_connection()


def _rectangle_center_and_extent_cm(
    gx0: int,
    gy0: int,
    gx1: int,
    gy1: int,
) -> Tuple[float, float, float, float]:
    x0, y0 = cell_center_world_xy_cm(gx0, gy0)
    x1, y1 = cell_center_world_xy_cm(gx1, gy1)
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    n_x = abs(gx1 - gx0) + 1
    n_y = abs(gy1 - gy0) + 1
    ext_x = n_x * geh.CUBE_SIZE_CM
    ext_y = n_y * geh.CUBE_SIZE_CM
    return cx, cy, ext_x, ext_y


def spawn_temp_floor(
    ucv: UnrealCV,
    geometry: LayerGeometry,
) -> bool:
    """Spawn a kinematic platform covering TEMP_FLOOR_* cells."""
    geh.destroy_if_exists(ucv, TEMP_FLOOR_ACTOR)
    cx, cy, ext_x, ext_y = _rectangle_center_and_extent_cm(
        TEMP_FLOOR_GX0,
        TEMP_FLOOR_GY0,
        TEMP_FLOOR_GX1,
        TEMP_FLOOR_GY1,
    )
    half_t = TEMP_FLOOR_THICKNESS_CM / 2.0
    center_z = geometry.temp_floor_top_z_cm - half_t
    if not geh.spawn_bp(ucv, BOX_BP, TEMP_FLOOR_ACTOR):
        print(f"[TempFloor] spawn failed: {BOX_BP}")
        return False
    ucv.set_physics(TEMP_FLOOR_ACTOR, False)
    ucv.set_collision(TEMP_FLOOR_ACTOR, True)
    ucv.set_movable(TEMP_FLOOR_ACTOR, True)
    scale_x = ext_x / 100.0
    scale_y = ext_y / 100.0
    scale_z = TEMP_FLOOR_THICKNESS_CM / 100.0
    ucv.set_scale((scale_x, scale_y, scale_z), TEMP_FLOOR_ACTOR)
    ucv.set_location((cx, cy, center_z), TEMP_FLOOR_ACTOR)
    ucv.set_orientation((0.0, 0.0, 0.0), TEMP_FLOOR_ACTOR)
    print(
        f"[TempFloor] {TEMP_FLOOR_ACTOR} cells=({TEMP_FLOOR_GX0},{TEMP_FLOOR_GY0})"
        f"-({TEMP_FLOOR_GX1},{TEMP_FLOOR_GY1}) "
        f"top_z={geometry.temp_floor_top_z_cm:.1f}cm "
        f"center={geh._fmt_xyz((cx, cy, center_z))}"
    )
    return True


def spawn_demo_wall_obstacle(
    ucv: UnrealCV,
    gx: int,
    gy: int,
    geometry: LayerGeometry,
) -> bool:
    """One elevated box at block-bottom height to produce a 'wall' probe hit."""
    geh.destroy_if_exists(ucv, DEMO_WALL_ACTOR)
    x, y = cell_center_world_xy_cm(gx, gy)
    actor_z = block_bottom_to_actor_z(geometry.block_bottom_z_cm)
    if not geh.spawn_bp(ucv, BOX_BP, DEMO_WALL_ACTOR):
        return False
    ucv.set_physics(DEMO_WALL_ACTOR, False)
    ucv.set_collision(DEMO_WALL_ACTOR, True)
    ucv.set_movable(DEMO_WALL_ACTOR, True)
    ucv.set_scale((0.28, 0.28, 0.28), DEMO_WALL_ACTOR)
    ucv.set_location((x, y, actor_z), DEMO_WALL_ACTOR)
    print(
        f"[DemoWall] {DEMO_WALL_ACTOR} at cell ({gx},{gy}) "
        f"z_bottom={geometry.block_bottom_z_cm:.1f}cm"
    )
    return True


def _place_semantic_cube(
    ucv: UnrealCV,
    gx: int,
    gy: int,
    geometry: LayerGeometry,
    semantic: BlockSemantic,
    *,
    mode: BlockMode = "F",
) -> Optional[SemanticBlockRecord]:
    name = block_actor_name(gx, gy)
    x, y = cell_center_world_xy_cm(gx, gy)
    bottom_z = geometry.block_bottom_z_cm
    loc = (x, y, block_bottom_to_actor_z(bottom_z))
    geh.destroy_if_exists(ucv, name)
    if not geh.spawn_bp(ucv, geh.CUBE_BP, name):
        print(f"  warn: spawn failed {name}")
        return None
    blocking = mode == "T"
    ok = geh._place_cube_kinematic_on_floor(
        ucv,
        name,
        loc,
        blocking=blocking,
        apply_tint=False,
    )
    if not ok:
        return None
    try:
        ucv.set_color(name, list(SEMANTIC_COLORS[semantic]))
    except Exception as exc:
        print(f"  warn: set_color {name}: {exc}")
    return SemanticBlockRecord(
        gx=gx,
        gy=gy,
        semantic=semantic,
        mode=mode,
        block_bottom_z_cm=bottom_z,
        actor_name=name,
        world_cm=loc,
        on_temp_floor=cell_on_temp_floor(gx, gy),
    )


def fill_labeled_blocks(
    ucv: UnrealCV,
    semantics: Dict[BlockIndex, BlockSemantic],
    geometry: LayerGeometry,
    *,
    default_mode: BlockMode = "F",
    spawn_interval_s: float = 0.02,
) -> Dict[str, SemanticBlockRecord]:
    registry: Dict[str, SemanticBlockRecord] = {}
    cells = sorted(semantics.keys())
    total = len(cells)
    print(f"[Fill] placing {total} labeled blocks (mode={default_mode}) ...")
    for i, (gx, gy) in enumerate(cells, start=1):
        sem = semantics[(gx, gy)]
        rec = _place_semantic_cube(
            ucv,
            gx,
            gy,
            geometry,
            sem,
            mode=default_mode,
        )
        if rec is not None:
            registry[rec.actor_name] = rec
        if spawn_interval_s > 0:
            time.sleep(spawn_interval_s)
        if i % 20 == 0 or i == total:
            print(f"[Fill] {i}/{total} placed={len(registry)}")
    return registry


def save_registry(
    path: Path,
    geometry: LayerGeometry,
    semantics: Dict[BlockIndex, BlockSemantic],
    blocks: Dict[str, SemanticBlockRecord],
) -> None:
    payload = {
        "geometry": asdict(geometry),
        "fill_region": [FILL_GX0, FILL_GY0, FILL_GX1, FILL_GY1],
        "temp_floor_region": [TEMP_FLOOR_GX0, TEMP_FLOOR_GY0, TEMP_FLOOR_GX1, TEMP_FLOOR_GY1],
        "semantics": {f"{gx:03d}_{gy:03d}": sem for (gx, gy), sem in sorted(semantics.items())},
        "blocks": {name: asdict(rec) for name, rec in sorted(blocks.items())},
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[Registry] saved {path}")


def summarize_semantics(semantics: Dict[BlockIndex, BlockSemantic]) -> Dict[str, int]:
    counts: Dict[str, int] = {"wall": 0, "floor": 0, "air": 0}
    for sem in semantics.values():
        counts[sem] += 1
    return counts


def cleanup_semantic_layer(ucv: UnrealCV, blocks: Optional[Dict[str, SemanticBlockRecord]] = None) -> None:
    geh.destroy_if_exists(ucv, TEMP_FLOOR_ACTOR)
    geh.destroy_if_exists(ucv, DEMO_WALL_ACTOR)
    if blocks:
        for rec in blocks.values():
            geh.destroy_if_exists(ucv, rec.actor_name)
    prefix = f"{BLOCK_ACTOR_PREFIX}_"
    for name in geh.actor_names(ucv):
        if name.startswith(prefix):
            geh.destroy_if_exists(ucv, name)
    print("[Cleanup] semantic layer actors removed.")


def run_semantic_layer_demo(
    ucv: Optional[UnrealCV] = None,
    *,
    spawn_demo_wall: bool = True,
    demo_wall_cell: BlockIndex = (4, 4),
    default_block_mode: BlockMode = "F",
    cleanup_before: bool = True,
    save_path: Path = REGISTRY_PATH,
) -> SemanticLayerResult:
    """Full demo: temp floor → scan → labeled fill → registry save."""
    if ucv is None:
        ucv, _ = ensure_connection()
    if not ucv.client.isconnected():
        raise RuntimeError("UnrealCV not connected — start grid_100x100 PIE first.")

    if cleanup_before:
        cleanup_semantic_layer(ucv)

    floor_top = geh.resolve_floor_top_z_cm(ucv)
    geometry = compute_layer_geometry(floor_top_z_cm=floor_top)
    print(
        f"[Geometry] existing_block_top≈{geometry.existing_block_top_z_cm:.1f}cm "
        f"temp_floor_top={geometry.temp_floor_top_z_cm:.1f}cm "
        f"block_bottom={geometry.block_bottom_z_cm:.1f}cm "
        f"(gap={BLOCK_GAP_ABOVE_FLOOR_M:.2f}m)"
    )

    if not spawn_temp_floor(ucv, geometry):
        raise RuntimeError("temp floor spawn failed")

    if spawn_demo_wall:
        spawn_demo_wall_obstacle(ucv, demo_wall_cell[0], demo_wall_cell[1], geometry)

    cells = list(iter_rectangle_indices(FILL_GX0, FILL_GY0, FILL_GX1, FILL_GY1))
    print(
        f"[SemanticScan] region ({FILL_GX0},{FILL_GY0})-({FILL_GX1},{FILL_GY1}) "
        f"cells={len(cells)} ..."
    )
    semantics = scan_region_semantics(
        ucv,
        cells,
        cell_center_xy_cm_fn=cell_center_world_xy_cm,
        block_bottom_z_cm=geometry.block_bottom_z_cm,
        block_height_cm=geh.CUBE_SIZE_CM,
        progress_every=20,
    )
    counts = summarize_semantics(semantics)
    print(f"[SemanticScan] summary: {counts}")

    blocks = fill_labeled_blocks(
        ucv,
        semantics,
        geometry,
        default_mode=default_block_mode,
    )
    save_registry(save_path, geometry, semantics, blocks)

    return SemanticLayerResult(
        geometry=geometry,
        semantics=semantics,
        blocks=blocks,
        registry_path=save_path,
    )
