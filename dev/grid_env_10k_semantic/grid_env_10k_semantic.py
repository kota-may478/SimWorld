#!/usr/bin/env python3
"""grid_100x100 隅: 仮床 3×3 + 5×5 ブロックの floor/air ラベル付けテスト。

ラベル規則（ブロック配置前に各セルで評価）:
  z_initial = 仮床上面 + 0.15 m（ブロック下面の生成高度）
  1. z_initial で干渉あり → wall
  2. なし → z_lower = z_initial − 0.30 m で干渉あり → floor
  3. なし → air

テスト期待: 3×3 内 → floor×9、外周 16 マス → air×16、wall×0
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Dict, Iterator, List, Literal, Optional, Tuple

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
from block_semantic_scan import BlockSemantic, ObstacleBox, scan_region_semantics  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

# 5×5 敷き詰め、内側 3×3 が仮床
FILL_GX0, FILL_GY0 = 1, 1
FILL_GX1, FILL_GY1 = 5, 5
TEMP_FLOOR_GX0, TEMP_FLOOR_GY0 = 1, 1
TEMP_FLOOR_GX1, TEMP_FLOOR_GY1 = 3, 3

BLOCK_GAP_ABOVE_FLOOR_M = 0.15
BLOCK_GAP_ABOVE_FLOOR_CM = BLOCK_GAP_ABOVE_FLOOR_M * 100.0
EXISTING_BLOCK_CLEARANCE_CM = 200.0
TEMP_FLOOR_THICKNESS_CM = 20.0
DEFAULT_BLOCK_MODE: BlockMode = "F"
# 見た目: floor=透過(F), air/wall=実体(T)
SEMANTIC_VISUAL_MODES: Dict[BlockSemantic, BlockMode] = {
    "floor": "F",
    "air": "T",
    "wall": "T",
}
Z_PLACEMENT_TOL_CM = 5.0
BLOCK_SPAWN_INTERVAL_S = 0.28
BLOCK_SPAWN_BATCH_SIZE = 5
BLOCK_SPAWN_MAX_ATTEMPTS = 2
UE_RECONNECT_WAIT_S = 120.0
DESTROY_PACE_S = 0.12
DESTROY_GONE_TIMEOUT_S = 5.0
SPAWN_TIMEOUT_FIRST_S = 90.0
SPAWN_TIMEOUT_NEXT_S = 45.0
POST_PHASE_SETTLE_S = 0.5
POST_CLEANUP_SETTLE_S = 2.0
POST_TEMP_FLOOR_SETTLE_S = 2.0
POST_WARMUP_SETTLE_S = 1.5
POST_CUBE_SETTLE_S = 0.12
POST_RECONNECT_SETTLE_S = 2.0

TEMP_FLOOR_ACTOR = "sem_temp_floor"
WARMUP_CUBE_ACTOR = "sem_bp_warmup"
BLOCK_ACTOR_PREFIX = "sem_block"
REGISTRY_PATH = THIS_DIR / ".semantic_layer_registry.json"
TEMP_FLOOR_BP = geh.FLOOR_BP
TEMP_FLOOR_NATIVE_EXT_CM = 3000.0  # BP_Floor_30x30 = 30 m


@dataclass(frozen=True)
class LayerGeometry:
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


def iter_rectangle_indices(gx0: int, gy0: int, gx1: int, gy1: int) -> Iterator[BlockIndex]:
    lo_gx, hi_gx = (gx0, gx1) if gx0 <= gx1 else (gx1, gx0)
    lo_gy, hi_gy = (gy0, gy1) if gy0 <= gy1 else (gy1, gy0)
    for gx in range(lo_gx, hi_gx + 1):
        for gy in range(lo_gy, hi_gy + 1):
            yield gx, gy


def mode_for_semantic(semantic: BlockSemantic) -> BlockMode:
    return SEMANTIC_VISUAL_MODES[semantic]


def cell_on_temp_floor(gx: int, gy: int) -> bool:
    return (
        TEMP_FLOOR_GX0 <= gx <= TEMP_FLOOR_GX1
        and TEMP_FLOOR_GY0 <= gy <= TEMP_FLOOR_GY1
    )


def block_actor_name(gx: int, gy: int) -> str:
    return f"{BLOCK_ACTOR_PREFIX}_{gx:03d}_{gy:03d}"


def iter_semantic_actor_names() -> Iterator[str]:
    """Fixed sem_* names for this demo — never scan ``vget /objects`` (10k+ actors)."""
    yield TEMP_FLOOR_ACTOR
    yield WARMUP_CUBE_ACTOR
    for gx, gy in iter_rectangle_indices(FILL_GX0, FILL_GY0, FILL_GX1, FILL_GY1):
        yield block_actor_name(gx, gy)


def _ue_alive(ucv: UnrealCV) -> bool:
    return geh._ping_ucv(ucv)


def prepare_semantic_spawn_session(ucv: UnrealCV) -> None:
    """Lightweight UE prep — no full actor list (grid_100x100 safe)."""
    geh._prepare_ue_spawn(ucv)
    time.sleep(POST_PHASE_SETTLE_S)


def _destroy_semantic_actor(ucv: UnrealCV, name: str) -> None:
    """Destroy sem_* and wait until the logical name is free (prevents UObject rename crash)."""
    if not geh.actor_exists(ucv, name):
        return
    geh.destroy_actor_safely(ucv, name, max_attempts=3, timeout_s=12.0)
    geh.wait_until_actor_gone(ucv, name, timeout_s=DESTROY_GONE_TIMEOUT_S)
    time.sleep(DESTROY_PACE_S)


def _ensure_spawn_name_free(ucv: UnrealCV, name: str) -> bool:
    """spawn_bp_asset renames the new actor — stale names cause fatal UObject rename errors."""
    if not geh.actor_exists(ucv, name):
        return True
    print(f"[Spawn] clearing stale {name!r} before spawn_bp_asset ...")
    _destroy_semantic_actor(ucv, name)
    if geh.actor_exists(ucv, name):
        print(f"[Spawn] FAIL: {name!r} still present — skip spawn to avoid UE rename crash")
        return False
    return True


def cell_center_world_xy_cm(gx: int, gy: int) -> Tuple[float, float]:
    col, row = gx - 1, gy - 1
    ox, oy = geh.MAP_ORIGIN_XY_CM
    x = ox + col * geh.CUBE_SIZE_CM + geh.CUBE_HALF_CM
    y = oy + row * geh.CUBE_SIZE_CM + geh.CUBE_HALF_CM
    return x, y


def compute_layer_geometry(*, floor_top_z_cm: Optional[float] = None) -> LayerGeometry:
    top = geh.FLOOR_TOP_Z_CM if floor_top_z_cm is None else floor_top_z_cm
    existing_block_top = top + geh.CUBE_ON_FLOOR_EPS_CM + geh.CUBE_SIZE_CM
    temp_floor_top = existing_block_top + EXISTING_BLOCK_CLEARANCE_CM
    block_bottom = temp_floor_top + BLOCK_GAP_ABOVE_FLOOR_CM
    return LayerGeometry(
        existing_block_top_z_cm=existing_block_top,
        temp_floor_top_z_cm=temp_floor_top,
        block_bottom_z_cm=block_bottom,
    )


def geometry_from_measured_floor_top(
    base: LayerGeometry,
    measured_floor_top_z_cm: float,
) -> LayerGeometry:
    block_bottom = measured_floor_top_z_cm + BLOCK_GAP_ABOVE_FLOOR_CM
    return replace(
        base,
        temp_floor_top_z_cm=measured_floor_top_z_cm,
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


def _wait_for_ue_port(timeout_s: float = UE_RECONNECT_WAIT_S) -> bool:
    import socket

    host, port = "127.0.0.1", 9000
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2.0):
                return True
        except OSError:
            time.sleep(2.0)
    return False


def _reconnect_ucv(ucv: Optional[UnrealCV]) -> UnrealCV:
    print("[UE] waiting for port 9000 ...")
    if not _wait_for_ue_port():
        raise ConnectionError("UnrealCV port 9000 not reachable — restart PIE and retry.")
    geh.release_connection(ucv)
    time.sleep(1.0)
    new_ucv, _ = g10k.ensure_connection(force_new=True)
    if not _ue_alive(new_ucv):
        raise ConnectionError("UnrealCV reconnected but not responding — restart PIE.")
    prepare_semantic_spawn_session(new_ucv)
    time.sleep(POST_RECONNECT_SETTLE_S)
    return new_ucv


def _recover_session_after_dropout(
    ucv: Optional[UnrealCV],
    geometry: LayerGeometry,
) -> Tuple[UnrealCV, LayerGeometry]:
    """Reconnect and restore temp floor if UE dropped mid-fill."""
    new_ucv = _reconnect_ucv(ucv)
    if geh.actor_exists(new_ucv, TEMP_FLOOR_ACTOR):
        return new_ucv, geometry
    print("[Recover] sem_temp_floor missing — respawning ...")
    ok, restored = spawn_temp_floor(new_ucv, geometry)
    if not ok:
        raise ConnectionError("temp floor restore failed after UE dropout")
    return new_ucv, restored


def ensure_verify_session(
    ucv: UnrealCV,
    geometry: LayerGeometry,
) -> Tuple[UnrealCV, LayerGeometry]:
    if not _ue_alive(ucv):
        return _recover_session_after_dropout(ucv, geometry)
    if not geh.actor_exists(ucv, TEMP_FLOOR_ACTOR):
        print("[Verify] sem_temp_floor missing — respawning before verify ...")
        ok, geometry = spawn_temp_floor(ucv, geometry)
        if not ok:
            raise RuntimeError("temp floor respawn failed before verify")
    return ucv, geometry


def _rectangle_center_and_extent_cm(
    gx0: int, gy0: int, gx1: int, gy1: int,
) -> Tuple[float, float, float, float]:
    x0, y0 = cell_center_world_xy_cm(gx0, gy0)
    x1, y1 = cell_center_world_xy_cm(gx1, gy1)
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    n_x, n_y = abs(gx1 - gx0) + 1, abs(gy1 - gy0) + 1
    return cx, cy, n_x * geh.CUBE_SIZE_CM, n_y * geh.CUBE_SIZE_CM


def build_temp_floor_obstacle(geometry: LayerGeometry) -> ObstacleBox:
    cx, cy, ext_x, ext_y = _rectangle_center_and_extent_cm(
        TEMP_FLOOR_GX0, TEMP_FLOOR_GY0, TEMP_FLOOR_GX1, TEMP_FLOOR_GY1,
    )
    z_max = geometry.temp_floor_top_z_cm
    z_min = z_max - TEMP_FLOOR_THICKNESS_CM
    return ObstacleBox(
        x_min=cx - ext_x / 2.0,
        x_max=cx + ext_x / 2.0,
        y_min=cy - ext_y / 2.0,
        y_max=cy + ext_y / 2.0,
        z_min=z_min,
        z_max=z_max,
        source=TEMP_FLOOR_ACTOR,
    )


def spawn_temp_floor(
    ucv: UnrealCV,
    geometry: LayerGeometry,
) -> Tuple[bool, LayerGeometry]:
    """Spawn 3×3 temp floor via scaled BP_Floor_30x30 (90×90 cm at map corner)."""
    cx, cy, ext_x, ext_y = _rectangle_center_and_extent_cm(
        TEMP_FLOOR_GX0, TEMP_FLOOR_GY0, TEMP_FLOOR_GX1, TEMP_FLOOR_GY1,
    )
    top_z = geometry.temp_floor_top_z_cm
    scale_xy = ext_x / TEMP_FLOOR_NATIVE_EXT_CM
    scale = (scale_xy, scale_xy, TEMP_FLOOR_THICKNESS_CM / 100.0)
    loc = (cx, cy, top_z)

    if geh.actor_exists(ucv, TEMP_FLOOR_ACTOR):
        ucv.set_physics(TEMP_FLOOR_ACTOR, False)
        ucv.set_collision(TEMP_FLOOR_ACTOR, True)
        ucv.set_movable(TEMP_FLOOR_ACTOR, False)
        ucv.set_scale(scale, TEMP_FLOOR_ACTOR)
        ucv.set_location(loc, TEMP_FLOOR_ACTOR)
        ucv.set_orientation((0.0, 0.0, 0.0), TEMP_FLOOR_ACTOR)
    else:
        if not _ensure_spawn_name_free(ucv, TEMP_FLOOR_ACTOR):
            return False, geometry
        if not geh.spawn_bp(
            ucv, TEMP_FLOOR_BP, TEMP_FLOOR_ACTOR, timeout_s=SPAWN_TIMEOUT_FIRST_S,
        ):
            print(f"[TempFloor] spawn failed: {TEMP_FLOOR_BP}")
            return False, geometry
        ucv.set_physics(TEMP_FLOOR_ACTOR, False)
        ucv.set_collision(TEMP_FLOOR_ACTOR, True)
        ucv.set_movable(TEMP_FLOOR_ACTOR, False)
        ucv.set_scale(scale, TEMP_FLOOR_ACTOR)
        ucv.set_location(loc, TEMP_FLOOR_ACTOR)
        ucv.set_orientation((0.0, 0.0, 0.0), TEMP_FLOOR_ACTOR)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)

    if not geh.actor_exists(ucv, TEMP_FLOOR_ACTOR):
        print("[TempFloor] FAIL: sem_temp_floor not found after spawn")
        return False, geometry

    actual = geh.try_get_location_cm(ucv, TEMP_FLOOR_ACTOR)
    if actual is None:
        print("[TempFloor] FAIL: cannot read location")
        return False, geometry

    measured_top = actual[2]
    updated = geometry_from_measured_floor_top(geometry, measured_top)
    print(
        f"[TempFloor] {TEMP_FLOOR_ACTOR} cells=({TEMP_FLOOR_GX0},{TEMP_FLOOR_GY0})"
        f"-({TEMP_FLOOR_GX1},{TEMP_FLOOR_GY1}) "
        f"size={ext_x:.0f}x{ext_y:.0f} cm scale_xy={scale_xy:.4f} "
        f"top_z={measured_top:.1f}cm (planned {top_z:.1f}) "
        f"block_bottom_z={updated.block_bottom_z_cm:.1f}cm "
        f"loc={geh._fmt_xyz(actual)}"
    )
    return True, updated


def spawn_temp_floor_with_retry(
    ucv: UnrealCV,
    geometry: LayerGeometry,
    *,
    max_attempts: int = 2,
) -> Tuple[bool, UnrealCV, LayerGeometry]:
    for attempt in range(1, max_attempts + 1):
        if not _ue_alive(ucv):
            ucv = _reconnect_ucv(ucv)
        ok, updated = spawn_temp_floor(ucv, geometry)
        if ok:
            return True, ucv, updated
        if attempt < max_attempts:
            print(f"[TempFloor] retry {attempt + 1}/{max_attempts} after UE settle ...")
            if not _ue_alive(ucv):
                ucv = _reconnect_ucv(ucv)
            else:
                prepare_semantic_spawn_session(ucv)
    return False, ucv, geometry


def prime_first_semantic_cube(
    ucv: UnrealCV,
    geometry: LayerGeometry,
    semantics: Dict[BlockIndex, BlockSemantic],
    *,
    use_semantic_modes: bool = True,
) -> Tuple[UnrealCV, Optional[SemanticBlockRecord]]:
    """Place cell (1,1) before the fill loop — loads BP without destroy→respawn churn."""
    gx, gy = 1, 1
    sem = semantics[(gx, gy)]
    mode = mode_for_semantic(sem) if use_semantic_modes else DEFAULT_BLOCK_MODE
    print("[Prime] placing sem_block_001_001 (loads BP_TransparentCube) ...")
    time.sleep(POST_TEMP_FLOOR_SETTLE_S)
    rec: Optional[SemanticBlockRecord] = None
    for attempt in range(1, BLOCK_SPAWN_MAX_ATTEMPTS + 1):
        if not _ue_alive(ucv):
            ucv = _reconnect_ucv(ucv)
            floor_ok, ucv, geometry = spawn_temp_floor_with_retry(ucv, geometry)
            if not floor_ok:
                return ucv, None
        rec = _place_semantic_cube(
            ucv, gx, gy, geometry, sem, mode=mode, spawn_timeout_s=SPAWN_TIMEOUT_FIRST_S,
        )
        if rec is not None:
            time.sleep(POST_WARMUP_SETTLE_S)
            print("[Prime] first cube OK — fill continues with warmed BP")
            return ucv, rec
        if attempt < BLOCK_SPAWN_MAX_ATTEMPTS:
            print(f"[Prime] retry attempt {attempt + 1}/{BLOCK_SPAWN_MAX_ATTEMPTS}")
            prepare_semantic_spawn_session(ucv)
    return ucv, None


def _set_cube_visual_mode(ucv: UnrealCV, name: str, *, blocking: bool) -> None:
    """Solid cubes need vbp SetBlocking; translucent uses collision off only (lighter on UE)."""
    ucv.set_collision(name, blocking)
    if blocking:
        geh.set_cube_blocking_mode(ucv, name, blocking=True, apply_tint=False)
    else:
        raw = geh._ue_request(ucv, f"vbp {name} SetBlocking False", timeout_s=10.0)
        if raw is not None and str(raw).strip().lower().startswith("error"):
            geh._ue_request(ucv, f"vbp {name} SetBlocking 0", timeout_s=10.0)


def _configure_semantic_cube(
    ucv: UnrealCV,
    name: str,
    loc: Tuple[float, float, float],
    *,
    blocking: bool,
) -> None:
    ucv.set_physics(name, False)
    ucv.set_movable(name, False)
    ucv.set_location(list(loc), name)
    ucv.set_orientation((0.0, 0.0, 0.0), name)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)
    _set_cube_visual_mode(ucv, name, blocking=blocking)
    time.sleep(POST_CUBE_SETTLE_S)


def _place_semantic_cube(
    ucv: UnrealCV,
    gx: int,
    gy: int,
    geometry: LayerGeometry,
    semantic: BlockSemantic,
    *,
    mode: BlockMode = DEFAULT_BLOCK_MODE,
    spawn_timeout_s: float = SPAWN_TIMEOUT_FIRST_S,
) -> Optional[SemanticBlockRecord]:
    name = block_actor_name(gx, gy)
    x, y = cell_center_world_xy_cm(gx, gy)
    bottom_z = geometry.block_bottom_z_cm
    actor_z = block_bottom_to_actor_z(bottom_z)
    loc = (x, y, actor_z)
    blocking = mode == "T"

    if geh.actor_exists(ucv, name):
        _configure_semantic_cube(ucv, name, loc, blocking=blocking)
    else:
        if not _ensure_spawn_name_free(ucv, name):
            return None
        if not geh.spawn_bp(ucv, geh.CUBE_BP, name, timeout_s=spawn_timeout_s):
            print(f"  warn: spawn failed {name}")
            return None
        _configure_semantic_cube(ucv, name, loc, blocking=blocking)

    if not geh.actor_exists(ucv, name):
        print(f"  warn: missing after place {name}")
        return None

    actual = tuple(float(v) for v in ucv.get_location(name))
    actual_bottom = actor_z_to_block_bottom(actual[2])
    if abs(actual_bottom - bottom_z) > Z_PLACEMENT_TOL_CM:
        print(
            f"  warn: {name} bottom z={actual_bottom:.1f} "
            f"expected {bottom_z:.1f} (actor {geh._fmt_xyz(actual)})"
        )
        return None

    return SemanticBlockRecord(
        gx=gx,
        gy=gy,
        semantic=semantic,
        mode=mode,
        block_bottom_z_cm=bottom_z,
        actor_name=name,
        world_cm=actual,
        on_temp_floor=cell_on_temp_floor(gx, gy),
    )


def fill_labeled_blocks(
    ucv: UnrealCV,
    semantics: Dict[BlockIndex, BlockSemantic],
    geometry: LayerGeometry,
    *,
    use_semantic_modes: bool = True,
    default_mode: BlockMode = DEFAULT_BLOCK_MODE,
    spawn_interval_s: float = BLOCK_SPAWN_INTERVAL_S,
    bp_warmed: bool = False,
    prefilled: Optional[Dict[str, SemanticBlockRecord]] = None,
) -> Tuple[UnrealCV, Dict[str, SemanticBlockRecord], int]:
    prepare_semantic_spawn_session(ucv)
    registry: Dict[str, SemanticBlockRecord] = dict(prefilled or {})
    disconnects = 0
    cells = sorted(semantics.keys())
    pending = [
        (gx, gy) for gx, gy in cells
        if block_actor_name(gx, gy) not in registry
    ]
    total = len(cells)
    mode_desc = (
        "floor=F air=T wall=T"
        if use_semantic_modes
        else f"uniform={default_mode}"
    )
    print(
        f"[Fill] placing {len(pending)} blocks "
        f"({len(registry)} already primed) at bottom_z={geometry.block_bottom_z_cm:.1f}cm "
        f"({mode_desc}, interval={spawn_interval_s}s) ..."
    )
    for i, (gx, gy) in enumerate(pending, start=1):
        sem = semantics[(gx, gy)]
        mode = mode_for_semantic(sem) if use_semantic_modes else default_mode
        spawn_timeout = SPAWN_TIMEOUT_NEXT_S if bp_warmed else SPAWN_TIMEOUT_FIRST_S
        rec: Optional[SemanticBlockRecord] = None
        for attempt in range(1, BLOCK_SPAWN_MAX_ATTEMPTS + 1):
            if not _ue_alive(ucv):
                print(f"[Fill] UE not responding before ({gx},{gy}) — recovering ...")
                disconnects += 1
                ucv, geometry = _recover_session_after_dropout(ucv, geometry)
            rec = _place_semantic_cube(
                ucv,
                gx,
                gy,
                geometry,
                sem,
                mode=mode,
                spawn_timeout_s=spawn_timeout,
            )
            if rec is not None:
                bp_warmed = True
                break
            if attempt < BLOCK_SPAWN_MAX_ATTEMPTS:
                print(f"[Fill] retry ({gx},{gy}) attempt {attempt + 1}/{BLOCK_SPAWN_MAX_ATTEMPTS}")
                if not _ue_alive(ucv):
                    disconnects += 1
                    ucv, geometry = _recover_session_after_dropout(ucv, geometry)
                else:
                    prepare_semantic_spawn_session(ucv)
                    time.sleep(0.5)
        if rec is not None:
            registry[rec.actor_name] = rec
        elif not _ue_alive(ucv):
            print("[Fill] abort: UE lost during fill")
            break
        if spawn_interval_s > 0:
            time.sleep(spawn_interval_s)
        if i % BLOCK_SPAWN_BATCH_SIZE == 0:
            prepare_semantic_spawn_session(ucv)
        placed = len(registry)
        if i % 5 == 0 or i == len(pending):
            print(f"[Fill] {i}/{len(pending)} pending placed={placed}/{total}")
    if disconnects:
        print(f"[Fill] WARN: UE dropout/recover events={disconnects}")
    else:
        print("[Fill] completed with no UE dropout")
    return ucv, registry, disconnects


def validate_test_semantics(semantics: Dict[BlockIndex, BlockSemantic]) -> bool:
    """3×3 → floor, outer ring → air, wall → 0."""
    ok = True
    counts = {"wall": 0, "floor": 0, "air": 0}
    for (gx, gy), sem in sorted(semantics.items()):
        counts[sem] += 1
        expected: BlockSemantic = "floor" if cell_on_temp_floor(gx, gy) else "air"
        if sem != expected:
            print(
                f"[Validate] ({gx},{gy}): got {sem!r}, expected {expected!r}"
            )
            ok = False
    if counts["wall"] != 0:
        print(f"[Validate] FAIL: wall count={counts['wall']} (expected 0)")
        ok = False
    if counts["floor"] != 9:
        print(f"[Validate] FAIL: floor count={counts['floor']} (expected 9)")
        ok = False
    if counts["air"] != 16:
        print(f"[Validate] FAIL: air count={counts['air']} (expected 16)")
        ok = False
    if ok:
        print(f"[Validate] semantics OK: {counts}")
    return ok


def verify_semantic_layer(
    ucv: UnrealCV,
    geometry: LayerGeometry,
    blocks: Dict[str, SemanticBlockRecord],
    *,
    semantics: Optional[Dict[BlockIndex, BlockSemantic]] = None,
) -> bool:
    ok = True
    if semantics is not None:
        counts = {"wall": 0, "floor": 0, "air": 0}
        for sem in semantics.values():
            counts[sem] += 1
        print(f"[Verify] labels wall/floor/air={counts}")
    if not geh.actor_exists(ucv, TEMP_FLOOR_ACTOR):
        print("[Verify] FAIL: sem_temp_floor missing")
        ok = False
    else:
        loc = geh.try_get_location_cm(ucv, TEMP_FLOOR_ACTOR)
        measured_top = loc[2] if loc else float("nan")
        print(
            f"[Verify] temp floor top≈{measured_top:.1f}cm "
            f"block_bottom target={geometry.block_bottom_z_cm:.1f}cm"
        )
    if len(blocks) != 25:
        print(f"[Verify] FAIL: blocks={len(blocks)} (expected 25)")
        ok = False
    mode_counts = {"F": 0, "T": 0}
    for rec in blocks.values():
        mode_counts[rec.mode] += 1
        expected_mode = mode_for_semantic(rec.semantic)
        if rec.mode != expected_mode:
            print(
                f"[Verify] FAIL: {rec.actor_name} semantic={rec.semantic!r} "
                f"mode={rec.mode!r} expected {expected_mode!r}"
            )
            ok = False
    if mode_counts["F"] != 9 or mode_counts["T"] != 16:
        print(
            f"[Verify] FAIL: modes F/T={mode_counts} (expected F=9, T=16)"
        )
        ok = False
    else:
        print(f"[Verify] modes OK: F(floor)={mode_counts['F']} T(air)={mode_counts['T']}")
    gap_cm = geometry.block_bottom_z_cm - geometry.temp_floor_top_z_cm
    if abs(gap_cm - BLOCK_GAP_ABOVE_FLOOR_CM) > Z_PLACEMENT_TOL_CM:
        print(
            f"[Verify] FAIL: floor-to-block gap={gap_cm:.1f}cm "
            f"(expected {BLOCK_GAP_ABOVE_FLOOR_CM:.1f})"
        )
        ok = False
    return ok


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


def cleanup_semantic_layer(ucv: UnrealCV, blocks: Optional[Dict[str, SemanticBlockRecord]] = None) -> None:
    """Remove only known sem_* actors — never ``vget /objects`` on grid_100x100."""
    if not _ue_alive(ucv):
        print("[Cleanup] skip: UE not responding")
        return
    names = set(iter_semantic_actor_names())
    if blocks:
        names.update(rec.actor_name for rec in blocks.values())
    print(f"[Cleanup] removing {len(names)} sem_* actors (fixed list, no /objects scan) ...")
    geh._prepare_ue_spawn(ucv)
    for name in sorted(names, reverse=True):
        _destroy_semantic_actor(ucv, name)
    geh._prepare_ue_spawn(ucv)
    time.sleep(POST_CLEANUP_SETTLE_S)
    print("[Cleanup] semantic layer actors removed.")


def run_semantic_layer_demo(
    ucv: Optional[UnrealCV] = None,
    *,
    use_semantic_modes: bool = True,
    default_block_mode: BlockMode = DEFAULT_BLOCK_MODE,
    cleanup_before: bool = True,
    save_path: Path = REGISTRY_PATH,
) -> SemanticLayerResult:
    if ucv is None:
        ucv, _ = ensure_connection()
    if not ucv.client.isconnected():
        raise RuntimeError("UnrealCV not connected — start grid_100x100 PIE first.")

    if cleanup_before:
        cleanup_semantic_layer(ucv)

    prepare_semantic_spawn_session(ucv)

    base_geom = compute_layer_geometry(floor_top_z_cm=geh.resolve_floor_top_z_cm(ucv))
    print(
        f"[Geometry] plan temp_floor_top≈{base_geom.temp_floor_top_z_cm:.1f}cm "
        f"block_bottom≈{base_geom.block_bottom_z_cm:.1f}cm "
        f"(gap={BLOCK_GAP_ABOVE_FLOOR_M:.2f}m)"
    )

    obstacles = [build_temp_floor_obstacle(base_geom)]
    cells = list(iter_rectangle_indices(FILL_GX0, FILL_GY0, FILL_GX1, FILL_GY1))
    print(
        f"[SemanticScan] region ({FILL_GX0},{FILL_GY0})-({FILL_GX1},{FILL_GY1}) "
        f"cells={len(cells)} z_initial_bottom={base_geom.block_bottom_z_cm:.1f}cm ..."
    )
    semantics = scan_region_semantics(
        ucv,
        cells,
        cell_center_xy_cm_fn=cell_center_world_xy_cm,
        z_initial_bottom_cm=base_geom.block_bottom_z_cm,
        block_height_cm=geh.CUBE_SIZE_CM,
        obstacles=obstacles,
        progress_every=10,
    )
    if not validate_test_semantics(semantics):
        raise RuntimeError("semantic labeling validation failed")

    floor_ok, ucv, geometry = spawn_temp_floor_with_retry(ucv, base_geom)
    if not floor_ok:
        raise RuntimeError("temp floor spawn failed")
    time.sleep(POST_TEMP_FLOOR_SETTLE_S)

    if not _ue_alive(ucv):
        ucv = _reconnect_ucv(ucv)
        floor_ok, ucv, geometry = spawn_temp_floor_with_retry(ucv, geometry)
        if not floor_ok:
            raise RuntimeError("temp floor respawn failed after reconnect")

    ucv, first_rec = prime_first_semantic_cube(
        ucv, geometry, semantics, use_semantic_modes=use_semantic_modes,
    )
    if first_rec is None:
        raise RuntimeError("first cube prime failed")

    ucv, blocks, disconnects = fill_labeled_blocks(
        ucv,
        semantics,
        geometry,
        use_semantic_modes=use_semantic_modes,
        default_mode=default_block_mode,
        bp_warmed=True,
        prefilled={first_rec.actor_name: first_rec},
    )
    if disconnects > 0:
        raise RuntimeError(
            f"UE disconnected {disconnects} time(s) during fill — "
            "restart PIE and retry (see stability constants in grid_env_10k_semantic.py)"
        )
    ucv, geometry = ensure_verify_session(ucv, geometry)
    if not verify_semantic_layer(ucv, geometry, blocks, semantics=semantics):
        raise RuntimeError("semantic layer verification failed")
    save_registry(save_path, geometry, semantics, blocks)

    return SemanticLayerResult(
        geometry=geometry,
        semantics=semantics,
        blocks=blocks,
        registry_path=save_path,
    )
