#!/usr/bin/env python3
"""Level map: labeled 0.3 m block layer inside corner rectangle + 3 m margin.

Label rule (0.3 m cube, r = 0.15 m sphere at cube center):
  at initial bottom → wall; else bottom −30 cm → floor if hit, else air.

Height scan: −30 cm while all air; first wall → initial bottom = wall height + 30 cm.
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Literal, Optional, Tuple

BlockIndex = Tuple[int, int]
BlockMode = Literal["T", "F"]
BlockSemantic = Literal["wall", "floor", "air"]


def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
SEM_DIR = ROOT / "dev" / "grid_env_10k_semantic"
THIS_DIR = Path(__file__).resolve().parent
for p in (ROOT, GEH_DIR, G10K_DIR, SEM_DIR, THIS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from level_region import (  # noqa: E402
    HEIGHT_STEP_CM,
    MAX_HEIGHT_TRIES_ALL_AIR,
    LevelRegionConfig,
    all_air_height_scan_action,
    default_level_region,
    initial_bottom_on_wall_detected,
)
from level_semantic_scan import (  # noqa: E402
    compute_depth_sample_cam_z_cm,
    destroy_collision_probe,
    ensure_collision_probe,
    scan_region_collision,
)
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

LEVEL_MAP_PATH = "/Game/Maps/Level"
SAVE_MAP_PATH = "/Game/Maps/Level_semantic"

HEIGHT_STEP_M = HEIGHT_STEP_CM / 100.0
DEFAULT_MAX_CELLS_WITHOUT_OPT_IN = 5000

BLOCK_ACTOR_PREFIX = "level_sem_block"
REGISTRY_PATH = THIS_DIR / ".level_semantic_registry.json"

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
POST_WARMUP_SETTLE_S = 1.5
POST_CUBE_SETTLE_S = 0.12
POST_RECONNECT_SETTLE_S = 2.0


@dataclass(frozen=True)
class LevelLayerGeometry:
    region: LevelRegionConfig
    block_bottom_z_cm: float


@dataclass
class LevelBlockRecord:
    gx: int
    gy: int
    semantic: BlockSemantic
    mode: BlockMode
    block_bottom_z_cm: float
    actor_name: str
    world_cm: Tuple[float, float, float]


@dataclass(frozen=True)
class LevelSemanticResult:
    geometry: LevelLayerGeometry
    semantics: Dict[BlockIndex, BlockSemantic]
    blocks: Dict[str, LevelBlockRecord]
    registry_path: Path
    height_adjust_steps: int


def mode_for_semantic(semantic: BlockSemantic) -> BlockMode:
    return SEMANTIC_VISUAL_MODES[semantic]


def block_actor_name(gx: int, gy: int) -> str:
    return f"{BLOCK_ACTOR_PREFIX}_{gx:03d}_{gy:03d}"


def iter_fill_indices(
    region: LevelRegionConfig,
    *,
    subgrid: Optional[Tuple[int, int, int, int]] = None,
) -> Iterator[BlockIndex]:
    if subgrid is None:
        yield from region.iter_indices()
        return
    gx0, gy0, gx1, gy1 = subgrid
    lo_gx, hi_gx = (gx0, gx1) if gx0 <= gx1 else (gx1, gx0)
    lo_gy, hi_gy = (gy0, gy1) if gy0 <= gy1 else (gy1, gy0)
    for gx in range(lo_gx, hi_gx + 1):
        for gy in range(lo_gy, hi_gy + 1):
            if 1 <= gx <= region.grid_nx and 1 <= gy <= region.grid_ny:
                yield gx, gy


def iter_block_actor_names(
    region: LevelRegionConfig,
    *,
    subgrid: Optional[Tuple[int, int, int, int]] = None,
) -> Iterator[str]:
    for gx, gy in iter_fill_indices(region, subgrid=subgrid):
        yield block_actor_name(gx, gy)


def _ue_alive(ucv: UnrealCV) -> bool:
    return geh._ping_ucv(ucv)


def ensure_connection() -> Tuple[UnrealCV, object]:
    return g10k.ensure_connection()


def prepare_spawn_session(ucv: UnrealCV) -> None:
    geh._prepare_ue_spawn(ucv)
    time.sleep(POST_PHASE_SETTLE_S)


def _destroy_level_actor(ucv: UnrealCV, name: str) -> None:
    if not geh.actor_exists(ucv, name):
        return
    geh.destroy_actor_safely(ucv, name, max_attempts=3, timeout_s=12.0)
    geh.wait_until_actor_gone(ucv, name, timeout_s=DESTROY_GONE_TIMEOUT_S)
    time.sleep(DESTROY_PACE_S)


def _ensure_spawn_name_free(ucv: UnrealCV, name: str) -> bool:
    if not geh.actor_exists(ucv, name):
        return True
    print(f"[Spawn] clearing stale {name!r} ...")
    _destroy_level_actor(ucv, name)
    if geh.actor_exists(ucv, name):
        print(f"[Spawn] FAIL: {name!r} still present")
        return False
    return True


def block_bottom_to_actor_z(block_bottom_z_cm: float) -> float:
    if geh.CUBE_PIVOT_AT_CENTER:
        return block_bottom_z_cm + geh.CUBE_HALF_CM
    return block_bottom_z_cm


def actor_z_to_block_bottom(actor_z_cm: float) -> float:
    if geh.CUBE_PIVOT_AT_CENTER:
        return actor_z_cm - geh.CUBE_HALF_CM
    return actor_z_cm


def _wait_for_ue_port(timeout_s: float = UE_RECONNECT_WAIT_S) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for host in geh._ue_host_candidates():
            if geh._probe_unrealcv_endpoint(host, geh.UE_PORT, timeout_s=2.0):
                return True
        time.sleep(2.0)
    return False


def _reconnect_ucv(ucv: Optional[UnrealCV]) -> UnrealCV:
    print("[UE] waiting for port 9000 ...")
    if not _wait_for_ue_port():
        raise ConnectionError("UnrealCV port 9000 not reachable — restart Level PIE.")
    geh.release_connection(ucv)
    time.sleep(1.0)
    new_ucv, _ = g10k.ensure_connection(force_new=True)
    if not _ue_alive(new_ucv):
        raise ConnectionError("UnrealCV reconnected but not responding.")
    prepare_spawn_session(new_ucv)
    time.sleep(POST_RECONNECT_SETTLE_S)
    return new_ucv


def _set_cube_visual_mode(ucv: UnrealCV, name: str, *, blocking: bool) -> None:
    ucv.set_collision(name, blocking)
    if blocking:
        geh.set_cube_blocking_mode(ucv, name, blocking=True, apply_tint=False)
    else:
        raw = geh._ue_request(ucv, f"vbp {name} SetBlocking False", timeout_s=10.0)
        if raw is not None and str(raw).strip().lower().startswith("error"):
            geh._ue_request(ucv, f"vbp {name} SetBlocking 0", timeout_s=10.0)


def _configure_block(
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


def _place_block(
    ucv: UnrealCV,
    region: LevelRegionConfig,
    geometry: LevelLayerGeometry,
    gx: int,
    gy: int,
    semantic: BlockSemantic,
    *,
    mode: BlockMode,
    spawn_timeout_s: float = SPAWN_TIMEOUT_FIRST_S,
) -> Optional[LevelBlockRecord]:
    name = block_actor_name(gx, gy)
    x, y = region.cell_center_xy_cm(gx, gy)
    bottom_z = geometry.block_bottom_z_cm
    actor_z = block_bottom_to_actor_z(bottom_z)
    loc = (x, y, actor_z)
    blocking = mode == "T"

    if geh.actor_exists(ucv, name):
        _configure_block(ucv, name, loc, blocking=blocking)
    else:
        if not _ensure_spawn_name_free(ucv, name):
            return None
        if not geh.spawn_bp(ucv, geh.CUBE_BP, name, timeout_s=spawn_timeout_s):
            print(f"  warn: spawn failed {name}")
            return None
        _configure_block(ucv, name, loc, blocking=blocking)

    if not geh.actor_exists(ucv, name):
        return None

    actual = tuple(float(v) for v in ucv.get_location(name))
    actual_bottom = actor_z_to_block_bottom(actual[2])
    if abs(actual_bottom - bottom_z) > Z_PLACEMENT_TOL_CM:
        print(
            f"  warn: {name} bottom z={actual_bottom:.1f} "
            f"expected {bottom_z:.1f}"
        )
        return None

    return LevelBlockRecord(
        gx=gx,
        gy=gy,
        semantic=semantic,
        mode=mode,
        block_bottom_z_cm=bottom_z,
        actor_name=name,
        world_cm=actual,
    )


def prime_first_block(
    ucv: UnrealCV,
    region: LevelRegionConfig,
    geometry: LevelLayerGeometry,
    semantics: Dict[BlockIndex, BlockSemantic],
) -> Tuple[UnrealCV, Optional[LevelBlockRecord]]:
    if not semantics:
        return ucv, None
    gx, gy = min(semantics.keys())
    sem = semantics[(gx, gy)]
    mode = mode_for_semantic(sem)
    print(f"[Prime] placing {block_actor_name(gx, gy)} ...")
    for attempt in range(1, BLOCK_SPAWN_MAX_ATTEMPTS + 1):
        if not _ue_alive(ucv):
            ucv = _reconnect_ucv(ucv)
        rec = _place_block(
            ucv, region, geometry, gx, gy, sem, mode=mode,
            spawn_timeout_s=SPAWN_TIMEOUT_FIRST_S,
        )
        if rec is not None:
            time.sleep(POST_WARMUP_SETTLE_S)
            return ucv, rec
        if attempt < BLOCK_SPAWN_MAX_ATTEMPTS:
            prepare_spawn_session(ucv)
    return ucv, None


def fill_labeled_blocks(
    ucv: UnrealCV,
    region: LevelRegionConfig,
    semantics: Dict[BlockIndex, BlockSemantic],
    geometry: LevelLayerGeometry,
    *,
    prefilled: Optional[Dict[str, LevelBlockRecord]] = None,
) -> Tuple[UnrealCV, Dict[str, LevelBlockRecord], int]:
    prepare_spawn_session(ucv)
    registry: Dict[str, LevelBlockRecord] = dict(prefilled or {})
    disconnects = 0
    cells = sorted(semantics.keys())
    pending = [
        (gx, gy) for gx, gy in cells
        if block_actor_name(gx, gy) not in registry
    ]
    total = len(cells)
    print(
        f"[Fill] {len(pending)} blocks at bottom_z={geometry.block_bottom_z_cm:.1f}cm "
        f"({len(registry)} primed) ..."
    )
    bp_warmed = bool(registry)
    for i, (gx, gy) in enumerate(pending, start=1):
        sem = semantics[(gx, gy)]
        mode = mode_for_semantic(sem)
        spawn_timeout = SPAWN_TIMEOUT_NEXT_S if bp_warmed else SPAWN_TIMEOUT_FIRST_S
        rec: Optional[LevelBlockRecord] = None
        for attempt in range(1, BLOCK_SPAWN_MAX_ATTEMPTS + 1):
            if not _ue_alive(ucv):
                disconnects += 1
                ucv = _reconnect_ucv(ucv)
            rec = _place_block(
                ucv, region, geometry, gx, gy, sem, mode=mode,
                spawn_timeout_s=spawn_timeout,
            )
            if rec is not None:
                bp_warmed = True
                break
            if attempt < BLOCK_SPAWN_MAX_ATTEMPTS:
                prepare_spawn_session(ucv)
        if rec is not None:
            registry[rec.actor_name] = rec
        elif not _ue_alive(ucv):
            break
        if BLOCK_SPAWN_INTERVAL_S > 0:
            time.sleep(BLOCK_SPAWN_INTERVAL_S)
        if i % BLOCK_SPAWN_BATCH_SIZE == 0:
            prepare_spawn_session(ucv)
        if i % 5 == 0 or i == len(pending):
            print(f"[Fill] {i}/{len(pending)} placed={len(registry)}/{total}")
    if disconnects:
        print(f"[Fill] WARN: UE dropout events={disconnects}")
    else:
        print("[Fill] completed with no UE dropout")
    return ucv, registry, disconnects


def count_semantics(semantics: Dict[BlockIndex, BlockSemantic]) -> Dict[str, int]:
    counts = {"wall": 0, "floor": 0, "air": 0}
    for sem in semantics.values():
        counts[sem] += 1
    return counts


def scan_with_height_adjust(
    ucv: UnrealCV,
    region: LevelRegionConfig,
    cells: List[BlockIndex],
    *,
    initial_bottom_z_cm: float,
) -> Tuple[UnrealCV, Dict[BlockIndex, BlockSemantic], float, int]:
    bottom_z = initial_bottom_z_cm
    steps = 0
    semantics: Dict[BlockIndex, BlockSemantic] = {}
    depth_cam_z = compute_depth_sample_cam_z_cm(initial_bottom_z_cm, geh.CUBE_SIZE_CM)
    total_cells = len(cells)
    probe_active = False
    probe_ok, probe_name = ensure_collision_probe(ucv, force_respawn=False)
    use_collision = probe_ok
    if probe_ok:
        probe_active = True
        print(f"[HeightScan] collision probe ready ({probe_name}, reuse ok)")
    else:
        print("[HeightScan] collision probe unavailable — depth labeling fallback")
    try:
        for attempt in range(1, MAX_HEIGHT_TRIES_ALL_AIR + 1):
            if not _ue_alive(ucv):
                print("[HeightScan] UE not responding — reconnecting ...")
                ucv = _reconnect_ucv(ucv)
            limit = MAX_HEIGHT_TRIES_ALL_AIR
            print(
                f"[HeightScan] attempt {attempt}/{limit} "
                f"bottom_z={bottom_z:.1f}cm ({bottom_z/100:.3f}m) cells={total_cells} ..."
            )
            try:
                semantics, geometry_hits = scan_region_collision(
                    ucv,
                    cells,
                    cell_center_xy_cm_fn=region.cell_center_xy_cm,
                    z_initial_bottom_cm=bottom_z,
                    block_height_cm=geh.CUBE_SIZE_CM,
                    depth_sample_cam_z_cm=depth_cam_z,
                    progress_every=50,
                    manage_probe=False,
                    use_collision_probe=use_collision,
                    probe_actor=probe_name,
                )
            except ConnectionError as exc:
                print(f"[HeightScan] scan failed: {exc} — reconnecting ...")
                ucv = _reconnect_ucv(ucv)
                probe_ok, probe_name = ensure_collision_probe(ucv, force_respawn=False)
                use_collision = probe_ok
                if probe_ok:
                    probe_active = True
                semantics, geometry_hits = scan_region_collision(
                    ucv,
                    cells,
                    cell_center_xy_cm_fn=region.cell_center_xy_cm,
                    z_initial_bottom_cm=bottom_z,
                    block_height_cm=geh.CUBE_SIZE_CM,
                    depth_sample_cam_z_cm=depth_cam_z,
                    progress_every=50,
                    manage_probe=False,
                    use_collision_probe=use_collision,
                    probe_actor=probe_name,
                )
            counts = count_semantics(semantics)
            hit_label = "collision_geom" if use_collision else "nadir_surface"
            action = all_air_height_scan_action(counts, total_cells)
            print(
                f"[HeightScan] labels wall/floor/air={counts} "
                f"{hit_label}={geometry_hits}/{total_cells} action={action!r}"
            )
            if action == "lock_wall":
                detect_z = bottom_z
                bottom_z = initial_bottom_on_wall_detected(bottom_z)
                steps += 1
                print(
                    f"[HeightScan] wall at {detect_z:.1f}cm → "
                    f"initial bottom {bottom_z:.1f}cm (+{HEIGHT_STEP_M:.2f}m)"
                )
                semantics, _ = scan_region_collision(
                    ucv,
                    cells,
                    cell_center_xy_cm_fn=region.cell_center_xy_cm,
                    z_initial_bottom_cm=bottom_z,
                    block_height_cm=geh.CUBE_SIZE_CM,
                    depth_sample_cam_z_cm=depth_cam_z,
                    progress_every=50,
                    manage_probe=False,
                    use_collision_probe=use_collision,
                    probe_actor=probe_name,
                )
                print(
                    f"[HeightScan] final labels wall/floor/air="
                    f"{count_semantics(semantics)} at bottom_z={bottom_z:.1f}cm"
                )
                return ucv, semantics, bottom_z, steps
            if action == "stop":
                return ucv, semantics, bottom_z, steps
            if action == "lower":
                if attempt >= MAX_HEIGHT_TRIES_ALL_AIR:
                    print(
                        f"[HeightScan] max tries ({MAX_HEIGHT_TRIES_ALL_AIR}) "
                        "all air — using last scan"
                    )
                    return ucv, semantics, bottom_z, steps
                bottom_z -= HEIGHT_STEP_CM
                steps += 1
                print(f"[HeightScan] all air — lower -{HEIGHT_STEP_M:.2f}m → {bottom_z:.1f}cm")
                continue
            return ucv, semantics, bottom_z, steps
    finally:
        # Keep collision probe alive for the PIE session (destroy→respawn crashes SimWorld).
        pass
    return ucv, semantics, bottom_z, steps


def cleanup_level_semantic_layer(
    ucv: UnrealCV,
    region: LevelRegionConfig,
    *,
    subgrid: Optional[Tuple[int, int, int, int]] = None,
    blocks: Optional[Dict[str, LevelBlockRecord]] = None,
) -> None:
    if not _ue_alive(ucv):
        print("[Cleanup] skip: UE not responding")
        return
    names = set(iter_block_actor_names(region, subgrid=subgrid))
    if blocks:
        names.update(rec.actor_name for rec in blocks.values())
    print(f"[Cleanup] removing {len(names)} {BLOCK_ACTOR_PREFIX}_* actors ...")
    prepare_spawn_session(ucv)
    for name in sorted(names, reverse=True):
        _destroy_level_actor(ucv, name)
    prepare_spawn_session(ucv)
    time.sleep(POST_CLEANUP_SETTLE_S)


def save_registry(
    path: Path,
    geometry: LevelLayerGeometry,
    semantics: Dict[BlockIndex, BlockSemantic],
    blocks: Dict[str, LevelBlockRecord],
    *,
    height_adjust_steps: int,
    subgrid: Optional[Tuple[int, int, int, int]] = None,
) -> None:
    region = geometry.region
    payload = {
        "source_map": LEVEL_MAP_PATH,
        "save_map": SAVE_MAP_PATH,
        "region": {
            "corner_a_xy_cm": region.corner_a_xy_cm,
            "corner_b_xy_cm": region.corner_b_xy_cm,
            "outward_margin_m": region.outward_margin_cm / 100.0,
            "grid_origin_xy_cm": region.grid_origin_xy_cm,
            "grid_nx": region.grid_nx,
            "grid_ny": region.grid_ny,
            "subgrid": list(subgrid) if subgrid else None,
        },
        "block_bottom_z_cm": geometry.block_bottom_z_cm,
        "height_adjust_steps": height_adjust_steps,
        "semantics": {
            f"{gx:03d}_{gy:03d}": sem for (gx, gy), sem in sorted(semantics.items())
        },
        "blocks": {name: asdict(rec) for name, rec in sorted(blocks.items())},
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[Registry] saved {path}")


def _check_region_size(
    region: LevelRegionConfig,
    *,
    allow_large_region: bool,
    subgrid: Optional[Tuple[int, int, int, int]],
) -> int:
    if subgrid is not None:
        cells = list(iter_fill_indices(region, subgrid=subgrid))
        n = len(cells)
        print(f"[Region] PIE subgrid {subgrid} → {n} cells")
        return n
    n = region.cell_count
    print(
        f"[Region] core=({region.core_x_min_cm:.0f},{region.core_y_min_cm:.0f})"
        f"-({region.core_x_max_cm:.0f},{region.core_y_max_cm:.0f}) cm "
        f"expanded +{region.outward_margin_cm:.0f}cm/side "
        f"grid={region.grid_nx}x{region.grid_ny}={n} cells "
        f"origin={region.grid_origin_xy_cm}"
    )
    if n > DEFAULT_MAX_CELLS_WITHOUT_OPT_IN and not allow_large_region:
        raise RuntimeError(
            f"Region has {n} cells (> {DEFAULT_MAX_CELLS_WITHOUT_OPT_IN}). "
            "Set allow_large_region=True for full run, or pie_subgrid=(gx0,gy0,gx1,gy1) "
            "for a small PIE test (e.g. (1,1,5,5))."
        )
    return n


def run_level_semantic_layer(
    ucv: Optional[UnrealCV] = None,
    *,
    region: Optional[LevelRegionConfig] = None,
    cleanup_before: bool = True,
    allow_large_region: bool = False,
    pie_subgrid: Optional[Tuple[int, int, int, int]] = (1, 1, 5, 5),
    save_path: Path = REGISTRY_PATH,
) -> LevelSemanticResult:
    """Run labeling + block placement on Level (PIE).

    Default ``pie_subgrid=(1,1,5,5)`` places 25 cells for PIE verification.
    Pass ``pie_subgrid=None`` with ``allow_large_region=True`` for the full region.
    """
    if ucv is None:
        ucv, _ = ensure_connection()
    if not ucv.client.isconnected():
        raise RuntimeError("UnrealCV not connected — open Level map and start PIE.")

    region = region or default_level_region()
    cell_count = _check_region_size(
        region, allow_large_region=allow_large_region, subgrid=pie_subgrid,
    )
    cells = list(iter_fill_indices(region, subgrid=pie_subgrid))

    if cleanup_before:
        cleanup_level_semantic_layer(ucv, region, subgrid=pie_subgrid)

    prepare_spawn_session(ucv)

    initial_z = region.block_bottom_z_cm
    print(
        f"[Geometry] block_bottom_z={initial_z:.1f}cm ({initial_z/100:.3f}m) "
        f"from corner A"
    )

    ucv, semantics, final_z, height_steps = scan_with_height_adjust(
        ucv, region, cells, initial_bottom_z_cm=initial_z,
    )
    geometry = LevelLayerGeometry(region=region, block_bottom_z_cm=final_z)

    ucv, first_rec = prime_first_block(ucv, region, geometry, semantics)
    if first_rec is None:
        raise RuntimeError("first block prime failed")

    prefilled = {first_rec.actor_name: first_rec}
    ucv, blocks, disconnects = fill_labeled_blocks(
        ucv, region, semantics, geometry, prefilled=prefilled,
    )
    if disconnects > 0:
        raise RuntimeError(f"UE disconnected {disconnects} time(s) during fill")

    counts = count_semantics(semantics)
    modes = {"F": 0, "T": 0}
    for rec in blocks.values():
        modes[rec.mode] += 1
    print(f"[Verify] labels={counts} modes F/T={modes} blocks={len(blocks)}/{cell_count}")

    save_registry(
        save_path, geometry, semantics, blocks,
        height_adjust_steps=height_steps,
        subgrid=pie_subgrid,
    )

    return LevelSemanticResult(
        geometry=geometry,
        semantics=semantics,
        blocks=blocks,
        registry_path=save_path,
        height_adjust_steps=height_steps,
    )
