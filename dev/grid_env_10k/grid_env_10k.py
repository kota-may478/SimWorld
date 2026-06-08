#!/usr/bin/env python3
"""100×100 半透明ブロック敷き詰め + Humanoid / SpotDog（grid_env_hri と同床・同エージェント位置）。

ブロック座標 (gx, gy) は **1 始まり**:
  - 床左下角のマス = (1, 1)
  - Humanoid から見て **右** = gx が増える: (1,1), (2,1), …, (100,1)
  - Humanoid から見て **正面** = gy が増える: (1,1), (1,2), …, (1,100)

内部 UE 格子は grid_env_hri と同じ row=gy-1, col=gx-1（0 始まり）。
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Literal, Optional, Tuple, Union

BlockMode = Literal["T", "F"]
ScenarioStep = Tuple[Union[str, int], ...]

# ---- SimWorld ルート + grid_env_hri ----
def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
GEH_DIR = ROOT / "dev" / "grid_env_hri"
THIS_DIR = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(GEH_DIR) not in sys.path:
    sys.path.insert(0, str(GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

# ==============================================================
# 定数（フェーズ2 以降の領域実体化用 API もここに追加予定）
# ==============================================================

BLOCK_GRID_N = int(os.environ.get("BLOCK_GRID_N", "100"))
BLOCK_SPAWN_INTERVAL_S = float(os.environ.get("BLOCK_SPAWN_INTERVAL_S", "0.01"))
BLOCK_ACTOR_PREFIX = os.environ.get("BLOCK_ACTOR_PREFIX", "block")
# 本番 10,000 前の接続確認用（例: BLOCK_GRID_N=5 → 25 個）
BLOCK_SPAWN_DRY_RUN_N = int(os.environ.get("BLOCK_SPAWN_DRY_RUN_N", "0"))

BlockIndex = Tuple[int, int]  # (gx, gy) 1-indexed


def validate_block_index(gx: int, gy: int, *, grid_n: int = BLOCK_GRID_N) -> None:
    if not (1 <= gx <= grid_n and 1 <= gy <= grid_n):
        raise ValueError(
            f"block index ({gx}, {gy}) out of range; "
            f"expected 1 <= gx, gy <= {grid_n}"
        )


def block_index_to_row_col(gx: int, gy: int) -> Tuple[int, int]:
    """1-indexed (gx, gy) → 0-indexed (row, col) for UE 配置。"""
    validate_block_index(gx, gy)
    return gy - 1, gx - 1


def row_col_to_block_index(row: int, col: int) -> BlockIndex:
    """0-indexed (row, col) → 1-indexed (gx, gy)。"""
    return col + 1, row + 1


def block_actor_name(gx: int, gy: int, *, prefix: str = BLOCK_ACTOR_PREFIX) -> str:
    validate_block_index(gx, gy)
    return f"{prefix}_{gx:03d}_{gy:03d}"


def parse_block_actor_name(name: str, *, prefix: str = BLOCK_ACTOR_PREFIX) -> Optional[BlockIndex]:
    """Actor 名から (gx, gy) を復元。不一致時は None。"""
    pre = f"{prefix}_"
    if not name.startswith(pre):
        return None
    parts = name[len(pre) :].split("_")
    if len(parts) != 2:
        return None
    try:
        gx, gy = int(parts[0]), int(parts[1])
    except ValueError:
        return None
    if gx < 1 or gy < 1:
        return None
    return gx, gy


def iter_block_indices(
    grid_n: int = BLOCK_GRID_N,
) -> Iterator[BlockIndex]:
    """gy 外側・gx 内側（正面方向に行が進む）。"""
    for gy in range(1, grid_n + 1):
        for gx in range(1, grid_n + 1):
            yield gx, gy


def ensure_connection() -> Tuple[UnrealCV, Communicator]:
    return geh.ensure_connection()


def reconnect_if_needed(ucv: Optional[UnrealCV]) -> Tuple[UnrealCV, Communicator]:
    """UnrealCV が切れたあと再接続（10,000 スポーン中の reset 対策）。"""
    if ucv is not None and ucv.client.isconnected():
        return ucv, Communicator(ucv)
    print("[UE] reconnecting after disconnect ...")
    return ensure_connection()


def prepare_ue_session(ucv: UnrealCV) -> None:
    """大量スポーン前に UE を整理（前回の block_* が残っていても続行可能に）。"""
    geh._prepare_ue_spawn(ucv)
    raw = geh._ue_request(ucv, "vget /objects", timeout_s=120.0)
    if raw is None:
        return
    prefix = f"{BLOCK_ACTOR_PREFIX}_"
    stale = [n for n in raw.split() if n.startswith(prefix)]
    if stale:
        print(f"[Prepare] destroying {len(stale)} stale {prefix}* actors ...")
    for name in stale:
        geh.destroy_if_exists(ucv, name)
    geh.destroy_if_exists(ucv, geh.ROBOT_ACTOR_NAME)


def spawn_floor_with_retry(
    ucv: UnrealCV,
    *,
    max_attempts: int = 2,
) -> Tuple[bool, UnrealCV]:
    """床スポーン。Connection reset 時は再接続して 1 回リトライ。"""
    for attempt in range(1, max_attempts + 1):
        if spawn_floor(ucv):
            return True, ucv
        if attempt < max_attempts:
            print(f"[Floor] retry {attempt + 1}/{max_attempts} after reconnect ...")
            ucv, _ = reconnect_if_needed(ucv)
            prepare_ue_session(ucv)
    return False, ucv


def configure_spawn_env(
    *,
    grid_n: int = BLOCK_GRID_N,
    spawn_interval_s: float = BLOCK_SPAWN_INTERVAL_S,
) -> None:
    """grid_env_hri の環境変数を本プロジェクト向けに設定。"""
    os.environ["GRID_N"] = str(grid_n)
    os.environ["SPAWN_INTERVAL_S"] = str(spawn_interval_s)
    os.environ["GRID_CUBE_BLOCKING"] = "0"
    os.environ["CUBE_ENABLE_PHYSICS"] = "0"
    os.environ["SPAWN_DEMO_MODE_CUBES"] = "0"
    os.environ["RUN_DEMO_PASSAGE_TESTS"] = "0"


def spawn_floor(ucv: UnrealCV) -> bool:
    return geh.spawn_fixed_floor(ucv)


def spawn_translucent_block_grid(
    ucv: UnrealCV,
    *,
    grid_n: int = BLOCK_GRID_N,
    spawn_interval_s: float = BLOCK_SPAWN_INTERVAL_S,
    progress_every_rows: int = 10,
) -> Dict[str, dict]:
    """床全面に半透明（SetBlocking False）ブロックを grid_n² 個スポーン。"""
    total = grid_n * grid_n
    registry: Dict[str, dict] = {}
    t0 = time.monotonic()
    print(
        f"[Blocks] spawning {total} translucent blocks "
        f"({grid_n}×{grid_n}, interval={spawn_interval_s}s, "
        f"pivot={'center' if geh.CUBE_PIVOT_AT_CENTER else 'bottom'})"
    )
    print(
        f"  map corner (0,0) m → block (1,1); "
        f"human {geh.HUMAN_MAP_XY_M} m, robot {geh.ROBOT_MAP_XY_M} m"
    )

    failed = 0
    for gy in range(1, grid_n + 1):
        for gx in range(1, grid_n + 1):
            row, col = gy - 1, gx - 1
            name = block_actor_name(gx, gy)
            ok, loc = geh.spawn_grid_cube_on_floor(
                ucv,
                name,
                row,
                col,
                blocking=False,
            )
            if ok:
                registry[name] = {
                    "gx": gx,
                    "gy": gy,
                    "row": row,
                    "col": col,
                    "blocking": False,
                    "world_cm": loc,
                }
            else:
                failed += 1
                if failed <= 5 or failed % 50 == 0:
                    print(f"  warn: spawn failed {name} (block {gx},{gy})")
                if not ucv.client.isconnected():
                    print("[Blocks] UE disconnected — aborting spawn loop")
                    break
            time.sleep(spawn_interval_s)

        if gy % progress_every_rows == 0 or gy == grid_n:
            elapsed = time.monotonic() - t0
            print(
                f"  row gy={gy}/{grid_n} "
                f"({len(registry)}/{total} ok, failed={failed}, "
                f"elapsed={elapsed:.0f}s)"
            )

    elapsed = time.monotonic() - t0
    print(
        f"[Blocks] done: {len(registry)}/{total} in {elapsed:.1f}s "
        f"({failed} failed)"
    )
    if len(registry) != total:
        print(
            "[Blocks] FAIL: incomplete spawn — "
            "SimWorld 再起動 / SPAWN_INTERVAL_S を 0.02〜0.05 に増やして再試行"
        )
    return registry


def spawn_agents(
    communicator: Communicator,
    ucv: UnrealCV,
) -> Tuple[Optional[str], bool]:
    """Humanoid / SpotDog を grid_env_hri と同位置にスポーン。"""
    human_name = geh.spawn_humanoid(communicator, ucv)
    robot_ok = geh.spawn_robot(ucv)
    return human_name, robot_ok


def verify_block_samples(
    ucv: UnrealCV,
    registry: Dict[str, dict],
    *,
    grid_n: int = BLOCK_GRID_N,
) -> bool:
    """四隅と中央のブロックが存在し床上にあることを確認。"""
    samples: List[BlockIndex] = [
        (1, 1),
        (grid_n, 1),
        (1, grid_n),
        (grid_n, grid_n),
        (grid_n // 2, grid_n // 2),
    ]
    expected_z = geh.cube_actor_z_on_floor_cm()
    all_ok = True
    print("[Verify] sample blocks:")
    for gx, gy in samples:
        name = block_actor_name(gx, gy)
        if name not in registry:
            print(f"  {name} block({gx},{gy}): MISSING from registry")
            all_ok = False
            continue
        if not geh.actor_exists(ucv, name):
            print(f"  {name} block({gx},{gy}): MISSING in UE")
            all_ok = False
            continue
        loc = tuple(ucv.get_location(name))
        z_ok = abs(loc[2] - expected_z) <= 3.0
        mark = "OK" if z_ok else f"Z? (z={loc[2]:.1f}, expect≈{expected_z:.1f})"
        print(f"  block({gx:3d},{gy:3d}) {name} → {geh._fmt_xyz(loc)} {mark}")
        all_ok = all_ok and z_ok
    return all_ok


def cleanup_all(
    ucv: UnrealCV,
    block_registry: Dict[str, dict],
    human_name: Optional[str],
) -> None:
    """床・全ブロック・Robot・Humanoid を削除。"""
    geh.destroy_if_exists(ucv, geh.FLOOR_ACTOR_NAME)
    geh.destroy_if_exists(ucv, geh.ROBOT_ACTOR_NAME)
    for name in block_registry:
        geh.destroy_if_exists(ucv, name)
    if human_name:
        geh.destroy_if_exists(ucv, human_name)
    try:
        ucv.clean_garbage()
    except Exception:
        pass
    print("[Cleanup] floor, blocks, agents destroyed.")


def run_phase1_spawn(
    *,
    grid_n: int = BLOCK_GRID_N,
    spawn_interval_s: float = BLOCK_SPAWN_INTERVAL_S,
    spawn_agents_after_blocks: bool = True,
    verify_samples: bool = True,
) -> bool:
    """フェーズ1: 床 + 半透明ブロック敷き詰め + エージェント。"""
    configure_spawn_env(grid_n=grid_n, spawn_interval_s=spawn_interval_s)
    importlib = __import__("importlib")
    importlib.reload(geh)

    ucv, communicator = ensure_connection()
    if not ucv.client.isconnected():
        print("[Phase1] UE not connected")
        return False

    prepare_ue_session(ucv)
    floor_ok, ucv = spawn_floor_with_retry(ucv)
    if not floor_ok:
        return False

    registry = spawn_translucent_block_grid(
        ucv, grid_n=grid_n, spawn_interval_s=spawn_interval_s
    )
    expected = grid_n * grid_n
    if len(registry) != expected:
        return False

    human_name: Optional[str] = None
    robot_ok = False
    if spawn_agents_after_blocks:
        human_name, robot_ok = spawn_agents(communicator, ucv)
        if not robot_ok:
            print("[Phase1] warn: robot spawn failed")
        geh.report_spawn_state(ucv, {}, human_name, marker_registry=None)

    if verify_samples and not verify_block_samples(ucv, registry, grid_n=grid_n):
        return False

    print(
        f"[Phase1] SUCCESS — {len(registry)} translucent blocks, "
        f"human={human_name!r}, robot_ok={robot_ok}"
    )
    return True


# ==============================================================
# T / F レイアウト API（案 A）
# T = 不透明（SetBlocking True） / F = 半透明（SetBlocking False）
# ==============================================================

MAP_ASSET_PATH = "/Game/Maps/grid_100x100"
MAP_LAUNCH_ARG = "/Game/Maps/grid_100x100.umap"


def parse_block_mode(mode: str) -> BlockMode:
    token = mode.strip().upper()
    if token not in ("T", "F"):
        raise ValueError(f"block mode must be 'T' or 'F', got {mode!r}")
    return token  # type: ignore[return-value]


def mode_to_set_blocking(mode: BlockMode) -> bool:
    """T → True（不透明）, F → False（半透明）。"""
    return mode == "T"


def set_block_mode(
    ucv: UnrealCV,
    gx: int,
    gy: int,
    mode: BlockMode,
    *,
    grid_n: int = BLOCK_GRID_N,
) -> bool:
    validate_block_index(gx, gy, grid_n=grid_n)
    blocking = mode_to_set_blocking(mode)
    name = block_actor_name(gx, gy)
    return geh.set_cube_blocking_mode(
        ucv, name, blocking=blocking, apply_tint=blocking
    )


def set_blocks_mode(
    ucv: UnrealCV,
    cells: Iterable[BlockIndex],
    mode: BlockMode,
    *,
    progress_every: int = 500,
    label: str = "",
) -> Tuple[int, int]:
    """複数マスに同一モードを適用。戻り値 (成功数, 失敗数)。"""
    ok_n = 0
    fail_n = 0
    cells_list = list(cells)
    total = len(cells_list)
    t0 = time.monotonic()
    prefix = f"[Layout]{f' {label}' if label else ''}"
    print(f"{prefix} apply {mode} to {total} cells ...")
    for i, (gx, gy) in enumerate(cells_list, start=1):
        if set_block_mode(ucv, gx, gy, mode):
            ok_n += 1
        else:
            fail_n += 1
        if progress_every > 0 and (i % progress_every == 0 or i == total):
            print(
                f"{prefix} {i}/{total} ok={ok_n} fail={fail_n} "
                f"elapsed={time.monotonic() - t0:.0f}s"
            )
    print(f"{prefix} done ok={ok_n} fail={fail_n} elapsed={time.monotonic() - t0:.1f}s")
    return ok_n, fail_n


def reset_all_blocks(
    ucv: UnrealCV,
    mode: BlockMode = "F",
    *,
    grid_n: int = BLOCK_GRID_N,
    progress_every: int = 1000,
) -> Tuple[int, int]:
    """全マスを同一モードにする（シミュ開始時は通常 F）。"""
    return set_blocks_mode(
        ucv,
        iter_block_indices(grid_n),
        mode,
        progress_every=progress_every,
        label=f"reset_all {mode}",
    )


def iter_perimeter_indices(
    grid_n: int = BLOCK_GRID_N,
) -> Iterator[BlockIndex]:
    """外周 396 マス（100×100 固定時）。gx∈{1,N} または gy∈{1,N}。"""
    for gx in range(1, grid_n + 1):
        for gy in range(1, grid_n + 1):
            if gx in (1, grid_n) or gy in (1, grid_n):
                yield gx, gy


def iter_rectangle_indices(
    gx0: int,
    gy0: int,
    gx1: int,
    gy1: int,
    *,
    grid_n: int = BLOCK_GRID_N,
) -> Iterator[BlockIndex]:
    """(gx0, gy0) と (gx1, gy1) を **両端含む** 矩形のマスを列挙。"""
    lo_gx, hi_gx = (gx0, gx1) if gx0 <= gx1 else (gx1, gx0)
    lo_gy, hi_gy = (gy0, gy1) if gy0 <= gy1 else (gy1, gy0)
    for gx in range(lo_gx, hi_gx + 1):
        for gy in range(lo_gy, hi_gy + 1):
            validate_block_index(gx, gy, grid_n=grid_n)
            yield gx, gy


def set_rectangle(
    ucv: UnrealCV,
    gx0: int,
    gy0: int,
    gx1: int,
    gy1: int,
    mode: BlockMode,
    *,
    grid_n: int = BLOCK_GRID_N,
) -> Tuple[int, int]:
    cells = list(iter_rectangle_indices(gx0, gy0, gx1, gy1, grid_n=grid_n))
    return set_blocks_mode(
        ucv,
        cells,
        mode,
        progress_every=max(50, len(cells) // 5),
        label=f"rect ({gx0},{gy0})-({gx1},{gy1}) {mode}",
    )


def set_perimeter(
    ucv: UnrealCV,
    mode: BlockMode,
    *,
    grid_n: int = BLOCK_GRID_N,
) -> Tuple[int, int]:
    return set_blocks_mode(
        ucv,
        iter_perimeter_indices(grid_n),
        mode,
        progress_every=100,
        label=f"perimeter {mode}",
    )


def apply_scenario(
    ucv: UnrealCV,
    steps: Iterable[ScenarioStep],
    *,
    grid_n: int = BLOCK_GRID_N,
) -> None:
    """シナリオを順に適用（案 A）。

    例:
        apply_scenario(ucv, [
            ("all", "F"),
            ("perimeter", "T"),
            ("rect", 10, 10, 20, 20, "T"),
        ])

    マップを全 F で保存済みなら、初回の ("all", "F") は省略可（約 1 万回 vbp を省く）。
    """
    for step in steps:
        kind = str(step[0]).lower()
        if kind == "all":
            if len(step) != 2:
                raise ValueError(f"all step: ('all', mode), got {step!r}")
            reset_all_blocks(ucv, parse_block_mode(str(step[1])), grid_n=grid_n)
        elif kind == "perimeter":
            if len(step) != 2:
                raise ValueError(f"perimeter step: ('perimeter', mode), got {step!r}")
            set_perimeter(ucv, parse_block_mode(str(step[1])), grid_n=grid_n)
        elif kind == "rect":
            if len(step) != 6:
                raise ValueError(
                    f"rect step: ('rect', gx0, gy0, gx1, gy1, mode), got {step!r}"
                )
            _, gx0, gy0, gx1, gy1, mode_s = step
            set_rectangle(
                ucv,
                int(gx0),
                int(gy0),
                int(gx1),
                int(gy1),
                parse_block_mode(str(mode_s)),
                grid_n=grid_n,
            )
        elif kind == "block":
            if len(step) != 4:
                raise ValueError(f"block step: ('block', gx, gy, mode), got {step!r}")
            _, gx, gy, mode_s = step
            set_block_mode(ucv, int(gx), int(gy), parse_block_mode(str(mode_s)), grid_n=grid_n)
        else:
            raise ValueError(f"unknown scenario step kind {kind!r} in {step!r}")


def audit_level_for_map_save(
    ucv: UnrealCV,
    *,
    grid_n: int = BLOCK_GRID_N,
    label: str = "",
) -> Dict[str, object]:
    """vget /objects から床・block_*・削除対象の有無を集計。"""
    raw = geh._ue_request(ucv, "vget /objects", timeout_s=120.0)
    names = raw.split() if raw else []
    blocks: List[str] = []
    humanoids: List[str] = []
    demos: List[str] = []
    legacy_cubes: List[str] = []
    robots: List[str] = []
    floor_present = False
    extras: List[str] = []

    floor_present = geh.actor_exists(ucv, geh.FLOOR_ACTOR_NAME)

    for name in names:
        if parse_block_actor_name(name) is not None:
            blocks.append(name)
            continue
        if name.startswith("block_"):
            blocks.append(name)
            continue
        # 少数のランタイム Actor のみ location で生存確認（1 万 block は名前列挙のみ）
        if name.startswith("GEN_BP_Humanoid"):
            if geh.actor_exists(ucv, name):
                humanoids.append(name)
        elif name == geh.ROBOT_ACTOR_NAME:
            if geh.actor_exists(ucv, name):
                robots.append(name)
        elif name.startswith("demo_") or name == geh.SINGLE_TOGGLE_CUBE_NAME:
            if geh.actor_exists(ucv, name):
                demos.append(name)
        elif name.startswith("cube_"):
            if geh.actor_exists(ucv, name):
                legacy_cubes.append(name)
        elif name.startswith("GEN_") or name.startswith("GridEnv_"):
            if geh.actor_exists(ucv, name):
                extras.append(name)

    expected_blocks = grid_n * grid_n
    tag = f"[MapSave]{f' {label}' if label else ''}"
    print(
        f"{tag} audit: floor={floor_present} blocks={len(blocks)}/{expected_blocks} "
        f"humanoids={len(humanoids)} robots={len(robots)} demos={len(demos)} "
        f"legacy_cube={len(legacy_cubes)} extras={len(extras)}"
    )
    if humanoids:
        print(f"{tag}   humanoids: {humanoids[:5]}{'...' if len(humanoids) > 5 else ''}")
    if robots:
        print(f"{tag}   robots: {robots}")
    if demos:
        print(f"{tag}   demos: {demos}")
    if legacy_cubes:
        print(f"{tag}   legacy cube_*: {legacy_cubes[:5]}{'...' if len(legacy_cubes) > 5 else ''}")
    if extras:
        print(f"{tag}   extras: {extras[:8]}{'...' if len(extras) > 8 else ''}")

    return {
        "floor_present": floor_present,
        "block_count": len(blocks),
        "expected_blocks": expected_blocks,
        "humanoids": humanoids,
        "robots": robots,
        "demos": demos,
        "legacy_cubes": legacy_cubes,
        "extras": extras,
    }


def prepare_runtime_actors_for_map_save(ucv: UnrealCV) -> Dict[str, object]:
    """マップ保存前: Humanoid / Robot / デモ立方体などを削除し block_* のみ残す。"""
    print("[MapSave] removing runtime-only actors (keep floor + block_*) ...")
    audit_level_for_map_save(ucv, label="before cleanup")
    geh.destroy_if_exists(ucv, geh.ROBOT_ACTOR_NAME)
    geh.destroy_if_exists(ucv, geh.SINGLE_TOGGLE_CUBE_NAME)
    for name in list(DEMO_CLEANUP_NAMES):
        geh.destroy_if_exists(ucv, name)
    raw = geh._ue_request(ucv, "vget /objects", timeout_s=120.0)
    if raw is not None:
        for name in raw.split():
            if name.startswith("GEN_BP_Humanoid"):
                geh.destroy_if_exists(ucv, name)
            elif name.startswith("demo_"):
                geh.destroy_if_exists(ucv, name)
            elif name.startswith("cube_"):
                print(f"[MapSave] warn: legacy cube actor {name!r} — remove or rename before save")
    try:
        ucv.clean_garbage()
    except Exception:
        geh._prepare_ue_spawn(ucv)
    time.sleep(0.5)
    after = audit_level_for_map_save(ucv, label="after cleanup")
    print("[MapSave] ready for UE Editor save (floor + block_* expected)")
    return after


# grid_env_hri デモ立方体 + 通過試験用（保存前に削除）
DEMO_CLEANUP_NAMES: Tuple[str, ...] = (
    "demo_solid_00",
    "demo_solid_01",
    "demo_solid_02",
    "demo_translucent_00",
    geh.SINGLE_TOGGLE_CUBE_NAME,
)


if __name__ == "__main__":
    dry = BLOCK_SPAWN_DRY_RUN_N
    n = dry if dry > 0 else BLOCK_GRID_N
    if dry > 0:
        print(f"[Phase1] dry-run grid_n={n} ({n * n} blocks)")
    ok = run_phase1_spawn(grid_n=n)
    raise SystemExit(0 if ok else 1)
