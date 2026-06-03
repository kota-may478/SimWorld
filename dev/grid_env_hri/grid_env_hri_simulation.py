#!/usr/bin/env python3
"""30 m グリッド床 + 透明箱 10,000 + Humanoid + SpotDog（empty.umap 用）。

前提:
  1. Windows: C:\\SimWorldServer で SimWorld.exe を起動
       .\\SimWorld.exe -windowed -log /Game/Maps/empty.umap
  2. pakchunk9002 に BP_Floor_30x30 / BP_TransparentCube が含まれている
  3. WSL: conda activate simworld

座標系（UE cm、empty マップ）:
  - 床 30 m 四方の **左下隅** が (x, y) = (0, 0)
  - 床上面の高さ = FLOOR_TOP_Z_CM（既定 100 cm = empty 原点から 1 m）
  - 床は物理 OFF・固定。箱は短い落下で設置。Humanoid / Robot は床面上へ直接配置

使い方:
  python grid_env_hri_simulation.py              # GRID_N=100（10,000 箱）
  GRID_N=3 python grid_env_hri_simulation.py    # 小規模テスト
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path
from typing import Iterable, Optional, Tuple

# ---- SimWorld ルートをパスに追加 ----
def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simworld.agent.humanoid import Humanoid
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.utils.vector import Vector

# ==============================================================
# アセット / 定数
# ==============================================================

FLOOR_BP = "/Game/CustomAssets/BP_Floor_30x30.BP_Floor_30x30_C"
CUBE_BP = "/Game/CustomAssets/BP_TransparentCube.BP_TransparentCube_C"
HUMAN_BP = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"
ROBOT_BP = "/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C"

FLOOR_ACTOR_NAME = "grid_floor_main"
ROBOT_ACTOR_NAME = "GridEnv_SpotRobot"

FLOOR_SIZE_M = 30.0
CUBE_SIZE_M = 0.3
CUBE_SIZE_CM = CUBE_SIZE_M * 100.0
CUBE_HALF_CM = CUBE_SIZE_CM / 2.0

# 床左下隅（マップ座標原点）[cm]
MAP_ORIGIN_XY_CM: Tuple[float, float] = (0.0, 0.0)
FLOOR_HALF_CM = FLOOR_SIZE_M * 50.0  # 1500 cm

# empty 原点から床上面まで 1 m
FLOOR_TOP_Z_CM = 100.0
# 床メッシュ pivot: spawn_grid 検証時、actor Z=0 で上面 ≈ Z=0 → 上面を FLOOR_TOP_Z_CM に合わせる
FLOOR_ACTOR_Z_CM = FLOOR_TOP_Z_CM

# 箱のみ短い落下（大きい値だと角の格子から床外へ弾き飛ばされる）
CUBE_SPAWN_ABOVE_FLOOR_CM = float(os.environ.get("CUBE_SPAWN_ABOVE_FLOOR_CM", "5.0"))
# Humanoid / Robot は床面上へ直接配置（hri_spotdog_follow / material_transport と同様）
HUMAN_SPAWN_Z_CM = float(os.environ.get("HUMAN_SPAWN_Z_CM", str(FLOOR_TOP_Z_CM)))
ROBOT_SPAWN_Z_CM = float(os.environ.get("ROBOT_SPAWN_Z_CM", str(FLOOR_TOP_Z_CM)))
PHYSICS_ENABLE_DELAY_S = 0.08
SETTLE_AFTER_SPAWN_S = 6.0

# Human / Robot マップ座標 [m]（左下原点、material_transport grid_map と同じ比率）
HUMAN_MAP_XY_M = (1.0, 1.0)
ROBOT_MAP_XY_M = (1.0, 3.0)

GRID_N = int(os.environ.get("GRID_N", "100"))
SPAWN_INTERVAL_S = float(os.environ.get("SPAWN_INTERVAL_S", "0.005"))
CUBE_ENABLE_PHYSICS = os.environ.get("CUBE_ENABLE_PHYSICS", "1") not in {"0", "false", "False"}
# Humanoid / Robot に Simulated Physics を有効にするとラグドール化するため既定 OFF
AGENT_ENABLE_PHYSICS = os.environ.get("AGENT_ENABLE_PHYSICS", "0") not in {"0", "false", "False"}


# ==============================================================
# 座標変換
# ==============================================================

def floor_center_xy_cm() -> Tuple[float, float]:
    """床 Static Mesh 中心（左下隅が MAP_ORIGIN 時）。"""
    ox, oy = MAP_ORIGIN_XY_CM
    return ox + FLOOR_HALF_CM, oy + FLOOR_HALF_CM


def map_xy_m_to_world_cm(map_xy_m: Tuple[float, float]) -> Tuple[float, float]:
    """マップ座標 [m]（左下原点）→ UE 世界 XY [cm]。"""
    mx, my = map_xy_m
    ox, oy = MAP_ORIGIN_XY_CM
    return ox + mx * 100.0, oy + my * 100.0


def cube_center_cm(
    row: int,
    col: int,
    *,
    above_floor_cm: Optional[float] = None,
    on_floor: bool = False,
) -> Tuple[float, float, float]:
    """格子 (row, col) の箱中心 [cm]。

    on_floor=True のとき床上面 + 半辺 + 2 cm に静置配置。
    物理落下時は above_floor_cm（既定 CUBE_SPAWN_ABOVE_FLOOR_CM）だけ上げる。
    """
    ox, oy = MAP_ORIGIN_XY_CM
    x = ox + col * CUBE_SIZE_CM + CUBE_HALF_CM
    y = oy + row * CUBE_SIZE_CM + CUBE_HALF_CM
    if on_floor or not CUBE_ENABLE_PHYSICS:
        z = FLOOR_TOP_Z_CM + CUBE_HALF_CM + 2.0
    else:
        drop_cm = CUBE_SPAWN_ABOVE_FLOOR_CM if above_floor_cm is None else above_floor_cm
        z = FLOOR_TOP_Z_CM + CUBE_HALF_CM + drop_cm
    return x, y, z


def agent_spawn_xyz_cm(
    map_xy_m: Tuple[float, float],
    *,
    spawn_z_cm: float,
) -> Tuple[float, float, float]:
    """Humanoid / Robot スポーン位置（床面上、物理落下なし）。"""
    x, y = map_xy_m_to_world_cm(map_xy_m)
    return x, y, spawn_z_cm


# ==============================================================
# UnrealCV ヘルパ
# ==============================================================

def actor_names(ucv: UnrealCV) -> set[str]:
    return {str(name) for name in ucv.get_objects().tolist()}


def actor_exists(ucv: UnrealCV, name: str) -> bool:
    return name in actor_names(ucv)


def destroy_if_exists(ucv: UnrealCV, name: str) -> None:
    if actor_exists(ucv, name):
        ucv.set_physics(name, False)
        ucv.set_collision(name, False)
        ucv.destroy(name)


def spawn_bp(ucv: UnrealCV, bp_path: str, name: str) -> bool:
    ucv.spawn_bp_asset(bp_path, name)
    time.sleep(0.02)
    return actor_exists(ucv, name)


def enable_cube_blocking(ucv: UnrealCV, cube_id: str) -> None:
    """BP_TransparentCube を可視化し、コリジョンを有効化する。"""
    try:
        ucv.client.request(f"vbp {cube_id} SetBlocking True")
    except Exception as exc:
        print(f"  warn: SetBlocking failed for {cube_id}: {exc}")
    ucv.set_collision(cube_id, True)


def spawn_with_physics_drop(
    ucv: UnrealCV,
    bp_path: str,
    name: str,
    location: Tuple[float, float, float],
    *,
    enable_physics: bool = True,
    use_set_blocking: bool = False,
) -> bool:
    """物理 OFF で上空配置 → コリジョン ON → 物理 ON（material_transport と同順）。"""
    if not spawn_bp(ucv, bp_path, name):
        return False

    ucv.set_physics(name, False)
    ucv.set_collision(name, False)
    ucv.set_movable(name, True)
    ucv.set_location(list(location), name)
    ucv.set_orientation((0.0, 0.0, 0.0), name)
    time.sleep(PHYSICS_ENABLE_DELAY_S)

    if use_set_blocking:
        enable_cube_blocking(ucv, name)
    else:
        ucv.set_collision(name, True)

    if enable_physics:
        time.sleep(PHYSICS_ENABLE_DELAY_S)
        ucv.set_physics(name, True)
    return True


def spawn_fixed_floor(ucv: UnrealCV) -> bool:
    """30 m 床を 1 m 高度に固定配置（物理 OFF、落ちない）。"""
    cx, cy = floor_center_xy_cm()
    loc = (cx, cy, FLOOR_ACTOR_Z_CM)

    destroy_if_exists(ucv, FLOOR_ACTOR_NAME)
    if not spawn_bp(ucv, FLOOR_BP, FLOOR_ACTOR_NAME):
        print("[Floor] spawn failed — PAK / BP パス / SimWorld 再起動を確認")
        return False

    ucv.set_physics(FLOOR_ACTOR_NAME, False)
    ucv.set_movable(FLOOR_ACTOR_NAME, False)
    ucv.set_collision(FLOOR_ACTOR_NAME, True)
    ucv.set_location(list(loc), FLOOR_ACTOR_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), FLOOR_ACTOR_NAME)
    print(f"[Floor] fixed at center={loc} (corner @ {MAP_ORIGIN_XY_CM}, top Z≈{FLOOR_TOP_Z_CM} cm)")
    return True


def spawn_cubes(ucv: UnrealCV, grid_n: int) -> dict[str, dict]:
    registry: dict[str, dict] = {}
    total = grid_n * grid_n
    print(f"[Cubes] spawning {total} (GRID_N={grid_n}, physics={CUBE_ENABLE_PHYSICS})")

    for row in range(grid_n):
        for col in range(grid_n):
            cube_id = f"cube_{row:03d}_{col:03d}"
            loc = cube_center_cm(row, col)
            ok = spawn_with_physics_drop(
                ucv,
                CUBE_BP,
                cube_id,
                loc,
                enable_physics=CUBE_ENABLE_PHYSICS,
                use_set_blocking=True,
            )
            if ok:
                registry[cube_id] = {
                    "row": row,
                    "col": col,
                    "x_cm": loc[0],
                    "y_cm": loc[1],
                    "spawn_z_cm": loc[2],
                }
            else:
                print(f"  warn: failed {cube_id}")
            time.sleep(SPAWN_INTERVAL_S)

        if (row + 1) % 10 == 0 or row == grid_n - 1:
            print(f"  row {row + 1}/{grid_n} ({len(registry)}/{total})")

    print(f"[Cubes] done: {len(registry)}")
    return registry


def spawn_humanoid(communicator: Communicator, ucv: UnrealCV) -> Optional[str]:
    """Humanoid を床面上に配置（物理シミュレーションは使わない）。"""
    loc = agent_spawn_xyz_cm(HUMAN_MAP_XY_M, spawn_z_cm=HUMAN_SPAWN_Z_CM)
    human = Humanoid(position=Vector(loc[0], loc[1]), direction=Vector(1, 0))
    communicator.spawn_agent(
        agent=human,
        name=None,
        position=loc,
        model_path=HUMAN_BP,
        type="humanoid",
    )
    human_name = communicator.get_humanoid_name(human.id)
    ucv.set_physics(human_name, False)
    ucv.set_movable(human_name, True)
    ucv.set_collision(human_name, True)
    if AGENT_ENABLE_PHYSICS:
        print(
            f"  warn: AGENT_ENABLE_PHYSICS=True は Humanoid をラグドール化させるため無視します"
        )
    try:
        communicator.humanoid_set_speed(human.id, 0.0)
    except Exception:
        pass
    print(f"[Humanoid] {human_name} spawn @ {loc} (kinematic, no sim physics)")
    return human_name


def spawn_robot(ucv: UnrealCV) -> bool:
    """SpotDog を material_transport と同様に配置しコントローラを有効化。"""
    loc = agent_spawn_xyz_cm(ROBOT_MAP_XY_M, spawn_z_cm=ROBOT_SPAWN_Z_CM)
    destroy_if_exists(ucv, ROBOT_ACTOR_NAME)
    if not spawn_bp(ucv, ROBOT_BP, ROBOT_ACTOR_NAME):
        print("[Robot] spawn failed")
        return False

    ucv.set_physics(ROBOT_ACTOR_NAME, False)
    ucv.set_movable(ROBOT_ACTOR_NAME, True)
    ucv.set_location(list(loc), ROBOT_ACTOR_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), ROBOT_ACTOR_NAME)
    ucv.set_collision(ROBOT_ACTOR_NAME, True)
    ucv.enable_controller(ROBOT_ACTOR_NAME, True)
    if AGENT_ENABLE_PHYSICS:
        print(
            f"  warn: AGENT_ENABLE_PHYSICS=True は SpotDog を転倒させるため無視します"
        )
    print(f"[Robot] {ROBOT_ACTOR_NAME} @ {loc} (controller on, no sim physics)")
    return True


def _fmt_xyz(loc: Tuple[float, float, float]) -> str:
    return f"({loc[0]:.1f}, {loc[1]:.1f}, {loc[2]:.1f})"


def report_spawn_state(
    ucv: UnrealCV,
    cube_registry: dict[str, dict],
    human_name: Optional[str],
    *,
    floor_z_min_cm: float = FLOOR_TOP_Z_CM - 5.0,
) -> None:
    """スポーン後の位置をログ出力（箱の床外落下・エージェント転倒の確認用）。"""
    print("[Verify] actor locations after settle:")
    if actor_exists(ucv, FLOOR_ACTOR_NAME):
        floor_loc = ucv.get_location(FLOOR_ACTOR_NAME)
        print(f"  floor {FLOOR_ACTOR_NAME}: {_fmt_xyz(tuple(floor_loc))}")

    if human_name and actor_exists(ucv, human_name):
        loc = tuple(ucv.get_location(human_name))
        ok = loc[2] >= floor_z_min_cm
        print(f"  humanoid {human_name}: {_fmt_xyz(loc)} {'OK' if ok else 'LOW-Z?'}")
    elif human_name:
        print(f"  humanoid {human_name}: MISSING")

    if actor_exists(ucv, ROBOT_ACTOR_NAME):
        loc = tuple(ucv.get_location(ROBOT_ACTOR_NAME))
        ok = loc[2] >= floor_z_min_cm
        print(f"  robot {ROBOT_ACTOR_NAME}: {_fmt_xyz(loc)} {'OK' if ok else 'LOW-Z?'}")
    else:
        print(f"  robot {ROBOT_ACTOR_NAME}: MISSING")

    on_floor = 0
    below_floor = 0
    missing = 0
    sample_ids = sorted(cube_registry.keys())[:3]
    for cube_id in cube_registry:
        if not actor_exists(ucv, cube_id):
            missing += 1
            continue
        loc = tuple(ucv.get_location(cube_id))
        if loc[2] >= floor_z_min_cm:
            on_floor += 1
        else:
            below_floor += 1
    print(
        f"  cubes: on/above floor={on_floor}, below floor z={below_floor}, "
        f"missing={missing}, total={len(cube_registry)}"
    )
    for cube_id in sample_ids:
        if actor_exists(ucv, cube_id):
            loc = tuple(ucv.get_location(cube_id))
            print(f"    sample {cube_id}: {_fmt_xyz(loc)}")


def cleanup_spawned(ucv: UnrealCV, cube_ids: Iterable[str]) -> None:
    destroy_if_exists(ucv, FLOOR_ACTOR_NAME)
    destroy_if_exists(ucv, ROBOT_ACTOR_NAME)
    for cid in cube_ids:
        destroy_if_exists(ucv, cid)
    try:
        ucv.clean_garbage()
    except Exception:
        pass


# ==============================================================
# メイン
# ==============================================================

def main() -> None:
    print(
        f"[GridEnvHRI] map={ROOT.name}, GRID_N={GRID_N}, "
        f"floor_top_z={FLOOR_TOP_Z_CM} cm, origin={MAP_ORIGIN_XY_CM}"
    )
    print(
        "  Launch: SimWorld.exe -windowed -log /Game/Maps/empty.umap"
    )

    ucv = UnrealCV()
    communicator = Communicator(ucv)

    if not spawn_fixed_floor(ucv):
        return

    cube_registry = spawn_cubes(ucv, GRID_N)
    human_name = spawn_humanoid(communicator, ucv)
    robot_ok = spawn_robot(ucv)

    if SETTLE_AFTER_SPAWN_S > 0:
        print(f"[Settle] waiting {SETTLE_AFTER_SPAWN_S}s for physics ...")
        time.sleep(SETTLE_AFTER_SPAWN_S)

    report_spawn_state(ucv, cube_registry, human_name)

    print("[Done]")
    print(f"  floor: {FLOOR_ACTOR_NAME}")
    print(f"  cubes: {len(cube_registry)}")
    print(f"  humanoid: {human_name}")
    print(f"  robot: {ROBOT_ACTOR_NAME if robot_ok else 'FAILED'}")
    print(
        f"  human map=({HUMAN_MAP_XY_M[0]}m,{HUMAN_MAP_XY_M[1]}m) "
        f"robot map=({ROBOT_MAP_XY_M[0]}m,{ROBOT_MAP_XY_M[1]}m)"
    )


if __name__ == "__main__":
    main()
