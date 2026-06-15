# grid_env_level_nav

`/Game/Maps/Level` 向け **階層ナビゲーション**（NavMesh L0 + コストマップ L1/L2）の開発ディレクトリ。

手順書: Obsidian #320 `simWorld_LevelNavMeshNavigation_ForHRCMaterialTransport.md`

## ファイル一覧

| Phase | ファイル | 用途 |
| --- | --- | --- |
| 2 | `nav_query.py`, `level_coords.py`, `ue_native/` | NavQueryService / 座標変換 |
| 3 | `l0_nav_mask.py`, `build_l0_nav_mask.py`, `visualize_merged_costmap.py` | L0 静的マスク |
| 4 | `zone_catalog.py`, `zone_registry.py`, `costmap_layers.py` | L1 ゾーン封鎖 |
| 5 | `spotdog_nav_follower.py`, `run_spotdog_layered_nav_smoke.py` | A* + SpotDog 追従 |
| 6 | `perception_layer.py` | L2 エゴセントリック depth |
| 7 | `level_nav_adapter.py` | material transport 接続アダプタ |

## Phase 3 — L0 マスク（PIE 必須）

```bash
# 本番（30 cm, 約 5 時間 @300ms/cell）
python dev/grid_env_level_nav/build_l0_nav_mask.py \
  --resolution-cm 30 \
  --output dev/grid_env_level_nav/cache/l0_mask_30cm.npz

# 開発用クイック（100 cm, stride 2, 約 8 分）
python dev/grid_env_level_nav/build_l0_nav_mask.py --quick \
  --output dev/grid_env_level_nav/cache/l0_mask_100cm_quick.npz
```

可視化:

```bash
python dev/grid_env_level_nav/visualize_merged_costmap.py \
  --l0 dev/grid_env_level_nav/cache/l0_mask_100cm_quick.npz \
  --output dev/grid_env_level_nav/cache/l0_viz_quick.png --no-show
```

## Phase 4 — ゾーンカタログ（後から座標を登録）

ラベル（`RoomA`, `RoomB`, `RoomD`, `AreaA`, …）と **ローカル座標矩形**（UE 変換前、原点 = 世界 (-1000,-2200)）を
JSON ピックリストで管理。セル index は L0 解像度に合わせて **実行時に自動計算**。

1. テンプレートをコピーして編集:

```bash
cp dev/grid_env_level_nav/cache/zone_catalog.template.json \
   dev/grid_env_level_nav/cache/zone_catalog.json
# 各 zone の local_xy_cm: [[lx0,ly0],[lx1,ly1]] を UE で測った値に更新
```

2. 登録内容の確認:

```bash
python dev/grid_env_level_nav/print_zone_picklist.py \
  --catalog dev/grid_env_level_nav/cache/zone_catalog.json \
  --resolution-cm 30
```

3. 封鎖スモーク（plan-only）:

```bash
python dev/grid_env_level_nav/run_spotdog_layered_nav_smoke.py --plan-only \
  --l0 dev/grid_env_level_nav/cache/l0_mask_30cm.npz \
  --catalog dev/grid_env_level_nav/cache/zone_catalog.json \
  --close-zone RoomD
```

`world_xy_cm` 形式も利用可（`kind: rect_world`）。legacy の `zone_registry.json`（セル直書き）も `--zones` で可。

## Phase 5 — スモーク

```bash
# 経路のみ（UE 不要）
python dev/grid_env_level_nav/run_spotdog_layered_nav_smoke.py --plan-only \
  --l0 dev/grid_env_level_nav/cache/l0_mask_100cm_quick.npz

# Room D 封鎖込み
python dev/grid_env_level_nav/run_spotdog_layered_nav_smoke.py --plan-only \
  --l0 dev/grid_env_level_nav/cache/l0_mask_100cm_quick.npz \
  --zones dev/grid_env_level_nav/cache/zone_registry_100cm.json \
  --close-zone RoomD

# SpotDog 実走（PIE + 目視確認）
python dev/grid_env_level_nav/run_spotdog_layered_nav_smoke.py \
  --l0 dev/grid_env_level_nav/cache/l0_mask_100cm_quick.npz \
  --start-local 500 500 --goal-local 5000 6000
```

## Phase 7 — material transport

```python
from level_nav_adapter import LevelNavSession, apply_instruction_to_zones

session = LevelNavSession.from_cache(
    "dev/grid_env_level_nav/cache/l0_mask_30cm.npz",
    "dev/grid_env_level_nav/cache/zone_registry.json",
)
apply_instruction_to_zones(session, "Room D は通行できません")
session.navigate_robot_local(ucv, goal_local=(50.0 * 100, 60.0 * 100))
```

`material_transport_llm.py` の `set_transport_costmap(session.merged_costmap())` と
`robot_navigate_planned_leg` の差し替えは、Level マップへ切り替える際に上記セッションを使う。

## 単体テスト（UE 不要）

```bash
python -m unittest discover -s dev/grid_env_level_nav -p 'test_*.py' -v
```
