# grid_env_level_nav

Layered navigation for `/Game/Maps/Level`: **L0** NavMesh static mask, **L1** zone closures, **L2** FusionCam perception obstacles, A* planning, and SpotDog PIE tests.

Design note: Obsidian #320 `simWorld_LevelNavMeshNavigation_ForHRCMaterialTransport.md`

---

## Directory layout (English)

| Path | Purpose |
|------|---------|
| **`paths.py`** | Canonical paths for caches, registries, and run artifacts |
| **`bootstrap.py`** | `sys.path` setup for scripts and scenarios |
| **`pie_spawn_safety.py`** | Level PIE destroy→spawn helpers (reduces UE Editor crashes) |
| **Core modules** (package root) | Shared libraries: `level_coords.py`, `costmap_layers.py`, `l0_nav_mask.py`, `nav_query.py`, `perception_layer.py`, `level_nav_robot.py`, `prop_catalog.py`, … |
| **`scenarios/compact_nav/`** | 30 m × 30 m FusionCam L2 test (3 props, SpotDog to 25 m goal) |
| **`scenarios/construction_site/`** | Larger material-transport scenario (20 prop types, carry visual) |
| **`scripts/`** | Unreal Editor utility scripts (Blueprint generation, mesh repair) |
| **`ue_native/`** | C++ NavQueryService plugin sources |
| **`cache/l0/`** | Built L0 NavMesh masks (`.npz`) and preview PNGs |
| **`cache/registries/`** | Placement registries, zone catalogs, prop catalogs (`.json`) |
| **`cache/runs/compact_nav/`** | Compact-nav run outputs: costmap PNG/NPZ, trajectory JSON, RMSE plots |
| **Root entry scripts** | Thin wrappers (`run_compact_nav_test.py`, …) delegating to `scenarios/*` |

### `scenarios/compact_nav/` files

| File | Role |
|------|------|
| `region.py` | 30 m work region constants (start, goal, L0 path) |
| `placement.py` | 3-prop layout registry |
| `l0_crop.py` | Crop full L0 mask to 30 m sub-region |
| `l2_fusion.py` | FusionCam detections → L2 lethal cells |
| `layered_nav.py` | L0 + L2 navigation with replanning |
| `viz.py` | Post-run artifacts (costmap PNG, trajectory JSON, RMSE plots) |
| `spawn_pie.py` | PIE spawn props + SpotDog (PIE-safe destroy/spawn) |
| `run_test.py` | End-to-end test runner (single UE session: spawn + nav) |
| `regenerate_viz.py` | Re-render PNG from saved NPZ + trajectory |
| `test_placement.py` | Unit tests (no UE) |

### `cache/runs/compact_nav/` artifacts (per run)

| Artifact | Description |
|----------|-------------|
| `latest_costmap_png.png` | L0 / L2 / Merged + trajectory overlay |
| `latest_costmap_npz.npz` | Final L0, L1, L2, merged arrays |
| `latest_trajectory_json.json` | Robot path, replan events, GT prop positions |
| `latest_distance_png.png` | Distance GT vs estimate time series + RMSE |
| `latest_bearing_png.png` | Bearing GT vs estimate time series + RMSE |
| `latest_rmse_json.json` | Numeric RMSE summary |

Prop markers on costmap PNGs labelled **`(GT)`** use registry placement coordinates (not FusionCam estimates). Blue **`+`** markers on the L2 panel are perception-based position estimates.

### UE Editor crash mitigation

Level PIE is fragile when:

1. **Destroy → spawn too fast** — UE may reset UnrealCV (`Connection reset by peer`) or crash the Editor.
2. **Two scripts connect to :9000** — stale sockets in `CLOSE-WAIT` block reconnects (`ue_client_guard.py` in `grid_env_hri`).

Fixes applied:

- `pie_safety.cooldown_before_spawn_batch()` — 3 s idle after destroy batches
- `pie_spawn_safety.destroy_by_prefix()` — per-actor destroy + settle + cooldown
- `run_test.py` — **one UnrealCV session** for spawn and navigation (no disconnect between phases)
- `ue_client_guard.prepare_ue_connection()` — exclusive lock, stale client cleanup, graceful shutdown
- `MPLBACKEND=Agg` in `run_test.py` / `viz.py` — headless matplotlib (avoids Qt abort after nav on WSL)

---

## Quick start — compact nav (English)

**Prerequisites:** UE Editor, `/Game/Maps/Level` in PIE, L0 cache at `cache/l0/l0_mask_30cm_strict.npz`

```bash
# Plan only (no UE)
conda run -n simworld python dev/grid_env_level_nav/run_compact_nav_test.py --plan-only

# Full test: spawn + navigate + artifacts (single UE session)
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld \
  python dev/grid_env_level_nav/run_compact_nav_test.py
```

Spawn only:

```bash
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld \
  python dev/grid_env_level_nav/spawn_compact_nav_scene_pie.py --force-rebuild
```

Unit tests (no UE):

```bash
conda run -n simworld python dev/grid_env_level_nav/scenarios/compact_nav/test_placement.py
python -m unittest discover -s dev/grid_env_level_nav -p 'test_*.py' -v
```

---

## L0 mask build

```bash
python dev/grid_env_level_nav/build_l0_nav_mask.py \
  --resolution-cm 30 \
  --output dev/grid_env_level_nav/cache/l0/l0_mask_30cm_strict.npz
```

---

## ディレクトリ構成（日本語）

| パス | 目的 |
|------|------|
| **`paths.py`** | キャッシュ・レジストリ・実行成果物の標準パス |
| **`bootstrap.py`** | スクリプト用 `sys.path` 初期化 |
| **`pie_spawn_safety.py`** | PIE 向け destroy→spawn 安全化（UE クラッシュ対策） |
| **コアモジュール**（ルート直下） | 共通ライブラリ（座標変換、コストマップ、L0、NavQuery 等） |
| **`scenarios/compact_nav/`** | 30 m 四方の FusionCam L2 コンパクトテスト |
| **`scenarios/construction_site/`** | 建設現場シナリオ（大規模配置・運搬） |
| **`scripts/`** | UE Editor 用ユーティリティ |
| **`cache/l0/`** | L0 NavMesh マスクと可視化 PNG |
| **`cache/registries/`** | 配置レジストリ・ゾーン・カタログ JSON |
| **`cache/runs/compact_nav/`** | コンパクトナビの実行結果（コストマップ、軌跡、RMSE 図） |

### コンパクトナビ成果物

| ファイル | 内容 |
|----------|------|
| `latest_costmap_png.png` | L0 / L2 / マージ＋経路 |
| `latest_trajectory_json.json` | 走行軌跡・再計画履歴 |
| `latest_distance_png.png` / `latest_bearing_png.png` | 距離・方位の GT と推定値、RMSE |
| `latest_rmse_json.json` | RMSE 数値 |

コストマップ上の **`(GT)`** ラベルはレジストリの設置座標（真値）です。L2 パネルの青 **`+`** が FusionCam による位置推定です。

### UE Editor クラッシュ対策

主な原因:

1. **アクター削除直後の spawn** — UnrealCV 切断や Editor クラッシュ
2. **:9000 への二重接続** — ソケット未解放による接続失敗

対策:

- 削除バッチ後 **3 秒クールダウン**（`cooldown_before_spawn_batch`）
- **spawn とナビを同一 UnrealCV セッション**で実行（`run_test.py`）
- `ue_client_guard` による排他ロックと graceful shutdown
- `MPLBACKEND=Agg` — WSL 上での matplotlib Qt クラッシュ防止

---

## クイックスタート — コンパクトナビ（日本語）

**前提:** UE Editor で `/Game/Maps/Level` を PIE 起動、L0 が `cache/l0/l0_mask_30cm_strict.npz` に存在

```bash
# 経路計画のみ
conda run -n simworld python dev/grid_env_level_nav/run_compact_nav_test.py --plan-only

# 本番（spawn + ナビ + 成果物、単一 UE セッション）
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld \
  python dev/grid_env_level_nav/run_compact_nav_test.py
```

---

## Legacy paths

Pre-reorganization paths under `cache/` root are moved. Use `paths.py` constants. Old entry script names at package root still work via thin wrappers.
