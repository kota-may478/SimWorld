# grid_env_10k

## English

### Purpose

Builds and exercises a **100×100 grid** (10,000 actors) of semi-transparent blocks on top of `grid_env_hri`, targeting the saved map `/Game/Maps/grid_100x100`. Covers initial mass spawn, Humanoid/SpotDog patrol in PIE, a four-rooms sub-layout, traffic-system integration, and Editor-side baking scripts to persist the grid into a `.umap` asset.

Design reference: extends `dev/grid_env_hri` with 1-indexed block naming `block_{gx:03d}_{gy:03d}`.

### File Reference

| File | Role |
|------|------|
| `grid_env_10k.py` | Core API: block indices, actor names, spawn batches, phase-1 spawn orchestration, imports `grid_env_hri_simulation` |
| `grid_env_10k.ipynb` | Interactive notebook for grid spawn and exploration |
| `grid_env_10k_pie_patrol.py` | SpotDog/Humanoid patrol helpers in PIE (`get_pos2d`, `get_yaw`, `dist2d`, movement loops) |
| `grid_env_10k_pie_patrol.ipynb` | Notebook for PIE patrol demos |
| `grid_env_10k_four_rooms_layout.py` | Defines a four-rooms block layout subset on the 100×100 grid |
| `grid_env_10k_four_rooms_pie.py` | PIE spawn for the four-rooms layout |
| `grid_env_10k_four_rooms.ipynb` | Notebook for four-rooms scenario |
| `run_phase1_spawn.py` | CLI: spawn all blocks + agents (`BLOCK_SPAWN_DRY_RUN_N` for small test) |
| `run_humanoid_spawn_test.py` | Humanoid spawn verification after traffic-system setup |
| `wait_for_ue_port.py` | Poll until UnrealCV port 9000 is ready |
| `prepare_map_for_save.py` | Remove runtime-only actors before saving `grid_100x100` |
| `verify_phase1_log.py` | Parse spawn logs for 10,000/10,000 completion |
| `bake_grid_100x100_editor.py` | UE Editor: bake grid blocks from registry |
| `load_grid_region_editor.py` | UE Editor: load a grid region into the level |
| `dedupe_grid_blocks_editor.py` | UE Editor: remove duplicate block actors |
| `_pie_diagnostic.py` | PIE connection diagnostic script |
| `.pie_block_registry.json` | Cached block registry for PIE sessions |
| `SAVE_MAP_GRID_100x100.md` | Step-by-step guide to save `/Game/Maps/grid_100x100` |
| `test_grid_env_10k_coords.py` | Unit tests for coordinate transforms |
| `test_grid_env_10k_layout.py` | Unit tests for layout definitions |
| `test_grid_env_10k_four_rooms_layout.py` | Unit tests for four-rooms layout |
| `scripts/run_pie_patrol_with_wait.py` | Wrapper: wait for UE then run patrol |
| `scripts/run_four_rooms_with_wait.py` | Wrapper: wait for UE then run four-rooms PIE |
| `scripts/mount_simworld_runtime_paks_pie.py` | Mount runtime paks for PIE |
| `scripts/mount_simworld_runtime_paks_editor.py` | Mount runtime paks in Editor |
| `scripts/install_simworld_runtime_paks_editor.sh` | Shell: install paks for Editor |
| `scripts/prepare_spotdog_pie_assets.sh` | Shell: prepare SpotDog PIE assets |
| `scripts/install_robot_dog_editor.sh` | Shell: install robot dog Editor assets |
| `scripts/compile_robot_dog_editor.py` | UE Editor: compile robot dog blueprints |
| `scripts/create_interactable_stub_editor.py` | UE Editor: create interactable stub BP |
| `scripts/finalize_agent_for_editor.sh` | Shell: finalize agent assets |
| `scripts/compile_traffic_system_editor.py` | UE Editor: compile traffic system |
| `scripts/verify_traffic_system_editor.py` | UE Editor: verify traffic system |
| `scripts/verify_traffic_system_preflight.sh` | Shell: preflight checks before traffic compile |
| `scripts/install_traffic_system_editor.sh` | Shell: install traffic system for Editor |
| `scripts/disable_human_avatar_editor.ps1` | PowerShell: disable human avatar in Editor |
| `scripts/_probe_ue_port.py` | Diagnostic: probe UE port |
| `scripts/_probe_humanoid_bps.py` | Diagnostic: list humanoid blueprint paths |
| `scripts/_mount_all_paks_probe_humanoid.py` | Diagnostic: mount paks and probe humanoid |
| `scripts/_release_jupyter_ue_session.py` | Release Jupyter-held UE connection |

### Running Simulations

**Notebooks (interactive):**

| Notebook | Use when |
|----------|----------|
| `grid_env_10k.ipynb` | Exploring block spawn on empty/grid map |
| `grid_env_10k_pie_patrol.ipynb` | SpotDog patrol in PIE |
| `grid_env_10k_four_rooms.ipynb` | Four-rooms sub-layout demo |

**CLI (reproducible):**

```bash
# Phase 1: spawn 10k blocks (or dry run)
BLOCK_SPAWN_DRY_RUN_N=5 python dev/grid_env_10k/run_phase1_spawn.py

# PIE patrol (UE Play on grid_100x100 or empty per script docs)
python dev/grid_env_10k/grid_env_10k_pie_patrol.py
```

Prerequisites: UE running, `conda activate simworld`, paks mounted per `SAVE_MAP_GRID_100x100.md`.

### Configurable Parameters

| Parameter | Location | Effect |
|-----------|----------|--------|
| `BLOCK_GRID_N` | `grid_env_10k.py` / env | Grid size (default 100) |
| `BLOCK_SPAWN_INTERVAL_S` | env | Delay between block spawns |
| `BLOCK_SPAWN_DRY_RUN_N` | `run_phase1_spawn.py` | If >0, spawn only N×N blocks |
| `BLOCK_ACTOR_PREFIX` | env | Actor name prefix (default `block`) |
| `BLOCK_GRID_N`, scenario steps | notebooks / `grid_env_10k_four_rooms_layout.py` | Sub-region layout bounds |

### Future Extensibility

- Complete migration from `empty.umap` one-shot spawn to loading pre-baked `grid_100x100` only.
- Share patrol helpers (`grid_env_10k_pie_patrol.py`) via a thin `dev/_common/ue_motion.py` to reduce cross-imports from `grid_env_level_nav`.
- CI job with `BLOCK_SPAWN_DRY_RUN_N=3` for fast regression without full 80-minute spawn.

---

## 日本語

### 目的

`grid_env_hri` 上に **100×100 格子**（1 万アクター）の半透明ブロックを構築・検証し、保存マップ `/Game/Maps/grid_100x100` を対象とします。大量スポーン、PIE パトロール、四部屋サブレイアウト、交通システム連携、Editor 焼き付けを含みます。

ブロック名: `block_{gx:03d}_{gy:03d}`（1 始まりインデックス）。

### ファイル一覧

| ファイル | 役割 |
|------|------|
| `grid_env_10k.py` | コア API: ブロックインデックス、アクター名、スポーン、フェーズ1オーケストレーション、`grid_env_hri_simulation` インポート |
| `grid_env_10k.ipynb` | 格子スポーン・探索用対話ノートブック |
| `grid_env_10k_pie_patrol.py` | PIE パトロールヘルパ（`get_pos2d`、`get_yaw`、`dist2d`、移動ループ） |
| `grid_env_10k_pie_patrol.ipynb` | PIE パトロールデモ用ノートブック |
| `grid_env_10k_four_rooms_layout.py` | 100×100 格子上の四部屋サブレイアウト定義 |
| `grid_env_10k_four_rooms_pie.py` | 四部屋レイアウトの PIE スポーン |
| `grid_env_10k_four_rooms.ipynb` | 四部屋シナリオ用ノートブック |
| `run_phase1_spawn.py` | CLI: 全ブロック+エージェントスポーン（`BLOCK_SPAWN_DRY_RUN_N` で小規模テスト） |
| `run_humanoid_spawn_test.py` | 交通システム設定後の Humanoid スポーン検証 |
| `wait_for_ue_port.py` | UnrealCV 9000 番ポート待機 |
| `prepare_map_for_save.py` | `grid_100x100` 保存前にランタイム専用アクター削除 |
| `verify_phase1_log.py` | スポーンログの 10000/10000 完了解析 |
| `bake_grid_100x100_editor.py` | UE Editor: レジストリから格子ブロック焼き付け |
| `load_grid_region_editor.py` | UE Editor: 格子領域をレベルに読み込み |
| `dedupe_grid_blocks_editor.py` | UE Editor: 重複ブロックアクター削除 |
| `_pie_diagnostic.py` | PIE 接続診断 |
| `.pie_block_registry.json` | PIE セッション用ブロックレジストリキャッシュ |
| `SAVE_MAP_GRID_100x100.md` | `/Game/Maps/grid_100x100` 保存手順 |
| `test_grid_env_10k_coords.py` | 座標変換のユニットテスト |
| `test_grid_env_10k_layout.py` | レイアウト定義のユニットテスト |
| `test_grid_env_10k_four_rooms_layout.py` | 四部屋レイアウトのユニットテスト |
| `scripts/run_pie_patrol_with_wait.py` | UE 待機後にパトロール実行 |
| `scripts/run_four_rooms_with_wait.py` | UE 待機後に四部屋 PIE 実行 |
| `scripts/mount_simworld_runtime_paks_pie.py` | PIE 用ランタイム pak マウント |
| `scripts/mount_simworld_runtime_paks_editor.py` | Editor 用ランタイム pak マウント |
| `scripts/install_simworld_runtime_paks_editor.sh` | Editor 用 pak インストール |
| `scripts/prepare_spotdog_pie_assets.sh` | SpotDog PIE アセット準備 |
| `scripts/install_robot_dog_editor.sh` | ロボット犬 Editor アセットインストール |
| `scripts/compile_robot_dog_editor.py` | UE Editor: ロボット犬 BP コンパイル |
| `scripts/create_interactable_stub_editor.py` | UE Editor: インタラクトスタブ BP 作成 |
| `scripts/finalize_agent_for_editor.sh` | エージェントアセット最終化 |
| `scripts/compile_traffic_system_editor.py` | UE Editor: 交通システムコンパイル |
| `scripts/verify_traffic_system_editor.py` | UE Editor: 交通システム検証 |
| `scripts/verify_traffic_system_preflight.sh` | コンパイル前プリフライト |
| `scripts/install_traffic_system_editor.sh` | Editor 用交通システムインストール |
| `scripts/disable_human_avatar_editor.ps1` | Editor でヒューマンアバター無効化 |
| `scripts/_probe_ue_port.py` | UE ポート診断 |
| `scripts/_probe_humanoid_bps.py` | Humanoid BP パス一覧 |
| `scripts/_mount_all_paks_probe_humanoid.py` | pak マウント+Humanoid 診断 |
| `scripts/_release_jupyter_ue_session.py` | Jupyter の UE 接続解放 |

### シミュレーションの実行

**ノートブック:** `grid_env_10k.ipynb`、`grid_env_10k_pie_patrol.ipynb`、`grid_env_10k_four_rooms.ipynb`

**CLI 例:**

```bash
BLOCK_SPAWN_DRY_RUN_N=5 python dev/grid_env_10k/run_phase1_spawn.py
python dev/grid_env_10k/grid_env_10k_pie_patrol.py
```

前提: UE 起動、`conda activate simworld`、`SAVE_MAP_GRID_100x100.md` の pak 手順。

### 変更可能なパラメータ

| パラメータ | 場所 | 効果 |
|-----------|------|------|
| `BLOCK_GRID_N` | 環境変数 | 格子サイズ（既定 100） |
| `BLOCK_SPAWN_INTERVAL_S` | 環境変数 | スポーン間隔 |
| `BLOCK_SPAWN_DRY_RUN_N` | `run_phase1_spawn.py` | 小規模ドライラン |
| `BLOCK_ACTOR_PREFIX` | 環境変数 | アクター名接頭辞 |

### 今後の拡張性

- 焼き付け済み `grid_100x100` のみ読み込む運用への移行。
- パトロールヘルパの共通化で `grid_env_level_nav` からの横断 import 削減。
- `BLOCK_SPAWN_DRY_RUN_N=3` による高速 CI 回帰。
