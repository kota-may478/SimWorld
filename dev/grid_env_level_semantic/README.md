# grid_env_level_semantic

## English

### Purpose

Builds a **semantic block layer** on `/Game/Maps/Level`: for each cell in a defined height/rectangle region, assigns labels **`wall`**, **`floor`**, or **`air`** using **collision probing** (Approach C), spawns labeled blocks in PIE, writes a registry, and bakes actors into the saved map **`/Game/Maps/Level_semantic`**.

Related: `dev/grid_env_10k_semantic/` (grid map corner); this project targets the large Level terrain.

### File Reference

| File | Role |
|------|------|
| `grid_env_level_semantic.py` | Main orchestration: region grid, scan, spawn, registry updates |
| `grid_env_level_semantic.ipynb` | Interactive 5×5 PIE validation notebook |
| `level_camera_probe.ipynb` | Camera corner snapshot for region alignment |
| `level_camera_probe.py` | Script form of camera probe |
| `level_region.py` | Two corners + margin → `gx`/`gy` grid indices |
| `level_semantic_scan.py` | Approach C: collision probe labels (depth fallback) |
| `level_collision_probe.py` | `ProbePointHit` Blueprint wrapper |
| `level_semantic_registry_io.py` | Load/save `.level_semantic_registry.json` |
| `level_semantic_spawn_status.py` | Track spawn progress / resume |
| `run_level_semantic_layer.py` | CLI: `--subgrid` for partial runs |
| `bake_level_semantic_editor.py` | UE Editor: registry → persistent actors |
| `create_semantic_collision_probe_editor.py` | UE Editor: create `BP_SemanticCollisionProbe` |
| `cleanup_all_level_sem_blocks.py` | Remove all semantic blocks from level |
| `release_ue_connection.py` | Close UnrealCV before CLI (single-client rule) |
| `spawn_fixed_height_verify.py` | Verify block spawn heights |
| `run_label_test_world_center.py` | Label correctness test at world center |
| `SAVE_LEVEL_SEMANTIC.md` | Full map save workflow |
| `ue_native/SemanticCollisionProbe.h` | Optional C++ parent for probe BP |
| `ue_native/SemanticCollisionProbe.cpp` | C++ implementation |
| `.level_semantic_registry.json` | Cell → label + spawn metadata |
| `.level_camera_snapshot.json` | Camera probe output |
| `test_level_region.py` | Unit tests for region math |
| `test_level_semantic_registry_io.py` | Registry I/O tests |
| `test_level_semantic_scan.py` | Scan logic tests |
| `_collision_probe_smoke_test.py` | PIE smoke: `ProbePointHit` returns JSON |
| `_cleanup_level_pie_actors.py` | PIE actor cleanup helper |
| `_label_height_compare_diagnostic.py` | Height vs label diagnostic |
| `_collision_*_diagnostic.py` | Various collision/depth sweep diagnostics |
| `_depth_*` | Depth fallback probe diagnostics |

### Running Simulations

| Entry | Type | Description |
|-------|------|-------------|
| `grid_env_level_semantic.ipynb` | Notebook | 5×5 PIE validation (recommended first) |
| `level_camera_probe.ipynb` | Notebook | Capture region corner references |
| `run_level_semantic_layer.py` | CLI | Scriptable subgrid or full region |

```bash
# 5×5 test
python dev/grid_env_level_semantic/run_level_semantic_layer.py --subgrid 1,1,5,5

# Full region (hours; use --allow-large-region)
python dev/grid_env_level_semantic/run_level_semantic_layer.py --subgrid none --allow-large-region
```

Before CLI: `python dev/grid_env_level_semantic/release_ue_connection.py` if a notebook kernel holds port 9000.

Full bake/save: `SAVE_LEVEL_SEMANTIC.md`.

### Configurable Parameters

| Parameter | Effect |
|-----------|--------|
| `--subgrid gx0,gy0,gx1,gy1` | Limit scan/spawn rectangle |
| `--allow-large-region` | Permit full ~74k cell run |
| Region corners | In notebook / `level_region.py` |
| Probe Z offsets | In `level_semantic_scan.py` (wall/floor/air rules) |
| Block visual modes | Transparent floor vs solid air/wall |

### Future Extensibility

- Share `level_semantic_scan.py` with `grid_env_10k_semantic/block_semantic_scan.py`.
- Parallelize subgrid runs with merged registry checkpoints.
- Deprecate depth fallback once `BP_SemanticCollisionProbe` is always present.

---

## 日本語

### 目的

`/Game/Maps/Level` 上に **意味ブロック層** を構築。矩形領域の各セルに **コリジョン probe**（Approach C）で `wall` / `floor` / `air` を付与し、PIE でスポーン、レジストリ出力、Editor 焼き付けで **`/Game/Maps/Level_semantic`** を保存。

関連: `grid_env_10k_semantic`（格子マップ隅）。本プロジェクトは Level 大規模地形向け。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `grid_env_level_semantic.py` | メイン: 領域格子、スキャン、スポーン、レジストリ更新 |
| `grid_env_level_semantic.ipynb` | 5×5 PIE 検証用対話ノートブック |
| `level_camera_probe.ipynb` | 領域合わせ用カメラ角スナップショット |
| `level_camera_probe.py` | カメラプローブのスクリプト版 |
| `level_region.py` | 2 角 + マージン → `gx`/`gy` 格子インデックス |
| `level_semantic_scan.py` | Approach C: コリジョン probe ラベル（深度フォールバック） |
| `level_collision_probe.py` | `ProbePointHit` Blueprint ラッパ |
| `level_semantic_registry_io.py` | `.level_semantic_registry.json` 読み書き |
| `level_semantic_spawn_status.py` | スポーン進捗・再開管理 |
| `run_level_semantic_layer.py` | CLI: `--subgrid` で部分実行 |
| `bake_level_semantic_editor.py` | UE Editor: レジストリ → 永続アクター |
| `create_semantic_collision_probe_editor.py` | UE Editor: `BP_SemanticCollisionProbe` 作成 |
| `cleanup_all_level_sem_blocks.py` | レベル上の意味ブロック全削除 |
| `release_ue_connection.py` | CLI 前の UnrealCV 切断（単一クライアント） |
| `spawn_fixed_height_verify.py` | ブロックスポーン高度の検証 |
| `run_label_test_world_center.py` | ワールド中心でのラベル正しさテスト |
| `SAVE_LEVEL_SEMANTIC.md` | マップ保存ワークフロー |
| `ue_native/SemanticCollisionProbe.h` | probe BP 用 C++ 親（任意） |
| `ue_native/SemanticCollisionProbe.cpp` | C++ 実装 |
| `.level_semantic_registry.json` | セル → ラベル + スポーンメタデータ |
| `.level_camera_snapshot.json` | カメラプローブ出力 |
| `test_level_region.py` | 領域計算のユニットテスト |
| `test_level_semantic_registry_io.py` | レジストリ I/O テスト |
| `test_level_semantic_scan.py` | スキャンロジックテスト |
| `_collision_probe_smoke_test.py` | PIE スモーク: `ProbePointHit` が JSON を返すこと |
| `_cleanup_level_pie_actors.py` | PIE アクター掃除ヘルパ |
| `_label_height_compare_diagnostic.py` | 高度とラベルの診断 |
| `_collision_*_diagnostic.py` | コリジョン/深度スイープ診断 |
| `_depth_*` | 深度フォールバック probe 診断 |

### シミュレーションの実行

| エントリ | 種別 | 説明 |
|---------|------|------|
| `grid_env_level_semantic.ipynb` | ノートブック | 5×5 PIE 検証 |
| `level_camera_probe.ipynb` | ノートブック | 領域角のカメラ取得 |
| `run_level_semantic_layer.py` | CLI | サブグリッド/全領域 |

```bash
python dev/grid_env_level_semantic/run_level_semantic_layer.py --subgrid 1,1,5,5
```

CLI 前: 必要なら `release_ue_connection.py`。保存: `SAVE_LEVEL_SEMANTIC.md`。

### 変更可能なパラメータ

| パラメータ | 効果 |
|-----------|------|
| `--subgrid` | 実行矩形 |
| `--allow-large-region` | 全セル実行の許可 |
| 領域角 | `level_region.py` / ノートブック |
| Probe Z | `level_semantic_scan.py` の wall/floor/air 規則 |

### 今後の拡張性

- `grid_env_10k_semantic` との scan モジュール共通化。
- サブグリッド並列とチェックポイントマージ。
- `BP_SemanticCollisionProbe` 常備後の深度フォールバック廃止。
