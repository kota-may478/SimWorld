# grid_env_10k_semantic

## English

### Purpose

Assigns **semantic labels** (`wall`, `floor`, `air`) to cells on the `grid_100x100` map by probing collision geometry before placing blocks. Validates the labeling pipeline on a **5×5 corner** (3×3 temporary floor inside) before scaling to larger regions. Builds on `grid_env_10k` block indexing and `grid_env_hri` UE access.

### File Reference

| File | Role |
|------|------|
| `grid_env_10k_semantic.py` | Main orchestration: temp floor, block fill, semantic scan, registry I/O, visual modes (transparent vs solid) |
| `grid_env_10k_semantic.ipynb` | Interactive notebook for corner semantic labeling demo |
| `block_semantic_scan.py` | Per-cell collision probe logic → `BlockSemantic` label |
| `run_semantic_layer_demo.py` | CLI demo runner for the 5×5 corner test |
| `test_semantic_classifier.py` | Unit tests for label rules (wall/floor/air) |
| `_probe_diagnostic.py` | Diagnostic probes for collision/depth during development |
| `.semantic_layer_registry.json` | Persisted semantic labels per `(gx, gy)` block index |

### Running Simulations

| Entry | Type | Description |
|-------|------|-------------|
| `grid_env_10k_semantic.ipynb` | Notebook | Primary interactive run on `grid_100x100` corner (UE PIE required) |
| `run_semantic_layer_demo.py` | CLI | Headless/scriptable 5×5 demo |

```bash
conda activate simworld
python dev/grid_env_10k_semantic/run_semantic_layer_demo.py
```

Prerequisites: `/Game/Maps/grid_100x100` (or compatible grid) in **PIE**, blocks from `grid_env_10k` phase-1 spawn.

### Configurable Parameters

Defined in `grid_env_10k_semantic.py` and `block_semantic_scan.py`:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `FILL_GX0/GY0` – `FILL_GX1/GY1` | 1–5 | 5×5 fill region |
| `TEMP_FLOOR_GX0/GY0` – `TEMP_FLOOR_GX1/GY1` | 1–3 | 3×3 temp floor |
| `BLOCK_GAP_ABOVE_FLOOR_M` | `0.15` | Block bottom height above floor |
| `DEFAULT_BLOCK_MODE` | `"F"` | Transparent (`F`) vs solid (`T`) |
| `SEMANTIC_VISUAL_MODES` | dict | Maps semantic class → block mode for visualization |

### Future Extensibility

- Reuse `block_semantic_scan.py` from `grid_env_level_semantic` via a shared package (both implement Approach C-style probing).
- Scale CLI to full 100×100 with checkpointed registry writes.
- Add visualization notebook cell for semantic heatmap on the corner region.

---

## 日本語

### 目的

`grid_100x100` 上のセルに **意味ラベル**（`wall` / `floor` / `air`）を、ブロック配置前のコリジョン probe で付与します。**5×5 隅**（内側 3×3 仮床）で検証後、拡大を想定。`grid_env_10k` のインデックスと `grid_env_hri` の UE アクセスを利用。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `grid_env_10k_semantic.py` | メイン: 仮床・充填・スキャン・レジストリ |
| `grid_env_10k_semantic.ipynb` | 対話デモ用ノートブック |
| `block_semantic_scan.py` | セル単位のラベル判定 |
| `run_semantic_layer_demo.py` | 5×5 CLI デモ |
| `test_semantic_classifier.py` | ラベル規則のユニットテスト |
| `_probe_diagnostic.py` | 開発用診断 |
| `.semantic_layer_registry.json` | `(gx, gy)` ごとのラベル永続化 |

### シミュレーションの実行

| エントリ | 種別 | 説明 |
|---------|------|------|
| `grid_env_10k_semantic.ipynb` | ノートブック | 隅デモ（PIE 必須） |
| `run_semantic_layer_demo.py` | CLI | スクリプト可能な 5×5 デモ |

```bash
conda activate simworld
python dev/grid_env_10k_semantic/run_semantic_layer_demo.py
```

前提: `grid_100x100` を PIE、`grid_env_10k` でブロック配置済み。

### 変更可能なパラメータ

| パラメータ | 既定 | 効果 |
|-----------|------|------|
| `FILL_*` | 1–5 | 5×5 充填領域 |
| `TEMP_FLOOR_*` | 1–3 | 3×3 仮床 |
| `BLOCK_GAP_ABOVE_FLOOR_M` | `0.15` | ブロック下面の高さオフセット |
| `DEFAULT_BLOCK_MODE` | `"F"` | 半透明/実体 |
| `SEMANTIC_VISUAL_MODES` | dict | ラベル→表示モード |

### 今後の拡張性

- `grid_env_level_semantic` と probe モジュールの共通化。
- 全 100×100 向けチェックポイント付き CLI。
- 隅領域の意味ヒートマップ可視化。
