# Level_semantic マップ保存手順

## 概要

1. **PIE** で `grid_env_level_semantic` がブロックをスポーンし `.level_semantic_registry.json` を書く
2. **UE Editor** で registry から Actor を焼き付け、**Level_semantic** として保存

保存先アセット: `/Game/Maps/Level_semantic`

---

## Phase A — PIE でラベル付きブロック配置（WSL）

1. UE Editor で `/Game/Maps/Level` を開き **PIE 開始**
2. WSL:

```bash
conda activate simworld
cd ~/01_Private/Program/SimWorld/dev/grid_env_level_semantic
```

3. まず **5×5 検証**（ノート `grid_env_level_semantic.ipynb` または CLI）:

```bash
python run_level_semantic_layer.py --subgrid 1,1,5,5
```

4. 全領域（約 7.4 万セル、数時間級）:

```bash
python run_level_semantic_layer.py --subgrid none --allow-large-region
```

5. 成功時: `.level_semantic_registry.json` が更新される

---

## Phase B — Editor で registry を焼き付け

1. **PIE を停止**（Editor に戻る）
2. `/Game/Maps/Level` を開いたまま（または作業用レベル）
3. **Tools → Execute Python Script** → `bake_level_semantic_editor.py`
4. World Outliner で `level_sem_block_` を検索し Actor があることを確認
5. **File → Save Current Level As…** → `/Game/Maps/Level_semantic`

### ブロック名

`level_sem_block_{gx:03d}_{gy:03d}`（例: `level_sem_block_001_001`）

### 見た目

- **floor** → 透過 (SetBlocking False / F)
- **air / wall** → 実体 (T)

PIE 中の見た目を Editor 保存後も揃える場合は、必要に応じて各 BP の collision / SetBlocking を目視確認してください。

---

## 座標・領域（現在の設定）

| 項目 | 値 |
|------|-----|
| 角 A XY (cm) | (-1140.012, -2139.228) |
| 角 B XY (cm) | (6013.30, 5860.17) |
| ブロック下面 Z (cm) | 6477.101（角 A の Z、自動補正あり ±0.15 m） |
| 外側マージン | 各辺 +3 m |
| セル | 0.3 m、gx/gy は 1 始まり |

変更する場合は `level_region.py` の定数を編集してください。
