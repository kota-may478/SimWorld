# grid_env_level_semantic

`/Game/Maps/Level` 上の指定高度・矩形領域に、wall / floor / air ラベル付きブロックを配置し、`/Game/Maps/Level_semantic` として保存するための作業ディレクトリです。

## ファイル

| ファイル | 用途 |
|----------|------|
| `level_camera_probe.ipynb` | カメラ座標取得（角の目印） |
| `grid_env_level_semantic.ipynb` | PIE でラベル付きブロック配置 |
| `grid_env_level_semantic.py` | 配置オーケストレーション |
| `level_region.py` | 2 角 + 3 m マージン → gx/gy グリッド |
| `level_semantic_scan.py` | Approach C コリジョン probe ラベル（深度はフォールバック） |
| `level_collision_probe.py` | `ProbePointHit` vbp ラッパー |
| `create_semantic_collision_probe_editor.py` | UE Editor: `BP_SemanticCollisionProbe` 作成 |
| `ue_native/SemanticCollisionProbe.*` | （任意）C++ 親クラスを SimWorld Source にコピー |
| `run_level_semantic_layer.py` | CLI |
| `bake_level_semantic_editor.py` | Editor: registry → Actor 焼き付け |
| `SAVE_LEVEL_SEMANTIC.md` | マップ保存手順 |

## UnrealCV 接続（単一クライアント）

UnrealCV は **TCP 1 本のみ**。ノートブックの Kernel が接続したままだと CLI がタイムアウトします。

```bash
# ノートを開いたまま CLI を回す前に
python release_ue_connection.py
# または Jupyter: Kernel → Restart
```

Windows `Get-NetTCPConnection -LocalPort 9000` で `Established` + `CloseWait` が残る場合も、WSL 側 Python の解放が必要です。

## Approach C（コリジョン probe）セットアップ

1. **UE Editor（Windows）** で `create_semantic_collision_probe_editor.py` を実行  
   （Tools → Execute Python Script）。ネイティブクラスが無い場合はログの手順で
   `ProbePointHit(X,Y,Z)` を Blueprint に追加して Compile / Save。
2. PIE 起動後、WSL で:
   `python _collision_probe_smoke_test.py` → `ProbePointHit` が JSON を返すこと。
3. ラベル検証: `python run_label_test_world_center.py`

`BP_SemanticCollisionProbe` が無い間は深度フォールバック（室内・崖端では不十分）。

## クイックスタート（PIE 5×5 検証）

1. Level を開き PIE
2. `conda activate simworld`
3. 他ノートの Kernel を Restart（または `release_ue_connection.py`）
4. `grid_env_level_semantic.ipynb` を実行（既定: gx,gy 1..5 の 25 セル）

全領域・保存: `SAVE_LEVEL_SEMANTIC.md` を参照。

参照: `dev/grid_env_10k_semantic/`
