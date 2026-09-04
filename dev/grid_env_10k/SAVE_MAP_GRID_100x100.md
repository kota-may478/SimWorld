# grid_100x100 専用マップの保存手順（Windows UE Editor）

前提: 10,000 個の `block_{gx:03d}_{gy:03d}` が SimWorld 上に存在し、**すべて F（半透明・SetBlocking False）** の状態で保存する。

## Q4 と Q5 の関係

| 項目 | 内容 |
| --- | --- |
| **Q4** | レベルアセット `/Game/Maps/grid_100x100` として保存する |
| **Q5** | 各ブロック Actor 名は `block_001_001` … `block_100_100`（ログで確認済み） |
| 関係 | マップに「何が置かれているか」は Q5 の命名規則。Q4 はその配置を `.umap` に焼き付ける作業 |

旧ドキュメントの `cube_*` 命名は **使わない**（`dev/grid_env_10k` の正本は `block_*`）。

---

## Phase A — Python で 10,000 個をスポーン（初回のみ）

1. Windows: `empty.umap` で SimWorld を起動（または既存セッションをクリア）。
2. WSL:

```bash
conda activate simworld
cd ~/01_Private/Program/SimWorld/dev/grid_env_10k
python run_phase1_spawn.py
```

3. 完了ログに `10000/10000` と `block_100_100` 等が出ることを確認（約 80 分）。

---

## Phase B — 保存用にランタイム Actor を除去（WSL）

SimWorld を **起動したまま**:

```bash
python prepare_map_for_save.py
```

削除対象: Humanoid、SpotDog、デモ立方体、`toggle_test_cube` など。  
残すもの: `grid_floor_main`（床）と `block_*` 10,000 個。

---

## Phase C — Windows UE Editor でレベルを保存

### Step 1. SimWorld を一度終了する

PowerShell で SimWorld プロセスを終了し、port 9000 を空ける。

### Step 2. UE プロジェクトを開く

1. Epic Launcher またはショートカットから **UE 5.3.2**（プロジェクトで使っているバージョン）を起動。
2. プロジェクト: SimWorld 用の **ベースプロジェクト**（`C:\UEProjects\SimWorld` 等、社内で §3.7 に記載のパス）を開く。
3. まだ pakchunk9002 を Editor に読み込んでいない場合は、§3.7 D の手順どおり `BP_Floor_30x30` / `BP_TransparentCube` が Content Browser に見える状態にする。

### Step 3. 実行中に作ったマップを開くか、現レベルを確認

- **推奨**: 直前に Python スポーンしたのと同じマップ（多くの場合 `empty` または作業用マップ）を **File → Open Level** で開く。
- 既に Editor に統合済みなら、そのレベルを開く。

### Step 4. World Outliner で中身を確認

1. **Window → World Outliner** を開く。
2. 検索欄に `block_` と入力し、Actor が大量にあることを確認。
3. 次が **無い** ことを確認（あれば選択 → Delete）:
   - `GridEnv_SpotRobot` / Humanoid 系
   - `demo_*` / `toggle_test_cube` / `cube_*`（レガシー）
4. `grid_floor_main`（床）は **残す**。

### Step 5. ブロックの初期状態（F）を目視・サンプル確認

Editor 上で数個の `BP_TransparentCube` を選び、半透明・コリジョン OFF 相当になっているか確認。  
Python 側で全 F にした直後ならそのまま保存してよい。

必要なら WSL からサンプル 1 個だけ再確認:

```python
from simworld.communication import UnrealCV
ucv = UnrealCV()
print(ucv.client.request("vbp block_050_050 SetBlocking False"))
```

### Step 6. レベルを新規パスで保存

1. **File → Save Current Level As…**
2. パス: **`Content/Maps/grid_100x100`**（アセット名 `grid_100x100`）
3. 保存後、Content Browser で `/Game/Maps/grid_100x100` が表示されることを確認。

### Step 7. （任意）デフォルトマップに登録

**Edit → Project Settings → Maps & Modes** で Editor Startup Map / Game Default Map を `grid_100x100` にしてもよいが、運用では **起動引数で指定** する方が安全（下記コマンド）。

### Step 8. パッケージ / PAK へ反映（SimWorldServer で使う場合）

Editor で編集したマップを **実行用 bundle** に載せる手順は、Obsidian `simWorld.md` の **Step 9（UE で編集した内容を SimWorldServer 実行へ反映）** に従う。

要点:

1. **Package / Cook**（プロジェクトの Packaging 設定に従う）
2. 生成された `pakchunk*` または Maps を `C:\SimWorldServer\...` へ配置
3. `SimWorld.exe` を再起動してマップが読めることを確認

---

## Phase D — 保存後の日常運用

### SimWorld 起動（Windows PowerShell）

```powershell
cd C:\SimWorldServer\SimWorld\Binaries\Win64
.\SimWorld.exe -windowed /Game/Maps/grid_100x100.umap
```

### シナリオ適用（WSL）

```python
import grid_env_10k as g10k

ucv = g10k.connect_unrealcv()
# マップが全 F で保存済みなら ("all", "F") は省略可
g10k.apply_scenario(ucv, [
    ("perimeter", "T"),
    ("rect", 10, 10, 20, 20, "T"),
])
```

`("all", "F")` を毎回実行すると約 10,000 回の vbp になり **約 80 分級** の可能性がある。全 F 保存マップでは **T にする領域だけ** 指定する運用を推奨。

---

## トラブルシューティング

| 症状 | 対処 |
| --- | --- |
| Editor に `block_*` が無い | Phase A の Python スポーンを同じマップでやり直す |
| マップが真っ暗 / 床だけ | 保存前に Outliner で `block_` を検索 |
| 起動後ブロックが無い | PAK / Server bundle へ Maps がコピーされていない → Step 9 |
| 名前が `cube_*` | 旧スクリプト。`grid_env_10k` の `block_*` に統一 |
