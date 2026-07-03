# NavMesh Phase 5 — UE MoveTo セットアップ

Python 側は `--nav-mode navmesh --nav-exec moveto`（実装予定）で、経路計画は Python・**実行は UE**（`SpotDogNavController`）に委譲します。

> **Phase 1–4** は [`NAVMESH_UE_SETUP.md`](NAVMESH_UE_SETUP.md) を参照。  
> **Sight の「Phase 5」**（`GetVisibleSightTargetsJson`）は [`SIGHT_PERCEPTION_UE_SETUP.md`](SIGHT_PERCEPTION_UE_SETUP.md) — **別物**です。

---

## アーキテクチャ（Phase 5-A）

```
Python                          UE (PIE)
  NavFindPath ───────────────► (計画のみ)
  NavFollowPathJson(points) ─► SpotDogNavController
  GetNavMoveStatusJson() ◄────  Tick → Move_Speed / Rotate_Angle (Pawn vbp 内部呼び出し)
```

Phase 5-A は SpotDog の四足アニメ（`Move_Speed`）を維持しつつ、**制御ループを UE Tick に移す**段階です。

---

## Step 1-1 — C++ をコピー

| コピー元（本リポジトリ） | コピー先（UE プロジェクト） |
|---|---|
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.h` | `Source/SimWorld/Public/SpotDogNavController.h` |
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.cpp` | `Source/SimWorld/Private/SpotDogNavController.cpp` |

**上書き**で構いません。

提供 API（`BP_SpotDogAIController` が親クラス `ASpotDogNavController` を継承すると vbp 可能）:

| 関数 | 用途 |
|------|------|
| `NavMoveToGoal(X,Y,Z,AcceptanceRadius)` | 内部 NavFindPath + 追従開始 |
| `NavFollowPathJson(PathJson)` | Python 計画パス `{"points":[...]}` を追従 |
| `NavStopMove()` | 停止 |
| `GetNavMoveStatusJson()` | `idle` / `moving` / `success` / `failed` |

---

## Step 1-2 — `SimWorld.Build.cs` に依存追加

`Source/SimWorld/SimWorld.Build.cs` の `PrivateDependencyModuleNames` に未追加なら:

```csharp
"NavigationSystem",
"AIModule",
"Json",
"JsonUtilities",
```

Phase 1–4 時点で `NavigationSystem` / `AIModule` は入っている想定。Phase 5 で **`Json` / `JsonUtilities`** を追加（`NavFollowPathJson` のパース用）。

---

## Step 1-3 — Visual Studio で Rebuild

1. **UE Editor を完全終了**
2. `SimWorld.sln` を Visual Studio で開く
3. 構成: **Development Editor** / **Win64**
4. **ビルド → ソリューションのリビルド**
5. 成功後、UE Editor 再起動

---

## Step 2 — `BP_SpotDogAIController` の親クラス変更

1. **PIE を停止**
2. Content Drawer → `Content/Robot_Dog/Blueprint/` → **`BP_SpotDogAIController`** を開く
3. ツールバー **Class Settings** をクリック
4. **Parent Class** → **Browse** → **`SpotDogNavController`** を選択 → **Reparent**
5. **Compile** → **Save**

My Blueprint → Functions に `NavMoveToGoal` 等が C++ 継承で表示されます。

---

## Step 3 — Pawn ラッパー（vbp 用）

Python は **`vbp GridEnv_SpotRobot ...`** と **Pawn 名**に対して呼びます。  
C++ の `NavMoveToGoal` 等は **AI Controller** 側にあるため、**`BP_SpotRobot`** に薄いラッパーを 4 本足します。

> **対象 BP**: `Content/Robot_Dog/Blueprint/BP_SpotRobot`  
> （`Move_Speed` / `Rotate_Angle` / `AttachCarryActor` と **同じ BP**）  
> spawn 名は `GridEnv_SpotRobot`（`grid_env_hri_simulation.ROBOT_ACTOR_NAME`）。

> **Class Defaults ではない**  
> 関数グラフを編集するときは、ツールバー **Class Defaults** が **押されていない**（通常モード）ことを確認してください。

### 3-0. 全体チェックリスト

| # | Pawn 関数名 | 入力 | 戻り値 |
|---|-------------|------|--------|
| 1 | `NavMoveToGoal` | `GoalX,Y,Z` (Float), `AcceptanceRadius` (Float) | String |
| 2 | `NavFollowPathJson` | `PathJson` (String) | String |
| 3 | `NavStopMove` | なし | String |
| 4 | `GetNavMoveStatusJson` | なし | String |

4 本ともグラフの型は同じです:

```
関数入口 → Get Controller → Cast to BP_SpotDogAIController
  ├─ Cast 成功 → Controller の C++ 関数 → Return Node
  └─ Cast 失敗 → Make Literal String（エラー JSON）→ Return Node
```

---

### 3-1. `BP_SpotRobot` を開く

1. **PIE を停止**（ツールバー **Stop** ■、または **Esc**）
2. 画面下 **Content Drawer** タブを **クリック**  
   （無い場合: メニュー **Window → Content Browser → Content Browser 1**）
3. フォルダ **`Content/Robot_Dog/Blueprint/`** に移動
4. **`BP_SpotRobot`** を **ダブルクリック**
5. 中央に Viewport、左上 **Components**、左 **My Blueprint**、右 **Details** が出れば OK

---

### 3-2. 関数 `NavMoveToGoal` を新規作成

#### 3-2-A. 関数を追加

1. 左 **My Blueprint** パネルで **Functions** 行を探す
2. **Functions** の右にある **＋（プラス）** を **クリック**
3. 一覧から **Function** を **クリック**
4. 新しい関数名が `NewFunction_0` 等になっている → **F2** またはゆっくり **2 回クリック**で名前を **`NavMoveToGoal`** に変更 → **Enter**

#### 3-2-B. Details — vbp 公開設定と入出力

1. **My Blueprint → Functions** で **`NavMoveToGoal`** を **1 回クリック**（選択）
2. 右 **Details** 上部の検索欄に何か入っていて **「All results have been filtered」** と出る場合 → 検索欄右の **×** を **クリック**
3. **Graph** セクション:
   - **Category** は `default` または `SimWorld`（どちらでも可）
   - **Access Specifier** = **`Public`**（`Private` だと vbp から呼べない場合あり）
   - **Call In Editor** = **OFF のまま**（エディタ Details のボタン用。vbp とは無関係）
4. **Inputs** 行の右 **＋** を **4 回クリック**し、次を設定:

| Name | Type |
|------|------|
| `GoalX` | **Float** |
| `GoalY` | **Float** |
| `GoalZ` | **Float** |
| `AcceptanceRadius` | **Float** |

5. **Outputs** 行の右 **＋** を **1 回クリック**
   - **Name**: `ReturnValue`（そのまま）
   - **Type**: **String**
6. ツールバー **Compile**（緑のチェックマーク）を **1 回クリック**
7. 中央グラフに紫の **`NavMoveToGoal`** 入口ノードが出て、左に 4 つの Float 入力ピン、右に Return 用の出口があることを確認

#### 3-2-C. グラフ配線（ピン単位）

**白いピン** = 実行の順序、**色付きピン** = データです。

##### C-1. Get Controller

1. グラフの **空白** を **右クリック**
2. 検索欄に **`Get Controller`** と入力
3. **`Get Controller`** を **クリック**して配置  
   - Pawn 関数内なので **Target は Self（自分）** のまま

##### C-2. Cast to BP_SpotDogAIController

1. **Get Controller** の **青い Return Value ピン** を **左クリックしたまま** 空白へ **ドラッグ** → 離す
2. 検索欄に **`Cast to BP_SpotDogAIController`** と入力 → **選択**  
   （無い場合: **`Cast to SpotDogNavController`** や **`Cast to AIController`** ではなく、**`BP_SpotDogAIController`** を選ぶ）
3. **白い実行線**を接続:  
   **`NavMoveToGoal` 入口（紫）の白ピン** → **`Cast` ノードの入力 Exec（左の白三角）**

##### C-3. Cast 成功 → Controller 側の Nav Move To Goal（重要）

> **よくあるエラー**  
> `BP Spot Dog AIController Object Reference is not compatible with Self Object Reference`  
> → ノードのサブタイトルが **`Target is BP Spot Robot`** のまま＝**今編集中の Pawn ラッパー自身**を呼ぼうとしている（誤り）。  
> 正しくは **`Target is BP Spot Dog AI Controller`**（または **`Spot Dog Nav Controller`**）。

1. いまの **青い `Nav Move to Goal` ノード**（Target is BP Spot Robot）を **選択** → **Delete**
2. **`Cast` ノードの `As BP Spot Dog AIController`（青いピン）** を **左クリックしたまま** グラフ空白へ **ドラッグ** → 離す
3. 出たメニューの検索欄に **`Nav Move`** と入力
4. 一覧から次を選ぶ（**表記は環境により多少異なる**）:
   - **`Nav Move to Goal`** でサブタイトルが **`Target is BP Spot Dog AI Controller`**
   - または **`Target is Spot Dog Nav Controller`**
   - **`Target is BP Spot Robot` のものは選ばない**（自分自身のラッパーで再帰になる）
5. **メニューに Controller 版が出ない場合**（下記「C++ 関数が見えないとき」へ）
6. **白い実行線**: **`Cast` の Then** → 追加した **Nav Move to Goal** の Exec
7. **青い線**: **`As BP Spot Dog AIController`** → **Target**（手順 2 のドラッグで自動接続されているはず）
8. 関数入力を接続:
   - **`GoalX`** → **`Goal X`**
   - **`GoalY`** → **`Goal Y`**
   - **`GoalZ`** → **`Goal Z`**
   - **`AcceptanceRadius`** → **`Acceptance Radius`**
9. **Return Value（String）** → **Return Node**

**別の置き方（手順 2 で出ないとき）**

1. グラフ空白 **右クリック** → 検索窓左下 **Context Sensitive（コンテキストに応じた）** の **チェックを外す**
2. **`Nav Move to Goal`** を検索
3. 候補が複数ある場合は **`BP Spot Dog AI Controller`** / **`Spot Dog Nav Controller`** 側を選択
4. 配置後、**Target** ピンに **`As BP Spot Dog AIController`** を **手動で接続**

**C++ 関数が見えないとき（Controller 版が一覧に無い）**

1. **`BP_SpotDogAIController`** を開く（Pawn ではない）
2. 左 **My Blueprint → Functions** に **`Nav Move to Goal`** 等が **継承関数**として見えるか確認
3. 無い → **Step 2** を再確認（Parent Class = `SpotDogNavController`）→ VS **Rebuild** → **Editor 再起動**
4. **`BP_SpotDogAIController`** で **Compile** → **Save** してから **`BP_SpotRobot`** に戻る

##### C-4. Cast 失敗 → エラー JSON

1. **`Cast` ノードの Cast Failed（白ピン）** から空白へ **右クリック**
2. **`Make Literal String`** を **追加**
3. **Value** に次を **コピー＆ペースト**（引用符含む）:  
   `{"ok":false,"error":"no_controller"}`
4. **Make Literal String** の **Return Value** → **別の Return Node**（または同じ Return Node へマージ）  
   - 実行線: **Cast Failed** → **Make Literal String** の白ピン（あれば）→ **Return Node**

##### C-5. Compile

1. ツールバー **Compile** を **クリック**
2. 下部 **Compiler Results** に **Error 0** であることを確認

---

### 3-3. 関数 `NavFollowPathJson` を追加

**3-2 と同じ手順**で、入出力だけ変えます。

1. **Functions → ＋ → Function** → 名前 **`NavFollowPathJson`**
2. **Details**:
   - **Access Specifier** = **`Public`**
   - **Call In Editor** = OFF
   - **Inputs ＋** ×1: **`PathJson`** / **String**
   - **Outputs ＋** ×1: **`ReturnValue`** / **String**
3. **Compile**
4. グラフ:
   - **Get Controller** → **Cast to BP_SpotDogAIController**
   - Cast 成功 → **`Nav Follow Path Json`**（`PathJson` を接続）
   - Cast 失敗 → `{"ok":false,"error":"no_controller"}`
   - Return Value → **Return Node**
5. **Compile** → **Save**（Ctrl+S）

---

### 3-4. 関数 `NavStopMove` を追加

1. **Functions → ＋** → 名前 **`NavStopMove`**
2. **Access Specifier** = **`Public`**（**Call In Editor** は OFF）
3. **Inputs** なし、**Outputs**: `ReturnValue` / **String**
4. グラフ:
   - Cast 成功 → **`Nav Stop Move`**（引数なし）
   - Cast 失敗 → `{"ok":false,"error":"no_controller"}`
5. **Compile**

---

### 3-5. 関数 `GetNavMoveStatusJson` を追加

1. **Functions → ＋** → 名前 **`GetNavMoveStatusJson`**
2. **Access Specifier** = **`Public`**（**Call In Editor** は OFF）
3. **Inputs** なし、**Outputs**: `ReturnValue` / **String**
4. グラフ:
   - Cast 成功 → **`Get Nav Move Status Json`**
   - Cast 失敗 → `{"status":"failed","error":"no_controller"}`  
     （ポーリング用なので `status` キーを使う）
5. **Compile** → **Save**

---

### 3-6. 保存と確認

1. ツールバー **Save**（**Ctrl+S**）を **クリック**
2. 左 **My Blueprint → Functions** に次の 4 つがあることを確認:
   - `NavMoveToGoal`
   - `NavFollowPathJson`
   - `NavStopMove`
   - `GetNavMoveStatusJson`
3. 各関数をクリックし、**Access Specifier = Public** であることを確認

#### 用語メモ（UE 5.3）

| Details の項目 | vbp に必要？ | 説明 |
|----------------|-------------|------|
| **Access Specifier = Public** | **はい** | 外部（UnrealCV vbp）から呼べるようにする |
| **Call In Editor** | **いいえ（OFF）** | レベル上の Actor Details にボタンを出す用。vbp とは別 |
| ~~Callable~~ | UE 5.3 には **この名前のチェックボックスは無い** | 旧ドキュメント表記。`My Blueprint → Functions` に追加した関数は、**Public** なら通常 vbp 可能 |

#### よくあるミス

| 症状 | 原因 | 対処 |
|------|------|------|
| vbp が `not found` | Access Specifier が Private | Details → **Public** → Compile |
| エディタに謎のボタンが出る | Call In Editor ON | **Call In Editor** を OFF |
| Cast 先が無い | Step 2 未完了 | `BP_SpotDogAIController` の Parent Class = `SpotDogNavController` |
| C++ 関数が検索に出ない | Editor 再起動前 | UE Editor 終了→再起動→`BP_SpotRobot` を開き直す |
| Child にだけ書いた | spawn は親 BP | **`BP_SpotRobot`** に書く（`GridEnv_SpotRobot` は親を spawn） |

---

### 3-7. Step 3 完了チェック（PIE 前でも可）

**My Blueprint → Functions** に 4 関数があり、各グラフに **Get Controller → Cast → C++ 呼び出し** があること。

次は **Step 4**（PIE スモーク）へ。

---

## Step 4 — PIE スモーク（UE 単体）

PIE Play 中、WSL から:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

期待: `NavFollowPathJson` → `status` が `moving` → `success`（約 2–5 秒）。

手動確認（短距離 + Python 経路）:

```python
from simworld.communicator.unrealcv import UnrealCV
import time
ucv = UnrealCV()
robot = "GridEnv_SpotRobot"
loc = ucv.get_location(robot)
gx, gy, gz = loc[0] + 300, loc[1], loc[2]
print(ucv.client.request(f"vbp {robot} NavMoveToGoal {gx} {gy} {gz} 130"))
for _ in range(40):
    print(ucv.client.request(f"vbp {robot} GetNavMoveStatusJson"))
    time.sleep(0.5)
```

> **Note:** `NavMoveToGoal` は C++ 内部 `BuildPathToGoal` が `no_path` になることがあります。ミッション同等条件では **`NavFollowPathJson` + Python `NavFindPath`** を使います（スモークスクリプト参照）。

### Step 4 トラブル: `moving` のまま不動 / すぐ `failed`

| 症状 | 意味 |
|------|------|
| `moving` が続くが位置・Yaw 不変 | C++ Tick が回っていない、または `ProcessEvent` で Pawn が動いていない |
| 数秒で `failed`、位置不変 | stuck 判定または `rotate_vbp_missing` / `move_vbp_missing` |
| 直接 `vbp Move_Speed` は動く | Pawn の `Move_Speed` は正常。**Controller → Pawn 呼び出し経路**に問題 |

**詳細な UE 側調査手順** → 下記 **「Step 4-D — UE 側調査（ボタン単位）」** を順に実行してください。

---

## Step 4-D — UE 側調査（ボタン単位）

> **前提**: Level を開き **PIE Play** 可能な状態。  
> **症状の例**: `_nav_moveto_smoke_test.py` が `moving` のままタイムアウト、ロボットが動かない。  
> **直接 `vbp Move_Speed` は動く**（WSL から確認済みの場合）。

調査は **A → B → C → D → E → F** の順がおすすめです。  
どこかで **NG** が出たら、その Step の「対処」まで実施してから次へ。

---

### 4-D-A. ベースライン確認（Python / PIE）

**目的**: Pawn の `Move_Speed` と NavMove API が PIE 中に生きているか確認。

1. UE Editor で **PIE Play**（ツールバー **Play** ▶、または **Alt+P**）
2. WSL ターミナルで次を実行:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

3. 結果をメモ:
   - **PASS** → 調査不要。Step 5 へ。
   - **`moving` タイムアウト / `failed`** → 4-D-B へ。

**追加確認（Move_Speed 単体）** — スモーク FAIL 時:

```bash
conda run -n simworld python -c "
import sys, time
from pathlib import Path
D=Path('~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav').expanduser()
sys.path[:0]=[str(D), str(D.parent/'grid_env_hri'), str(D.parent/'grid_env_10k')]
import grid_env_10k as g10k, grid_env_hri_simulation as geh
ucv,_=g10k.ensure_connection()
r=geh.ROBOT_ACTOR_NAME
loc0=ucv.get_location(r)
geh._ue_request(ucv, f'vbp {r} Move_Speed 180 1.0 0', timeout_s=10)
time.sleep(1.2)
loc1=ucv.get_location(r)
print('delta', loc1[0]-loc0[0], loc1[1]-loc0[1])
"
```

| 結果 | 意味 |
|------|------|
| delta が 50cm 以上 | Pawn モーション OK → **Controller 経路**を疑う（4-D-B 以降） |
| delta ≈ 0 | Pawn / spawn / PIE 自体を疑う（`GridEnv_SpotRobot` がレベルにいるか） |

---

### 4-D-B. `BP_SpotDogAIController` — 親クラスと Tick

**目的**: C++ `SpotDogNavController::Tick` が Blueprint 側で **無効化・上書き**されていないか確認。

1. **PIE を停止**（ツールバー **Stop** ■）
2. **Content Drawer** → `Content/Robot_Dog/Blueprint/` → **`BP_SpotDogAIController`** を **ダブルクリック**
3. ツールバー **Class Settings** を **1 回クリック**
4. 右 **Details → Class Options → Parent Class** が **`Spot Dog Nav Controller`**（または `SpotDogNavController`）であることを確認  
   - 違う場合 → [Step 2](#step-2--bp_spotdogaicontroller-の親クラス変更) をやり直し → **Compile** → **Save**

#### B-1. Event Graph に Tick 上書きがないか

1. ツールバー **Class Settings** の **ハイライトを解除**（もう一度 **Class Settings** をクリック、または **Event Graph** タブをクリック）
2. 上部タブ **Event Graph** を **クリック**
3. グラフ内を **右クリック** → **Find References** は使わず、目視または **Ctrl+F** で **`Event Tick`** を検索
4. **Event Tick** ノードがある場合:
   - **親 `SpotDogNavController` の Tick を止めている可能性大**
   - **対処（推奨）**: **Event Tick** ノードを **選択** → **Delete**
   - または Tick 内の最後に **Parent: Tick** ノードを追加（検索: `Parent Tick` / `Call Parent Function`）
5. **Event Tick** が **無い** → OK（C++ Tick のみ）

#### B-2. Class Defaults — Tick 有効

1. ツールバー **Class Defaults** を **クリック**（ハイライト ON）
2. 右 **Details** 検索欄に **`tick`** と入力
3. 次を確認:

| 項目 | 期待値 |
|------|--------|
| **Start with Tick Enabled**（Actor Tick） | **チェック ON** |
| **Tick Interval (secs)** | **0.0**（毎フレーム） |

4. 変更したら **Compile** → **Save**（**Ctrl+S**）

#### B-3. 継承 C++ 関数の存在確認

1. 左 **My Blueprint → Functions** を展開
2. 次が **継承関数**として見えるか確認（アイコン付きでグレー表示など）:
   - `Nav Follow Path Json`
   - `Get Nav Move Status Json`
   - `Nav Stop Move`
3. **1 つも無い** → VS **Rebuild** → Editor **再起動** → 4-D-B を最初から

---

### 4-D-C. `BP_SpotRobot` — AI Controller と Possess

**目的**: spawn された `GridEnv_SpotRobot` が **`BP_SpotDogAIController` に Possess** されているか確認。

1. **`BP_SpotRobot`** を **ダブルクリック**（`BP_SpotDogAIController` タブは閉じてよい）
2. ツールバー **Class Defaults** を **クリック**
3. 右 **Details** 検索欄に **`AI Controller`** と入力
4. 設定:

| 項目 | 期待値 |
|------|--------|
| **AI Controller Class** | **BP_SpotDogAIController** |
| **Auto Possess AI** | **Placed in World or Spawned** |

5. **Compile** → **Save**

#### C-1. PIE 中に Possess を目視確認

1. **PIE Play** 開始
2. メニュー **Window → World Partition** ではなく **Window → World Outliner**（無ければ **Window → Outliner**）
3. Outliner 上部 **検索欄**に **`SpotDog`** と入力
4. 次が **両方** あるか確認:
   - **`GridEnv_SpotRobot`**（または `BP_SpotRobot_C_*`）
   - **`BP_SpotDogAIController_C_0`**（番号は環境依存）
5. Outliner で **`BP_SpotDogAIController_C_0`** を **1 回クリック**
6. 右 **Details** を下へスクロール → **Pawn** または **Controller** 関連:
   - **Controlled Pawn** / **Pawn** に **`GridEnv_SpotRobot`** が表示されていれば **Possess OK**
7. **空 / None** → Possess 失敗。C-2 へ。

#### C-2. Possess 失敗時の対処

1. **PIE 停止**
2. `BP_SpotRobot` の **Class Defaults** で 4-D-C の表を再設定
3. Level に **`GridEnv_SpotRobot`** を手動配置している場合:
   - World Outliner で Robot を選択 → Details → **Auto Possess AI** が Blueprint デフォルト通りか確認
4. Python spawn のみの場合: 次回 `--force-respawn` で spawn し直す
5. 参考: [`SIGHT_PERCEPTION_UE_SETUP.md`](SIGHT_PERCEPTION_UE_SETUP.md) の Phase 2-B（AI Controller Class）

---

### 4-D-D. `BP_SpotRobot` — `Move_Speed` / `Rotate_Angle`

**目的**: C++ が `FindFunction("Move_Speed")` できる **BlueprintCallable 関数**が Pawn に存在するか。

1. **PIE 停止** → **`BP_SpotRobot`** を開く
2. 左 **My Blueprint → Functions** に次があるか確認:
   - **`Move_Speed`**
   - **`Rotate_Angle`**
3. **無い** → 親 BP または `Move_Speed` 実装 BP を特定（Content 検索 **`Move_Speed`**）→ 4-D-F のラッパー案を検討
4. **`Move_Speed`** を **1 回クリック** → 右 **Details**:
   - **Access Specifier** = **Public**（または Blueprint から呼べる設定）
5. グラフを開き、**入力ピン**が次の順序か確認（C++ と一致）:
   - **Move_Speed**: `Speed` (Float), `Duration` (Float), `Direction` (Int)
   - **Rotate_Angle**: `Duration` (Float), `Angle` (Float), `Clockwise` (Int)

---

### 4-D-E. Output Log で C++ ログ確認

**目的**: C++ が `rotate_vbp_missing` / `move_vbp_missing` / `stuck` で **MarkFailed** していないか。

1. **PIE Play**
2. メニュー **Window → Developer Tools → Output Log**
3. Output Log 左下 **Filters** 横の **検索欄**に **`SpotDogNavController`** と入力
4. WSL からスモーク再実行:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

5. Output Log に次のような行が出るか確認:

| ログ | 意味 | 対処 |
|------|------|------|
| `SpotDogNavController failed: rotate_vbp_missing` | Pawn に `Rotate_Angle` が見つからない | 4-D-D、`FindFunction` 名の一致 |
| `SpotDogNavController failed: move_vbp_missing` | 同上 `Move_Speed` | 同上 |
| `SpotDogNavController failed: stuck` | 移動量が増えない | 障害物・ProcessEvent 未実行 |
| `SpotDogNavController failed: no_pawn` | Controller が Pawn を保持していない | 4-D-C |
| **ログ無し** + `moving` のまま不動 | Tick 未実行 or **Rotate_Angle BP 故障** | 4-D-B + **Step 4-E** + **4-D-G** |

### Step 4-G — `Rotate_Angle` BP 故障（2026-07 追記）

**症状**（Output Log）:

    GridEnv_SpotRobot Rotate SetTimer passed a negative or zero time
    Divide by zero: Divide_DoubleDouble

**WSL 確認**（PIE 中）:

    vbp GridEnv_SpotRobot Rotate_Angle 1.0 30 -1
    → Yaw が変わらない（Move_Speed は動く）

**原因**: 親 BP（`BP_AgentBase` 等）の `Rotate_Angle` グラフ内で Duration=0 や Speed=0 による **除算ゼロ** が発生し、回転タイマーが起動しない。

**C++ 回避**（リポジトリ済み）: `SpotDogNavController` の **`bUseDirectYawRotation=true`**（デフォルト）で `SetActorRotation` による直接 Yaw 更新。`Move_Speed` は従来通り BP 呼び出し。

#### あなたの作業

1. **Step 4-E** と同様に C++ Rebuild + PIE 再起動
2. 診断再実行 — `Rotate_Angle vbp probe: BROKEN` でも NavFollow が PASS すれば Step 4 完了
3. （任意・後日）Content Browser で `Move_Speed` がある BP を開き、`Rotate_Angle` グラフを修正:
   - 関数入口の **Duration / Angle / Clockwise** ピンが `Rotate` 内部の SetTimer / 除算に正しく接続されているか
   - **Speed=0** や **Duration=0** にならないよう Clamp / 分岐

> **見た目改善の詳細手順（段階1）**: [`NAVMESH_PHASE5_VISUAL_STAGE1.md`](NAVMESH_PHASE5_VISUAL_STAGE1.md)

### Step 4-H — NavMesh スナップ + waypoint 方向移動（2026-07 追記）

**症状**: E2E leg1 は完了するが leg2 で `start_not_on_navmesh`。軌跡が NavFindPath から逸脱。

**原因**: `bUseDirectTranslation` のテレポート移動が NavMesh ポリラインから外れ、Dynamic 障害物込みの有効 NavMesh 上に載らない。

**C++ 修正**（`SpotDogNavController.*`）:
- **`bSnapPawnToNavMesh=true`**: 各移動/回転後に `ProjectPointToNavigation`（失敗時 `NavProjectRetryExtentCm=120` で再試行）
- **`ApplyDirectMoveToward`**: 現在 WP 方向へ移動（前方ベクトルのみではない）
- **`BeginFollowPath` 開始時**にもスナップ

**Python 修正**（`navmesh_mission_nav.py`）:
- `_nav_plan_xyz` が投影失敗時 **`None` を返す**（生 XY を NavFindPath に渡さない）
- `_start_xyz_for_robot`: ロボット実 Z を優先して投影

Rebuild 後 E2E 再実行:

```bash
NAV_MOVETO_UE=1 conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --nav-exec moveto --force-respawn
```

---

### Step 4-E — C++ Timer 修正（2026-07 追記）

**症状**: Output Log に `SpotDogNavController failed` が **出ない** のに、`moving` のまま位置・Yaw 不変。

**原因**: `BP_SpotDogAIController` で **Actor Tick が無効** の場合、`SpotDogNavController::Tick` が回らず、最初の `Rotate` 命令だけ発行されて永久 `moving` になる。

**リポジトリ修正**（`ue_native/SpotDogNavController.*`）:
- `BeginPlay` で `SetActorTickEnabled(true)`
- **World Timer**（0.05s）で `TickFollowPath` を駆動（Actor Tick 無効でも動作）
- `ProcessEvent` を UFunction パラメータ反射で呼び出し
- 関数名的フォールバック: `Move_Speed` / `Rotate_Angle` / `Rotate` / `NavExecMoveSpeed` / `NavExecRotate`
- **`bUseDirectYawRotation`**（デフォルト true）: `Rotate_Angle` BP 故障時は `SetActorRotation` で Yaw 更新（→ 4-D-G）

#### あなたの作業（Rebuild 必須）

1. **PIE 停止**
2. 次を UE プロジェクトへコピー（WSL から既に同期済みの場合はスキップ可）:
   - `dev/grid_env_level_nav/ue_native/SpotDogNavController.h` → `Source/SimWorld/Public/`
   - `dev/grid_env_level_nav/ue_native/SpotDogNavController.cpp` → `Source/SimWorld/Private/`
3. **UE Editor 終了** → VS **Rebuild** → Editor 起動
4. Missing Modules → **Yes**
5. Level → **PIE Play**
6. WSL:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py
```

`PASS` なら Step 4 完了。

#### トラブル: Editor 起動時 `SimWorld could not be compiled`

**原因例**: `SpotDogNavController.cpp` で引数名 `Pawn` が `AAIController::Pawn` を隠し **C4458** になる（プロジェクト設定で警告がエラー扱い）。

**対処**: 最新 `ue_native/SpotDogNavController.*` を再コピー（引数は `InPawn`）→ VS **Rebuild** → Editor 再起動。

---

### 4-D-F. 修正候補 — Pawn ラッパー `NavExecMoveSpeed` / `NavExecRotate`（推奨ワークアラウンド）

**目的**: C++ `ProcessEvent` が既存 `Move_Speed` を直接呼べない場合、**薄い BP 関数**を挟む。

> **いつ使うか**: 4-D-B〜E まで OK なのに、スモークが依然 `moving` タイムアウトのとき。

#### F-1. `NavExecMoveSpeed` を追加（`BP_SpotRobot`）

1. **PIE 停止** → **`BP_SpotRobot`** を開く
2. **My Blueprint → Functions → ＋ → Function**
3. 関数名 **`NavExecMoveSpeed`**
4. **Details**:
   - **Access Specifier** = **Public**
   - **Inputs ＋** ×3:

| Name | Type |
|------|------|
| `Speed` | Float |
| `Duration` | Float |
| `Direction` | Int |

5. **Outputs** なし（Return 不要）
6. グラフ:
   - 関数入口 → **`Move Speed`** ノード（**Target = Self**）を配置
   - `Speed` / `Duration` / `Direction` を **Move Speed** の同名ピンへ接続
7. **Compile** → **Save**

#### F-2. `NavExecRotate` を追加

1. 同様に関数 **`NavExecRotate`**
2. **Inputs**:

| Name | Type |
|------|------|
| `Duration` | Float |
| `AngleDeg` | Float |
| `Clockwise` | Int |

3. グラフ: 入口 → **`Rotate Angle`**（Self）← 入力接続
4. **Compile** → **Save**

#### F-3. C++ をラッパー名に変更（開発者作業）

`SpotDogNavController.cpp` の `FindFunction` を:

| 旧 | 新 |
|----|-----|
| `Move_Speed` | `NavExecMoveSpeed` |
| `Rotate_Angle` | `NavExecRotate` |

に変更 → `ue_native/` から UE `Source/` へコピー → **VS Rebuild** → Editor 再起動。

（リポジトリ側の変更は別途コミット予定。未反映の場合は手動で cpp を編集。）

#### F-4. ラッパー単体テスト（PIE）

WSL:

```bash
conda run -n simworld python -c "
import sys, time
from pathlib import Path
D=Path('~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav').expanduser()
sys.path[:0]=[str(D), str(D.parent/'grid_env_hri'), str(D.parent/'grid_env_10k')]
import grid_env_10k as g10k, grid_env_hri_simulation as geh
ucv,_=g10k.ensure_connection()
r=geh.ROBOT_ACTOR_NAME
loc0=ucv.get_location(r)
geh._ue_request(ucv, f'vbp {r} NavExecMoveSpeed 180 1.0 0', timeout_s=10)
time.sleep(1.2)
loc1=ucv.get_location(r)
print('delta', loc1[0]-loc0[0], loc1[1]-loc0[1])
"
```

delta > 50 → ラッパー OK → スモーク再実行。

---

### 4-D-G. 調査チェックリスト（記入用）

| # | 項目 | OK / NG | メモ |
|---|------|---------|------|
| A | 直接 `Move_Speed` vbp | | |
| B | Parent Class = SpotDogNavController | | |
| B | Event Graph に **Event Tick 上書き無し** | | |
| B | Start with Tick Enabled = ON | | |
| C | AI Controller Class = BP_SpotDogAIController | | |
| C | Auto Possess AI = Placed in World or Spawned | | |
| C | PIE 中 Controlled Pawn = GridEnv_SpotRobot | | |
| D | Move_Speed / Rotate_Angle が Public | | |
| E | Output Log に failed 理由 | | |
| F | NavExecMoveSpeed ラッパー（必要時） | | |
| — | `_nav_moveto_smoke_test.py` PASS | | |

**すべて OK なのに FAIL** → WSL から `/mnt/c/UEProjects/SimWorld/Source/SimWorld/Private/SpotDogNavController.cpp` が最新か再確認 → **Rebuild**。

---


## Step 5 — Python 統合

| ファイル | 内容 |
|----------|------|
| `nav_move.py` | vbp ラッパー + `moveto_use_ue_controller()` |
| `navmesh_mission_nav.py` | `--nav-exec moveto` 分岐（UE 未検証時 vbp フォールバック） |
| `run_test.py` | `--nav-exec {vbp,moveto}` CLI |
| `_nav_moveto_smoke_test.py` | Step 4 自動スモーク |

```bash
# vbp（従来どおり）
conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --force-respawn

# moveto（UE Tick 未検証時は vbp フォールバック）
conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --nav-exec moveto --force-respawn

# UE Tick 検証後
NAV_MOVETO_UE=1 conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --nav-exec moveto --force-respawn
```

---

## 事前条件チェック

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_project_point_smoke_test.py
conda run -n simworld python dev/grid_env_level_nav/_phase5_prereq_check.py
```

`NavFindPath` は公式スモークが foot Z で FAIL することがあります。ミッション同等条件では PASS（`_nav_find_path_check2.py` 参照）。

---

## 関連ドキュメント

| ファイル | 内容 |
|----------|------|
| [`NAVMESH_UE_SETUP.md`](NAVMESH_UE_SETUP.md) | Phase 1–4（NavQueryService・Dynamic NavMesh） |
| **本ファイル** | Phase 5（UE MoveTo / SpotDogNavController） |
| [`NAVMESH_PHASE5_VISUAL_STAGE1.md`](NAVMESH_PHASE5_VISUAL_STAGE1.md) | Phase 5 見た目改善・段階1（BP 復旧・direct OFF） |
| [`SIGHT_PERCEPTION_UE_SETUP.md`](SIGHT_PERCEPTION_UE_SETUP.md) | AI Sight（別系統の Phase 5） |
| [`CARRY_ATTACH_UE_SETUP.md`](CARRY_ATTACH_UE_SETUP.md) | Leg2 運搬 Socket |
| [`MISSION_ARCHITECTURE.md`](MISSION_ARCHITECTURE.md) | ミッション全体設計 |
| [`ue_native/INSTALL_NATIVE.md`](../../ue_native/INSTALL_NATIVE.md) | C++ コピー先一覧 |

---

## トラブルシュート

| 症状 | 対処 |
|------|------|
| `move_vbp_missing` / `rotate_vbp_missing` | `BP_SpotRobot` に `Move_Speed` / `Rotate_Angle` があるか確認 |
| `no_pawn` | AI Controller が SpotDog を Possess しているか（Auto Possess AI） |
| `bad_path_json` | `NavFollowPathJson` の JSON 形式を確認 |
| C++ ビルドエラー Json | `SimWorld.Build.cs` に Json / JsonUtilities を追加して Rebuild |
| Reparent 後に Compile エラー | 親 `SpotDogNavController` がビルド済みか VS Rebuild を再実行 |
