# NavMesh Phase 5 — 見た目改善 段階1（SpotDog 歩行復旧）

Phase 5 の **NavMove（`SpotDogNavController` + `--nav-exec moveto`）** はミッション到達性を優先するため、暫定実装では次が **デフォルト ON** になっています。

| フラグ | 暫定動作 | 見 = 見た目への影響 |
|--------|----------|-------------------|
| `bUseDirectYawRotation` | `SetActorRotation` で即回転 | 回転アニメなし |
| `bUseDirectTranslation` | `SetActorLocation` + TeleportPhysics | 足アニメと位置が不一致 |
| `bSnapPawnToNavMesh` | 移動・回転のたびに XYZ スナップ | 上下にガタつく |

**段階1** は、既存 BP（`Move_Speed` / `Rotate_Angle`）を復活させ、direct + full snap を段階的に OFF にして **「床の上を走る」見た目** に戻す手順です。

> **前提**: [Phase 5 セットアップ](NAVMESH_PHASE5_UE_SETUP.md)（C++ コピー・Reparent・Pawn ラッパー・smoke PASS）が完了していること。  
> **注意**: 本ファイルの **「段階1-Step 1-1」** は [`NAVMESH_PHASE5_UE_SETUP.md` の Step 1-1（C++ コピー）](NAVMESH_PHASE5_UE_SETUP.md#step-1-1--c-をコピー) とは **別物** です。

---

## 進め方

**1 ステップ完了 → WSL 確認 → 次** の順。NG なら次に進まない。

| Step | 内容 | 主な作業場所 |
|------|------|-------------|
| [1-1](#step-1-1--rotate_angle-bp-修正) | `Rotate_Angle` BP 修正 | `BP_SpotRobot` または親 BP |
| [1-2](#step-1-2--直接-yaw-更新を-off) | `bUseDirectYawRotation = OFF` | `BP_SpotDogAIController` |
| [1-3](#step-1-3--move_speed-ラッパー) | `Move_Speed` / `NavExecMoveSpeed` | `BP_SpotRobot` |
| [1-4](#step-1-4--直接テレポート移動を-off) | `bUseDirectTranslation = OFF` | `BP_SpotDogAIController` |
| [1-5](#step-1-5--navmesh-スナップを弱める) | XY のみスナップ（C++ + Rebuild） | `SpotDogNavController.cpp` |
| [1-6](#step-1-6--layout_01-e2e-最終確認) | E2E + 目視 | WSL + PIE |

段階2・3は本ファイルの範囲外（補間・UE 標準 Path Following 等）。

---

## 事前準備

| # | 操作 |
|---|------|
| 0-1 | UE Editor で **`/Game/Maps/Level`** を開く |
| 0-2 | **PIE Stop**（ツールバー **■ Stop** または **Esc**） |
| 0-3 | WSL で `conda run -n simworld` が使えること |

**ベースライン記録**（後で比較）:

1. **PIE Play**（▶ または **Alt+P**）
2. WSL:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py
```

3. `Rotate_Angle vbp probe: BROKEN` / `OK` をメモ
4. **PIE Stop**

---

## Step 1-1 — `Rotate_Angle` BP 修正

**目的**: vbp で回転アニメが動く（Divide by zero / SetTimer 0 を解消）

### 1-1-A. 実装 BP の特定

1. **PIE Stop**
2. 画面下 **Content Drawer** をクリック  
   （無ければ **Window → Content Browser → Content Browser 1**）
3. 検索欄に **`Rotate_Angle`** と入力
4. ヒットした Blueprint を開き、**My Blueprint → Functions** に **`Rotate_Angle`** があるか確認
5. **`BP_SpotRobot`**（`Content/Robot_Dog/Blueprint/`）を開く
   - **Functions** に無い → **親 BP**（例: `BP_AgentBase`）側にあることが多い
   - **`Rotate_Angle`** をクリック → Details に **Inherited from …** → **親 BP** を編集

**親 BP を開く**:

1. **`BP_SpotRobot`** → ツールバー **Class Settings**
2. **Details → Parent Class** 名を **Ctrl+クリック**、または Content Drawer で親を検索

### 1-1-B. 故障箇所の確認

1. **My Blueprint → Functions → Rotate_Angle** を **ダブルクリック**
2. **Window → Developer Tools → Output Log** を開く
3. グラフ内 **Ctrl+F** で **`Set Timer`** / **`Divide`** を検索

**典型故障**（[`NAVMESH_PHASE5_UE_SETUP.md` Step 4-G](NAVMESH_PHASE5_UE_SETUP.md#step-4-g--rotate_angle-bp-故障2026-07-追記) 参照）:

- Output Log: `Divide by zero`, `SetTimer passed a negative or zero time`
- **Duration** / **Angle** が 0 のまま除算・SetTimer に入る

**入口ピン**（C++ / vbp と一致）:

| 名前 | 型 |
|------|-----|
| Duration | Float |
| Angle | Float |
| Clockwise | Int |

vbp 例: `vbp GridEnv_SpotRobot Rotate_Angle 1.0 30 -1`

### 1-1-C. 修正

1. 入口 **Duration** が SetTimer / 除算に **接続されているか** 確認
2. **Duration** に **Max (float)** を追加（B = **0.05**）→ SetTimer の Time へ
3. **Angle** が分母になる **Divide** がある場合、**Max (float)**（B = **0.01**）を挟む
4. **Clockwise** が回転方向ロジックに接続されているか確認
5. **Compile** → **Save**（**Ctrl+S**）

**親関数が編集不可（Inherited）の場合**:

1. **`BP_SpotRobot` → Functions → Rotate_Angle** を右クリック → **Override**
2. 入口 → **Rotate Angle**（Target = **Self**）へ接続、または親ロジックをコピーして Clamp を追加

### 1-1-D. 確認

1. **PIE Play**
2. WSL:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py
```

3. 期待: **`Rotate_Angle vbp probe: OK`**（|delta| ≥ 3°）
4. Output Log に Divide by zero / SetTimer zero **が出ない**
5. **NG → Step 1-1 に留まる**
6. **PIE Stop**

---

## Step 1-2 — 直接 Yaw 更新を OFF

**目的**: C++ の `SetActorRotation` バypassを止め、`Rotate_Angle` BP を使う

### 1-2-A. Class Defaults 変更

1. **PIE Stop**
2. **`BP_SpotDogAIController`** を **ダブルクリック**
3. ツールバー **Class Defaults** をクリック（青ハイライト ON）
4. **Details** 検索: **`direct yaw`** または **`Nav Move`**

| 項目 | 設定 |
|------|------|
| **Use Direct Yaw Rotation** (`bUseDirectYawRotation`) | **☐ OFF** |

5. **Compile** → **Save**

### 1-2-B. 確認

1. **PIE Play**
2. WSL: `_nav_moveto_diagnose.py` → **`PASS`**、`Rotate_Angle … OK`
3. PIE で **瞬間回転ではなく** 時間をかけた回転
4. **PIE Stop**

---

## Step 1-3 — `Move_Speed` ラッパー

**目的**: C++ `ProcessEvent` から四足歩行 BP を確実に呼ぶ

### 1-3-A. 直接 `Move_Speed` テスト

1. **PIE Play**
2. WSL:

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
print('delta_cm', ((loc1[0]-loc0[0])**2+(loc1[1]-loc0[1])**2)**0.5)
"
```

3. **delta_cm ≥ 50** → **Step 1-4 へ**（ラッパー不要）
4. **delta_cm ≈ 0** → **1-3-B へ**

### 1-3-B. `NavExecMoveSpeed` 追加（`BP_SpotRobot`）

詳細は [`NAVMESH_PHASE5_UE_SETUP.md` 4-D-F](NAVMESH_PHASE5_UE_SETUP.md#4-d-f-修正候補--pawn-ラッパー-navexecmovespeed--navexecrotate推奨ワークアラウンド) も参照。

1. **PIE Stop** → **`BP_SpotRobot`** を開く（**Class Defaults** は OFF）
2. **My Blueprint → Functions → ＋ → Function** → 名前 **`NavExecMoveSpeed`**
3. **Details**: **Access Specifier = Public**
4. **Inputs ＋** ×3:

| Name | Type |
|------|------|
| Speed | Float |
| Duration | Float |
| Direction | Integer |

5. グラフ: **Move Speed**（Target = **Self**）← Speed / Duration / Direction
6. **Compile** → **Save**

### 1-3-C. `NavExecRotate` 追加（推奨）

1. 関数 **`NavExecRotate`**、Public、Inputs: Duration, AngleDeg, Clockwise
2. グラフ: **Rotate Angle**（Self）← 同名ピン（AngleDeg → Angle）
3. **Compile** → **Save**

C++ は `NavExecMoveSpeed` / `NavExecRotate` を **フォールバック候補**として既に探索します（[`SpotDogNavController.cpp`](../../ue_native/SpotDogNavController.cpp)）。

### 1-3-D. 確認

1. **PIE Play**
2. ラッパー作成時:

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
print('delta_cm', ((loc1[0]-loc0[0])**2+(loc1[1]-loc0[1])**2)**0.5)
"
```

3. **delta_cm ≥ 50** かつ PIE で足が動く
4. **PIE Stop**

---

## Step 1-4 — 直接テレポート移動を OFF

**目的**: `SetActorLocation(TeleportPhysics)` をやめ、`Move_Speed` アニメ移動に戻す

### 1-4-A. Class Defaults 変更

1. **PIE Stop** → **`BP_SpotDogAIController`**
2. **Class Defaults** → **Nav Move**:

| 項目 | 設定 |
|------|------|
| **Use Direct Translation** (`bUseDirectTranslation`) | **☐ OFF** |
| **Use Direct Yaw Rotation** | **☐ OFF**（1-2 済） |

3. **Compile** → **Save**

### 1-4-B. 確認

1. **PIE Play**
2. WSL:

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

3. 期待: **`PASS`**
4. Output Log（**Window → Developer Tools → Output Log**）で **`SpotDogNavController failed:`** を確認
   - `move_vbp_missing` → Step 1-3
   - `stuck` → Move_Speed / 障害物を再確認
5. PIE で滑り・瞬間移動が減り歩行に近づく
6. **PIE Stop**

---

## Step 1-5 — NavMesh スナップを弱める

**目的**: 毎ステップの **XYZ テレポート** による上下ガタつきを減らす

現状の `SnapPawnToNavMesh` は **NavLoc.Location 全体（XYZ）** を `SetActorLocation` します。段階1では **XY のみ NavMesh に合わせ、Z は現在値を維持** する C++ 変更が必要です。

### 1-5-A. C++ 修正（`SnapPawnToNavMesh`）

**ファイル**: `dev/grid_env_level_nav/ue_native/SpotDogNavController.cpp`

`SnapPawnToNavMesh` 内、`SetActorLocation` の直前を次の方針に変更:

```cpp
const FVector Current = InPawn->GetActorLocation();
const FVector Snapped(
    NavLoc.Location.X,
    NavLoc.Location.Y,
    Current.Z);  // Z は維持（または FMath::FInterpTo で緩やかに補間）
InPawn->SetActorLocation(Snapped, false, nullptr, ETeleportType::TeleportPhysics);
```

**コピー先**（[Phase 5 Step 1-1](NAVMESH_PHASE5_UE_SETUP.md#step-1-1--c-をコピー)）:

| コピー元 | コピー先（UE プロジェクト） |
|---------|---------------------------|
| `ue_native/SpotDogNavController.cpp` | `Source/SimWorld/Private/` |
| `ue_native/SpotDogNavController.h` | `Source/SimWorld/Public/` |

### 1-5-B. Rebuild

1. **PIE Stop** → **UE Editor 完全終了**
2. **`SimWorld.sln`** → **Development Editor / Win64**
3. **ビルド → ソリューションのリビルド**
4. Editor 起動 → Missing Modules → **Yes**

### 1-5-C. Editor 設定

1. **`BP_SpotDogAIController` → Class Defaults → Nav Move**:

| 項目 | 推奨 |
|------|------|
| **Snap Pawn To Nav Mesh** | **☑ ON** |
| **Nav Project Extent Cm** | `30` |
| **Nav Project Retry Extent Cm** | `120` |

2. **Compile** → **Save**

### 1-5-D. 確認

1. **PIE Play** → `_nav_moveto_diagnose.py` **PASS**
2. PIE で **上下のふらつきが減る** ことを目視
3. **PIE Stop**

**暫定（C++ 未反映時）**: **Snap Pawn To Nav Mesh = OFF** で見た目だけ改善可能。ただし leg2 で `start_not_on_navmesh` が再発しうるため、**Step 1-6 で必ず E2E 確認**。

---

## Step 1-6 — layout_01 E2E（最終確認）

1. **PIE Play**
2. WSL:

```bash
NAV_MOVETO_UE=1 conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --nav-exec moveto --force-respawn
```

3. 期待:
   - **`[Site20] PASS`**
   - **`delivered=True`**
   - PIE で床沿いの歩行に近い見た目

| FAIL 症状 | 戻る Step |
|-----------|-----------|
| 回転しない / 瞬間回転 | 1-1, 1-2 |
| 動かない / stuck | 1-3, 1-4 |
| leg2 `start_not_on_navmesh` | 1-5（スナップ方針） |

---

## チェックリスト

| Step | 内容 | OK |
|------|------|-----|
| 0 | ベースライン diagnose 実行 | ☐ |
| 1-1 | `Rotate_Angle` BP 修正 | ☐ |
| 1-1-D | diagnose: `Rotate_Angle … OK` | ☐ |
| 1-2 | `bUseDirectYawRotation = OFF` | ☐ |
| 1-3 | `Move_Speed` または `NavExecMoveSpeed` ≥50cm | ☐ |
| 1-4 | `bUseDirectTranslation = OFF` + smoke PASS | ☐ |
| 1-5 | XY のみスナップ C++ + Rebuild | ☐ |
| 1-6 | layout_01 E2E PASS + 目視 OK | ☐ |

---

## よくある分岐

**`Rotate_Angle` が My Blueprint に無い**  
→ Content 検索で親 BP を開く。**親を編集**するか **`BP_SpotRobot` で Override**。

**1-4 で PASS だが見た目がまだ変**  
→ Step 1-5（Z スナップ）が効くことが多い。それでも足りなければ段階2（補間・閾値調整）。

**Phase 5 smoke がまだ FAIL**  
→ 本ファイルの前に [NAVMESH_PHASE5_UE_SETUP.md Step 4-D](NAVMESH_PHASE5_UE_SETUP.md#step-4-d--pietick-が回らない--moving-のまま動かない調査) を完了すること。

---

## 関連ドキュメント

| ファイル | 内容 |
|----------|------|
| [`NAVMESH_PHASE5_UE_SETUP.md`](NAVMESH_PHASE5_UE_SETUP.md) | Phase 5 本体（C++・Reparent・smoke・4-D 調査） |
| [`NAVMESH_UE_SETUP.md`](NAVMESH_UE_SETUP.md) | Phase 1–4（NavQueryService） |
| [`ue_native/SpotDogNavController.cpp`](../../ue_native/SpotDogNavController.cpp) | NavMove C++ 実装 |
| [`ue_native/INSTALL_NATIVE.md`](../../ue_native/INSTALL_NATIVE.md) | C++ コピー先一覧 |

---

## トラブルシュート

| 症状 | 対処 |
|------|------|
| `Rotate_Angle vbp probe: BROKEN` | Step 1-1（BP グラフ・Clamp） |
| `rotate_vbp_missing` / `move_vbp_missing` | Step 1-3、`Move_Speed` Public 確認（[4-D-D](NAVMESH_PHASE5_UE_SETUP.md#4-d-d-bp_spotrobot--move_speed--rotate_angle)） |
| smoke `stuck` | Move_Speed 単体 delta、NavExec ラッパー、障害物 |
| 上下にふらつく | Step 1-5（XY のみスナップ） |
| leg2 `start_not_on_navmesh` | Snap ON + XY のみスナップ；[Step 4-H](NAVMESH_PHASE5_UE_SETUP.md#step-4-h--navmesh-スナップ--waypoint-方向移動2026-07-追記) 参照 |
