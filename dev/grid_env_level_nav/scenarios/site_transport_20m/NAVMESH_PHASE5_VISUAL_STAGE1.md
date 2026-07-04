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
| [1-1](#step-1-1--回転-bp-修正-navexecrotate--rotationmethod-安全装置) | 回転 BP 修正（`NavExecRotate` + `RotationMethod` 安全装置） | `BP_SpotRobot` |
| [1-2](#step-1-2--直接-yaw-更新を-off) | `bUseDirectYawRotation = OFF` | `BP_SpotDogAIController` |
| [1-3](#step-1-3--move_speed-ラッパー) | `Move_Speed` / `NavExecMoveSpeed` | `BP_SpotRobot` |
| [1-4](#step-1-4--直接テレポート移動を-off) | `bUseDirectTranslation = OFF` | `BP_SpotDogAIController` |
| [1-5](#step-1-5--navmesh-スナップを弱める) | XY のみスナップ（C++ + Rebuild） | `SpotDogNavController.cpp` |
| [1-6](#step-1-6--layout_01-e2e-最終確認) | E2E + 目視 | WSL + PIE |
| [Plan B](#plan-b--本来の目的bp-非同期対応) | C++ BP 非同期対応（本来の目的） | `SpotDogNavController` + Rebuild |

段階2・3は本ファイルの範囲外（補間・UE 標準 Path Following 等）。

---

## 現在の進捗（2026-07 時点）

### Editor 設定（`BP_SpotDogAIController` → Class Defaults）

| 項目 | 現在 | 段階1 最終目標 |
|------|------|---------------|
| **Use Direct Yaw Rotation** | **☑ ON**（暫定） | ☐ OFF |
| **Use Direct Translation** | **☐ OFF** | ☐ OFF |
| **Snap Pawn To Nav Mesh** | ☑ ON（FULL XYZ） | ☑ ON（1-5 後は XY のみ） |

> **Direct Yaw ON** は [Step 1-2-C（暫定ワークアラウンド）](#step-1-2-c-暫定ワークアラウンドdirect-yaw-on) のため。**本来の目的**（両 Direct OFF + BP アニメ）へ進むには [Plan B](#plan-b--本来の目的bp-非同期対応) の C++ 修正が必要です。

### BP 作業

| Step | 内容 | 状態 |
|------|------|------|
| 1-1-B | `RotationMethod` Max(Total Iterations, 1) | ✅ 完了 |
| 1-1-C | `NavExecRotate` 作成 | ✅ 完了 |
| 1-1-D | NavExecRotate vbp: \|yaw_delta\| ≥ 3° | ✅ 完了 |
| 1-3-B | `NavExecMoveSpeed` 作成 | ✅ 完了 |
| 1-3-D | NavExecMoveSpeed vbp: delta ≥ 50 cm | ✅ 完了 |

### 検証結果

| テスト | 設定 | 結果 |
|--------|------|------|
| diagnose（Direct Yaw ON） | Yaw ON / Trans OFF | ✅ PASS |
| diagnose（Direct Yaw OFF） | 両方 OFF 相当 | ❌ 0 cm / yaw 固定 |
| smoke（Direct Trans OFF） | Yaw ON / Trans OFF | ❌ `stuck` |

**結論**: vbp 直叩きでは `NavExecRotate` / `NavExecMoveSpeed` は動くが、**Controller の NavFollow ループ**は BP の **タイマー非同期**と C++ の **同期完了判定**がずれて失敗する。**Plan B** で C++ を修正してから Direct Yaw を OFF に戻す。

---

## Plan B — 本来の目的（BP 非同期対応）

**目的**: `Move_Speed` / `NavExecRotate` の **四足歩行 BP アニメ**で NavFollow を完走させ、`bUseDirectYawRotation` / `bUseDirectTranslation` を **両方 OFF** にする。

**目標状態（Class Defaults）**:

| 項目 | 設定 |
|------|------|
| **Use Direct Yaw Rotation** | ☐ OFF |
| **Use Direct Translation** | ☐ OFF |
| **Snap Pawn To Nav Mesh** | ☑ ON（Step 1-5 後は XY のみ） |

### なぜ Direct OFF だけでは足りないか

| 経路 | 回転 | 移動 |
|------|------|------|
| **vbp 直叩き** | `NavExecRotate` ✅ | `NavExecMoveSpeed` ✅ |
| **Controller + Direct ON** | `SetActorRotation`（瞬間） | `SetActorLocation`（瞬間）→ PASS だが見た目が悪い |
| **Controller + Direct OFF** | `NavExecRotate`（タイマー非同期） | `NavExecMoveSpeed`（タイマー非同期）→ **stuck / 0 cm** |

**根本原因**: [`SpotDogNavController.cpp`](../../ue_native/SpotDogNavController.cpp) が **同期移動前提**で書かれている。

```cpp
// IssueNextMotionCommand — Move 分支
const float Duration = FMath::Max(0.12f, MoveCm / SafeSpeed);
CallPawnMoveSpeed(SafeSpeed, Duration, 0, Target);
CommandEndWorldTime = GetWorld()->GetTimeSeconds() + Duration;

// TickFollowPath — Duration 経過後すぐ stuck 判定
if (Dist2D(SnappedLoc, LastProgressLocation) < StuckMoveThresholdCm)
    UnchangedCommandCycles++;  // 3 回で MarkFailed("stuck")
```

BP の `Move Speed` / `Rotation Method` は **Set Timer by Function Name** で非同期実行される。C++ は `Duration` 秒後に「終わった」とみなすが、**その時点では位置がほとんど変わっていない** → `stuck` になる。

### Plan B 全体フロー

```
Phase A  BP 確認（ほぼ完了）
   ↓
Phase B  C++ を「BP 非同期対応」に修正  ← 本丸
   ↓
Phase C  VS Rebuild + UE 再起動
   ↓
Phase D  Class Defaults を Direct 両方 OFF
   ↓
Phase E  1-2-B → 1-4-B → 1-6 で検証
   ↓
Phase F  Step 1-5（XY のみスナップ）で見た目調整
```

---

### Phase A — BP 確認（済みチェック）

| # | 項目 | 確認 |
|---|------|------|
| A-1 | `NavExecRotate`（Public） | ☐ |
| A-2 | `NavExecMoveSpeed`（Duration → Move Speed の Time） | ☐ |
| A-3 | `RotationMethod` Max(Iterations, 1) + Max(Duration, 0.05) | ☐ |
| A-4 | vbp: NavExecRotate 30°、NavExecMoveSpeed 180 cm | ☐ |

**Compile → Save** 済みであること。未完了なら [Step 1-1](#step-1-1--回転-bp-修正-navexecrotate--rotationmethod-安全装置) / [Step 1-3](#step-1-3--move_speed-ラッパー) に戻る。

---

### Phase B — C++ 修正（`SpotDogNavController`）

**ファイル**:

| コピー元 | コピー先（UE プロジェクト） |
|---------|---------------------------|
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.h` | `Source/SimWorld/Public/` |
| `dev/grid_env_level_nav/ue_native/SpotDogNavController.cpp` | `Source/SimWorld/Private/` |

#### B-1. 新プロパティ追加（`.h`）

`Nav Move` カテゴリに追加:

| プロパティ | 型 | デフォルト | 意味 |
|-----------|-----|-----------|------|
| `BpMotionGraceSec` | float | `0.15` | BP コマンド完了後の余裕時間 [s] |
| `BpRotateMinDurationSec` | float | `0.5` | Direct Yaw OFF 時、NavExecRotate に渡す最短 Duration |
| `bSkipStuckCheckForBpMotion` | bool | `true` | BP 非同期移動中は stuck 判定をスキップ |

#### B-2. 移動コマンドの待ち時間（`.cpp` `IssueNextMotionCommand`）

**Move 分支**（`CallPawnMoveSpeed` の直後）:

```cpp
// 変更前
CommandEndWorldTime = GetWorld()->GetTimeSeconds() + Duration;

// 変更後（bUseDirectTranslation == false のとき）
const float WaitSec = Duration + BpMotionGraceSec;
CommandEndWorldTime = GetWorld()->GetTimeSeconds() + WaitSec;
```

`Duration` は **NavExecMoveSpeed / Move_Speed に渡す Time** と同じ値（既存の `MoveCm / SafeSpeed` で OK）。

#### B-3. 回転コマンドの Duration 延長（`.cpp` `IssueNextMotionCommand`）

**Rotate 分支**:

```cpp
// 変更前
const float Duration = FMath::Max(0.12f, TurnDeg / SafeSpeed);

// 変更後
const float Duration = bUseDirectYawRotation
    ? FMath::Max(0.12f, TurnDeg / SafeSpeed)
    : FMath::Max(BpRotateMinDurationSec, TurnDeg / 30.0f);  // 30°≈1s 目安
const float WaitSec = bUseDirectYawRotation ? Duration : (Duration + BpMotionGraceSec);
CommandEndWorldTime = GetWorld()->GetTimeSeconds() + WaitSec;
```

`NavExecRotate` に **0.12 s** など短すぎる Duration を渡すと、タイマー回転が完了する前に C++ が次コマンドへ進む。

#### B-4. stuck 判定の見直し（`.cpp` `TickFollowPath`）

**Move 完了時**（`ActiveCommand == Move` ブロック内）:

```cpp
if (bUseDirectTranslation || !bSkipStuckCheckForBpMotion)
{
    // 既存: SnapPawnToNavMesh → 8cm 未満なら UnchangedCommandCycles++
}
else
{
    // BP 非同期: stuck 判定しない
    UnchangedCommandCycles = 0;
}
```

**Rotate 完了時**も同様に、`bUseDirectYawRotation == false` かつ `bSkipStuckCheckForBpMotion == true` なら stuck カウントをリセットする。

#### B-5. コマンド開始時に進捗リセット（`.cpp` `IssueNextMotionCommand`）

Move / Rotate 命令を発行した直後:

```cpp
UnchangedCommandCycles = 0;
bHasLastProgressLocation = false;
```

前コマンドの位置で stuck カウントが引き継がれないようにする。

#### B-6. （任意）回転完了の Yaw 確認

Direct Yaw OFF 時、Rotate コマンド終了後:

- `|Yaw - YawAtCommandStart| < 2°` かつ `|AngleDiff| > RotateThresholdDeg` なら **1 回だけ Duration を延長して再試行**
- 3 回失敗で `MarkFailed("rotate_stuck")`

> **実装状況**: B-1〜B-5 + **B-7（二重 Tick 修正）** は [`ue_native/SpotDogNavController.*`](../../ue_native/SpotDogNavController.cpp) に反映済み。UE プロジェクトへコピー → **Rebuild 必須**（[Phase C](#phase-c--rebuild)）。

#### B-7. 二重 TickFollowPath の除去（2026-07 追記 — 根本原因 Fix）

**症状**: vbp `NavExecMoveSpeed` / `NavExecRotate` は動くが、NavFollow 中は **0 cm / yaw 固定** で `moving` ハング。

**原因**: `SpotDogNavController` が **同一フレーム内で 2 経路**から `TickFollowPath` を呼んでいた。

| 経路 | 周期 |
|------|------|
| `AAIController::Tick` | 毎フレーム（~60 Hz） |
| `FTimerManager` タイマー | 0.05 s（20 Hz） |

BP の `Move Speed` / `Rotation Method` は **Set Timer by Function Name** で非同期実行される。二重呼び出しで **BP タイマーが上書き・中断**され、`Is Available` が 0 のまま固定 → 以降の ProcessEvent が無視される。

**修正**（`.cpp`）:

1. `StartFollowTimer()` — タイマー登録を **削除**し、`SetActorTickEnabled(true)` のみ
2. `TickFollowPath()` — **再入ガード**（`bTickFollowPathRunning`）
3. **進捗待ち** — 最小待ち時間経過後、Yaw / XY が動くまで `CommandMaxWaitSec` まで延長
4. 進捗なしで上限超過 → `rotate_stuck` / `stuck` で fail（ハングしない）

**Rebuild 後**に Direct 両方 OFF で smoke / diagnose を再実行すること。

---

### Phase C — Rebuild

| # | 操作 |
|---|------|
| 1 | `ue_native/SpotDogNavController.*` を UE `Source/SimWorld/` にコピー |
| 2 | **PIE Stop** → **UE Editor 完全終了** |
| 3 | **`SimWorld.sln`** → **Development Editor / Win64** → **ソリューションのリビルド** |
| 4 | Editor 起動 → Missing Modules → **Yes** |

---

### Phase D — Class Defaults（Plan B 完了後）

**`BP_SpotDogAIController` → Class Defaults**:

| 項目 | 設定 |
|------|------|
| **Use Direct Yaw Rotation** | **☐ OFF** |
| **Use Direct Translation** | **☐ OFF** |
| **Snap Pawn To Nav Mesh** | **☑ ON** |
| **Bp Motion Grace Sec** | `0.15`（B-1 反映後に表示） |
| **Bp Rotate Min Duration Sec** | `0.5` |
| **Skip Stuck Check For Bp Motion** | **☑ ON** |

**Compile → Save → PIE Play**

> **Plan B 未実装の間**は **Direct Yaw = ON**（現在の設定）のまま E2E を通す。Direct Translation は **OFF** のままでよい（移動は BP 経由だが、回転だけ Direct のハイブリッド状態）。

---

### Phase E — 検証（順番固定）

#### E-1. vbp 単体（PIE 中）

回転:

```bash
conda run -n simworld python -c "
import sys,time
from pathlib import Path
D=Path('~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav').expanduser()
sys.path[:0]=[str(D), str(D.parent/'grid_env_hri'), str(D.parent/'grid_env_10k')]
import grid_env_10k as g10k, grid_env_hri_simulation as geh
from grid_env_10k_pie_patrol import get_yaw
ucv,_=g10k.ensure_connection()
r=geh.ROBOT_ACTOR_NAME
y0=get_yaw(ucv,r)
geh._ue_request(ucv, f'vbp {r} NavExecRotate 1.0 30 -1', timeout_s=10)
time.sleep(1.5)
print('yaw_delta', get_yaw(ucv,r)-y0)
"
```

移動:

```bash
conda run -n simworld python -c "
import sys,time
from pathlib import Path
D=Path('~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav').expanduser()
sys.path[:0]=[str(D), str(D.parent/'grid_env_hri'), str(D.parent/'grid_env_10k')]
import grid_env_10k as g10k, grid_env_hri_simulation as geh
ucv,_=g10k.ensure_connection()
r=geh.ROBOT_ACTOR_NAME
loc0=ucv.get_location(r)
geh._ue_request(ucv, f'vbp {r} NavExecMoveSpeed 180 1.0 0', timeout_s=10)
time.sleep(1.5)
loc1=ucv.get_location(r)
print('delta_cm', ((loc1[0]-loc0[0])**2+(loc1[1]-loc0[1])**2)**0.5)
"
```

| 期待 |
|------|
| \|yaw_delta\| ≥ 3° |
| delta_cm ≥ 50 |

#### E-2. Step 1-2-B（Direct Yaw OFF）

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py
```

| 期待 |
|------|
| 末尾 **PASS**、max moved ≥ 50 cm |
| Output Log に `stuck` / `rotate_vbp_missing` **なし** |

#### E-3. Step 1-4-B（Direct Translation OFF — 現状と同設定）

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

| 期待 |
|------|
| 末尾 **PASS**（status=success） |

#### E-4. Step 1-6 E2E

```bash
NAV_MOVETO_UE=1 conda run -n simworld python dev/grid_env_level_nav/run_site_transport_20m_test.py \
  --layout-id layout_01 --nav-mode navmesh --nav-exec moveto --force-respawn
```

| 期待 |
|------|
| `[Site20] PASS`、`delivered=True` |
| PIE で **足が動く**（テレポート感が減る） |

---

### Phase F — Step 1-5（見た目調整）

Phase E PASS 後、[Step 1-5](#step-1-5--navmesh-スナップを弱める) の XY のみスナップ C++ を適用 → Rebuild → E-4 再実行。

---

### Plan B 失敗時の切り分け

| 症状 | 疑う箇所 | 対処 |
|------|----------|------|
| vbp OK、NavFollow `stuck` | C++ 待ち時間 / stuck 判定 | Phase B-2〜B-4 |
| `rotate_vbp_missing` | 関数名不一致 | `NavExecRotate` Public 確認 |
| `move_vbp_missing` | 同上 | `NavExecMoveSpeed` |
| 動くが瞬間回転 | Direct Yaw まだ ON | Phase D |
| 動くが滑る・瞬間移動 | Direct Translation まだ ON | Phase D |
| leg2 `start_not_on_navmesh` | スナップ | Phase F（Step 1-5） |

---

## Blueprint 操作の読み方（最初に1回だけ）

UE Blueprint エディタでグラフを編集するときの **線の色・形** の意味です。

| 線 / ピン | 色・形 | 意味 | 接続例 |
|-----------|--------|------|--------|
| **実行ピン** | **白い三角 ▶** | 「この処理が終わったら次へ」 | 関数入口 ▶ → **Set** ノード ▶ → **Set Timer** ▶ |
| **Float** | **緑** の丸 | 小数（速度・角度・時間） | Duration → Divide |
| **Integer** | **シアン（水色）** の丸 | 整数（回数・方向フラグ） | Clockwise → Equal |
| **Boolean** | **赤** の丸 | true / false | Equal の出力 → Branch の **Condition** |
| **Object** | **青** の丸 | Actor / Self 参照 | **Self** → Rotation Method の **Target** |

**用語**:

| 用語 | 説明 |
|------|------|
| **Pure ノード** | 実行ピン（白三角）が **無い** ノード。Max / Divide / Multiply など。いつでもデータだけつなげる |
| **Impure ノード** | 実行ピンがあるノード。SET / Branch / Set Timer / 関数呼び出し など |
| **Promote to Constant** | ピンを右クリック → 定数にする（数値を直接入力） |
| **Get 変数** | 左 **My Blueprint → Variables** からグラフへドラッグ → **Get** |
| **Compile** | ツールバー左上 **✓ Compile** |
| **Save** | **Ctrl+S** または **File → Save** |

**配線の切り方**: 線を **1 回クリック** → **Alt + 左クリック**、またはピンからドラッグして空白で離す。

**Class Defaults モード**: ツールバー **Class Defaults** が **青く光っている** と関数グラフが編集しづらい。**もう一度 Class Defaults を押して OFF** にしてから Functions を開く。

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

## Step 1-1 — 回転 BP 修正（`NavExecRotate` + `RotationMethod` 安全装置）

**目的**: C++ / Python が期待する **Duration + Angle + Clockwise** を、SpotDog の回転 BP に正しく渡し、**Divide by zero / SetTimer 0** を解消する。

> **結論（先に）**: `Rotate_Angle`（親 `BP_AgentBase`）は第3引数が **Rotation Per Tick** で、vbp/C++ の **Clockwise** と不一致。**`Rotate_Angle` 本体は直さず**、`BP_SpotRobot` に **`NavExecRotate`** を新規作成して迂回する（C++ は `NavExecRotate` を **最初**に呼ぶ）。

---

### 1-1-0. 回転 BP の構造（確認済み）

| BP | 関数 | 状態 |
|----|------|------|
| `BP_AgentBase` | `Rotate Angle` | 入口 → `Rotation Method` を呼ぶ。**第3引数 = Rotation Per Tick** |
| `BP_AgentBase` | `RotationMethod` | **空**（Override なしの親のみ） |
| `BP_SpotRobot` | **`RotationMethod`** | **実装あり**（Override）— ここを修正 |
| `BP_SpotRobot` | **`Cal Rotation Iteration`** | 反復回数計算 — 下記 1-1-A |
| `BP_SpotRobot` | **`Rotate`** | タイマーで繰り返し `Add Actor Local Rotation` |

**呼び出しチェーン**:

```
Rotate Angle (BP_AgentBase)
  └─ Rotation Method (BP_SpotRobot Override)
       ├─ Cal Rotation Iteration → Total Iterations
       ├─ Divide: Duration / Total Iterations → Set Timer("Rotate")
       └─ Rotate (毎 tick) → Add Actor Local Rotation
```

**Python / C++ が渡す引数**（`Rotate_Angle` / `NavExecRotate` vbp）:

| 引数 | 型 | 例 |
|------|-----|-----|
| Duration | Float | `1.0` |
| Angle | Float | `30` |
| Clockwise | Int | `-1` または `1` |

**`Rotate Angle` が受け取る第3引数**（不一致）:

| 引数 | 型 | 意味 |
|------|-----|------|
| Rotation Per Tick | Float | 1 tick あたりの回転量 [deg] |

→ vbp `Rotate_Angle 1.0 30 -1` では **`-1` が Rotation Per Tick として解釈**され、反復回数・タイマーが破綻する。

---

### 1-1-A. `Cal Rotation Iteration` のロジック（読むだけ・編集不要）

> **作業**: **なし**（確認のみ）。既に確認済みなら **スキップして [1-1-B](#1-1-b-rotationmethod-に安全装置を入れる)** へ。

1. **`BP_SpotRobot`** を開く
2. **My Blueprint → Functions → `Cal Rotation Iteration`** をダブルクリック
3. 下表と一致していれば OK（**Compile 不要**）

グラフの分岐（要約）:

| 条件 | 返す Iterations |
|------|-----------------|
| `Angle == 0` | **0** |
| `Angle` と `Rotational Magnitude` が **同符号** | `Truncate(Angle / Rotational Magnitude)` |
| `Angle < 0` かつ `Rotational Magnitude > 0` | `Truncate((Angle + 360) / Rotational Magnitude)` |
| 上記以外（例: `Angle > 0` かつ `Magnitude < 0`） | `Truncate((Angle - 360) / Rotational Magnitude)` |

**故障パターン**:

| 原因 | 結果 |
|------|------|
| `Rotational Magnitude == 0` | **Divide by zero**（Output Log） |
| `Truncate(Angle / Magnitude) == 0`（例: Angle=22, Magnitude=30） | **Total Iterations = 0** → `RotationMethod` で `Duration / 0` |
| vbp が **Clockwise (-1/1)** を Magnitude として渡す | 意図しない反復回数（回転しない・異常に遅い） |

**安全な Magnitude の目安**: `|Magnitude|` は **1〜10 deg/tick** 程度。`|Angle| / |Magnitude|` が **10 前後** になるよう設定（例: Angle=30 → Magnitude=3 → 10 iterations）。

---

### 1-1-B. `RotationMethod` に安全装置を入れる

**目的**: `Total Iterations == 0` でも **Divide by zero / SetTimer 0** にならない。

**開く BP**: `Content/Robot_Dog/Blueprint/BP_SpotRobot` → **Functions → `RotationMethod`**

> **重要**: **Max (Integer)** / **Max (Float)** は **Pure ノード**（白い実行ピンなし）。**実行線（白三角）は切らない**。

#### 完成後の配線図

```
[Rotation Method 入口]
  Duration (緑) ──→ [Max (Float) B=0.05] ──→ [Divide 上ピン]
  Angle, Rotational Magnitude → （既存の SET / Cal Rotation Iteration へそのまま）

[Cal Rotation Iteration] → [SET Total Iterations]
       │（白 ▶ 実行線はそのまま）
       └──────────────────────────────→ [Set Timer by Function Name] → [SET Rotation Start Timer]

[GET Total Iterations] (緑) ──→ [Max (Integer) B=1] ──→ [Divide 下ピン]
[Divide 出力] (緑) ──→ [Set Timer の Time ピン]
```

#### B-1. Max (Integer) を Divide の除数に挿入

| # | 操作 |
|---|------|
| 1 | **PIE Stop** |
| 2 | **`RotationMethod`** グラフを開く |
| 3 | **Divide** ノードを探す（**Duration ÷ Total Iterations** の **/** ノード） |
| 4 | **Divide の下ピン（除数・緑）** に入っている線を **Alt+クリック** で切断 |
| 5 | **Divide の左付近** の空白を **右クリック** |
| 6 | 検索 **`integer max`** → **`Max (Integer)`** を選択（**Float の Max ではない**） |
| 7 | 左 **My Blueprint → Variables → Rotation → Total Iterations** をグラフの空白へ **ドラッグ** |
| 8 | 出たメニューで **Get Total Iterations** を選ぶ |
| 9 | **Get Total Iterations** の **緑ピン（右）** → **Max (Integer) の A** にドラッグ |
| 10 | **Max (Integer) の B ピン** を **右クリック** → **Promote to Literal Integer** → 値 **`1`** |
| 11 | **Max (Integer) の Return Value（緑・右）** → **Divide の下ピン（除数）** にドラッグ |

#### B-2. （推奨）Duration に Max (Float) を挿入

| # | 操作 |
|---|------|
| 12 | **Rotation Method 入口** の **Duration（緑）** から **Divide 上ピン** への線があれば **Alt+クリック** で切断 |
| 13 | 空白 **右クリック** → 検索 **`float max`** → **`Max (Float)`** |
| 14 | **入口 Duration（緑）** → **Max (Float) の A** |
| 15 | **Max (Float) の B** → 右クリック → **Promote to Literal Float** → **`0.05`** |
| 16 | **Max (Float) の Return Value（緑）** → **Divide の上ピン（被除数）** |

#### B-3. 実行線は触らない

| # | 確認 |
|---|------|
| 17 | **SET Total Iterations** の **白 ▶（右）** → **Set Timer by Function Name** の **白 ▶（左）** が **そのまま繋がっている** |
| 18 | **Set Timer** の **Function Name** = **`Rotate`**、**Looping** = **☑ ON** |
| 19 | **Divide の出力（緑）** → **Set Timer の Time（緑）** |
| 20 | ツールバー **Compile** → **Save** |

**よくあるミス**: Float の **Max** を Integer 用に使う / 実行線を Max に通そうとする / B=**0** のまま。

---

### 1-1-C. `NavExecRotate` を新規作成（本 Fix）

C++ は **`NavExecRotate` → `Rotate_Angle` → `Rotate`** の順で探します。  
**`NavExecRotate` があれば `Rotate_Angle` は呼ばれません。**

**開く BP**: `BP_SpotRobot`（**Class Defaults** は **OFF**）

#### 完成後の配線図（全体）

```
[NavExecRotate 入口]
  Duration (緑) ─────────────────────────────→ [Rotation Method の Duration]
  AngleDeg (緑) ──┬→ [Abs] → [/10] → [Max 1.0] ────────────────┐
                  │                                              ↓
                  └→ [Select Float] → [SignedAngle] → [Sign] → [×] → Rotational Magnitude
                       ↑ Condition ← [Equal: Clockwise == 1]
                       A ← [AngleDeg × -1]
                       B ← AngleDeg
  Clockwise (水色) → [Equal (Integer)]

[入口 白 ▶] ─────────────────────────────────→ [Rotation Method 白 ▶]
```

#### C-1. 関数の新規作成

| # | 操作 |
|---|------|
| 1 | **`BP_SpotRobot`** を開く |
| 2 | ツールバー **Class Defaults** が青い場合 → **もう一度クリックして OFF** |
| 3 | 左 **My Blueprint → Functions** 右の **＋** → **Function** |
| 4 | 名前 **`NavExecRotate`** → **Enter** |
| 5 | 右 **Details**: **Access Specifier** → **Public** |
| 6 | **Inputs** 横 **＋** ×3: |

| Name | Type |
|------|------|
| `Duration` | Float |
| `AngleDeg` | Float |
| `Clockwise` | Integer |

入口ノード（紫・グラフ左）のピン:

| ピン | 色 | 接続先（後述） |
|------|-----|----------------|
| **白 ▶** | 白三角 | → **Rotation Method** の白 ▶ |
| **Duration** | 緑 | → **Rotation Method の Duration** |
| **AngleDeg** | 緑 | → Select / Abs / Multiply |
| **Clockwise** | 水色 | → **Equal (Integer)** |

#### C-2. `SignedAngle`（Select Float）

**ルール**: `Clockwise == 1` → **-AngleDeg**、それ以外 → **+AngleDeg**（C++ 定義に合わせる）。

| # | 操作 | 線の色 | 接続 |
|---|------|--------|------|
| 7 | **Clockwise** ピンを右へドラッグ → 離す → **`Equal (Integer)`** | 水色→水色 | Clockwise → **Equal の A** |
| 8 | **Equal の B** → 右クリック → **Promote to Literal Integer** → **`1`** | | |
| 9 | 空白右クリック → **`Select Float`** | | |
| 10 | **Equal の Return Value（赤）** → **Select Float の Pick（Condition・赤）** | 赤→赤 | |
| 11 | **AngleDeg** → ドラッグ → **`float *` (Multiply)** | 緑→緑 | AngleDeg → **Multiply A** |
| 12 | **Multiply B** → 定数 **`-1.0`** | | |
| 13 | **Multiply 出力（緑）** → **Select Float の A**（True 側） | 緑→緑 | |
| 14 | **AngleDeg（緑）** → **Select Float の B**（False 側） | 緑→緑 | |

**SignedAngle** = **Select Float の Return Value（緑）** — 後の **Sign** へ。

#### C-3. `Rotational Magnitude`（Abs → Divide → Max → Sign × MagAbs）

**Cal Rotation Iteration** の「同符号」分岐を使うため、**SignedAngle と Magnitude を同符号**にする。

| # | 操作 | 接続 |
|---|------|------|
| 15 | **AngleDeg** → ドラッグ → **`Abs (Float)`** または **`Absolute (Float)`** | AngleDeg → **Abs 入力** |
| 16 | **Abs 出力** → ドラッグ → **`float /` (Divide)** の **A** | |
| 17 | **Divide B** → 右クリック → 定数 **`10.0`** | |
| 18 | **Divide 出力** → **`Max (Float)`** の **A** | |
| 19 | **Max (Float) B** → 定数 **`1.0`** | 出力 = **MagAbs** |
| 20 | **Select Float 出力（SignedAngle）** → ドラッグ → **`Sign (Float)`** | |
| 21 | 空白 → **`float *` (Multiply)** | **Sign 出力** → **Multiply A**、**MagAbs** → **Multiply B** |
| 22 | **Multiply 出力（緑）** = **Rotational Magnitude** | → **Rotation Method の Rotational Magnitude** |

**数値例**（AngleDeg=30, Clockwise=-1）: SignedAngle=+30, MagAbs=3, Magnitude=+3 → Iterations=10。

#### C-4. `Rotation Method` を呼ぶ

| # | 操作 | 線 | 接続 |
|---|------|-----|------|
| 23 | 空白右クリック → 検索 **`Rotation Method`**（**Call Function**） | | Target = **Self**（自動） |
| 24 | 実行 | **白 ▶** | **NavExecRotate 入口 白 ▶** → **Rotation Method 白 ▶（左）** |
| 25 | 時間 | **緑** | **Duration** → **Rotation Method の Duration** |
| 26 | 角度 | **緑** | **Select Float 出力** → **Rotation Method の Angle** |
| 27 | 1 tick 量 | **緑** | **Multiply 出力（C-3）** → **Rotation Method の Rotational Magnitude** |
| 28 | **Compile** → **Save** | | |

> **回転方向が逆**なら: C-2 の **Select A/B** を入れ替える（Multiply を B 側へ）。

> **Compile エラー「型が合わない」**: Integer ピンに Float を繋いでいないか確認。色が一致するピン同士のみ接続。

---

### 1-1-D. 確認

#### D-1. `NavExecRotate` 単体（PIE 中）

| # | 操作 |
|---|------|
| 1 | **PIE Play** |
| 2 | WSL: |

```bash
conda run -n simworld python -c "
import sys, time
from pathlib import Path
D=Path('~/00_kotaprivate/Program/SimWorld/dev/grid_env_level_nav').expanduser()
sys.path[:0]=[str(D), str(D.parent/'grid_env_hri'), str(D.parent/'grid_env_10k')]
import grid_env_10k as g10k, grid_env_hri_simulation as geh
from grid_env_10k_pie_patrol import get_yaw
ucv,_=g10k.ensure_connection()
r=geh.ROBOT_ACTOR_NAME
y0=get_yaw(ucv,r)
geh._ue_request(ucv, f'vbp {r} NavExecRotate 1.0 30 -1', timeout_s=10)
time.sleep(1.2)
y1=get_yaw(ucv,r)
print('yaw_delta', y1-y0)
"
```

| # | 期待 |
|---|------|
| 3 | **\|yaw_delta\| ≥ 3°** |
| 4 | **Output Log**（**Window → Developer Tools → Output Log**）に **Divide by zero** / **SetTimer … zero** **が無い** |
| 5 | PIE で SpotDog が **段階的に回転** |

#### D-2. 診断スクリプト

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py
```

| 項目 | 期待 |
|------|------|
| `Rotate_Angle vbp probe` | **BROKEN のままでも可** |
| `NavFollowPathJson` | **ok** |
| 末尾 | **`PASS`** |

| # | 操作 |
|---|------|
| 6 | **PIE Stop** |

**Step 1-1 完了条件**: D-1 の **yaw_delta ≥ 3°** かつ Output Log に Divide/SetTimer エラーなし。

---

### 1-1-E. （任意）`Cal Rotation Iteration` 入口に Magnitude ガード

Step 1-1-B だけでもクラッシュは防げます。余裕があれば:

1. **`Cal Rotation Iteration`** を開く
2. **Angle == 0** 分岐の **前** に **Abs(Rotational Magnitude) < 0.01** → **Return 0** 分岐を追加
3. または各 **Divide** の除数に **Max(Magnitude, 0.01)** を挟む

---

## Step 1-2 — 直接 Yaw 更新を OFF

**目的**: C++ の `SetActorRotation` バypassを止め、**`NavExecRotate`**（Step 1-1 で作成）経由の回転アニメを使う。

> **前提**: Step 1-1-D が PASS していること。未完了のまま OFF にすると回転しません。

### 1-2-A. Class Defaults 変更

| # | 操作 |
|---|------|
| 1 | **PIE Stop**（ツールバー **■**） |
| 2 | **Content Drawer** → `Content/Robot_Dog/Blueprint/` → **`BP_SpotDogAIController`** を **ダブルクリック** |
| 3 | ツールバー **Class Defaults** を **1 回クリック** → ボタンが **青くハイライト** |
| 4 | 右 **Details** パネル上部の **検索欄** に **`direct yaw`** と入力 |
| 5 | **Nav Move** カテゴリを展開 |
| 6 | **Use Direct Yaw Rotation** の **チェックボックスを OFF**（☐ 空にする） |
| 7 | ツールバー **Compile（✓）** → **Save（Ctrl+S）** |

| 項目 | 変更前（暫定） | 変更後 |
|------|---------------|--------|
| **Use Direct Yaw Rotation** | ☑ ON | **☐ OFF** |

### 1-2-B. 確認

| # | 操作 |
|---|------|
| 1 | **PIE Play** |
| 2 | WSL: `conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_diagnose.py` |
| 3 | 期待: 末尾 **`PASS`** |
| 4 | PIE で **段階的に回転**（パッと向きが変わらない） |
| 5 | **PIE Stop** |

> **Plan B 未実装時**: Direct Yaw OFF だけでは NavFollow が **0 cm / yaw 固定** になることがある（vbp `NavExecRotate` は動く）。その場合は [Step 1-2-C](#step-1-2-c-暫定ワークアラウンドdirect-yaw-on) へ。

---

### Step 1-2-C. 暫定ワークアラウンド（Direct Yaw ON）

**いつ使うか**: Step 1-2-B が FAIL（NavFollow 0 cm）かつ [Plan B](#plan-b--本来の目的bp-非同期対応) の C++ 修正が **未反映** のとき。

**目的**: 回転だけ Direct Yaw（`SetActorRotation`）に戻し、移動は BP（Direct Translation OFF）のまま E2E を通す。

| # | 操作 |
|---|------|
| 1 | **PIE Stop** |
| 2 | **`BP_SpotDogAIController` → Class Defaults** |
| 3 | **Use Direct Yaw Rotation** → **☑ ON** |
| 4 | **Use Direct Translation** → **☐ OFF**（変更しない） |
| 5 | **Compile → Save → PIE Play** |
| 6 | WSL: `_nav_moveto_diagnose.py` → **PASS** を確認 |

| 項目 | 暫定設定（現在） | 備考 |
|------|-----------------|------|
| **Use Direct Yaw Rotation** | **☑ ON** | 回転アニメなし（瞬間回転） |
| **Use Direct Translation** | **☐ OFF** | 移動は `NavExecMoveSpeed` / `Move_Speed` |

> **注意**: この状態では smoke（1-4-B）が **`stuck`** になることがある。Plan B 実装後に Direct Yaw を OFF に戻し、1-2-B / 1-4-B を再検証すること。

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

**いつ必要か**: 1-3-A で **delta_cm ≈ 0** のとき。直接 `Move_Speed` vbp が動くなら **スキップ可**。

#### 完成後の配線図

```
[NavExecMoveSpeed 入口]
  Speed (緑) ────────┐
  Duration (緑) ───┼→ [Move Speed] (Target = Self)
  Direction (水色) ──┘
[入口 白 ▶] ─────────→ [Move Speed 白 ▶]
```

| # | 操作 | 線 | 接続 |
|---|------|-----|------|
| 1 | **PIE Stop** → **`BP_SpotRobot`**（**Class Defaults OFF**） | | |
| 2 | **Functions → ＋ → Function** → 名前 **`NavExecMoveSpeed`** | | |
| 3 | **Details**: **Public**、**Inputs ＋** ×3: | | |

| Name | Type |
|------|------|
| Speed | Float |
| Duration | Float |
| Direction | Integer |

| # | 操作 | 線 | 接続 |
|---|------|-----|------|
| 4 | 空白右クリック → 検索 **`Move Speed`**（**Call Function on Self**） | | |
| 5 | 実行 | **白 ▶** | 入口 **白 ▶** → **Move Speed 白 ▶** |
| 6 | 速度 | **緑** | **Speed** → **Move Speed の Speed** |
| 7 | 時間 | **緑** | **Duration** → **Move Speed の Duration** |
| 8 | 方向 | **水色** | **Direction** → **Move Speed の Direction** |
| 9 | **Compile** → **Save** | | |

> **Move Speed が見つからない**: 親 `BP_AgentBase` にある。**Target = Self** の関数呼び出しなら継承関数が使える。

### 1-3-C. `NavExecRotate` について

**回転ラッパー `NavExecRotate` は [Step 1-1-C](#1-1-c-navexecrotate-を新規作成本-fix) で作成済み** の想定。未作成なら Step 1-1 に戻る。

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

**目的**: `SetActorLocation(TeleportPhysics)` をやめ、`Move_Speed` アニメ移動に戻す。

> **前提**: Step 1-2（回転 OFF）と Step 1-3（Move_Speed 動作確認）が済んでいること。  
> **Plan B 未実装時**: Direct Translation OFF のまま smoke が **`stuck`** になるのは正常（C++ が BP 非同期を待てていない）。→ [Plan B Phase B](#phase-b--c-修正spotdognavcontroller) を先に実施するか、[Step 1-2-C](#step-1-2-c-暫定ワークアラウンドdirect-yaw-on) で Direct Yaw ON のハイブリッド状態で diagnose のみ PASS させる。

### 1-4-A. Class Defaults 変更

| # | 操作 |
|---|------|
| 1 | **PIE Stop** |
| 2 | **`BP_SpotDogAIController`** を開く |
| 3 | ツールバー **Class Defaults**（青ハイライト ON） |
| 4 | **Details** 検索: **`Nav Move`** |
| 5 | 次を **OFF（☐）** にする: |

| 項目 | 設定 |
|------|------|
| **Use Direct Translation** | **☐ OFF** |
| **Use Direct Yaw Rotation** | **☐ OFF**（1-2 で済） |

| # | 操作 |
|---|------|
| 6 | **Compile** → **Save** |

### 1-4-B. 確認

| # | 操作 |
|---|------|
| 1 | **PIE Play** |
| 2 | WSL でスモーク実行: |

```bash
conda run -n simworld python dev/grid_env_level_nav/_nav_moveto_smoke_test.py
```

| # | 操作 |
|---|------|
| 3 | 期待: 末尾 **`PASS`** |
| 4 | **Window → Developer Tools → Output Log** → 検索 **`SpotDogNavController failed`** |
| 5 | PIE で **足が動く**（滑り・瞬間移動が減る） |
| 6 | **PIE Stop** |

| ログ | 対処 |
|------|------|
| `move_vbp_missing` | Step 1-3（NavExecMoveSpeed） |
| `stuck` | [Plan B Phase B](#phase-b--c-修正spotdognavcontroller)（BP 非同期待ち）。暫定は Direct Yaw ON + diagnose PASS を確認 |

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

| # | 操作 |
|---|------|
| 1 | **`BP_SpotDogAIController`** を開く |
| 2 | **Class Defaults**（青ハイライト ON） |
| 3 | **Details** 検索: **`snap`** |
| 4 | **Nav Move** カテゴリで次を設定: |

| 項目 | 設定 |
|------|------|
| **Snap Pawn To Nav Mesh** | **☑ ON** |
| **Nav Project Extent Cm** | `30` |
| **Nav Project Retry Extent Cm** | `120` |

| # | 操作 |
|---|------|
| 5 | **Compile** → **Save** |

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
| 1-1-B | `RotationMethod` に Max(Total Iterations, 1) | ☑ |
| 1-1-C | `NavExecRotate` 作成 | ☑ |
| 1-1-D | NavExecRotate: \|yaw_delta\| ≥ 3°、Log に Divide/SetTimer なし | ☑ |
| 1-2 | `bUseDirectYawRotation = OFF` + diagnose PASS | ☑ |
| 1-2-C | 暫定: Direct Yaw ON + diagnose PASS | （不要 — 1-2 本番 PASS） |
| 1-3 | `Move_Speed` または `NavExecMoveSpeed` ≥50cm | ☑ |
| 1-4 | `bUseDirectTranslation = OFF` + smoke PASS | ☑ |
| Plan B | C++ BP 非同期対応 + Rebuild | ☑ |
| 1-5 | XY のみスナップ C++ + Rebuild | ☑（1-5-D diagnose/smoke PASS） |
| 1-6 | layout_01 E2E PASS + 目視 OK | ☑（delivered=True, leg1=55s leg2=69s） |

---

## よくある分岐

**Content Browser で `Rotate_Angle` を検索しても出ない**  
→ 正常。関数は **BP 内部** にある。**`BP_SpotRobot` を直接開く**（Step 1-1-0 参照）。

**`Rotate_Angle` は `BP_AgentBase` にしか無い**  
→ **`Rotate_Angle` は直さない**。`BP_SpotRobot` に **`NavExecRotate`** を作る（Step 1-1-C）。

**`RotationMethod` が `BP_AgentBase` では空、`BP_SpotRobot` では実装あり**  
→ 修正対象は **`BP_SpotRobot` の Override 版**。

**1-4 で PASS だが見た目がまだ変**  
→ Step 1-5（Z スナップ）が効くことが多い。それでも足りなければ段階2（補間・閾値調整）。

**Direct Yaw ON / Direct Translation OFF（現在の設定）で diagnose は PASS だが smoke が stuck**  
→ 正常。移動は BP 非同期だが C++ が完了を待てていない。[Plan B](#plan-b--本来の目的bp-非同期対応) を実施してから 1-4-B を再試行。

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
| `Rotate_Angle vbp probe: BROKEN` | **NavExecRotate** 未作成 or 配線ミス → Step 1-1-C/D（`Rotate_Angle` 自体は直さなくてよい） |
| Output Log: Divide by zero | Step 1-1-B（Max Iterations）、1-1-C（Magnitude 計算） |
| Output Log: SetTimer zero | Step 1-1-B（Total Iterations=0 時の Divide） |
| `rotate_vbp_missing` / `move_vbp_missing` | Step 1-3、`Move_Speed` Public 確認（[4-D-D](NAVMESH_PHASE5_UE_SETUP.md#4-d-d-bp_spotrobot--move_speed--rotate_angle)） |
| smoke `stuck` | [Plan B](#plan-b--本来の目的bp-非同期対応)（Direct Translation OFF 時）。vbp `NavExecMoveSpeed` が動くなら C++ 待ち時間の問題 |
| 上下にふらつく | Step 1-5（XY のみスナップ） |
| leg2 `start_not_on_navmesh` | Snap ON + XY のみスナップ；[Step 4-H](NAVMESH_PHASE5_UE_SETUP.md#step-4-h--navmesh-スナップ--waypoint-方向移動2026-07-追記) 参照 |
