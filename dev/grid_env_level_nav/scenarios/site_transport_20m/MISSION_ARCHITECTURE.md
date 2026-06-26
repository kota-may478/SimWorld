---
title: "site_transport_20m ミッション実行アーキテクチャ"
last_updated: 2026-06-25
related_files:
  - run_test.py
  - layered_nav.py
  - site_transport_config.py
  - l2_depth.py
  - object_registry.py
  - depth_frame_cache.py
  - perception_standoff.py
  - carry.py
  - metrics.py
  - viz.py
language: ja
---

# SimWorld `site_transport_20m` ミッション実行アーキテクチャ

> **TL;DR** — SpotDog（四足ロボット）が建設現場で資材を拾い、20m先の作業員（Humanoid）まで運んで足元に届ける E2E シナリオです。
> 中核は次の3つ:
> 1. **3層コストマップ**（L0 静的地図 / L1 禁止ゾーン / L2 動的障害物）を `max` 合成して A* で経路を引く
> 2. **「ゴール座標は Sight、障害物は Depth」という関心の分離** — Sight はゴール追跡、Depth は障害物地図に専念
> 3. **オープンループ移動** — 「N cm 進め / M 度回れ」を送って待つ方式で、姿勢はポーリングで補正

---

## 読み方ガイド

| 知りたいこと | 読むべき節 |
|---|---|
| 何をするミッションか | [1. ミッション概要](#1-ミッション概要) |
| 全体の設計思想をざっくり | [2. アーキテクチャ全体像](#2-アーキテクチャ全体像3つの核心アイデア) |
| 起動から完了までの流れ | [3. ミッションのライフサイクル](#3-ミッションのライフサイクル時系列) |
| ロボットが世界をどう持つか | [4. 世界の表現：3層コストマップ](#4-世界の表現3層コストマップ) |
| センサーの処理 | [5. 知覚パイプライン](#5-知覚パイプライン) |
| 経路計画と移動制御 | [6. プランニングと移動](#6-プランニングと移動) |
| 資材の持ち運び | [7. キャリー／デリバー](#7-キャリーデリバーミッション固有ロジック) |
| パラメータ調整 | [8. チューニング：NavProfile](#8-チューニングnavprofiledefault-vs-fast) |
| 出力ファイル | [9. 出力：メトリクスとアーティファクト](#9-出力メトリクスとアーティファクト) |
| 「なぜこう作ったか」 | [10. 設計判断まとめ](#10-設計判断まとめなぜそうしたか) |
| CLI / 詳細図 / 用語 | [付録](#付録-a-cli-リファレンス) |

---

## 1. ミッション概要

### 1コマンドで起きること

```bash
python run_test.py --profile fast --l2-mode sight
```

このコマンド一発で、以下が順に自動実行されます。

```
シーン生成 → SpotDog 配置 → 資材まで移動 → 資材を背負う → 作業員まで運搬 → 足元に配送
```

### 2レグ構造

ミッションは2つの「レグ（脚 / 区間）」に分かれます。

```
         Leg1: navigate_to_slot()              Leg2: deliver_to()
  ┌────────────────────────────┐      ┌──────────────────────────────┐
[Start] ───────────────────► [資材] ──[背負う]──► ………… ───────────► [作業員] ──[配送]──► 完了
                          shipping_crate          carry              human_worker
```

| レグ | 関数 | 出発 → 目標 | 区切りイベント |
|---|---|---|---|
| Leg1 | `navigate_to_slot()` | Start → 資材（shipping_crate） | 到達後に `begin_carry_from_material()` で背負う |
| Leg2 | `deliver_to()` | 資材 → 作業員（human_worker） | 到達後に `deliver_carry_at_humanoid()` で配送 |

### 成功条件

配送が完了し、かつロボットと作業員の距離が **`ARRIVE_TOLERANCE_CM × 2 = 260cm`** 以内であれば `PASS`（`run_test.py:1021`）。

---

## 2. アーキテクチャ全体像（3つの核心アイデア）

このシステムを理解する近道は、ファイルを順に追うことではなく、**3つの設計思想**を先に掴むことです。

### 核心①：3層コストマップ

ロボットの「世界地図」は3枚のレイヤーを重ねて作られます。プランニング時は常に3層を `max` で合成します。

```
L0  静的 NavMesh   … 壁・構造物（不変）
L1  禁止ゾーン      … 人が立ち入るべきでない矩形（不変）
L2  動的障害物      … 深度センサーで毎フレーム更新（唯一の可変層）
        │
        ▼  merged = max(L0, L1, L2) + 障害物周辺のソフトコスト
      A* プランナー
```

### 核心②：「ゴールは Sight、障害物は Depth」

知覚は2系統に**役割分担**されており、互いに地図を汚しません。

| 系統 | センサー | 書き込み先 | 役割 |
|---|---|---|---|
| セマンティック | AI Sight | `ObjectRegistry`（座標辞書） | **ゴール座標の追跡**（資材・作業員がどこにいるか） |
| 占有 | FusionCamera 深度 | `L2` コストマップ | **障害物の形状**（どこを避けるか） |

> **重要**：`ObjectRegistry` は L2 コストマップに**一切書き込みません**。
> これにより Sight が使えない環境でも Depth だけでナビゲーションが成立し、逆も成り立ちます。

### 核心③：オープンループ移動

SpotDog は NavMesh 追従ではなく、UnrealCV 経由の直接モーション制御（`Move_Speed` / `Rotate_Angle` VBP）で動きます。
「N cm 進め、M 度回れ」を送って所定時間待つだけ（=オープンループ）で、姿勢のズレは UE からのポーリングで毎ループ補正します。

### 登場コンポーネント早見表

| ファイル | 役割（ひとことで） |
|---|---|
| `run_test.py` | エントリポイント。ミッション全体のオーケストレーション |
| `layered_nav.py` | 1レグ分のナビゲーションメインループ（計画・移動・回復） |
| `site_transport_config.py` | NavProfile（default / fast）のパラメータ定義 |
| `l2_depth.py` | 深度画像 → L2 障害物セルの書き込み |
| `object_registry.py` | Sight 検出 → ゴール座標辞書の管理 |
| `depth_frame_cache.py` | 重い深度フェッチの結果をキャッシュ |
| `perception_standoff.py` | 障害物に近すぎるときの後退・退避判定 |
| `carry.py` | 資材の背負い／配送のメカニクス |
| `metrics.py` | タイミング・違反などのメトリクス計測 |
| `viz.py` | コストマップ画像・軌跡・JSON の出力 |

### 全体ブロック図

```
                         ┌─────────────────────────────┐
                         │        run_test.py          │  オーケストレーション
                         │  spawn → leg1 → carry →      │
                         │         leg2 → deliver       │
                         └───────────────┬─────────────┘
                                         │ navigate_to_slot / deliver_to
                                         ▼
   ┌──────────────────────── layered_nav.py（メインループ）─────────────────────────┐
   │                                                                                 │
   │   知覚サイクル ──► プランニング ──► 移動 ──► スタック検出/回復                  │
   │       │                  │             │                                        │
   └───────┼──────────────────┼─────────────┼────────────────────────────────────────┘
           │                  │             │
   ┌───────▼────────┐  ┌──────▼───────┐  ┌──▼──────────────┐
   │ 知覚パイプライン│  │ 3層コストマップ│  │ オープンループ  │
   │ Sight→Registry │  │ L0/L1/L2 + A* │  │ Move/Rotate VBP │
   │ Depth→L2       │  │               │  │                 │
   └───────┬────────┘  └───────────────┘  └─────────────────┘
           │  fetch
   ┌───────▼────────┐
   │ UnrealCV / UE  │  FusionCamera深度・AI Sight・SpotDog制御
   └────────────────┘
```

---

## 3. ミッションのライフサイクル（時系列）

`run_test.py` の `main()` がミッション全体を順次制御します。

### フェーズ一覧

| # | フェーズ | 主な処理 | 失敗時の戻り値 |
|---|---|---|---|
| 0 | 起動・引数解析 | プロファイル解決、L0 マスク確認 | 1 |
| 1 | 事前計画 | L0+L1 のみで Start→資材→作業員の経路が引けるか確認 | 1 |
| 2 | UE 接続・スポーン | `spawn_site_transport_scene()` でシーン生成 | spawn_rc |
| 3 | ロボット準備 | NavQueryService 確立、SpotDog の起立・開始位置確認 | 2 |
| 4 | **Leg1** | `navigate_to_slot()` で資材へ接近 | 3 |
| 5 | キャリー取得 | `begin_carry_from_material()` で背負う | 4 |
| 6 | **Leg2** | `deliver_to()` で作業員へ運搬 | 5 |
| 7 | 配送 | `deliver_carry_at_humanoid()` で足元に配置 | 6 |
| 8 | 出力 | メトリクス JSON・コストマップ画像・軌跡を保存 | — |

### ロボット起動シーケンス（フェーズ3）

`level_nav_robot.py` の関数群でロボットを整えます。

- `ensure_robot_upright_at_start()` … 転倒していたら復帰
- `verify_spotdog_at_start()` … 開始位置の確認
- `prepare_spotdog_mission_start()` … AI コントローラー有効化

### レグ間の L2 リセット挙動

レグの切り替わりで L2 の扱いが変わるのがポイントです（[7. キャリー／デリバー](#7-キャリーデリバーミッション固有ロジック)で詳述）。

| タイミング | 処理 | 意図 |
|---|---|---|
| Leg1 開始 | `_reset_depth_state("leg1")` | L2 を完全クリア |
| Leg2 開始 | `_reset_depth_state("leg2", carry_forward=True)` | **2-hit ラッチ済みの静的障害物だけ引き継ぐ** |

---

## 4. 世界の表現：3層コストマップ

`crop_l0_to_local_region()` がフルマップから現在のシナリオ領域（`REGION_SIZE_CM` 四方）を切り出し、`LayeredCostmap` を構築します。

### 3つのレイヤー

| 層 | 名前 | 生成元 | 可変性 | 内容 |
|---|---|---|---|---|
| **L0** | 静的 NavMesh | NavMesh マスク PNG（`crop_l0_to_local_region`） | 不変 | 壁・構造物・NavMesh 外。A* の「床」 |
| **L1** | 禁止ゾーン | プロップ配置（`apply_forbidden_zones_l1`） | 不変 | 立ち入り禁止の矩形領域。`--no-l1` で無効化可 |
| **L2** | 動的障害物 | 深度画像（`update_l2_depth`） | **可変** | 走行中にリアルタイム更新される唯一の層 |

### レイヤーフュージョン（合成）

プランニング時は `_planning_costmap(layers)` を使い、3層を合成したうえで障害物周辺にソフトコストを乗せます（`layered_nav.py:247`）。

```python
merged = max(L0, L1, L2)
# さらに、障害物から planning_clearance_cm 以内のセルに
# SITE_PLANNING_CLEARANCE_COST = 300 を付加（壁にも適用）
```

`planning_clearance_cm` は NavProfile 依存（default 100cm / fast **150cm**）。広いほど保守的に壁から離れた経路を選びます。

### L2 の中身（深度由来）

L2 は深度画像から書き込まれ、以下の工夫で品質を担保します（`l2_depth.py`）。

- **ログオッズ更新**（`use_log_odds=True`）… 誤検知を確率的に抑制
- **2-hit 静的ラッチ**（`latch_static=True`）… 2回以上検出されたセルを `l2_static_latch` で永続化
- **近接キープアウト**（`close_range_keepout_cells_from_depth`）… 100cm 以内の障害物に退避ゾーンを設定
- **レイクリア**（`apply_depth_ray_update`）… 深度線が通り抜けたセルは「空き」として消去

> **なぜ L2 を Depth 専用にしたのか**
> AI Sight は「見えているか」のブール情報でノイズが多く、遮蔽物の形状推定には深度画像のほうが適切。
> Sight はゴール追跡（`ObjectRegistry`）に専念させ、関心を分離した（[核心②](#核心ゴールは-sight障害物は-depth)）。

---

## 5. 知覚パイプライン

知覚は[核心②](#核心ゴールは-sight障害物は-depth)のとおり2系統に分かれます。

```
                       ┌──────────────────────────────┐
                       │   UnrealCV / UE              │
                       └──────┬───────────────┬───────┘
            AI Sight（意味）   │               │   FusionCamera（深度）
                              ▼               ▼
              update_object_registry      update_l2_depth
              _from_sight                       │
                              │                 ▼
                              ▼          depth_hits_from_image
                       ObjectRegistry    → apply_depth_ray_update
                       （ゴール座標辞書）  → close_range_keepout
                              │                 │
                              ▼                 ▼
                       goal_xy(slot_id)     LayeredCostmap.l2
                       （プランの目標）      （避ける障害物）
```

### 系統A：Sight → ObjectRegistry（ゴール座標）

`object_registry.py` の `ObjectRegistry` は **L2 に書かず**、`slot_id → ワールド座標` の辞書だけを管理します。

- UE の `GetVisibleSightTargetsJson` VBP で可視ターゲットを取得し `upsert()`
- `goal_xy(slot_id)` でナビゲーションの目標座標を提供
- 動的オブジェクト（`human_worker`）は視野外で自動エビクト。静的プロップは PlacementRegistry から初期化されるので見えていなくても座標を保持
- `sight_registry_every_n` で間引き可能（fast: 2サイクルに1回）
- Sight が使えない場合は幾何 FOV 判定にフォールバック

### 系統B：Depth → L2（障害物）

[4節](#l2-の中身深度由来)で述べた L2 書き込みの入口。`update_l2_depth()` が深度画像を受け取り、ヒット書き込み・レイクリア・キープアウトを実行します。

### DepthFrameCache — 重いフェッチを節約（`depth_frame_cache.py`）

深度フェッチ1回はカメラ移動＋UE tick 待機で重い（おおよそ100–300ms）。1ナビループ内で複数回使うため、以下3条件でキャッシュを再利用します。

| 条件 | パラメータ | default | fast |
|---|---|---|---|
| 有効期間 | `ttl_s` | 0.3s | 0.55s |
| 姿勢デルタ | `pose_delta_max_cm`（これ以上動いていなければ再利用） | 5cm | 12cm |
| 移動無効化 | `move_invalidate_cm`（これ以上動いたら破棄） | 30cm | 120cm |

加えて、移動完了直後に次フレームを温める**プリフェッチ**（`depth_prefetch_fn`）を備えます。

### perception_standoff — L2 書き込みのゲート（`perception_standoff.py`）

深度撮影の前に「障害物に近すぎないか」を確認します。**近すぎると深度画像が壊れる**ためです。

- `standoff_cm`（fast: 100cm）以内に障害物があれば後退してから撮影
- 後退後はキャッシュを自動失効
- 逆に深度が十分なクリアランスを示せば、前方コーン内の古い L2 セルを掃除（`evict_stale_l2_in_forward_cone`）

### 深度単位の正規化（`depth_object_perception.py`）

UE の深度バッファはピクセル値が cm で入ることがある（UnrealCV 実装依存）。`depth_npy_to_meters()` / `depth_npy_unit_hint()` が自動検出して常にメートルへ正規化します。
fast の `min_obstacle_height_cm=55cm` は、SpotDog 自身の脚をノイズとして拾わないための高さ閾値です（default は 45cm）。

---

## 6. プランニングと移動

`nav_stack/` が Nav2 相当のサーバを提供し、`layered_nav.py` の `navigate_layered_with_fusion()` が1レグのオーケストレーションを担います。

### Nav2 スタック構成（`nav_stack/`）

| モジュール | Nav2 相当 | 役割 |
|---|---|---|
| `perception_server.py` | perception_server | sight 深度→L2 + ObjectRegistry 更新 |
| `planner_server.py` | planner_server | merged L0+L1+L2 の段階的リプラン |
| `controller_server.py` + `controllers/rpp.py` | controller_server | RPP 閉ループ速度制御（default/fast プロファイル） |
| `behavior_server.py` + `stuck_recovery.py` | behavior_server | mark→backup→escape→spin→clear_local_l2→replan |
| `last_resort_recovery.py` | — | L2 全フラッシュ + L0+L1 リプラン（LAST RESORT） |
| `mission_bt.py` | BT Navigator | Leg1→carry→Leg2 のミッション遷移 |
| `nav_context.py` | — | `NavStackConfig` + `NavKpiTracker` をレグ横断で共有 |

`run_test.py` は `build_nav_context()` で `nav_ctx` を構築し、各レグへ `nav_ctx=` として渡します。KPI（`stuck_events`, `replan_success_rate`, `mean_cross_track_error_cm`, `local_costmap_updates`）は `metrics.json` の `nav_kpi` に出力されます。

### RPP コントローラ（default / fast）

| パラメータ | default | fast |
|---|---|---|
| `use_rpp_controller` | true | true |
| `rpp_lookahead_cm` | 80 | 100 |
| `segment_chunk_max_move_cm` | 50 | 70 |
| `local_costmap_resolution_cm` | 50 | 50 |

`navigate_layered_with_fusion()` が1レグの全ナビゲーションを担います。

### メインループ

```
while total_steps < max_total_steps:
    1. ロボット姿勢取得 (pos_xy, yaw_deg)
    2. ゴール到達判定 (dist ≤ tolerance_cm → 成功で return)
    3. 知覚間隔チェック → perception_interval_s ごとに知覚サイクル
    4. MOVES_PER_CYCLE 回の移動:
        a. ウェイポイント選択・到達チェック
        b. スタンドオフチェック → 必要なら後退
        c. セグメントコマンド生成 (_smooth_segment_command)
        d. 移動前ブロック判定 → ブロックならリプラン or スタック回復
        e. オープンループ移動実行 (_execute_segment_command)
        f. スタック検出 → _apply_stuck_recovery
```

### A* リプランの連鎖（`_replan_on_merged_layers`）

経路が引けないとき、地図を段階的に「緩めて」再試行します（`layered_nav.py:759`）。

```
① merged + clearance   （通常）
②      ↓ 失敗
② merged（clearance なし）
③      ↓ 失敗
③ L0 + L1（L2 を無視）
④      ↓ 失敗
④ L0 のみ（最後の手段）
```

**リプランのトリガー**
- L2 更新のセル差分が `l2_replan_cell_delta_threshold` 以上（default 1 / fast 10）
- 移動前チェックでセグメントがブロックされた
- 進捗退行（`PROGRESS_REGRESS_THRESHOLD_CM = 350cm`）を検出
- スタック回復時

### オープンループ移動と動的クランプ

向きの差に応じてコマンドの種類を切り替えます（`_smooth_segment_command`）。

| 向きの差 | コマンド |
|---|---|
| > 35° | turn のみ |
| 12°–35° | turn + move |
| < 12° | move のみ |

さらに、障害物が近いほど1回の移動量を縮めます（`_dynamic_max_move_cm`）。基準距離は `最寄り障害物` と `前方深度` の小さいほう。

| 最寄り距離 | 許容移動量 |
|---|---|
| ≥ 220cm（`NEAR_OBSTACLE_SLOW_CM`） | フル（default 120cm / fast 250cm） |
| ≥ standoff + 40cm | 140cm |
| ≥ standoff | 70cm |
| standoff 未満 | 35cm |

### スタック検出と回復（`stuck_recovery.py` → `behavior_server`）

`STUCK_CHECK_MOVES = 4` 回の移動で変位が `STUCK_MOVE_THRESHOLD_CM = 14cm` 以下ならスタックと判定し、次の順で回復します。

1. 現在位置の周囲に L2 lethal セルをマーク（`mark_l2`）
2. `UNSTUCK_BACKUP_CM = 100cm` バックアップ
3. エスケープ候補を探索し、コスト最小の方向へ短距離移動（`escape`）
4. 2回目以降: `spin + backup + wait`（`tiered_recovery`）
5. 4回目以降: `clear_local_l2`（非 aggressive soft reset）+ リプラン
6. 失敗が重なると `MAX_UNSTUCK_ATTEMPTS = 16` でミッション失敗 → `last_resort_recovery`（L2 積極フラッシュ、最大3回）

---

## 7. キャリー／デリバー（ミッション固有ロジック）

`carry.py` が資材の「持ち運び」を担当します。

### `begin_carry_from_material()` — 背負う

1. 資材アクター（shipping_crate）をマップ外に退避（`_hide_actor_offmap`）
2. CarryActor をロボット背部にスポーン・アニメーション移動
3. `attach_carry_to_robot_bone()` で UE スケルタルソケットにアタッチ
4. アタッチ不可なら Python 側で `sync_carry_pose()` を毎ステップ呼び出して同期

### `deliver_carry_at_humanoid()` — 配送

1. `detach_carry_from_robot_bone()` でデタッチ
2. 作業員の足元座標へアニメーション移動
3. `_hide_actor_offmap()` で資材を非表示化
4. ロボットと作業員の距離が `ARRIVE_TOLERANCE_CM × 2 = 260cm` 以内なら成功

### Leg2 の Carry-Forward マスク

Leg2 開始時、L2 を全リセットせず**2-hit ラッチされた静的障害物だけ**を引き継ぎます（`l2_depth.py:49` `snapshot_occupied`）。

```python
_reset_depth_state("leg2", carry_forward=True)
# → depth_tracker.snapshot_occupied(): 静的ラッチ済みセルのみ保存
# → object_registry.clear_dynamic(): 作業員の古い座標を削除
```

> **なぜか**：Leg1 の一時的な深度ノイズは捨てたい（ファントム消去）が、確実に確認した静的障害物は Leg2 でも避けたい。
> 「十分な証拠（2-hit）があるセルだけ残す」ことで両者のバランスをとる。

---

## 8. チューニング：NavProfile（default vs fast）

`--profile` で挙動を切り替えます。`careful` は `default` の別名です（`site_transport_config.py`）。

| パラメータ | default | fast | 意味 |
|---|---|---|---|
| `perception_interval_s` | 1.0s | **5.5s** | 知覚サイクルの最小間隔 |
| `site_robot_speed` | 180 cm/s | **285 cm/s** | 移動速度 |
| `site_max_open_loop_move_cm` | 120 cm | **250 cm** | 1回の最大移動距離 |
| `planning_clearance_cm` | 100 cm | **150 cm** | コストマップのクリアランス半径 |
| `perception_standoff_cm` | 50 cm | **100 cm** | 知覚前の安全距離 |
| `l2_replan_cell_delta_threshold` | 1 セル | **10 セル** | リプランをトリガーする L2 変化量 |
| `depth_move_invalidate_cm` | 30 cm | **120 cm** | この移動量でキャッシュ破棄 |
| `depth_cache_ttl_s` | 0.3s | **0.55s** | 深度キャッシュの有効期間 |
| `moves_per_cycle` | 2 | **3** | 1知覚間隔あたりの移動回数 |
| `nav_warmup_settle_s` | 4.0s | **1.0s** | ナビ開始前の安定待機 |
| `sight_registry_every_n` | 1 | **2** | Sight 更新の間引き |

> **fast の思想**：知覚頻度を下げ（1/5.5秒）、速度を上げ（285cm/s）、クリアランスを広げる（150cm）。
> 「周りをこまめに見ながらゆっくり」ではなく、**「大きく避けながら速く走り抜ける」**戦略。
> これが俗称「fast 150cm」の由来。

---

## 9. 出力：メトリクスとアーティファクト

### タイミングバケット（`metrics.py` `NavTimingAccumulator`）

ナビループの各フェーズの所要時間をミリ秒で蓄積します。

| バケット | 内容 |
|---|---|
| `perceive_ms` | 知覚サイクル全体（depth_fetch + l2_update + sight_registry） |
| `move_ms` | 移動・回転コマンドの送信時間 |
| `replan_ms` | A* リプラン時間 |
| `settle_ms` | `tick_settle()` の待機時間 |
| `standoff_ms` | スタンドオフ・バックオフ処理 |
| `depth_refresh_ms` | 知覚サイクル外の深度更新 |
| `pose_query_ms` | 姿勢クエリ時間 |
| `loop_overhead_ms` | ループの残余オーバーヘッド |

### ミッションメトリクス（`MissionRecorder`）

各ステップで `record_pose()` を呼び、以下を計測。

- 速度違反（5 km/h 超え）
- 禁止ゾーン侵入時間
- オブジェクト近接違反（1m 以内）

### 出力ファイル（`viz.py` `save_site_transport_artifacts`）

| ファイル | 内容 |
|---|---|
| `costMap_{suffix}.png` | L0/L1/L2 重ね合わせ + 軌跡 + 計画パス + L2 推定位置 |
| `metricsSummary_{suffix}.png` | ステータスバナー + タイミング表 + 違反グラフ |
| `metricsSummary_{suffix}.json` | 全メトリクス |
| `timing_{suffix}.json` | タイミングサマリー |
| `site_transport_costmap_{suffix}.npz` | L0/L1/L2/merged の NumPy 配列 |
| `site_transport_trajectory_{suffix}.json` | 軌跡・計画パス・L2 推定点 |
| `latest_*.json` | 最新実行へのコピー |

**命名規則**：`--run-label L0L2_fast_150cm --trial-index 3` を付けると `metricsSummary_L0L2_fast_150cm_3.json` のように固定名で保存。ラベルなしの場合は UTC タイムスタンプ（`20240101T120000Z`）が使われます（両者は同時指定が必須）。

---

## 10. 設計判断まとめ（なぜそうしたか）

| # | 判断 | 理由 |
|---|---|---|
| 1 | **L2 を Depth 専用にした** | AI Sight は遮蔽に敏感でノイズが多い。障害物形状の精密推定には深度画像が適切。Sight はゴール追跡に特化させ役割を分離 |
| 2 | **ObjectRegistry を L2 から分離した** | 動的ゴール（作業員）の座標はコストマップに焼き付けるべきでない。レジストリは目標座標の辞書、コストマップは障害物専用 |
| 3 | **オープンループ移動を採用** | SpotDog は NavMesh 追従ではなく直接モーション制御（`Move_Speed` / `Rotate_Angle` VBP）。姿勢フィードバックはポーリングで補う |
| 4 | **DepthFrameCache を設けた** | 深度フェッチは1回100–300ms。1ループで複数回使うため、TTL＋姿勢ゲートのキャッシュで無駄なフェッチを排除 |
| 5 | **Carry-Forward マスク** | Leg2 で L2 を全リセットすると Leg1 で確認した静的障害物が失われる。2-hit ラッチ済みセルだけ引き継ぎ、ファントム消去と信頼できる障害物保持を両立 |

---

## 付録 A: CLI リファレンス

| フラグ | 意味 | デフォルト |
|---|---|---|
| `--l0` | L0 NavMesh マスク PNG のパス | `L0_MASK_STRICT` |
| `--profile {default,careful,fast}` | NavProfile の選択 | `default` |
| `--l2-mode {sight,geom,camera,off}` | L2 知覚モード | `sight` |
| `--no-l2` | L2 無効（`--l2-mode off` のエイリアス） | — |
| `--no-l1` | L1 禁止ゾーンを無効化 | — |
| `--max-nav-steps` | ナビゲーション最大ステップ数 | 600 |
| `--skip-spawn` | シーンのスポーンをスキップ（再実行時） | — |
| `--spawn-only` / `--plan-only` | デバッグ用の部分実行 | — |
| `--force-rebuild-registry` | レジストリを強制再構築 | — |
| `--artifact-dir` | 出力先ディレクトリ | `DEFAULT_ARTIFACT_DIR` |
| `--run-label` / `--trial-index` | アーティファクトのラベリング（同時指定が必須） | — |

> `--l2-mode` の `sight` が現行の主経路。`geom`（幾何推定）/ `camera`（深度＋マスク）/ `off`（L0+L1 のみ）はフォールバックや比較実験用。

---

## 付録 B: 1ナビゲーションサイクル詳細シーケンス図

```mermaid
sequenceDiagram
    participant Main as run_test.py
    participant Nav as layered_nav.py
    participant DFC as DepthFrameCache
    participant UE as UnrealCV/UE
    participant L2 as l2_depth.py
    participant SO as perception_standoff.py
    participant OR as ObjectRegistry
    participant AP as A* Planner

    Main->>Nav: navigate_layered_with_fusion(goal_local)
    loop ナビゲーションメインループ
        Nav->>UE: get_pos2d() / get_yaw()
        Nav->>Nav: ゴール到達チェック (dist ≤ 130cm?)

        alt 知覚インターバル経過
            Nav->>SO: check_perception_standoff()
            alt 障害物が standoff_cm 以内
                Nav->>UE: dog_move(後退)
                Nav->>DFC: invalidate("standoff_backoff")
            end
            Nav->>DFC: get_or_wait(pose, fetch_fn)
            alt キャッシュ MISS
                DFC->>UE: fetch_depth_npy(fusion_cam)
                UE-->>DFC: depth_raw (cm単位)
                DFC->>DFC: depth_npy_to_meters() → depth_m
            end
            DFC-->>Nav: min_fwd_cm
            Nav->>L2: update_l2_depth(depth_m, layers, robot_pose)
            L2->>L2: depth_hits_from_image() → apply_depth_ray_update()
            L2->>L2: close_range_keepout_cells_from_depth()
            L2-->>Nav: DepthUpdateResult(hit_cells, cleared_cells)
            Nav->>OR: update_object_registry_from_sight()
            OR->>UE: GetVisibleSightTargetsJson VBP
            UE-->>OR: visible targets JSON
            OR->>OR: upsert(slot_id, world_xy)
            alt L2 セル変化 ≥ threshold
                Nav->>AP: _replan_on_merged_layers(L0+L1+L2+clearance)
                AP-->>Nav: new waypoints
            end
        end

        loop MOVES_PER_CYCLE 回
            Nav->>Nav: _smooth_segment_command(pos→waypoint)
            Nav->>Nav: _ensure_move_standoff() [前進前チェック]
            Nav->>Nav: _dynamic_max_move_cm(nearest, fwd_depth) [移動量クランプ]
            Nav->>Nav: world_segment_is_traversable() [セグメントブロック判定]
            alt セグメントブロック
                Nav->>AP: リプラン
            else 移動可能
                Nav->>UE: dog_move(speed, duration)
                Nav->>DFC: note_move_cm(move_cm)
                Nav->>DFC: prefetch_async() [次フレームプリフェッチ]
                Nav->>Nav: スタック検出チェック
                alt スタック
                    Nav->>Nav: _apply_stuck_recovery() [バックアップ+L2マーキング+リプラン]
                end
            end
        end
    end
    Nav-->>Main: True(到達) / False(失敗)
```

---

## 付録 C: 用語集

| 用語 | 意味 |
|---|---|
| **レグ (Leg)** | ミッションを構成する移動区間。Leg1=資材まで、Leg2=作業員まで |
| **L0 / L1 / L2** | コストマップの3層（静的地図 / 禁止ゾーン / 動的障害物） |
| **slot_id** | オブジェクトの識別子。`ObjectRegistry` でゴール座標と紐づく |
| **スタンドオフ (standoff)** | 知覚・移動の前に確保する障害物からの安全距離 |
| **オープンループ移動** | フィードバックなしで「N cm 進め / M 度回れ」を送る移動方式 |
| **2-hit ラッチ** | 2回以上検出されたセルを静的障害物として永続化する仕組み |
| **Carry-Forward** | レグ切替時に静的ラッチ済み L2 セルだけを引き継ぐこと |
| **ファントム** | 実在しないのに L2 に残ってしまった誤検知障害物 |
| **NavProfile** | 速度・知覚頻度などをまとめた挙動プリセット（default / fast） |
| **FusionCamera** | 深度画像を取得する UnrealCV のカメラ |
| **AI Sight** | UE の AI Perception。可視ターゲットを返す（ゴール追跡用） |
```
