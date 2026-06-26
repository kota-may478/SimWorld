---
title: "site_transport_20m Nav2 型ナビゲーション進化ロードマップ"
last_updated: 2026-06-26
status: draft
related_files:
  - MISSION_ARCHITECTURE.md
  - layered_nav.py
  - run_test.py
  - site_transport_config.py
  - metrics.py
language: ja
---

# Nav2 型ナビゲーション進化ロードマップ

> **目的** — `site_transport_20m` を、ROS 2 [Nav2](https://docs.nav2.org/) の設計思想を **featuring** した高性能ナビゲーションシミュレーションへ段階的に進化させる。
>
> **スコープ** — Phase 1・2 を先行実装し、完了後に Phase 3 へ移行する。本ドキュメントは実装方針・課題・完了条件の単一の参照元とする。
>
> **関連** — 現行アーキテクチャの詳細は [MISSION_ARCHITECTURE.md](./MISSION_ARCHITECTURE.md)。

---

## 0. ビジョンと成功の定義

### 0.1 目指す姿（Phase 1+2 完了時）

| 観点 | 現状 | 目標 |
|---|---|---|
| アーキテクチャ | `layered_nav.py` 単体の巨大ループ | Nav2 相当の **知覚 / 計画 / 制御 / ミッション** 分離 |
| 追従制御 | オープンループ `dog_move` + 単純ターン分岐 | **RPP 系ウェイポイント追従** + 障害物に応じた速度クランプ |
| 地図 | グローバル L0+L1+L2 のみ | **グローバル + ローカル（ローリング）** コストマップ |
| 再計画 | L2 セル差分 + ブロック + スタック | 上記に加え **制御層からのリプラン要求** |
| リカバリ | 固定シーケンス | **プラグイン可能な回復行動**（Nav2 recovery 相当） |
| 評価 | PASS/FAIL + タイミング | **定量 KPI**（到達率・スタック率・経路効率）でプロファイル比較可能 |

### 0.2 Nav2 を「そのまま入れる」わけではない

- ランタイムは **UE + UnrealCV + Python** のまま。ROS 2 ノード化は Phase 1/2 の必須条件にしない。
- 移植対象は **概念とアルゴリズム**（コストマップ二層、グローバル/ローカルプランナ、コントローラ、リカバリ、BT 的オーケストレーション）。
- 既存の **Sight（ゴール）/ Depth（L2）分離** は維持する（Nav2 導入の理由にならない）。

### 0.3 定量 KPI（全フェーズ共通）

| KPI | 測定方法 | Phase 1 目標（目安） | Phase 2 目標（目安） |
|---|---|---|---|
| **Leg1/ Leg2 到達率** | `run_l0_l2_slam_trials.py` 等で N 試行 | default プロファイル 95%+ | fast でも 90%+ |
| **スタック発生率** | `STUCK` ログ / `unstuck_attempts` | 50% 削減 | さらに 30% 削減 |
| **ゴール距離誤差** | 到達時 `dist_goal` cm | central 値 30% 改善 | 同上 |
| **リプラン成功率** | replan 失敗ログ比率 | 現状比改善 | ロールバック率低下 |
| **1 レグ所要時間** | `leg1_time_s` / `leg2_time_s` | 大幅悪化させない（±10%） | fast の意図した短縮を維持 |

---

## 1. 現状ギャップ分析（Nav2 対照表）

| Nav2 コンポーネント | 現行実装 | ギャップ |
|---|---|---|
| `bt_navigator` | `run_test.py` 直列 | BT なし・分岐固定 |
| `planner_server` | `_replan_on_merged_layers` + A* | 単一 2D A*、kinematic 制約なし |
| `controller_server` | `_smooth_segment_command` + `_execute_segment_command` | ローカルプランナなし・閉ループなし |
| `global_costmap` | L0+L1+L2 merged | あり（相当） |
| `local_costmap` | **なし** | ローリングウィンドウ未実装 |
| `behavior_server` | `_apply_stuck_recovery` 内蔵 | モジュール化・拡張性なし |
| SLAM / AMCL | L0 事前 PNG + UE pose | 自己位置は UE 依存（Phase 3 で検討） |
| `cmd_vel` 閉ループ | `vbp Move_Speed` 時間固定 | UE API 制約 |

### 1.1 既知の実装バグ・技術的負債（Phase 1 で先に直す）

| 項目 | 内容 | 影響 |
|---|---|---|
| **`cells_removed` 未伝播** | `run_test.py` の `PerceiveOutcome` が常に `cells_removed=0` | レイクリアがあってもリプラン閾値に効かない |
| **Leg 中ゴール固定** | `navigate_layered_with_fusion` の `goal_xy` はレグ開始時凍結 | Leg2 で作業員移動に非追従（Phase 3） |
| **`layered_nav.py` 肥大化** | 知覚・計画・制御・回復が 1 ファイル | Phase 2 分割の阻害要因 |
| **モジュール定数の globals** | `apply_profile_to_layered_nav` が `layered_nav` の global を書き換え | テスト並列化・複数プロファイル同時実行が困難 |

---

## 2. Phase 1 — 制御ループの近代化（最優先）

### 2.1 目的

オープンループ依存を減らし、Nav2 の **Controller Server 層** に相当する追従品質を実現する。グローバルプランナ（A*）は当面維持し、**「計画は悪くないが走れない」** 問題を解消する。

### 2.2 成果物一覧

| # | 成果物 | 新規/変更ファイル（案） |
|---|---|---|
| P1-1 | **Regulated Pure Pursuit (RPP) コントローラ** | `controllers/rpp.py`（新規） |
| P1-2 | **閉ループセグメント実行器** | `controllers/segment_executor.py`（新規） |
| P1-3 | **障害物に基づく速度スケーリング** | `controllers/velocity_scaler.py`（新規） |
| P1-4 | **`cells_removed` 修正** | `run_test.py`, `layered_nav.py` |
| P1-5 | **コントローラ単体テスト** | `test_rpp_controller.py`（新規） |
| P1-6 | **回帰試行スクリプト更新** | `run_l0_l2_slam_trials.py`, `metrics.py` |

### 2.3 P1-1: RPP コントローラ

#### 方針

Nav2 [Regulated Pure Pursuit](https://docs.nav2.org/) の核心を Python で再実装する。

**入力**

- 現在姿勢 `(pos_xy, yaw_deg)`（UE ポーリング）
- グローバル経路 `waypoints` と `wp_index`
- ローカル障害物情報（当面は merged L2 + `forward_depth_cm`）

**出力**

- `SegmentCommand`（`turn_deg`, `move_cm`, `turn_clockwise`）— 既存 `dog_move` / `dog_rotate` へのアダプタを維持

#### アルゴリズム要素（実装すべき項）

1. **先読み点（lookahead）** — 経路上でロボットから `lookahead_dist_cm` 先の点を選ぶ
2. **曲率制御** — 先読み点への曲率に応じて速度上限を下げる（Regulated 部分）
3. **障害物制御** — `nearest_dist_cm` / `forward_depth_cm` で速度上限をさらに下げる
4. **到達判定** — ウェイポイント / 最終ゴールの許容誤差は既存 `PATH_WP_REACH_TOLERANCE_CM` / `tolerance_cm` を踏襲

#### インターフェース（案）

```python
@dataclass(frozen=True)
class RppConfig:
    lookahead_cm: float = 80.0
    min_lookahead_cm: float = 40.0
    max_lookahead_cm: float = 150.0
    regulated_linear_scaling_min_radius_cm: float = 120.0
    regulated_linear_scaling_min_speed_frac: float = 0.4
    rotate_to_heading_threshold_deg: float = 35.0  # 既存 SITE_SMOOTH_TURN_MOVE_DEG と整合

def compute_rpp_command(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoints: Sequence[WorldXY],
    wp_index: int,
    *,
    config: RppConfig,
    max_move_cm: float,
) -> Optional[SegmentCommand]: ...
```

#### `_smooth_segment_command` からの移行

- Phase 1 では **フラグ切替**（`NavProfile.use_rpp_controller: bool`）で並走可能にする
- default は RPP OFF、検証後 default ON → 最終的に旧関数削除

### 2.4 P1-2: 閉ループセグメント実行器

#### 現状の問題

```python
# 現状: コマンド送信 → duration だけ sleep → 終了（実際の移動量は未検証）
_site_dog_move(ucv, robot_name, speed, move_duration_s, direction=0)
time.sleep(duration_s)
```

姿勢ドリフトが累積し、ウェイポイント追従誤差が拡大する。

#### 方針

**目標到達型の細切れオープンループ**（UE が真の `cmd_vel` を提供しない前提の現実解）。

```
while remaining_dist > epsilon:
    pose = get_pos2d / get_yaw
    cmd = rpp.compute(...)
    execute_small_chunk(cmd)   # 既存 MAX_MOVE 以下に分割
    if progress < min_progress: break  # スタック早期検出
```

| パラメータ | 推奨初期値 | 意味 |
|---|---|---|
| `chunk_max_move_cm` | 40〜60 | 1 回の `dog_move` 上限 |
| `chunk_max_turn_deg` | `MAX_TURN_DEG_PER_STEP` | UE クラッシュ回避 |
| `progress_epsilon_cm` | 8 | チャンク成功判定 |
| `max_chunks_per_segment` | 8 | 無限ループ防止 |

#### 課題: UE `Move_Speed` の非線形性

- 実移動距離が `speed × duration` と一致しない可能性がある
- **対策**: チャンクごとに実測 `dist2d` を記録し、経験的スケール係数を `NavProfile` で調整可能にする
- **検証**: 平地で 100cm コマンド × 10 回の散布を計測し、`metrics` に `open_loop_scale` を追加

### 2.5 P1-3: 速度スケーリング（DWA の1軸版）

Phase 1 では **フル DWA は入れない**。RPP の `max_move_cm` / `speed` に対するスカラー倍率のみ。

| 距離条件 | 倍率（案） |
|---|---|
| 最寄り障害 ≥ 220cm | 1.0 |
| ≥ standoff + 40cm | 0.7 |
| ≥ standoff | 0.5 |
| < standoff | 0.25（ほぼ旋回のみ） |

既存 `_dynamic_max_move_cm` のロジックを `velocity_scaler.py` に移し、RPP と共有する。

### 2.6 P1-4: `cells_removed` 修正

`update_l2_depth()` は `DepthUpdateResult.cleared_cells` を返す。`PerceiveOutcome` に正しく渡す。

```python
# run_test.py _perceive_sight 内（修正後イメージ）
return PerceiveOutcome(
    detections=...,
    cells_added=result.hit_cells + result.keepout_cells,
    cells_removed=result.cleared_cells,
    l2_applied=True,
)
```

`cells_added` を `total_cells_added` のままにするか、hit/keepout/clear を分離してログするかは実装時に統一する。

### 2.7 Phase 1 完了条件（Definition of Done）

- [ ] `NavProfile.use_rpp_controller=true` で Leg1/Leg2 が完走する（default プロファイル）
- [ ] `test_rpp_controller.py` が先読み点・曲率制御・到達判定をカバー
- [ ] `cells_removed` がリプラン閾値に反映される（ユニットテスト追加）
- [ ] KPI: スタック発生率 50% 削減（同一試行条件で Before/After 比較）
- [ ] `MISSION_ARCHITECTURE.md` の「オープンループ移動」節に RPP 追記（Phase 1 マージ時）

### 2.8 Phase 1 の主要リスクと対策

| リスク | 深刻度 | 対策 |
|---|---|---|
| UE `dog_move` の再現性が低い | 高 | チャンク化 + 実測フィードバック + スケール係数 |
| RPP 導入で低速化 | 中 | `fast` プロファイルは lookahead / chunk を別設定 |
| 大回転で UE クラッシュ | 高 | 既存 `_dog_rotate_chunked` を必ず経由 |
| キャリー中の物理干渉 | 中 | `carry_motion_cb` をチャンクループ内でも呼ぶ（現行踏襲） |

---

## 3. Phase 2 — Nav2 型スタック分離

### 3.1 目的

Phase 1 で改善した制御を、Nav2 と同様の **モジュール境界** に整理する。シミュレーションを「Nav2 featuring」として **説明可能・拡張可能・テスト可能** にする。

### 3.2 目標アーキテクチャ

```
run_test.py / mission_bt.py          ← bt_navigator 相当
    │
    ├── perception_server.py         ← 深度→L2, Sight→Registry（既存を集約）
    ├── planner_server.py            ← global costmap + A*
    ├── controller_server.py         ← RPP + segment_executor（Phase 1）
    ├── behavior_server.py           ← recovery プラグイン
    └── nav_context.py               ← 共有状態（layers, registry, trace, timing）
```

#### データフロー（1 ナビゲーション周期）

```
Controller (10〜20 Hz 相当)
    │  ← LocalCostmap (rolling)
    │  ← global path + wp_index
    ▼
cmd → UE

Planner (0.2〜1 Hz 相当)
    │  ← GlobalCostmap (L0+L1+L2)
    │  ← replan triggers
    ▼
new waypoints

Perception (NavProfile.perception_interval_s)
    │  ← depth cache, sight registry
    ▼
L2 update → may trigger Planner

Behavior (イベント駆動)
    ← stuck, plan_failed, progress_regress
    ▼
recovery actions → may clear L2 / backup / spin
```

### 3.3 成果物一覧

| # | 成果物 | 説明 |
|---|---|---|
| P2-1 | **`nav_context.py`** | ナビ状態の明示的コンテナ（globals 廃止方向） |
| P2-2 | **`global_costmap.py`** | L0+L1+L2 + clearance 合成（`_planning_costmap` 移動） |
| P2-3 | **`local_costmap.py`** | ロボット中心ローリングウィンドウ（例: 6m 四方） |
| P2-4 | **`planner_server.py`** | A* リプラン + 段階的緩和チェーン |
| P2-5 | **`controller_server.py`** | RPP + 閉ループ実行の窓口 |
| P2-6 | **`behavior_server.py`** | 回復行動プラグイン |
| P2-7 | **`mission_bt.py`** | leg1 / carry / leg2 / recover の BT 的実行 |
| P2-8 | **`layered_nav.py` 薄型化** | 上記への委譲ファサード（後方互換） |
| P2-9 | **統合テスト** | `test_nav_stack_integration.py` |

### 3.4 P2-3: ローカルコストマップ

#### 仕様

| 項目 | 値（案） |
|---|---|
| サイズ | 600cm × 600cm（`REGION` 解像度に合わせる） |
| 更新源 | L2 のうちローカル窓内 + 最新 depth ヒット |
| 更新頻度 | 移動ステップごと（または depth cache TTL に同期） |
| グローバルとの関係 | プランナは引き続きグローバル merged を使用。**コントローラと衝突判定はローカル** |

#### Nav2 との対応

- `global_costmap` = 既存 `LayeredCostmap` + `_planning_costmap`
- `local_costmap` = `RollingCostmap` 新規（L2 のクロップ + インフレーション）

#### 課題

- L2 はグローバル格子のインデックスで保持されているため、**クロップ時の原点ずれ**に注意
- ローカルとグローバルで障害物の見え方が異なると、コントローラが止まりプランナが通れる、という不整合が起きうる
- **対策**: コントローラの停止判定はローカル、リプラン要求はグローバル、という役割分担をドキュメント化してテストで検証

### 3.5 P2-6: 回復行動プラグイン

Nav2 `behavior_server` を参考に、回復を独立クラスにする。

| 行動 ID | 既存対応 | 新 API（案） |
|---|---|---|
| `backup` | `_unstuck_backup` | `RecoveryAction.run(ctx) -> RecoveryResult` |
| `spin` | `_lateral_unstuck_rotate_backup` | 同上 |
| `clear_local_l2` | `soft_l2_depth_reset` | 同上 |
| `clear_global_l2_aggressive` | aggressive reset | LAST RESORT のみ |
| `replan` | `_replan_on_merged_layers` | 同上 |
| `wait` | なし（新規） | `tick_settle` 固定待機 |

#### 実行ポリシー（BT 的）

```
on_stuck:
  1. backup
  2. replan
  if still stuck:
  3. spin + backup
  4. clear_local_l2 + replan
  if attempts >= MAX:
  5. clear_global_l2_aggressive + replan
  else:
  fail mission
```

既存 `_apply_stuck_recovery` のロジックをこの順序表に **明示的に写像** する。

### 3.6 P2-7: ミッション BT

Phase 2 では **軽量 BT**（Python の明示的状態機械 + テーブル）で十分。外部 BT ライブラリは必須としない。

```python
class MissionNode(Protocol):
    def tick(self, ctx: NavContext) -> NodeStatus: ...

# 例: Sequence[NavigateToMaterial, BeginCarry, DeliverToHumanoid]
```

`run_test.py` の `main()` はシーン生成・UE 接続後、`MissionRunner` に委譲する形へ。

### 3.7 `NavContext` — globals 廃止方針

```python
@dataclass
class NavContext:
    ucv: UnrealCV
    layers: LayeredCostmap
    local_costmap: RollingCostmap
    object_registry: ObjectRegistry
    profile: NavProfile
    trace: NavTrace
    timing: NavTimingAccumulator
    depth_cache: DepthFrameCache
    # ...
```

`apply_profile_to_layered_nav` は `NavProfile` を `ctx` に持たせ、各サーバーが読む形に移行する。段階的移行のため、Phase 2 初期は `layered_nav` globals の proxy を残してよい。

### 3.8 Phase 2 完了条件（Definition of Done）

- [ ] `layered_nav.navigate_layered_with_fusion` が内部で `controller_server` / `planner_server` を呼ぶ
- [ ] ローカルコストマップが移動中に更新され、コントローラが参照する
- [ ] 回復行動が `behavior_server` のプラグインとして追加・順序変更可能
- [ ] `mission_bt.py` から Leg1→carry→Leg2 が実行できる（`run_test.py` は薄いラッパ）
- [ ] ユニットテスト: global/local costmap, planner, behavior 各 1 ファイル以上
- [ ] KPI: Phase 1 比でスタック率さらに 30% 削減、fast 到達率 90%+
- [ ] `MISSION_ARCHITECTURE.md` に Nav2 対照図を追記

### 3.9 Phase 2 の主要リスクと対策

| リスク | 深刻度 | 対策 |
|---|---|---|
| リファクタで回帰 | 高 | Phase 1 完了時点の trial を golden に固定、CI 比較 |
| ローカル/グローバル不整合 | 高 | 統合テストで「コントローラ停止 + プランナ成功」ケースを再現 |
| ファイル分割による import 循環 | 中 | `nav_context` のみが下位モジュールを知る |
| UE セッション断 | 中 | 既存 `ensure_live_or_reconnect` を `NavContext` 経由に統一 |

---

## 4. Phase 3 — プレビュー（Phase 1+2 完了後に着手）

Phase 3 は本ロードマップの **実装スコープ外（将来）** だが、方針だけ固定しておく。

| 項目 | 内容 | 前提 |
|---|---|---|
| **Smac / Hybrid A*** | フットプリント・曲率制約付きグローバルプランナ | Phase 2 の `planner_server` 差し替え |
| **動的ゴール追従** | Leg 中の `object_registry` 更新 → goal 再設定 → replan | Sight 系統 A の拡張 |
| **L0 オンライン修正** | 軽量 pose graph / L0 差分層 | SLAM 要件の精査後 |
| **TEB / DWA** | ローカル軌道最適化 | `controller_server` プラグイン化後 |
| **本格 SLAM** | 未知環境シナリオ向け | 問題設定 A への拡張時のみ |

Phase 3 の詳細設計は **Phase 2 完了レビュー** で KPI と残課題を見てから `NAV2_NAV_ROADMAP.md` に追記する。

---

## 5. 実装順序（推奨スプリント）

### Sprint 1（Phase 1 基盤）

1. P1-4 `cells_removed` 修正 + テスト
2. P1-1 `controllers/rpp.py` + 単体テスト
3. P1-3 `velocity_scaler.py`（`_dynamic_max_move_cm` 移行）
4. `NavProfile.use_rpp_controller` フラグ

### Sprint 2（Phase 1 閉ループ）

5. P1-2 `segment_executor.py`（チャンク化閉ループ）
6. `layered_nav` への統合
7. KPI 試行・パラメータ調整（default / fast）

### Sprint 3（Phase 2 地図分離）

8. P2-1 `nav_context.py`
9. P2-2 `global_costmap.py`
10. P2-3 `local_costmap.py`

### Sprint 4（Phase 2 サーバー分離）

11. P2-4 `planner_server.py`
12. P2-5 `controller_server.py`
13. P2-6 `behavior_server.py`

### Sprint 5（Phase 2 統合）

14. P2-7 `mission_bt.py`
15. P2-8 `layered_nav` 薄型化
16. P2-9 統合テスト + KPI 最終評価
17. `MISSION_ARCHITECTURE.md` 更新

---

## 6. テスト戦略

### 6.1 テストピラミッド

| 層 | 内容 | 例 |
|---|---|---|
| 単体 | 幾何・コストマップ・RPP 数学 | `test_rpp_controller.py` |
| コンポーネント | planner / behavior / local_costmap | `test_planner_server.py` |
| 統合 | モック UE なしでは限定的 → **PIE 試行** | `run_l0_l2_slam_trials.py` |
| 回帰 | 固定 seed・固定 profile の PASS 率 | golden trial ログ |

### 6.2 UE 依存テストの注意

- CI で PIE が無い場合、単体テストのみ自動化
- PIE 試行は **手動または専用ランナー** で KPI 取得
- `metricsSummary_*.json` を試行ごとに保存し、Before/After 比較

### 6.3 新規メトリクス（Phase 1/2 で追加推奨）

| メトリクス | 用途 |
|---|---|
| `stuck_events` | スタック回数 |
| `replan_success_rate` | リプラン成功率 |
| `mean_cross_track_error_cm` | 経路からの横ずれ（RPP 評価） |
| `open_loop_scale_ema` | 移動スケール推定 |
| `local_costmap_updates` | Phase 2 デバッグ |

---

## 7. UE / UnrealCV 制約（全フェーズ共通）

| 制約 | 影響する設計 | 回避策 |
|---|---|---|
| `Move_Speed` / `Rotate_Angle` は時間ベース | 真の閉ループ速度制御不可 | チャンク化 + pose フィードバック |
| 大角度 `dog_rotate` でクラッシュ | 回転は必ず分割 | `_dog_rotate_chunked` |
| 深度フェッチ 100–300ms | 高頻度コントローラと競合 | `DepthFrameCache` 継続利用 |
| PIE セッション断 | 長時間試行で失敗 | `ensure_live_or_reconnect` |
| FusionCam = depth のみ（障害物） | LiDAR 相当は depth 依存 | L2 品質チューニングに集中 |

---

## 8. NavProfile 拡張（Phase 1/2 で追加予定のフィールド）

```python
@dataclass(frozen=True)
class NavProfile:
    # ... 既存 ...
    use_rpp_controller: bool = False          # Phase 1: 初期 False → 検証後 True
    rpp_lookahead_cm: float = 80.0
    rpp_regulated_min_radius_cm: float = 120.0
    segment_chunk_max_move_cm: float = 50.0
    open_loop_distance_scale: float = 1.0     # 実測補正
    local_costmap_size_cm: float = 600.0      # Phase 2
    local_costmap_resolution_cm: float = 50.0 # L2 と揃える
    controller_hz: float = 5.0                # 制御ループ目標（参考値）
```

---

## 9. ドキュメント更新義務

| タイミング | 更新対象 |
|---|---|
| Phase 1 完了 | `MISSION_ARCHITECTURE.md` §6 追従制御、本ファイルの Phase 1 DoD チェック |
| Phase 2 完了 | `MISSION_ARCHITECTURE.md` 全体ブロック図、本ファイル Phase 2 DoD |
| Phase 3 着手前 | 本ファイル §4 を詳細化した `PHASE3_DESIGN.md` を新規作成（その時点で判断） |

---

## 10. 用語対照（Nav2 ↔ SimWorld）

| Nav2 | SimWorld（Phase 2 目標） |
|---|---|
| `bt_navigator` | `mission_bt.py` |
| `planner_server` | `planner_server.py` |
| `controller_server` | `controller_server.py` |
| `behavior_server` | `behavior_server.py` |
| `global_costmap` | `global_costmap.py` / `LayeredCostmap` |
| `local_costmap` | `local_costmap.py` / `RollingCostmap` |
| `costmap_2d` | `costmap_layers.py` |
| `nav2_regulated_pure_pursuit` | `controllers/rpp.py` |
| `nav2_smac_planner` | Phase 3 |
| `dwb` / `mppi` | Phase 3（`controller_server` プラグイン） |
| `slam_toolbox` | なし（L0 事前 + L2 オンライン） |
| `amcl` | UE `get_pos2d` / `get_yaw` |

---

## 11. 未決事項（実装前に決める）

| # | 質問 | 推奨（デフォルト） | 決定期限 |
|---|---|---|---|
| Q1 | Phase 1 で RPP を fast プロファイルにも同時適用するか | default のみ先に ON | Sprint 2 開始前 |
| Q2 | ローカルコストマップの解像度を L2 と同一にするか | 同一（50cm） | Sprint 3 |
| Q3 | `layered_nav.py` を Phase 2 後も公開 API として残すか | 残す（薄いファサード） | Sprint 4 |
| Q4 | BT ライブラリを外部導入するか | 導入しない（自前状態機械） | Sprint 4 |
| Q5 | Phase 1/2 の golden trial 試行数 N | N=10（`run_l0_l2_slam_trials`） | Sprint 1 |

---

## 12. まとめ

| Phase | 一言 |  Nav2 で featuring する層 |
|---|---|---|
| **1** | 走れるようにする | **Controller**（RPP + 閉ループ） |
| **2** | 構造を Nav2 型にする | **Costmap 二層 + Planner/Controller/Behavior 分離 + BT** |
| **3** | 賢い計画と地図 | **Smac / 動的ゴール / 高度ローカルプランナ** |

Phase 1・2 を完了すれば、「Nav2 のアーキテクチャを体現した UE シミュレーション」として **デモ・論文・チューニング** に耐える基盤ができる。Phase 3 はその基盤の上に載せる拡張として位置づける。
