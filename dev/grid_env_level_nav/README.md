# grid_env_level_nav

## English

### Purpose

**Layered navigation** on `/Game/Maps/Level` for HRC material-transport research:

- **L0** — static NavMesh mask (offline `.npz`)
- **L1** — zone closures (human work areas)
- **L2** — dynamic obstacles from FusionCam depth, AI Sight, or geometry fusion
- **A\*** planning, SpotDog execution, carry/deliver workflows

Design note: Obsidian #320 `simWorld_LevelNavMeshNavigation_ForHRCMaterialTransport.md`

**No primary project notebook** — use scenario `run_test.py` scripts or root `run_*.py` wrappers.

### File Reference — package root

| File | Role |
|------|------|
| `paths.py` | Canonical paths for `cache/l0`, `cache/registries`, `cache/runs/*` |
| `bootstrap.py` | `sys.path` setup for scenarios |
| `pie_spawn_safety.py` | PIE-safe destroy→spawn (cooldown, per-actor settle) |
| `level_coords.py` | Level map local ↔ world UE coordinates |
| `work_region.py` | Work-region rectangle helpers |
| `l0_nav_mask.py` | Load/build L0 boolean nav mask arrays |
| `build_l0_nav_mask.py` | CLI: sample NavMesh into L0 `.npz` |
| `nav_query.py` | Python wrapper for UE NavFindPath / NavQueryService |
| `costmap_layers.py` | Merge L0+L1+L2 into lethal/planning grids |
| `perception_layer.py` | Generic L2 perception → costmap cells |
| `layered_nav_perception.py` | Higher-level L2 fusion orchestration |
| `level_nav_robot.py` | SpotDog motion, replan loop, step execution |
| `level_nav_adapter.py` | Adapter between costmap and robot commands |
| `spotdog_nav_follower.py` | Path-following controller |
| `zone_catalog.py` | Zone definition templates |
| `zone_registry.py` | Runtime zone registry I/O |
| `prop_catalog.py` | Construction VOL.1 prop blueprint catalog |
| `construction_site_placement.py` | **Canonical** construction-site prop layout registry |
| `construction_site_carry.py` | Carry attach/detach for construction scenario |
| `release_ue_connection.py` | Close UnrealCV client (port 9000) |
| `visualize_merged_costmap.py` | Plot merged L0/L1/L2 costmaps |
| `print_zone_picklist.py` | Print selectable zones for CLI |
| `create_nav_query_service_editor.py` | UE Editor: install NavQueryService BP |
| `run_compact_nav_test.py` | Wrapper → `scenarios/compact_nav/run_test.py` |
| `run_site_transport_20m_test.py` | Wrapper → `scenarios/site_transport_20m/run_test.py` |
| `run_construction_site_transport_test.py` | Construction-site E2E test entry |
| `spawn_compact_nav_scene_pie.py` | Wrapper → compact_nav spawn |
| `spawn_site_transport_20m_pie.py` | Wrapper → site_transport_20m spawn |
| `spawn_construction_site_pie.py` | Spawn construction-site props in PIE |
| `spawn_construction_vol1_props_pie.py` | Spawn full VOL.1 prop set for Editor/PIE |
| `regenerate_compact_nav_viz.py` | Re-render compact_nav plots from cache |
| `run_spotdog_layered_nav_smoke.py` | Short smoke test for layered nav |
| `build_zone_registry_smoke.py` | Build zone registry smoke |
| `test_level_coords.py` | Unit tests: coordinates |
| `test_l0_nav_mask.py` | Unit tests: L0 mask |
| `test_costmap_layers.py` | Unit tests: layer merge |
| `test_perception_layer.py` | Unit tests: perception → grid |
| `test_zone_catalog.py` | Unit tests: zone catalog |
| `test_zone_registry.py` | Unit tests: zone registry |
| `test_construction_site_placement.py` | Unit tests: placement registry |
| `_nav_find_path_smoke_test.py` | UE smoke: NavFindPath |
| `_nav_project_point_smoke_test.py` | UE smoke: project point to NavMesh |
| `_validate_l0_projection_spot.py` | Validate L0 projection at spot checks |

### File Reference — `scenarios/compact_nav/`

| File | Role |
|------|------|
| `region.py` | 30 m region constants (start, goal) |
| `placement.py` | 3-prop layout registry |
| `l0_crop.py` | Crop full L0 mask to 30 m sub-region |
| `l2_fusion.py` | FusionCam detections → L2 lethal cells |
| `layered_nav.py` | L0+L2 navigation with replanning |
| `viz.py` | Post-run costmap PNG, trajectory JSON, RMSE plots |
| `spawn_pie.py` | PIE spawn props + SpotDog |
| `run_test.py` | **E2E entry**: spawn + navigate + artifacts (single UE session) |
| `regenerate_viz.py` | Re-render from saved NPZ/JSON |
| `test_placement.py` | Unit tests (no UE) |

### File Reference — `scenarios/site_transport_20m/`

| File | Role |
|------|------|
| `region.py` | 20 m site transport region bounds |
| `placement.py` | Multi-prop layout registry |
| `zones.py` | L1 zone definitions for site |
| `l0_crop.py` | Crop L0 to 20 m region |
| `l2_fusion.py` | FusionCam L2 layer |
| `l2_sight.py` | AI Sight perception L2 |
| `l2_geom.py` | Geometry-based L2 fusion |
| `runtime_sight_sources.py` | Runtime Sight stimulus configuration |
| `layered_nav.py` | Full L0+L1+L2 nav with replanning |
| `carry.py` | Material pickup, carry attach, deliver to humanoid |
| `metrics.py` | Run metrics aggregation |
| `viz.py` | Trajectory/metrics plots → `cache/runs/site_transport_20m/` |
| `spawn_pie.py` | PIE scene spawn |
| `run_test.py` | **E2E entry**: spawn → nav → carry → deliver |
| `test_l2_sight.py` | Unit tests for sight L2 |
| `test_l2_geom.py` | Unit tests for geom L2 |
| `CARRY_ATTACH_UE_SETUP.md` | UE Blueprint setup for carry attach |
| `SIGHT_PERCEPTION_UE_SETUP.md` | UE setup for AI Sight on props |

### File Reference — `scenarios/construction_site/`

| File | Role |
|------|------|
| `placement.py` | Re-export of root `construction_site_placement.py` |
| `carry.py` | Scenario-local carry helpers (imports root placement) |

### File Reference — `scripts/` (UE Editor / diagnostics)

| File | Role |
|------|------|
| `create_level_prop_base_editor.py` | Create base Level prop Blueprint |
| `generate_construction_vol1_level_props_editor.py` | Generate VOL.1 prop BPs |
| `generate_missing_construction_vol1_props_editor.py` | Fill missing prop BPs |
| `repair_generated_level_prop_meshes_editor.py` | Repair prop meshes |
| `rebuild_generated_level_props_editor.py` | Rebuild all generated props |
| `diagnose_level_prop_blueprint_editor.py` | BP diagnostic |
| `level_prop_blueprint_utils.py` | Shared Editor BP utilities |
| `spawn_construction_vol1_props_editor.py` | Editor spawn props |
| `enable_custom_depth_on_level_props_editor.py` | Enable custom depth on props |
| `enable_ai_sight_stimuli_on_level_props_editor.py` | Enable AI Sight stimuli |
| `probe_carry_attach_vbp.py` | Test carry attach vbp in PIE |
| `probe_ue_sight.py` | Probe UE Sight API |
| `debug_ue_sight.py` | Debug Sight perception |
| `sweep_ue_sight_yaw.py` | Sweep robot yaw for Sight calibration |
| `run_ue_python_file.py` | Run arbitrary UE Python in Editor |
| `test_ue_python_console.py` | Test UE Python console connectivity |

### File Reference — `ue_native/`

| File | Role |
|------|------|
| `NavQueryService.h` | C++ NavQuery plugin header |
| `NavQueryService.cpp` | C++ NavQuery plugin implementation |
| `INSTALL_NATIVE.md` | Install native plugin into SimWorld UE source |

### File Reference — `cache/`

| Path | Role |
|------|------|
| `cache/l0/*.npz` | Built L0 NavMesh masks |
| `cache/l0/*.png` | L0 visualization previews |
| `cache/registries/*.json` | Placement, zone, prop catalogs |
| `cache/runs/compact_nav/` | Compact nav artifacts (`latest_*`) |
| `cache/runs/site_transport_20m/` | Site transport trajectories, metrics PNG/JSON |
| `cache/runs/construction_site/` | Construction-site run outputs |

### Running Simulations

| Scenario | Entry script | Map |
|----------|--------------|-----|
| Compact 30 m FusionCam test | `run_compact_nav_test.py` or `scenarios/compact_nav/run_test.py` | Level PIE |
| 20 m site transport + carry | `run_site_transport_20m_test.py` | Level PIE |
| Construction site transport | `run_construction_site_transport_test.py` | Level PIE |

```bash
# Plan only (no UE)
conda run -n simworld python dev/grid_env_level_nav/run_compact_nav_test.py --plan-only

# Full compact nav (spawn + nav + artifacts, single UE session)
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld \
  python dev/grid_env_level_nav/run_compact_nav_test.py

# Site transport 20 m
python dev/grid_env_level_nav/run_site_transport_20m_test.py
```

Build L0 mask first:

```bash
python dev/grid_env_level_nav/build_l0_nav_mask.py \
  --resolution-cm 30 \
  --output dev/grid_env_level_nav/cache/l0/l0_mask_30cm_strict.npz
```

Unit tests (no UE): `python -m unittest discover -s dev/grid_env_level_nav -p 'test_*.py' -v`

### Configurable Parameters

| Parameter | Location | Effect |
|-----------|----------|--------|
| `--plan-only` | `compact_nav/run_test.py` | Skip UE spawn/nav |
| `--skip-spawn` | `site_transport_20m/run_test.py` | Reuse existing actors |
| `--max-nav-steps` | site transport | Cap navigation iterations |
| `--force-rebuild` | spawn scripts | Regenerate placement registry |
| L0 resolution | `build_l0_nav_mask.py --resolution-cm` | Grid cell size |
| Replan thresholds | `layered_nav.py` per scenario | When to replan on L2 updates |
| `MPLBACKEND=Agg` | run_test / viz | Headless matplotlib on WSL |
| Registry paths | `paths.py` | Override via env only if code extended |

UE Editor crash mitigation: see `pie_spawn_safety.py`, `ue_client_guard` (in `grid_env_hri`), single-session spawn+nav in `run_test.py`.

### Future Extensibility

- Remove duplicate `scenarios/construction_site/placement.py` content — **done** (thin re-export shim).
- Add `scenarios/README.md` index (optional; covered in `dev/README.md`).
- Package as installable subpackage (`simworld_level_nav`) to replace `sys.path` bootstrap.
- Unified notebook for compact_nav visualization only (CLI remains canonical).
- Gitignore large `cache/runs/*` timestamps; keep `latest_*` symlinks or copies.

---

## 日本語

### 目的

`/Game/Maps/Level` 上の **層状ナビゲーション**（HRC 資材運搬研究向け）:

- **L0** — 静的 NavMesh マスク（オフライン `.npz`）
- **L1** — ゾーン閉鎖（人作業域）
- **L2** — FusionCam 深度・AI Sight・幾何融合による動的障害
- **A\*** 経路計画、SpotDog 実行、運搬・受け渡し

設計メモ: Obsidian #320

**プロジェクト全体の主ノートブックはなし** — シナリオの `run_test.py` またはルート `run_*.py` を使用。

### ファイル一覧

#### パッケージルート

| ファイル | 役割 |
|------|------|
| `paths.py` | `cache/l0`、`cache/registries`、`cache/runs/*` の標準パス |
| `bootstrap.py` | シナリオ用 `sys.path` 初期化 |
| `pie_spawn_safety.py` | PIE 向け destroy→spawn（クールダウン、安定待ち） |
| `level_coords.py` | Level ローカル ↔ UE ワールド座標 |
| `work_region.py` | 作業領域矩形ヘルパ |
| `l0_nav_mask.py` | L0 ナビマスク配列の読み込み/構築 |
| `build_l0_nav_mask.py` | CLI: NavMesh から L0 `.npz` を生成 |
| `nav_query.py` | UE NavFindPath / NavQueryService の Python ラッパ |
| `costmap_layers.py` | L0+L1+L2 を致命/計画グリッドにマージ |
| `perception_layer.py` | 汎用 L2 知覚 → コストマップセル |
| `layered_nav_perception.py` | L2 融合オーケストレーション |
| `level_nav_robot.py` | SpotDog 運動、再計画ループ、ステップ実行 |
| `level_nav_adapter.py` | コストマップとロボット命令のアダプタ |
| `spotdog_nav_follower.py` | 経路追従コントローラ |
| `zone_catalog.py` | ゾーン定義テンプレート |
| `zone_registry.py` | ランタイムゾーンレジストリ I/O |
| `prop_catalog.py` | Construction VOL.1 プロップ BP カタログ |
| `construction_site_placement.py` | **正本** 建設現場プロップ配置レジストリ |
| `construction_site_carry.py` | 建設シナリオの運搬アタッチ/デタッチ |
| `release_ue_connection.py` | UnrealCV クライアント切断（9000） |
| `visualize_merged_costmap.py` | マージ L0/L1/L2 コストマッププロット |
| `print_zone_picklist.py` | CLI 用ゾーン一覧表示 |
| `create_nav_query_service_editor.py` | UE Editor: NavQueryService BP 導入 |
| `run_compact_nav_test.py` | ラッパ → `scenarios/compact_nav/run_test.py` |
| `run_site_transport_20m_test.py` | ラッパ → `scenarios/site_transport_20m/run_test.py` |
| `run_construction_site_transport_test.py` | 建設現場 E2E エントリ |
| `spawn_compact_nav_scene_pie.py` | compact_nav スポーンラッパ |
| `spawn_site_transport_20m_pie.py` | site_transport_20m スポーンラッパ |
| `spawn_construction_site_pie.py` | 建設現場プロップ PIE スポーン |
| `spawn_construction_vol1_props_pie.py` | VOL.1 プロップ一式スポーン |
| `regenerate_compact_nav_viz.py` | キャッシュから compact_nav 図を再生成 |
| `run_spotdog_layered_nav_smoke.py` | 層ナビ短時間スモーク |
| `build_zone_registry_smoke.py` | ゾーンレジストリ構築スモーク |
| `test_level_coords.py` 他 `test_*.py` | 座標・L0・コストマップ等のユニットテスト |
| `_nav_*_smoke_test.py` 等 | UE スモーク・検証スクリプト |

#### `scenarios/compact_nav/`

| ファイル | 役割 |
|------|------|
| `region.py` | 30 m 領域定数（開始、ゴール） |
| `placement.py` | 3 プロップ配置レジストリ |
| `l0_crop.py` | 全 L0 を 30 m にクロップ |
| `l2_fusion.py` | FusionCam → L2 致命セル |
| `layered_nav.py` | L0+L2 再計画ナビ |
| `viz.py` | コストマップ PNG、軌跡 JSON、RMSE 図 |
| `spawn_pie.py` | プロップ + SpotDog PIE スポーン |
| `run_test.py` | **E2E**: 単一 UE セッションで spawn+nav+成果物 |
| `regenerate_viz.py` | 保存 NPZ/JSON から再描画 |
| `test_placement.py` | ユニットテスト（UE 不要） |

#### `scenarios/site_transport_20m/`

| ファイル | 役割 |
|------|------|
| `region.py` | 20 m 現場運搬領域 |
| `placement.py` | 複数プロップ配置レジストリ |
| `zones.py` | L1 ゾーン定義 |
| `l0_crop.py` | L0 を 20 m にクロップ |
| `l2_fusion.py` | FusionCam L2 |
| `l2_sight.py` | AI Sight L2 |
| `l2_geom.py` | 幾何ベース L2 |
| `runtime_sight_sources.py` | Sight 刺激のランタイム設定 |
| `layered_nav.py` | L0+L1+L2 再計画ナビ |
| `carry.py` | ピックアップ、運搬アタッチ、Humanoid へ受け渡し |
| `metrics.py` | 実行メトリクス集約 |
| `viz.py` | 軌跡/メトリクス図 |
| `spawn_pie.py` | PIE シーンスポーン |
| `run_test.py` | **E2E**: spawn → nav → carry → deliver |
| `test_l2_sight.py` / `test_l2_geom.py` | L2 ユニットテスト |
| `CARRY_ATTACH_UE_SETUP.md` | 運搬アタッチ UE セットアップ |
| `SIGHT_PERCEPTION_UE_SETUP.md` | プロップ AI Sight セットアップ |

#### `scenarios/construction_site/`

| ファイル | 役割 |
|------|------|
| `placement.py` | ルート `construction_site_placement` の再エクスポート |
| `carry.py` | シナリオローカル運搬ヘルパ |

#### `scripts/`（UE Editor / 診断）

Editor 用 BP 生成、VOL.1 プロップ、深度/Sight 有効化、carry/Sight プローブ等（英語表の 16 ファイルと同一）。

#### `ue_native/`

`NavQueryService.h` / `.cpp`、`INSTALL_NATIVE.md` — C++ NavQuery プラグイン。

#### `cache/`

`cache/l0/`（L0 npz/png）、`cache/registries/`（JSON）、`cache/runs/*`（実行成果物）。

### シミュレーションの実行

| シナリオ | エントリ | マップ |
|---------|---------|--------|
| コンパクト 30 m | `run_compact_nav_test.py` | Level PIE |
| 20 m 現場運搬 | `run_site_transport_20m_test.py` | Level PIE |
| 建設現場 | `run_construction_site_transport_test.py` | Level PIE |

```bash
conda run -n simworld python dev/grid_env_level_nav/run_compact_nav_test.py --plan-only
PYTHONUNBUFFERED=1 conda run --no-capture-output -n simworld \
  python dev/grid_env_level_nav/run_compact_nav_test.py
python dev/grid_env_level_nav/run_site_transport_20m_test.py
```

L0 構築: `build_l0_nav_mask.py --resolution-cm 30`

### 変更可能なパラメータ

| パラメータ | 場所 | 効果 |
|-----------|------|------|
| `--plan-only` | compact_nav | UE なし計画のみ |
| `--skip-spawn` | site_transport_20m | 既存アクター再利用 |
| `--max-nav-steps` | site transport | ナビ上限 |
| `--force-rebuild` | spawn | レジストリ再生成 |
| L0 解像度 | `build_l0_nav_mask.py` | セルサイズ |
| 再計画閾値 | 各 `layered_nav.py` | L2 更新時の再計画 |
| `MPLBACKEND=Agg` | run_test / viz | WSL ヘッドレス作図 |

クラッシュ対策: `pie_spawn_safety.py`、`grid_env_hri` の `ue_client_guard`、単一セッション spawn+nav。

### 今後の拡張性

- 未使用だった `scenarios/construction_site/placement.py` の重複コード削除 — **完了**（薄い再エクスポートに置換）。
- インストール可能サブパッケージ化で `bootstrap.py` 不要化。
- `cache/runs` のタイムスタンプ成果物の gitignore 整理。
