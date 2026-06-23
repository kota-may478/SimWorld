# grid_env_depth_perception

## English

### Purpose

Tests **egocentric object recognition** on `/Game/Maps/Level` using SpotDog's **depth camera** and UnrealCV **`object_mask`** rendering. Five Construction VOL.1 props are placed in a 30 m work region; the robot navigates to each prop in distance order, logging distance/bearing estimates vs ground truth and computing RMSE.

### File Reference

| File | Role |
|------|------|
| `depth_object_perception.py` | Core perception: mask color → prop ID, depth → distance, pixel bearing → angle |
| `object_mask_color.py` | Canonical color IDs from `vget /object/{name}/color` |
| `prop_placement.py` | Placement registry I/O, NavMesh-valid prop positions |
| `prop_signature.py` | Prop type signatures for mask matching |
| `ground_truth.py` | GT distance/bearing from registry poses |
| `robot_sensor.py` | Camera pose, depth fetch wrappers |
| `pie_safety.py` | PIE destroy/spawn cooldown helpers (imports `grid_env_level_nav.pie_spawn_safety` patterns) |
| `spawn_test_scene_pie.py` | PIE: spawn 5 props + SpotDog from registry |
| `run_depth_recognition_test.py` | Full E2E: navigate to each prop, log JSON + RMSE plots |
| `run_perception_smoke_test.py` | Short PIE regression gate |
| `nav_mesh_nav.py` | NavFindPath-following navigation (default `--nav-mode navmesh`) |
| `simple_nav.py` | Legacy turn-then-go navigation |
| `plot_results.py` | Plot distance/bearing time series from run JSON |
| `plot_from_run.py` | Regenerate plots from saved run directory |
| `mask_calibration.py` | Mask color tolerance calibration utilities |
| `compare_camera_masks.py` | Compare lit vs object_mask outputs |
| `debug_mask_probe.py` | Interactive mask debugging |
| `test_object_mask_color.py` | Unit tests for color parsing |
| `test_ground_truth.py` | Unit tests for GT geometry |
| `DETECTION.md` | Algorithm description and crash analysis |
| `VISION_APPROACHES.md` | Notes on vision backend alternatives |
| `cache/prop_placement_registry.json` | Fixed-seed layout (seed 42, 5 props) |
| `out/*.json` | Per-run trajectories and RMSE summaries |

### Running Simulations

**No primary notebook.** Use CLI scripts:

```bash
# 1. UE Editor: /Game/Maps/Level → Play (PIE)
# 2. Spawn scene
python dev/grid_env_depth_perception/spawn_test_scene_pie.py

# 3. Recognition + navigation test
python dev/grid_env_depth_perception/run_depth_recognition_test.py
# or combined:
python dev/grid_env_depth_perception/run_depth_recognition_test.py --spawn-first
```

Smoke test: `python dev/grid_env_depth_perception/run_perception_smoke_test.py`

### Configurable Parameters

CLI flags on `run_depth_recognition_test.py`:

| Flag / parameter | Default | Effect |
|------------------|---------|--------|
| `--nav-mode` | `navmesh` | `navmesh` or `simple` |
| `--spawn-first` | off | Run spawn before test |
| `--force-rebuild` | off | New prop picks/positions (avoid unless needed) |
| `--allow-lit-fallback` | off | Deprecated lit-RGB fallback |
| `--output-dir` | `out/` | Artifact directory |
| `PerceptionConfig.fov_deg` | `90` | Camera FOV |
| `PerceptionConfig.min_mask_pixels` | `48` | Minimum mask area |
| `PerceptionConfig.color_tolerance` | `6` | RGB match tolerance |

Registry path: `cache/prop_placement_registry.json` (reuse for identical layout).

### Future Extensibility

- Share `depth_object_perception.py` imports only via installed package path instead of duplicate `sys.path` to `grid_env_level_nav`.
- Fuse with `grid_env_level_nav` L2 FusionCam layer for unified perception stack.
- Notebook wrapper for interactive single-prop probing.

---

## 日本語

### 目的

`/Game/Maps/Level` で SpotDog の **深度カメラ** と **`object_mask`** による **自己中心的对象認識** を検証。30 m 作業域に Construction VOL.1 プロップ 5 個を配置し、距離順にナビして推定距離・方位と真値の RMSE を記録。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `depth_object_perception.py` | 認識コア（マスク・深度・方位） |
| `object_mask_color.py` | オブジェクト色 ID |
| `prop_placement.py` | 配置レジストリ |
| `prop_signature.py` | プロップ型シグネチャ |
| `ground_truth.py` | 真値幾何 |
| `robot_sensor.py` | カメラ・深度取得 |
| `pie_safety.py` | PIE 安全スポーン |
| `spawn_test_scene_pie.py` | シーンスポーン |
| `run_depth_recognition_test.py` | E2E テスト |
| `run_perception_smoke_test.py` | 短時間回帰 |
| `nav_mesh_nav.py` / `simple_nav.py` | NavMesh / レガシーナビ |
| `plot_*.py` | 可視化 |
| `mask_calibration.py` 等 | キャリブ・デバッグ |
| `test_*.py` | ユニットテスト |
| `DETECTION.md` / `VISION_APPROACHES.md` | 設計ドキュメント |
| `cache/*` | Registries (tracked) |
| `out/*` | Run artifacts: JSON trajectories, distance/bearing PNGs (gitignored) |

### シミュレーションの実行

**主ノートブックなし。** CLI:

```bash
python dev/grid_env_depth_perception/spawn_test_scene_pie.py
python dev/grid_env_depth_perception/run_depth_recognition_test.py
```

スモーク: `run_perception_smoke_test.py`

### 変更可能なパラメータ

| フラグ / パラメータ | 既定 | 効果 |
|---------------------|------|------|
| `--nav-mode` | `navmesh` | NavMesh / simple |
| `--spawn-first` | off | 事前スポーン |
| `--force-rebuild` | off | レイアウト再生成（非推奨頻用） |
| `--allow-lit-fallback` | off | 非推奨 lit フォールバック |
| `--output-dir` | `out/` | 成果物ディレクトリ |
| `PerceptionConfig.*` | （モジュール内） | FOV・マスク閾値等 |

### 今後の拡張性

- `grid_env_level_nav` L2 FusionCam との統合スタック。
- 単一プロップ用対話ノートブック。
- `sys.path` 依存のパッケージ化。
