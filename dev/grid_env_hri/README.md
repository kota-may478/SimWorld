# grid_env_hri

## English

### Purpose

Shared foundation for **30 m × 30 m grid environments** on `/Game/Maps/empty.umap`: spawns a floor blueprint, up to 10,000 semi-transparent cubes (0.3 m pitch), a Humanoid, and a SpotDog robot. Provides UE I/O helpers, coordinate conventions, and the **UnrealCV single-client guard** used across `dev/` Level and grid projects.

This project has **no primary simulation notebook**; it is imported by `grid_env_10k`, `grid_env_depth_perception`, `grid_env_level_nav`, and others.

### File Reference

| File | Role |
|------|------|
| `grid_env_hri_simulation.py` | Core module: floor/cube spawn, agent spawn, movement, camera/depth helpers, grid constants (`GRID_N`, `FLOOR_SIZE_M`, blueprint paths) |
| `ue_client_guard.py` | Exclusive lock on UnrealCV port 9000, stale-socket cleanup, graceful shutdown (`ensure_exclusive_ue_session`, `prepare_ue_connection`) |
| `grid_env_hri_simulation.ipynb` | Interactive notebook mirroring the main simulation script (exploration / debugging) |
| `run_notebook_flow.py` | Headless runner that replays the notebook's main flow for CI-style checks |
| `run_pipeline_until_settle.py` | Runs spawn pipeline until physics/settling completes |
| `run_single_cube_toggle_test.py` | Tests toggling a single cube's blocking state via UE |
| `test_grid_env_ue_helpers.py` | Unit tests for UE helper utilities (mocked where possible) |
| `test_ue_client_guard.py` | Unit tests for port lock and connection guard logic |
| `test_demo_passage_judge.py` | Tests passage/corridor judgment helpers used in demos |
| `_clear_nb_outputs.py` | Utility to strip Jupyter outputs from the notebook file |

### Running Simulations

**No dedicated end-to-end simulation entry point.** Use this project indirectly:

1. Start UE with `/Game/Maps/empty.umap` in Play mode.
2. Run a dependent project, e.g.:
   - `python dev/grid_env_10k/run_phase1_spawn.py`
   - `python dev/grid_env_hri/grid_env_hri_simulation.py` (direct spawn test; set `GRID_N=3` for small grid)

Optional notebook: `grid_env_hri_simulation.ipynb` (requires UE Play + `conda activate simworld`).

### Configurable Parameters

Environment variables and constants in `grid_env_hri_simulation.py`:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `GRID_N` | `100` | Grid dimension → `GRID_N²` cubes |
| `FLOOR_SIZE_M` | `30.0` | Floor extent in meters |
| `CUBE_SIZE_M` | `0.3` | Cube pitch |
| `FLOOR_TOP_Z_CM` | `100.0` | Floor top height in UE cm |
| `AGENT_SPAWN_ABOVE_FLOOR_CM` | (in module) | Humanoid / robot spawn clearance |
| `ROBOT_BP` / `HUMAN_BP` | TrafficSystem / Robot_Dog paths | Blueprint class paths |
| `UE_PORT_IDLE_WAIT_S` | `12.0` | Wait for port 9000 to free (via `ue_client_guard`) |
| `SIMWORLD_UE_LOCK` | `/tmp/simworld_ue9000.lock` | Filesystem lock path |

### Future Extensibility

- Extract `ue_client_guard.py` into a small shared `dev/_common/` package to avoid duplicate `sys.path` hacks.
- Formalize the grid coordinate API (1-based `gx,gy` vs 0-based row/col) as a standalone module.
- Add a minimal smoke-test script that only verifies UE connectivity without spawning 10k actors.

---

## 日本語

### 目的

`/Game/Maps/empty.umap` 上の **30 m × 30 m グリッド環境** の共通基盤。床 BP、最大 1 万個の半透明キューブ（0.3 m ピッチ）、Humanoid、SpotDog をスポーンし、UE I/O・座標規約・**UnrealCV 単一クライアントガード** を提供します。

**単体の本番シミュレーションノートブックはありません** — `grid_env_10k`、`grid_env_depth_perception`、`grid_env_level_nav` 等からインポートされます。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `grid_env_hri_simulation.py` | コア: 床/キューブ/エージェントスポーン、移動、カメラ/深度、定数 |
| `ue_client_guard.py` | UnrealCV 9000 番ポートの排他ロック・ソケット掃除・安全切断 |
| `grid_env_hri_simulation.ipynb` | メインスクリプトの対話用ノートブック |
| `run_notebook_flow.py` | ノートブック主要フローのヘッドレス実行 |
| `run_pipeline_until_settle.py` | スポーン後の物理安定まで待機するパイプライン |
| `run_single_cube_toggle_test.py` | 単一キューブのブロッキング切替テスト |
| `test_grid_env_ue_helpers.py` | UE ヘルパのユニットテスト |
| `test_ue_client_guard.py` | 接続ガードのユニットテスト |
| `test_demo_passage_judge.py` | デモ用通路判定のテスト |
| `_clear_nb_outputs.py` | ノートブック出力削除ユーティリティ |

### シミュレーションの実行

**専用 E2E エントリはありません。** 依存プロジェクト経由で利用:

1. UE で `/Game/Maps/empty.umap` を Play。
2. 例: `python dev/grid_env_10k/run_phase1_spawn.py` または `GRID_N=3 python dev/grid_env_hri/grid_env_hri_simulation.py`

ノートブック: `grid_env_hri_simulation.ipynb`（UE Play + `conda activate simworld`）。

### 変更可能なパラメータ

`grid_env_hri_simulation.py` の環境変数・定数（英語表と同じ項目）:

| パラメータ | 既定値 | 効果 |
|-----------|--------|------|
| `GRID_N` | `100` | 格子一辺 → `GRID_N²` キューブ |
| `FLOOR_SIZE_M` | `30.0` | 床サイズ [m] |
| `CUBE_SIZE_M` | `0.3` | キューブピッチ [m] |
| `FLOOR_TOP_Z_CM` | `100.0` | 床上面高さ [cm] |
| `ROBOT_BP` / `HUMAN_BP` | （モジュール内パス） | ブループリント |
| `UE_PORT_IDLE_WAIT_S` | `12.0` | ポート解放待ち [s] |
| `SIMWORLD_UE_LOCK` | `/tmp/simworld_ue9000.lock` | ロックファイル |

### 今後の拡張性

- `ue_client_guard.py` を `dev/_common/` 等に切り出し。
- 格子座標 API（1 始まり `gx,gy` と 0 始まり row/col）の独立モジュール化。
- 1 万アクターなしの UE 接続のみのスモークテスト追加。
