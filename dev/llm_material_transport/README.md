# llm_material_transport

## English

### Purpose

**LLM-directed material transport** on `/Game/Maps/Level`: a Humanoid manager uses an LLM to emit JSON task instructions; a SpotDog **RobotExecutor** navigates to material, simulates pickup, returns to home, and drops off. Integrates costmap-based path planning from `path_planning_costmap.py` and UE agents via SimWorld Communicator.

### File Reference

| File | Role |
|------|------|
| `material_transport_llm.py` | Main simulation: LLM prompts, `RobotExecutor` state machine, spawn, navigation phases, trajectory logging, plots (jupytext source) |
| `material_transport_llm.ipynb` | Primary notebook entry (paired with `.py`) |
| `path_planning_costmap.py` | A* / costmap obstacle integration for robot navigation |
| `costmap_obstacle_scan.py` | Scan Level map obstacles into costmap grids |
| `_run_integration_test.py` | Integration test runner (headless/log capture) |
| `_verify_handoff_geometry.py` | Verify pickup/drop geometry vs UE actors |
| `_scan_costmap_obstacles.py` | CLI wrapper for obstacle scan |
| `_debug_topdown_depth.py` | Debug top-down depth alignment |
| `_debug_collision_probe.py` | Debug collision probe for nav |
| `_test_costmap_viz_style.py` | Test costmap matplotlib styling |
| `_test_jupyter_server_connect.py` | Verify Jupyter server connectivity for Cursor |
| `_test_kernel_connect.py` | Verify kernel can reach UE |
| `_cursor_kernel_test.ipynb` | Minimal kernel connectivity notebook |
| `verify_cursor_notebook_setup.sh` | Shell: validate Cursor + Jupyter + simworld kernel |
| `start_jupyter_for_cursor.sh` | Shell: start Jupyter for remote Cursor editing |
| `_integration_*.log` | Historical integration run logs (safe to delete) |
| `_integration_verify.log` | Latest verification log |

### Running Simulations

**Notebook:** `material_transport_llm.ipynb`

1. Open `/Game/Maps/Level` in UE and start **Play**.
2. `conda activate simworld` (LLM API keys per SimWorld `BaseLLM` config).
3. Run all cells in `material_transport_llm.ipynb`.

Script equivalent:

```bash
python dev/llm_material_transport/material_transport_llm.py
```

For Cursor remote kernel setup, see `verify_cursor_notebook_setup.sh`.

### Configurable Parameters

In `material_transport_llm.py` (representative):

| Parameter | Effect |
|-----------|--------|
| LLM model name | Passed to `BaseLLM` (e.g. `gpt-4o`) |
| Material / home / robot spawn XY | Scenario geometry on Level map |
| `RobotExecutor` phase timeouts | Navigating, picking, carrying, dropping |
| Costmap resolution / inflation | In `path_planning_costmap.py` |
| `MPLBACKEND` | Set `Agg` for headless plotting on WSL |

Obstacle scan CLI flags in `costmap_obstacle_scan.py` and `_scan_costmap_obstacles.py`.

### Future Extensibility

- Delegate navigation to `grid_env_level_nav` layered L0+L2 stack instead of standalone costmap.
- Structured LLM output validation with Pydantic + retry policy.
- Move `_integration_*.log` under `cache/runs/` and gitignore.
- End-to-end test using mock LLM for CI without API calls.

---

## 日本語

### 目的

`/Game/Maps/Level` 上の **LLM 指示による資材搬送**。Humanoid 管理者が LLM で JSON タスクを生成し、SpotDog の **RobotExecutor** が移動・ピックアップ模擬・帰還・降ろしを実行。`path_planning_costmap.py` のコストマップ経路計画と SimWorld Communicator を使用。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `material_transport_llm.py` / `.ipynb` | メインシミュレーション（jupytext 連携） |
| `path_planning_costmap.py` | コストマップ A* ナビ |
| `costmap_obstacle_scan.py` | 障害物スキャン |
| `_run_integration_test.py` | 統合テスト |
| `_verify_handoff_geometry.py` 等 `_debug_*` | デバッグ・検証 |
| `_test_*` / `_cursor_kernel_test.ipynb` | Jupyter/カーネル接続確認 |
| `verify_cursor_notebook_setup.sh` 等 | Cursor 用シェル |
| `_integration_*.log` | 過去の統合ログ（削除可） |

### シミュレーションの実行

**ノートブック:** `material_transport_llm.ipynb`

1. Level を UE **Play**。
2. `conda activate simworld`（LLM API 設定）。
3. 全セル実行。

スクリプト: `python dev/llm_material_transport/material_transport_llm.py`

### 変更可能なパラメータ

| パラメータ | 効果 |
|-----------|------|
| LLM モデル名 | `BaseLLM` に渡す |
| スポーン座標 | 資材・ホーム・ロボット位置 |
| `RobotExecutor` タイムアウト | 各フェーズ |
| コストマップ解像度・膨張 | `path_planning_costmap.py` |
| `MPLBACKEND` | WSL ヘッドレスプロット |

### 今後の拡張性

- `grid_env_level_nav` の L0+L2 層ナビへの移行。
- Pydantic による LLM 出力検証とリトライ。
- ログを `cache/runs/` へ集約し gitignore。
- モック LLM による CI 統合テスト。
