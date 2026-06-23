# hri_agv

## English

### Purpose

Simulates **Human–AGV (SpotDog) interaction** in a **10 m × 10 m** square room using Unreal Engine and UnrealCV. The human walks randomly; the AGV adjusts speed by **proxemic zones** (Safety / Personal / Social / Far). Records trajectories and computes HRI metrics after the run.

### File Reference

| File | Role |
|------|------|
| `agv_hri_square_room.py` | Full simulation: wall spawn, Humanoid + SpotDog control threads, zone-based AGV policy, CSV logging, post-run plots (jupytext source) |
| `agv_hri_square_room.ipynb` | Primary notebook entry point (paired with `.py` via jupytext) |

### Running Simulations

**Notebook:** `agv_hri_square_room.ipynb`

1. Start Unreal Engine level in **Play** mode.
2. `conda activate simworld`
3. Open and run all cells in `agv_hri_square_room.ipynb`

Equivalent script (percent format):

```bash
python dev/hri_agv/agv_hri_square_room.py
```

### Configurable Parameters

Key constants at the top of `agv_hri_square_room.py`:

| Parameter | Typical value | Effect |
|-----------|---------------|--------|
| `ROOM_CM` | `1000` | Room size 10 m |
| `SAFETY_CM` / `PERSONAL_CM` / `SOCIAL_CM` | 50 / 120 / 300 | Proxemic zone radii |
| `SIM_DURATION_S` | (in module) | Total simulation time |
| `HUMAN_SPEED_CM_S` | (in module) | Human walking speed |
| `AGV_MAX_SPEED_CM_S` | (in module) | AGV speed cap |
| Wall segment counts / thickness | (in module) | Room geometry matching `hri_spotdog_follow` |

### Future Extensibility

- Parameterize room size and zone thresholds via CLI or YAML.
- Export metrics in a standard schema for cross-run comparison.
- Integrate with `grid_env_hri` floor blueprint instead of manual wall spawn for map parity.

---

## 日本語

### 目的

Unreal Engine + UnrealCV で **10 m 四方**の部屋における **人間–AGV（SpotDog）相互作用** をシミュレート。人間はランダム歩行、AGV は **Proxemics ゾーン**（Safety / Personal / Social / Far）に応じて速度調整。軌跡記録と事後メトリクス算出。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `agv_hri_square_room.py` | 壁スポーン、マルチスレッド制御、ゾーンポリシー、CSV、プロット（jupytext ソース） |
| `agv_hri_square_room.ipynb` | 主エントリ（`.py` と jupytext 連携） |

### シミュレーションの実行

**ノートブック:** `agv_hri_square_room.ipynb`

1. UE を **Play** 状態にする。
2. `conda activate simworld`
3. 全セル実行

スクリプト: `python dev/hri_agv/agv_hri_square_room.py`

### 変更可能なパラメータ

`agv_hri_square_room.py` 先頭の定数:

| パラメータ | 典型値 | 効果 |
|-----------|--------|------|
| `ROOM_CM` | `1000` | 部屋 10 m |
| `SAFETY_CM` / `PERSONAL_CM` / `SOCIAL_CM` | 50 / 120 / 300 | ゾーン半径 [cm] |
| `SIM_DURATION_S` | （モジュール内） | シミュレーション時間 |
| `HUMAN_SPEED_CM_S` | （モジュール内） | 人間の速度 |
| `AGV_MAX_SPEED_CM_S` | （モジュール内） | AGV 上限速度 |

### 今後の拡張性

- CLI/YAML で部屋サイズ・ゾーン閾値を外部化。
- 標準スキーマでのメトリクスエクスポート。
- `grid_env_hri` 床 BP との統合で `hri_spotdog_follow` と完全な幾何一致。
