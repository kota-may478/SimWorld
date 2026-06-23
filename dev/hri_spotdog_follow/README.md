# hri_spotdog_follow

## English

### Purpose

**SpotDog follows a walking Humanoid** in a 10 m × 10 m square room (same wall layout as `hri_agv`). Uses **camera-based human detection** for heading and **depth ranging** for ~1 m follow distance. On target loss, the robot **spins in place** to re-acquire. Optional OpenCV HOG or YOLO vision backends; real-time OpenCV monitor windows.

### File Reference

| File | Role |
|------|------|
| `spotdog_follow_human_square_room.py` | Full simulation: room spawn, human walk/turn-on-block, SpotDog follow/search, depth + vision pipeline, CSV log, monitor windows (jupytext source) |
| `README.md` | This file (quick reference; superseded in structure by bilingual sections below) |
| `HISTORY.md` | Changelog of vision/search tuning iterations |

**Note:** There is no `.ipynb` in this folder; run the `.py` script directly or convert via jupytext.

### Running Simulations

**Entry point:** `spotdog_follow_human_square_room.py` (no notebook)

1. Start Unreal Engine level in **Play** mode.
2. `conda activate simworld`
3. Run:

```bash
python dev/hri_spotdog_follow/spotdog_follow_human_square_room.py
```

Logs: `dev/hri_spotdog_follow/spotdog_follow_log.csv`

Close monitor windows with `q` or `ESC` in the OpenCV window after the run.

### Configurable Parameters

| Parameter | Effect |
|-----------|--------|
| `FOLLOW_DISTANCE_CM` | Target follow distance (~100 cm) |
| `FOLLOW_DISTANCE_TOL_CM` | Deadband around target distance |
| `ROBOT_HEADING_KP` | Heading correction gain |
| `ROBOT_MAX_TURN_DEG_PER_STEP` | Max turn per control step |
| `ROBOT_SPEED_MAX_FWD` / `ROBOT_SPEED_MAX_REV` | Forward/reverse speed limits |
| `SEARCH_SPIN_PERIOD_S` | Full rotation period when searching |
| `SEARCH_LOST_GRACE_S` | Delay before entering search mode |
| `SEARCH_ROTATE_SLICE_S` | Rotation slice duration |
| `ENABLE_REALTIME_MONITOR` | OpenCV live camera/range windows |
| `VISION_USE_YOLO` | Switch HOG → YOLO (`ultralytics`) |
| `VISION_ENABLE_CLAHE` | Contrast enhancement for far targets |
| `VISION_ENABLE_TILED_SEARCH` | Tiled multi-scale search |
| `VISION_FAR_UPSAMPLE` | Upsample pass for small/far person |
| `VISION_TEMPORAL_HOLD_S` | Hold last detection briefly |

### Future Extensibility

- Add `spotdog_follow_human_square_room.ipynb` paired via jupytext for parity with `hri_agv`.
- Integrate learned person detector exported from SimWorld ego-view datasets.
- Share room-spawn code with `hri_agv` as a single `square_room_env.py` module.

---

## 日本語

### 目的

**SpotDog が歩行する Humanoid を追従**（10 m 四方、`hri_agv` と同じ壁配置）。**カメラ検出**で方位、**深度**で約 1 m 追従。ロスト時は **その場旋回**で再捕捉。OpenCV HOG または YOLO、リアルタイム監視ウィンドウ。

### ファイル一覧

| ファイル | 役割 |
|----------|------|
| `spotdog_follow_human_square_room.py` | 部屋スポーン、人間/犬制御、ビジョン、CSV、監視 UI |
| `HISTORY.md` | ビジョン・探索チューニング履歴 |

**ノートブックはありません** — `.py` を直接実行。

### シミュレーションの実行

**エントリ:** `spotdog_follow_human_square_room.py`

1. UE を **Play**。
2. `conda activate simworld`
3. `python dev/hri_spotdog_follow/spotdog_follow_human_square_room.py`

ログ: `spotdog_follow_log.csv`。監視ウィンドウは `q` / `ESC` で閉じる。

### 変更可能なパラメータ

英語表と同じ（`FOLLOW_DISTANCE_CM`、`SEARCH_*`、`VISION_*` 等）。

### 今後の拡張性

- jupytext 付き `.ipynb` 追加。
- SimWorld 自我視点データセット由来の学習検出器。
- `hri_agv` と部屋スポーンコード共通化。
