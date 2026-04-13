# SpotDog Follow Human (Square Room)

This project is a new `dev` scenario based on `dev/hri_agv`.

Main script:
- `dev/hri_spotdog_follow/spotdog_follow_human_square_room.py`

## Behavior

Human:
- Walks straight continuously.
- If blocked (collision-like event), turns and keeps walking.

SpotDog:
- Tries to keep following at approximately 1 meter.
- Uses camera-based human detection for heading alignment.
- Uses depth sensing (LiDAR-like ranging) for distance control.
- If the human is not detected for a short period, spins in place to search.
   - Default: continuous spin, about 1 full turn in 2 seconds.

## Environment parity with `hri_agv`

The room and wall setup is intentionally kept the same as `hri_agv`:
- 10m x 10m square room (`ROOM_CM = 1000`)
- Segmented wall spawning with the same parameters

## Run notes

1. Start Unreal Engine level in Play mode.
2. Run the script.
3. After simulation, logs are saved to:
   - `dev/hri_spotdog_follow/spotdog_follow_log.csv`

During simulation, you can also watch:
- SpotDog camera stream (with tracking/search overlay)
- Real-time range sensor timeline window

Window lifecycle:
- Both monitor windows are opened right before simulation starts.
- They stay open after simulation ends.
- Close them manually by pressing `q` or `ESC` in an OpenCV window (or close the windows directly).
- On each run, monitor/runtime states are reset before simulation starts (safe for repeated runs in interactive sessions).

Related switches and parameters in the script:
- `ENABLE_REALTIME_MONITOR`
- `SEARCH_SPIN_PERIOD_S`
- `SEARCH_LOST_GRACE_S`
- `SEARCH_ROTATE_SLICE_S`
- `VISION_ENABLE_CLAHE`
- `VISION_ENABLE_TILED_SEARCH`
- `VISION_FAR_UPSAMPLE`
- `VISION_TEMPORAL_HOLD_S`

Far-distance robustness strategy (default ON):
- CLAHE-based contrast enhancement before detection
- Multi-scale search with upsampled pass for small/far targets
- Tiled search to improve off-center and tiny-person recall
- Short temporal hold to reduce one-frame detection drops

## Vision backend

Default backend is OpenCV HOG (portable, no extra package needed).

Optional YOLO:
- Install `ultralytics`.
- Set `VISION_USE_YOLO = True` in the script.
- Optionally set `VISION_YOLO_MODEL_PATH` to a local model file.

## Useful parameters to tune

- `FOLLOW_DISTANCE_CM`
- `FOLLOW_DISTANCE_TOL_CM`
- `ROBOT_HEADING_KP`
- `ROBOT_MAX_TURN_DEG_PER_STEP`
- `ROBOT_SPEED_MAX_FWD`
- `ROBOT_SPEED_MAX_REV`
- `VISION_HOG_SCALE`
