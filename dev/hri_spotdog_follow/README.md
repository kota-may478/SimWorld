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

## Environment parity with `hri_agv`

The room and wall setup is intentionally kept the same as `hri_agv`:
- 10m x 10m square room (`ROOM_CM = 1000`)
- Segmented wall spawning with the same parameters

## Run notes

1. Start Unreal Engine level in Play mode.
2. Run the script.
3. After simulation, logs are saved to:
   - `dev/hri_spotdog_follow/spotdog_follow_log.csv`

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
