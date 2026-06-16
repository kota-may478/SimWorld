# grid_env_depth_perception

Depth camera + `object_mask` object recognition test on `/Game/Maps/Level`.

## Scenario

- **Region**: local `(0, 0)` – `(30 m, 30 m)`, excluding `(0, 0)` – `(5 m, 5 m)` spawn exclusion.
- **Props**: 5 of 73 Construction VOL.1 BPs (fixed seed `42`), placed on NavMesh.
- **Robot**: SpotDog at local `(1 m, 1 m)`.
- **Recognition**: `object_mask` color → prop type; depth → distance; pixel bearing → angle (relative to forward, ±180°).
- **Navigation**: turn-then-go to each prop in **distance order** from spawn.
- **Outputs**: time-series JSON, distance/bearing plots (GT dashed, estimate solid), RMSE per prop and overall.

## Registry

Placement is stored in:

`cache/prop_placement_registry.json`

Re-run spawn with the same registry for identical layout. Use `--force-rebuild` only when you intentionally want new prop picks/positions.

## PIE workflow

1. UE Editor: open **`/Game/Maps/Level`** → **Play (PIE)**.
2. WSL: `conda activate simworld`
3. Spawn scene:

```bash
python dev/grid_env_depth_perception/spawn_test_scene_pie.py
```

4. Run recognition + navigation test:

```bash
python dev/grid_env_depth_perception/run_depth_recognition_test.py
```

Or combined:

```bash
python dev/grid_env_depth_perception/run_depth_recognition_test.py --spawn-first
```

Results land in `cache/runs/`.

## Offline tests

```bash
python -m unittest discover -s dev/grid_env_depth_perception -p 'test_*.py' -v
```

## Dependencies

Reuses:

- `dev/grid_env_hri/grid_env_hri_simulation.py` — UE I/O, SpotDog
- `dev/grid_env_level_nav/` — catalog, NavMesh, coordinates
- `dev/hri_spotdog_follow/` patterns — camera pose, depth npy

## Notes

- Each prop gets a unique `set_color` RGB for `object_mask` segmentation (not lit RGB).
- After spawn, **mask calibration** records observed BGR in the registry.
- RMSE is computed only on samples where the prop is in FOV **and** mask+depth detection succeeded.
- Stop PIE only for UE Editor asset work (BP generation, native plugin compile). Python spawn/tests require **PIE running**.
- **Avoid `--force-respawn`** unless necessary — batch destroy+spawn can crash Level PIE. Default reuse mode keeps existing actors.
- See `DETECTION.md` for the full perception algorithm and crash analysis.
