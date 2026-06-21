# grid_env_depth_perception

Depth camera + `object_mask` object recognition test on `/Game/Maps/Level`.

## Scenario

- **Region**: local `(0, 0)` – `(30 m, 30 m)`, excluding `(0, 0)` – `(5 m, 5 m)` spawn exclusion.
- **Props**: 5 of 73 Construction VOL.1 BPs (fixed seed `42`), placed on NavMesh.
- **Robot**: SpotDog at local `(1 m, 1 m)`.
- **Recognition**: `object_mask` color → prop type; depth → distance; pixel bearing → angle (relative to forward, ±180°).
- **Navigation**: turn-then-go to each prop in **distance order** from spawn (each leg soft-resets to spawn first).
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
# default: --nav-mode navmesh (NavFindPath). Legacy: --nav-mode simple
```

Or combined:

```bash
python dev/grid_env_depth_perception/run_depth_recognition_test.py --spawn-first
```

Results land in `cache/runs/`.

## Offline tests

```bash
conda run -n simworld python -m unittest discover -s dev/grid_env_depth_perception -p 'test_*.py' -v
```

## Dependencies

Reuses:

- `dev/grid_env_hri/grid_env_hri_simulation.py` — UE I/O, SpotDog
- `dev/grid_env_level_nav/` — catalog, NavMesh, coordinates
- `dev/hri_spotdog_follow/` patterns — camera pose, depth npy

## Baseline (pre Phase 1–2, run `20260616_172013`)

| Metric | Value |
|--------|-------|
| Overall distance RMSE | 6.32 m |
| Overall bearing RMSE | 8.00° |
| Pairs | 313 |
| E2E duration | ~9.6 min (5 legs) |

## After Phase 1–2 (run `20260616_185613`, 5 legs; legs 3–5 nav timeout but sampled)

| Metric | Value |
|--------|-------|
| Overall distance RMSE | **1.74 m** (was 6.32 m) |
| Overall bearing RMSE | **5.26°** (was 8.00°) |
| Pairs | 250 |

## After Phase 4 NavMesh nav (run `20260616_224932`, 5 legs all reached)

| Metric | Value |
|--------|-------|
| Overall distance RMSE | **2.93 m** |
| Overall bearing RMSE | **5.53°** |
| Pairs | 53 |
| Legs reached | **5 / 5** |

Use `--nav-mode navmesh` (default) for NavFindPath following; `--nav-mode simple` for legacy turn-then-go.

Regression gate: `run_perception_smoke_test.py` (PIE). Full E2E: `run_depth_recognition_test.py`.

## Notes

- Each prop gets a unique `set_color` RGB; canonical ID from `vget /object/{name}/color` after spawn.
- Lit-RGB fallback is **off** by default (`--allow-lit-fallback` to enable deprecated path).
- RMSE is computed only on samples where the prop is in FOV **and** mask+depth detection succeeded.
- Stop PIE only for UE Editor asset work (BP generation, native plugin compile). Python spawn/tests require **PIE running**.
- **Avoid `--force-respawn`** unless necessary — batch destroy+spawn can crash Level PIE. Default reuse mode keeps existing actors.
- See `DETECTION.md` for the full perception algorithm and crash analysis.
