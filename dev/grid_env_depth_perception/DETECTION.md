# Detection algorithm & PIE safety

See also `README.md` for run instructions.

## What sensors are used?

| Stream | UnrealCV mode | Purpose |
|--------|---------------|---------|
| **object_mask** | `vget /camera/{id}/object_mask png` | **Identity** — which registered prop is visible |
| **depth** | `vget /camera/{id}/depth npy` | **Distance** — range to masked region |
| lit (RGB) | not used | — |

This is **not** lit-texture color matching. We do **not** classify objects by their mesh paint color in the RGB camera.

Instead we use UnrealCV's **segmentation buffer** (`object_mask`). Each actor can be assigned a unique flat color via `vset /object/{name}/color R G B`. In the mask image, pixels belonging to that actor appear as that color (decoded as **BGR** after PNG load).

## Color registry (pre-acquired data)

> **Note:** One-pose mask calibration (`mask_calibration.py`) is deprecated for
> identity. See `VISION_APPROACHES.md`. Use `vget /object/{name}/color` per
> [UnrealCV GT tutorial](https://docs.unrealcv.org/en/latest/tutorials/generate_images_tutorial.html).

`cache/prop_placement_registry.json` stores per prop:

| Field | Meaning |
|-------|---------|
| `mask_color_set_rgb` | Color written at spawn (`set_color`) — intended segmentation ID |
| `mask_color_observed_bgr` | Dominant BGR measured in `object_mask` during calibration |
| `mask_color_rgb` | After calibration: observed color as RGB (human-readable); used as fallback |

**Detection uses `mask_color_observed_bgr` when present**, else reverses `mask_color_rgb` to BGR.

### Calibration (once after spawn)

1. Soft-teleport SpotDog ~4.5 m from each prop, facing it.
2. Sync camera pose to robot head.
3. Capture `object_mask`.
4. Take dominant non-background BGR in the image center ROI.
5. Save to registry as `mask_color_observed_bgr`.

This compensates for UE/UnrealCV not always rendering `set_color` exactly as requested.

## Per-frame detection pipeline

For each prop in the registry:

1. **Segmentation match** — threshold mask pixels where `|pixel_BGR - detection_bgr| ≤ tolerance` (default ±24).
2. **Pixel count** — skip if `< 48` pixels (not reliably in FOV).
3. **BBox** — axis-aligned box of matched pixels.
4. **Bearing** — horizontal angle from bbox center: `(cx - W/2) / (W/2) * (FOV/2)` (relative to camera forward).
5. **Depth** — 35th percentile of valid depth in a small ROI around bbox center (60% height); convert npy to metres (cm if value ≥ 20).
6. **Horizontal distance** — approximate ground distance from slant range + bearing + fixed camera pitch (−5°).

Output per detected prop: `prop_type_id`, `distance_m`, `bearing_deg`, `confidence` (from mask pixel count).

## Ground truth (for RMSE)

From robot world pose `(x, y, yaw)` and prop world position:

- `distance_m` = 2D Euclidean distance in the XY plane
- `bearing_deg` = `atan2(dy, dx) - yaw`, normalized to ±180°
- RMSE pairs only when prop is in FOV **and** mask+depth produced an estimate

---

## UE Editor crash — root cause analysis

Observed sequence:

1. **~9 min navigation test** completed (5 legs, open-loop `dog_move` / `dog_rotate`).
2. **Re-spawn** attempted: destroy 5 props → `spawn_bp` on first prop → `Connection reset by peer` → Editor crash.

### Contributing factors

| Factor | Mechanism |
|--------|-----------|
| **Destroy → immediate spawn** | Level PIE is fragile after batch `vset .../destroy`; GC / render thread still busy. `spawn_bp` during teardown stresses UE (documented in `level_nav_robot` / `spawn_construction_vol1_props_pie`). |
| **Aggressive reconnect** | On connection loss, `reconnect_if_needed(force_new=True)` kept sending commands while UE was crashing, masking failure. |
| **Open-loop navigation** | `turn-then-go` without NavMesh / obstacle avoidance; SpotDog can hit geometry, stressing Character movement on Level. |
| **High UnrealCV rate** | Perception every 0.25 s × 2 heavy requests (depth npy + mask png) + camera teleport each sample → sustained load ~9 min. |
| **No leg timeout** | Up to 400 steps × 5 legs; long hammering even when stuck. |
| **Hard destroy SpotDog** | Not used, but pawn destroy + `clean_garbage` is known to crash Level PIE. |

### Implemented mitigations (`pie_safety.py`)

- **Fail-fast** on connection loss (`PieSessionLost`) instead of reconnect during spawn/destroy.
- **Longer destroy settle** (6 s + ticks, no `clean_garbage`).
- **Batch pauses** every 2 spawns (2.5 s).
- **Reuse mode** — if all 5 `depth_test_prop_*` exist at registry poses, skip destroy/spawn.
- **`--force-respawn`** only when intentional re-layout is needed.
- **Soft teleport** for calibration (controller off → move → on).
- **Navigation**: slower perception (0.45 s), leg timeout 180 s, max 280 steps, speed 120, connection check aborts test.
- **`ue_client_guard.exclusive_ue_client_lock()`** — single TCP client during test.
