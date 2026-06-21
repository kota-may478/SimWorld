# Detection algorithm & PIE safety

See also `README.md` for run instructions.

## What sensors are used?

| Stream | UnrealCV mode | Purpose |
|--------|---------------|---------|
| **object_mask** | `FusionCamSensor` (head, cam 2) | **Identity** — segmentation buffer with stencil colors |
| **depth** | `FusionCamSensor` (same camera) | **Distance** — robot-mounted depth |
| **lit** | `FusionCamSensor` (optional fallback) | Deprecated RGB fallback only (`--allow-lit-fallback`) |

`resolve_mask_camera_id()` prefers **FusionCam** when segmentation is active; ThirdPerson is fallback only.

Each actor gets a unique flat color via `vset /object/{name}/color R G B`. Mask pixels use that color (decoded as **BGR** after PNG load).

## Color registry (Approach C)

Per [UnrealCV GT tutorial](https://docs.unrealcv.org/en/latest/tutorials/generate_images_tutorial.html):

1. At spawn: `vset /object/{slot_id}/color` with intended RGB.
2. Query **`vget /object/{slot_id}/color`** → store as `mask_color_canonical_rgb`.
3. Match mask pixels with per-channel tolerance (default ±6).

`cache/prop_placement_registry.json` fields:

| Field | Meaning |
|-------|---------|
| `mask_color_canonical_rgb` | **Primary ID** from `vget /object/{name}/color` |
| `mask_color_set_rgb` | Color written at spawn (`set_color`) |
| `mask_color_rgb` | Mirror of canonical (human-readable) |
| `mask_color_observed_bgr` | Deprecated — cleared on color sync |
| `lit_color_observed_bgr` | Deprecated — lit fallback only |

**Detection uses `mask_color_canonical_rgb` (as BGR)**. One-pose standoff calibration (`mask_calibration.py`, `prop_signature.py`) is deprecated and not run by default.

## Per-frame detection pipeline

For each prop in the registry:

1. **Segmentation match** — threshold mask pixels where `|pixel_BGR - detection_bgr| ≤ tolerance`.
2. **Pixel count** — skip if `< 48` pixels (not reliably in FOV).
3. **BBox + centroid** — axis-aligned box; bearing from mask **centroid** x.
4. **Bearing** — `(cx - W/2) / (W/2) * (FOV/2)` relative to camera forward.
5. **Depth** — 35th percentile of valid depth in ROI around bbox center (60% height).
6. **Horizontal distance** — slant range → ground distance using camera pitch; add forward camera offset to robot frame.
7. **Conflict resolution** — if two props share similar bearing (< 8°), keep higher `mask_pixels`.

Output per detected prop: `prop_type_id`, `distance_m`, `bearing_deg`, `confidence`.

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
| **Destroy → immediate spawn** | Level PIE is fragile after batch `vset .../destroy`; GC / render thread still busy. |
| **Aggressive reconnect** | On connection loss, reconnect during teardown stresses crashing UE. |
| **Open-loop navigation** | `turn-then-go` without NavMesh; SpotDog can hit geometry. **Mitigation:** `--nav-mode navmesh` (Phase 4). |

### FusionCam object_mask gray (UE fix pending rebuild)

Output Log smoking gun:

    LogRenderer: Scene Capture has ShowOnlyComponents ... ignored by the PrimitiveRenderMode setting!
    ...FusionCamSensor.FusionCamSensor_2_AnnotationCamSensor

BP instances can serialize `PRM_RenderScenePrimitives`, which ignores `ShowOnlyComponents`. Fix: set **Annotation Cam Sensor → Use Show Only List** in `BP_SpotRobot`, plus `AnnotationCamSensor.cpp` guard. After fix, FusionCam (`mask_cam=2`) carries segmentation; Python restores editor viewport to **Lit** after each `object_mask` fetch.
| **High UnrealCV rate** | Perception every 0.45 s × depth + mask per sample. |
| **Hard destroy SpotDog** | Known to crash Level PIE. |

### Implemented mitigations (`pie_safety.py`)

- **Fail-fast** on connection loss (`PieSessionLost`).
- **Reuse mode** — if all 5 `depth_test_prop_*` exist at registry poses, skip destroy/spawn.
- **`--force-respawn`** only when intentional re-layout is needed.
- **Navigation**: perception interval 0.45 s, leg timeout 240 s, connection check aborts test.
- **`ue_client_guard.exclusive_ue_client_lock()`** — single TCP client during test.
