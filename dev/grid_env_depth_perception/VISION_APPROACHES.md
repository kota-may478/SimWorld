# Vision simulation approaches for SpotDog (research notes)

This document responds to concerns about viewpoint-dependent mask colors and
off-axis object measurement. It surveys how Unreal Engine simulates character
vision and proposes implementation directions for `grid_env_depth_perception`.

## Current implementation — limitations (honest assessment)

### What we do today

1. Mount UnrealCV camera on SpotDog (pose synced each frame).
2. Fetch `object_mask` + `depth` per sample.
3. Match mask pixels to colors stored in `prop_placement_registry.json`.
4. Derive bearing from bbox center; distance from depth ROI.

### Your concern: calibration color varies with approach angle

**Partially valid, but the root cause is the wrong reference color.**

| Issue | Explanation |
|-------|-------------|
| Viewpoint-dependent **lit RGB** | Not used — correct to avoid |
| Viewpoint-dependent **segmentation ID** | Should **not** happen in principle; `object_mask` is a labeling buffer, not shaded albedo |
| Why calibration failed anyway | We sampled **dominant color in image center** from one pose — that mixes background, edges, anti-aliasing, occlusion with other props, PNG quantization |
| Official UnrealCV pattern | At spawn: `vset /object/{name}/color R G B`, then **`vget /object/{name}/color`** as canonical ID; match mask with small tolerance (see [UnrealCV GT tutorial](https://docs.unrealcv.org/en/latest/tutorials/generate_images_tutorial.html)) |

Remaining viewpoint effects on mask (even with correct ID):

- **Silhouette / pixel count** changes with angle (thin side vs broad face).
- **Edge pixels** vary under tolerance matching.
- **Occlusion**: only the front-most surface per pixel wins in the stencil buffer.
- **Not in frustum**: zero pixels — cannot detect.

So: drop one-pose BGR calibration; use per-actor `vget /object/{name}/color` instead.

### Can we measure objects NOT directly ahead?

| Situation | Current camera pipeline | Notes |
|-----------|-------------------------|-------|
| In front, center of FOV | Yes | Best accuracy |
| In front, left/right within 90° FOV | **Yes, in principle** | Bearing from bbox center; works if mask+depth valid |
| Behind the robot | **No** | Outside camera frustum |
| Outside FOV cone | **No** | Same as real narrow camera |
| Occluded by wall/other prop | **No / partial** | Mask may disappear; depth hits occluder |
| 360° awareness | **No** | Would need multiple sensors or engine perception |

The current code does **not** implement a UE-style vision cone in the engine — only
whatever the camera sees. FOV 90° is a rough cone, but there is **no line-of-sight
trace** unless we add it.

---

## How Unreal Engine simulates character vision (survey)

### 1. AI Perception — Sight Sense (the “transparent cone” model)

**Docs:** [AI Perception](https://dev.epicgames.com/documentation/unreal-engine/ai-perception-in-unreal-engine),
[UAISenseConfig_Sight](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/AIModule/UAISenseConfig_Sight)

```
Observer eyes (GetActorEyesViewPoint)
        │
        ▼
  ┌─────────────────┐
  │  Sight cone     │  SightRadius (max distance)
  │  half-angle =   │  PeripheralVisionAngleDegrees
  │  Peripheral...  │
  └────────┬────────┘
           │ targets in cone
           ▼
  Line trace (Visibility channel) ──► occluded? → not seen
           │
           ▼
  FAIStimulus: actor ref, strength, last seen location
```

| Parameter | Role |
|-----------|------|
| `SightRadius` | Max detection distance |
| `PeripheralVisionAngleDegrees` | **Half-angle** of cone (not full FOV) |
| `PointOfViewBackwardOffset` / `NearClippingRadius` | Near-field / peripheral awareness |
| `LoseSightRadius` / `AutoSuccessRangeFromLastSeenLocation` | Hysteresis after target seen |
| `GetActorEyesViewPoint` | Eye position + direction (head socket) |
| `IAISightTargetInterface` | Custom LoS points (body, camera, bounds) |

**Output:** structured list of **which actors** are visible — not an image.
Distance/bearing must be computed from observer ↔ target transforms (or from
`OutSeenLocation` on the interface).

**Pros:** Native cone + occlusion; identity = actor; no per-frame image processing;
viewpoint-independent ID.

**Cons:** No automatic depth image; requires C++/Blueprint on AIController; less
detail for ML-style evaluation unless you add geometry post-processing.

---

### 2. SceneCaptureComponent2D (robot-mounted sensor)

**Docs:** [USceneCaptureComponent2D](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Engine/USceneCaptureComponent2D)

Used in robotics sim tooling (e.g. [UnrealROX+](https://ar5iv.labs.arxiv.org/html/2104.11776),
[TempoSensors](https://github.com/tempo-sim/Tempo/blob/release/TempoSensors/README.md)):

- Attach capture component to pawn mesh socket (head).
- Render to `TextureRenderTarget2D` — **independent of main viewport**.
- Multiple components: RGB, SceneDepth, CustomStencil (one mode each).
- **CustomDepth Stencil** (1–255 per actor/type) for stable segmentation IDs.

**Pros:** True egocentric sensor; industry standard for synthetic datasets; stable
stencil IDs; can run at fixed rate on pawn.

**Cons:** Requires UE Editor setup on `BP_SpotRobot`; reading pixels from Python
needs UnrealCV/`vbp` bridge or export step; multiple captures cost GPU.

---

### 3. UnrealCV external camera (SimWorld current path, improved)

**Docs:** [UnrealCV commands](https://docs.unrealcv.org/en/latest/reference/commands.html)

- `vget /camera/{id}/object_mask` — instance segmentation
- `vget /camera/{id}/depth npy` — depth
- `vset /object/{name}/color R G B` + **`vget /object/{name}/color`**

Camera pose updated from Python (`set_camera_location/rotation`) to follow SpotDog.

**Pros:** Already integrated in SimWorld; no UE code change for basic tests.

**Cons:** TCP + full frame fetch per sample; camera not a real child component
(must sync pose); no built-in LoS; heavy for long runs.

---

### 4. CustomDepth Stencil (MATLAB / AD toolbox pattern)

**Ref:** [MathWorks stencil labeling](https://www.mathworks.com/help/driving/ug/apply-labels-to-unreal-scene-elements-for-semantic-segmentation-and-object-detection.html)

- Project setting: Custom Depth-Stencil Pass = Enabled with Stencil.
- Each mesh: `Render CustomDepth` + unique stencil byte.
- Post-process or SceneCapture reads stencil buffer.

**Pros:** IDs are **per-object constants** (not view-calibrated colors); supports
type-level IDs (all barrels = same stencil).

**Cons:** Needs asset/Editor work on Construction props; 255 ID limit; SimWorld
Level props may need batch stencil assignment.

---

## Proposed implementation directions

### Approach A — **AI Perception Sight + geometry ground-truth** (engine-native cone)

**Best match for “transparent cone on character”.**

| Item | Detail |
|------|--------|
| UE side | `AIPerception` on SpotDog AIController; Sight config (radius, half-angle); `AIPerceptionStimuliSource` on each `depth_test_prop_*`; override `GetActorEyesViewPoint` on robot |
| Python side | Poll visible actors via new UnrealCV/`vbp` API or lightweight UE service; compute distance/bearing from poses |
| Identity | Actor name / `prop_type_id` map — **no image color matching** |
| Occlusion | Engine line traces |
| Off-axis | Anything in cone ± half-angle, not only boresight |
| SimWorld effort | Medium–high (UE C++/BP + small bridge) |

---

### Approach B — **SceneCapture on SpotDog + stencil IDs** (robotics sim standard)

| Item | Detail |
|------|--------|
| UE side | Add SceneCapture components to `BP_SpotRobot`; assign CustomStencil per prop at spawn |
| Python side | UnrealCV reads dedicated robot camera render targets (or export via existing camera id if wired) |
| Identity | Stencil byte → `prop_type_id` lookup table |
| Off-axis | Full camera FOV; multiple props in one frame |
| SimWorld effort | High (asset + plugin wiring) but most scalable for depth+seg research |

---

### Approach C — **Fix UnrealCV pipeline (minimal change)** — recommended **short-term**

Keep architecture; fix identification and FOV semantics:

1. **Remove** one-pose mask calibration (`mask_calibration.py` as primary ID source).
2. At spawn: `vset /object/{slot_id}/color` then **`vget /object/{slot_id}/color`** → store in registry (canonical).
3. Each frame: match mask using canonical RGB (tolerance 3–8 per UnrealCV tutorial).
4. Explicit **visibility gate**: compute GT bearing; only report estimate if `|bearing| ≤ FOV/2` AND mask pixels > threshold.
5. Optional **LoS check**: `vget /object/{robot}/location` → prop location line trace via existing collision probe / `NavQuery` / simple `vbp` trace.
6. Bearing/distance from **mask bbox + depth**, not only boresight.

| Pros | Fastest path; uses existing PIE scripts |
| Cons | Still image-based; no true engine cone; LoS is extra work |

---

### Approach D — **Hybrid: Sight for “who is visible” + camera for metrics**

| Step | Source |
|------|--------|
| Visible set | AI Perception Sight (cone + occlusion) |
| Distance / fine bearing | Depth + mask only for actors in visible set |
| Evaluation | Compare against pose GT; reduces false positives from color bleeding |

**Best long-term balance** if UE-side work is acceptable.

---

### Approach E — **Depth-only + nearest-neighbor to registry** (not recommended for identity)

Cluster depth points → match to known prop positions.

| Pros | No segmentation colors |
| Cons | **Cannot reliably answer “what object”** when multiple same-type props or partial views; cheats identity via position oracle |

Use only as ablation baseline, not production perception.

---

## Recommendation

| Phase | Approach | Goal |
|-------|----------|------|
| **Now** | **C** — canonical `vget /object/color`, drop viewpoint calibration | Fix false “angle-dependent color” failure mode |
| **Next** | Add explicit FOV + optional LoS in Python | Align with cone semantics |
| **Medium** | **A** or **D** — AI Perception Sight on SpotDog | True UE vision cone + occlusion |
| **Research** | **B** — SceneCapture + CustomStencil on robot | Publication-grade egocentric sensor |

---

## References

- [Unreal Engine AI Perception](https://dev.epicgames.com/documentation/unreal-engine/ai-perception-in-unreal-engine)
- [UAISenseConfig_Sight API](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/AIModule/UAISenseConfig_Sight)
- [UnrealCV — Generate Images / GT](https://docs.unrealcv.org/en/latest/tutorials/generate_images_tutorial.html)
- [UnrealCV — Command reference (`vget /object/.../color`)](https://docs.unrealcv.org/en/latest/reference/commands.html)
- [UnrealROX+ — SceneCapture2D for multi-modal data](https://ar5iv.labs.arxiv.org/html/2104.11776)
- [TempoSensors — SceneCapture on robots](https://github.com/tempo-sim/Tempo/blob/release/TempoSensors/README.md)
- [MathWorks — Custom stencil labeling](https://www.mathworks.com/help/driving/ug/apply-labels-to-unreal-scene-elements-for-semantic-segmentation-and-object-detection.html)
- [AISense Sight notes (line trace, affiliation)](https://zomgmoz.tv/unreal/AI-Perception/AISense-Sight)
