# UE5 validation design (corrected)

## Role split (hard rule)

| Stage | Where | What |
|-------|--------|------|
| Offline Pareto $P$ | Kinematic oracle (`oracle/simulate.py`, `fronts/run_fronts.py`) | Grid + NSGA-II + SafeOpt. **Never UE5.** |
| Condition cards | `fronts/condition_cards.py` + oracle eval | Ground $\theta^*$ for L1–L4 / S1–S2 |
| Online / field check | Unreal Engine 5 PIE | Replay **selected** $\theta$ only: same field geometry, real locomotion, SSM/$d_{\min}$, progressive assembly |

Canonical offline run starts from UE-aligned geometry; after UE mission conditions stabilize, **rebuild the kinematic oracle to match UE** and recompute $P$ (do not freeze an outdated 0908 story if UE changed the field/protocol).

## Agents (required cast)

1. **Spot** ×1 — member transport (NavMesh / SpotDog controller).
2. **Yard humanoid** ×1 — handoff at kei truck / storage. **No $S_p$ / $d_{\min}$.** Spot may approach freely for loading.
3. **Assembler humanoid** ×1 — only agent under the study's keep-out: walks on the active deck, picks up the dropped member, assembles at the socket, retreats to refuge when Spot's scaffold keep-out triggers.

## Keep-out scope (hard rule)

| Agent | SSM $S_p$ | $d_{\min}$ | Notes |
|-------|-----------|------------|--------|
| Yard (truck) humanoid | **off** | **off** | Corridor / yard handoff unrestricted |
| Assembler humanoid | **on** (scaffold only) | **on** (scaffold only) | Same as kinematic oracle keep-out story |

Separation for metrics and stop-and-wait is measured **only** to the Assembler.

## What was wrong (2026-09-08 teleport runner)

`ue/run_mission_theta.py` (teleport revision) is **invalid** for paper numbers:

- Spot mostly `set_location` along Recast waypoints → looks like teleport / overspeed.
- Orientation + `dog_move` often skipped (`visual=False`).
- Only one humanoid (`ScaffoldHrc_yard_human`), teleported onto the deck as a statue.
- Full scaffold already spawned → not an erection mission.
- No live SSM / $d_{\min}$ stop-and-wait (only post-hoc TT penalties).
- Absolute TT did not match the oracle story.

Discard `out/20260908153515/ue_missions/` from that revision.

## Bare-ground assembly

UE and the kinematic oracle both start from **empty ground** and assemble
posts, braces, ledgers, boards, and stairs in the shared order of
`scene/erect_plan.py` (floor-by-floor). Do not pre-spawn a finished scaffold
for validation runs.

```bash
# UE yard only:
conda run -n simworld python ue/spawn_scaffold_pie.py --bare
conda run -n simworld python -u ue/hrc_mission.py --vmax 0.84 --dmin 1.6 --max-items 8 --max-floors 1
```

1. Yard human at truck; Assembler at floor refuge / standoff.
2. Spot approaches truck, waits handoff dwell (`truck_load_s`).
3. Spot carries a **member actor** along Recast path (corridor unconstrained; scaffold speed ≤ $v_{\max}$).
4. On scaffold, each tick: measure $S$ **to the Assembler only**; apply ISO SSM → commanded speed; if $S < d_{\min}$ or SSM stop → Spot stops; Assembler retreats to refuge; after clear, Spot resumes. The yard/truck humanoid never enters this gate.
5. Spot drops at drop point; Assembler walks to drop, picks, walks to socket, assemble dwell (`erect_s`), socket marked filled → reveal/spawn that member as fixed geometry; optional `nav_rebuild`.
6. Repeat for 4 sockets × 3 floors (or a paper subset with the same protocol).

## Locomotion

- **Spot**: prefer `nav_move.nav_move_to_goal` / path follow (Dynamic NavMesh CharacterMovement). Fallback: yaw toward next waypoint then `dog_move([v_cm_s, dt, 0])` with $v_{\mathrm{cm/s}} = 100\,v_{\max}$ on scaffold (corridor may use a fixed uncapped speed). **No teleport along the path for timed motion.**
- **Humanoids**: `humanoid_set_speed` + `humanoid_set_path` / `humanoid_follow_path`, or step-forward with yaw. Physics off (kinematic) to avoid ragdoll.

## Progressive geometry

- Keep posts / stairs / landings that NavMesh needs for the **current** climb.
- Deck boards / bay pieces for **unfilled** sockets start hidden or unspawned.
- On assemble complete: spawn/unhide that piece and rebuild nav if needed.

## Metrics for paper Table UE

Per condition-card $\theta$: completed, $\mathrm{TT}$, $S_{\min}$ (while keep-out active and Spot moving), $\mathrm{SI}_{\min}$, wait time due to SSM/$d_{\min}$.
Compare proposed vs B3/B4/SafeOpt under **identical** UE protocol.
