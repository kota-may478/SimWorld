# Stage-1 field alignment (oracle ↔ UE5)

Updated 2026-09-08 after the Recast stair / carryable-parts UE pass.

## Shared constants (`scene/field.py`)

| Quantity | Value | Notes |
|----------|-------|-------|
| Storage / kei yard | (−11.2, 1.5) m | Matches `STAGING_LOCAL_XY_CM=(150, 280)` |
| Recast stair run | 6.8 m | `RAMP_RUN_M`, L0 south then L1 north |
| Stair yard x | ≈ −7.6 m | Keep-out / SSM starts here (`scaffold_edge_x_m`) |
| Carry max (boards) | 2.4 m | Bay-length アンチ; braces may be longer |

## Bare-ground full assembly (oracle + UE)

Both sims start from an empty yard and install **every** member in order
(`scene/erect_plan.py`): per lift → posts → braces → ledgers/transoms → boards
→ stair treads, then the next lift.

| Piece | Oracle | UE |
|-------|--------|-----|
| Start | empty scaffold | `spawn_scaffold_pie.py --bare` |
| Work queue | `build_erect_sequence` | same |
| Place | Spot haul + Assembler dwell | Spot haul + spawn cube + Assembler walk |
| Keep-out | Assembler on scaffold only | same |

Pareto search should re-run after this change (`fronts/evaluate.py` defaults to
`erect_from_bare=True`). `--sockets-per-floor N` thins **boards only**; posts,
braces, ledgers, and stair treads remain. Further thinning:
`bare_max_floors` / `bare_max_items`. Oracle timeout defaults to 36000 s sim
(full bare ≈ 10k s at REF_THETA).

**`out/20260908153515/` is obsolete** for bare-ground protocol — recompute fronts
before paper numbers.

1. Stabilize the UE mission (`ue/hrc_mission.py`) until cast, keep-out scope, locomotion, and progressive place match the paper story.
2. Freeze shared numbers in `scene/mission_protocol.py`.
3. Rebuild / re-run the kinematic oracle and fronts so $P$ reflects those UE conditions (do not treat an older run as frozen if the field protocol changed).
4. Replay condition-card $\theta$ in UE for comparison tables only.

- Yard / truck humanoid: **no** $S_p$ / $d_{\min}$ (handoff unrestricted).
- Assembler humanoid: $S_p$ / $d_{\min}$ on scaffold only (study subject).

Language contrast (proposed vs B3 keyword vs B4 3-mode):

1. `L1_efficient` — 「急いで」
2. `L2_safe` — 「慎重に」
3. `L3_distance` — 「離れて作業して」
4. `L4_relative_slower` — 「もう少しゆっくり」 (proposed vs B4 only)

Safety-search contrast (proposed cell vs SafeOpt incumbent):

5. `S1_safeopt_vs_normal` — proposed (α,β)=(0.5, 0.5)
6. `S2_safeopt_vs_safe` — proposed (α,β)=(1.0, 0.5)

## Reproduce

Canonical full run: `out/20260908171041/` (NSGA-II ISO + SafeOpt, bare erect,
boards/floor=4). **Pareto $P$ is offline-only** (kinematic oracle). UE5 never
builds the front.

```bash
cd dev/scaffold_hrc
conda run -n simworld python fronts/run_fronts.py --sockets-per-floor 4
conda run -n simworld python scripts/run_condition_cards.py --fronts out/<run>/fronts.json --out out/<run>/cards
conda run -n simworld python scripts/export_paper_figs.py --fronts out/<run>/fronts.json --cards out/<run>/cards/condition_cards.json
# PIE field check under a frozen theta (see docs/ue_validation_design.md):
conda run -n simworld python -u ue/hrc_mission.py --vmax 0.84 --dmin 1.6 --max-items 3 --max-floors 1
```

