# COMAK / JAM improvement TODO

Working list for the step-by-step review of the settle, sweep, IK, COMAK and
contact-mesh code. Regression scripts live in `scripts/regression/`.

## Deferred (come back to these)

- [ ] **Settle: normalize coordinate units.** The settle convergence check takes
  the largest raw `|dq|` over all secondary coordinates, mixing radians and
  metres (`COMAKTool::equilibriateSecondaryCoordinates`, IK settle in
  `COMAKInverseKinematicsTool.cpp`). Idea: optional property that scales
  rotational changes by a characteristic length (e.g. ~0.05 m) so everything is
  compared in metres. Default off to keep results unchanged.
- [ ] **Settle start speeds:** `settle_zero_initial_speeds` was added (default
  off). Compare on all three example models, then decide whether to make it the
  default.
- [ ] **Settle plateau stopping:** `settle_plateau_tolerance` was added (default
  off). Measure how much earlier it stops and how much the settled coordinates
  change, then choose a recommended value.

## Plan

- [x] Step 0: build, baseline runs (walking, walking+meniscus, EMG-assisted),
  compare/summarize scripts
- [ ] Step 1: settle simulations (IK settle and COMAK settle)
- [ ] Step 2: sweep and constraint functions (`ind_pt_data` overflow, 0.8
  multiplier as a property, patella diagnostics)
- [ ] Step 3: COMAK input kinematics (`time_step = -1` checks, udot smoothing,
  disabled-muscle index alignment)
- [ ] Step 4: COMAK optimization (reserve bound and gradient, check IPOPT result
  before using it, fallback restore, trust region, mass-weighted rows, speed term)
- [ ] Step 5: COMAK speed (cached lookups, allocations, `flip_meshes`, parallel
  Jacobian columns)
- [ ] Step 6: contact and mesh (OBB triangle-index bug, leaf closest-hit,
  parallel tolerance, `max_proximity` cliff, force aggregation, Position-stage
  cache, culling)
- [ ] Step 7: stability on non-walking tasks, patella and meniscus handling,
  final regression, docs/CHANGELOG
