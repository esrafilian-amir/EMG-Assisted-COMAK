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

- [ ] **Skipping the first `initSystem` in the COMAK settle is not safe.** The model
  copy needs finalizing before its components can be listed. A cheaper
  `finalizeFromProperties()` was not tried.
- [ ] **Smoothing is on by default** (`smooth_input_kinematics`). On the EMG case
  (already 6 Hz filtered) it changes activations by up to ~0.03. Compare
  converged frames / iterations on walking and meniscus in Step 4, then decide
  whether to keep it as the default.
- [ ] Check the patellar tendon strain / slack length on the updated knee models
  (use the new `*_secondary_constraint_sweep_ligaments.sto`).

## Plan

- [x] Step 0: build, baseline runs (walking, walking+meniscus, EMG-assisted),
  compare/summarize scripts
- [x] Step 1: settle simulations (IK settle and COMAK settle)
- [x] Step 2: sweep and constraint functions (`ind_pt_data` overflow fixed, 0.8
  patella multiplier removed so constrained coordinates are free in the sweep,
  ligament diagnostics file, patella warning)
- [x] Step 3: COMAK input kinematics (`time_step = -1` checks, smoothing spline
  on by default, disabled-muscle index alignment, processed kinematics printout)
- [ ] Step 4: COMAK optimization (reserve bound and gradient, check IPOPT result
  before using it, fallback restore, trust region, mass-weighted rows, speed term)
- [ ] Step 5: COMAK speed (cached lookups, allocations, `flip_meshes`, parallel
  Jacobian columns)
- [ ] Step 6: contact and mesh (OBB triangle-index bug, leaf closest-hit,
  parallel tolerance, `max_proximity` cliff, force aggregation, Position-stage
  cache, culling)
- [ ] Step 7: stability on non-walking tasks, patella and meniscus handling,
  final regression, docs/CHANGELOG
