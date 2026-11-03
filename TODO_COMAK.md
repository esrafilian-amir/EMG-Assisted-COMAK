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
- [x] Step 4: COMAK optimization (reserve bound and gradient fixes, finite-result check,
  corrected fallback; optional numerical options; trust region, soft constraints and
  smoothing penalties tried and removed, see scripts/regression/RESULTS.md)
- [x] Step 5: COMAK speed (timing log, `flip_meshes` switched off for COMAK runs; the
  linearization is 80-97% of the run time; parallel Jacobian columns were dropped on
  purpose, Jacobian reuse across iterations diverges)
- [x] Step 6: contact and mesh (OBB triangle-index bug, leaf closest-hit, parallel
  tolerance, force aggregation; `max_proximity` cliff, Position-stage cache and culling
  not done)
- [x] Step 6b: optimization (see `scripts/regression/RESULTS.md`): force based
  convergence criterion (default), `max_change` as a limit on the total change in a time
  step and free (0) by default, exact-Hessian `casadi_ipopt` solver (default) with
  relaxed acceleration constraints, optional `casadi_qrqp`, activation change penalty
  (0.1), central difference Jacobian (default), range limits for the secondary
  coordinates (optional), per-group timing of the linearization
- [x] Step 7: stability on a non-walking task (the EMG example is a bouncing task), final
  regression (full-length runs of all three examples converge on every frame), docs and
  `CHANGELOG_JAM.md`

## Open

- [x] Full-length confirmation runs: see the last section of `scripts/regression/RESULTS.md`.
- [ ] Meniscus run time: the constraint sweep (440 s) and the COMAK settle (about 200 s) now dominate;
  see RESULTS.md for options (reuse the sweep, stop the settle earlier, per-pair contact cache).
- [ ] Meniscus vertical translation zig-zag in the settle (about 10 um): damping in the settle only,
  smoother contact, or output smoothing.
- [ ] `exampleWalking_Patella_Constrained` and `exampleWalkingMuscleCoordination` were
  changed to the new defaults but not run.
- [ ] Calibrate `force_tolerance_translation`: the logs only show the worst coordinate,
  which was always rotational for accepted frames.
- [ ] Tight `max_change` values (twice the IK step) make walking fail; if a limit is
  wanted for a new task, set it from a free run (`scripts/regression/free_steps.py`).
- [ ] Apply the numerical options only to light coordinates (selected by inertia).
- [ ] `max_proximity` cliff, Position-stage cache and culling in the contact model.
