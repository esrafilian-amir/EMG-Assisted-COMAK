# COMAK / JAM review: results so far

Work on `master` (local commits, nothing pushed). Steps 0 to 4 are done; Steps 5 to 7
are open (see `TODO_COMAK.md`). All numbers below were measured with the scripts in this
folder on the example models (EMG-assisted, walking, walking with menisci), RelWithDebInfo
build, MATLAB examples driven by `run_case.py`.

## 1. Baseline (unmodified `master`, full runs)

| model | frames converged | mean iterations | IPOPT failures | time |
|---|---|---|---|---|
| EMG-assisted (81 frames) | 81 / 81 | 5.1 | 399 of 411 solves | 19 min |
| walking (63 frames) | 46 / 63 | 11.9 | 713 | 30 min incl. IK |
| walking + menisci (63 frames) | 13 / 63 | 22.2 | 1233 | 73 min |

The instability is real and concentrated in the loading and push-off phases of walking and
almost everywhere in the meniscus model.

## 2. What was fixed (committed)

| step | change | evidence |
|---|---|---|
| 1 | Settle simulations: `std::abs`, cached coordinate pointers, states stored only when printed, shared `SettleMonitor` (threshold, optional plateau stop, safety limit), hard-coded 0.02 and 0.005 s are properties, optional zero start speeds | settle results identical to baseline (IK walking and meniscus, COMAK EMG) |
| 2 | IK constraint functions: one-element heap overflow and missing last point fixed (functions stopped at 95 of 100 deg); the hard-coded 0.8 patella coupler removed (constrained coordinates are free in the sweep); ligament length/strain/force diagnostics file; warning if a constrained coordinate barely moves | `check_constraint_functions.py`: baseline fails, new passes; free patella follows 0.63 (no menisci) to 0.73 (menisci) times the knee flexion |
| 3 | `time_step = -1` no longer throws; processed-kinematics printout read past the matrices; disabled muscles no longer shift the cost parameters of later muscles; non-uniform time warning; quintic smoothing spline for the input kinematics (`smooth_input_kinematics`, default on; off reproduces the old results exactly) | tests in `regression_runs` (old build throws / misassigns `vasmed_r`; new does not) |
| 4 | Reserve actuator upper bound and cost gradient fixed (the gradient did not match the cost); non-finite solver results are rejected; the fallback after a failed frame no longer adds the previous change twice; the verbose-0 logging line fixed | see section 3 |

## 3. Effect of the Step 4 fixes (short windows, new reference IK)

| window | before (Step 3) | after the fixes |
|---|---|---|
| walking 1.36 to 1.48 s, 13 frames | 11 converged, 12.0 iterations, 133 IPOPT failures, 323 s | 13 converged, 7.3 iterations, 1 failure, 177 s |
| EMG 0 to 0.3 s, 31 frames | baseline: 31 converged, 6.4 iterations | 31 converged, 6.0 iterations, 0 failures; activations differ from the baseline by 0.005 RMS (0.0015 on EMG-driven muscles) |
| meniscus 1.28 to 1.40 s, 12 frames | 7 converged, 17.8 iterations, 187 failures, 770 s | 6 converged, 16.1 iterations, 11 failures, 732 s |

The reserve gradient bug caused most of the IPOPT "restoration failed" errors. It removes
the failures but does not by itself fix the meniscus model.

## 4. Tried and removed

| idea | result | why removed |
|---|---|---|
| trust region (revert to best iteration, halve `max_change`) | walking 7 / 13 converged; meniscus 0 / 12 (coordinates never moved) | the rule shrinks the allowed step until nothing moves |
| soft acceleration constraints (slack per row) | marginal and inconsistent: walking 13/13 (w=1000), 12/13 (100, 10000) | no reliable gain |
| penalty on the second difference (acceleration) of the secondary coordinates | rotations 65% smoother at best, but translations up to 2.7 times rougher, convergence falls to 11 of 13 frames at weight 1e-2 and to 3 to 5 of 13 at larger weights; same with a soft constraint | the secondary coordinates are set by hard force equilibrium, so smoothing them only shifts the error to the translations and muscles |
| penalty on activation change between frames | walking 4.4 iterations (from 7.3), meniscus 9/12 converged (from 6/12), but it distorted EMG tracking (0.13 RMS) | removed on request; the data above is a reason to reconsider it |

## 5. Optional numerical options (all default off)

New properties of `COMAKTool`: `use_central_difference_jacobian`, `ignore_secondary_speed_term`,
`normalize_constraint_rows`, `secondary_step_penalty_weight` (+ `step_penalty_rotation_length`),
`secondary_velocity_carryover_to_next_step` (first called `position_extrapolation_factor`). They act on every
secondary coordinate, patella and menisci alike. The projection factor now defaults to 0 (see below).

Meniscus window (12 frames, control: 6 converged, 16.1 iterations, 732 s):

| option | converged | mean iterations | time |
|---|---|---|---|
| central differences | 8 | 16.1 | 824 s |
| ignore speed term | 0 | 25.0 | 1366 s |
| row normalization | 7 | 16.8 | 1061 s |
| step penalty 1e4 / 1e5 | 5 / 3 | 18.5 / 23.1 | 796 / 927 s |
| extrapolation factor 0.5 | 6 | 17.9 | 768 s |
| extrapolation factor 0 | 8 | 14.6 | 682 s |

Walking window (13 frames, control 13/13, 7.3 iterations, 176 s): extrapolation 0 gives
13/13, 5.3 iterations, 141 s; central differences 13/13, 7.0, 189 s; row normalization
13/13, 7.6, 222 s. EMG (31 frames, control 6.0 iterations, 505 s): extrapolation 0 gives
31/31, 6.3 iterations, 531 s.

Windows are short (12 to 31 frames), so these are indications, not proof.

Combination of extrapolation factor 0 and central differences: meniscus 8 converged, 14.5
iterations, 750 s (no gain over extrapolation 0 alone); walking 13/13, 7.1 iterations, 188 s
(worse than extrapolation 0 alone, 5.3 iterations, 141 s); EMG 31/31, 6.2 iterations, 595 s
(18% slower than the control). The two do not add up.

**Decision:** `secondary_velocity_carryover_to_next_step` defaults to 0 (it was called
`position_extrapolation_factor` in the test runs). It is the best single option: meniscus 8/12
instead of 6/12, walking 27% fewer iterations and 20% less time, EMG about equal (6.3 against 6.0
iterations, 3 IPOPT failures against 0, all frames still converge). The converged solutions barely
change (walking and EMG activations differ by at most 0.005, secondary rotations by 0.002 deg,
translations by 0.003 mm). Set it to 1 to get the old constant-velocity projection back.
Central differences help the worst case of the meniscus window (worst acceleration error 8.5
against 50) at about 13% more time and are neutral on walking, so they stay off. The step penalty,
ignoring the speed term and row normalization did not help.

## 6. Reference IK

The Step 2 IK (full-range constraint functions, free patella) is the reference for COMAK tests.
It differs from the old IK by about 1 deg RMS at the knee. Controls show the cause is the very
small change of the constraint functions (at most 0.04 deg, 0.4 mm) amplified by the loosely
determined marker fit; the code and the solver tolerance are not the cause.

## 7. Open items

- Full-length confirmation runs of the three models with the final defaults (only short windows were run).
- The meniscus model still converges only about 2/3 of the frames in the window; it needs more than the fixes.
- Check the patellar tendon strain and slack length on the updated knee models
  (`*_secondary_constraint_sweep_ligaments.sto`).
- Steps 5 (speed), 6 (contact and mesh, including the OBB triangle-index bug) and 7 (stability on non-walking tasks).
- Optional: apply the numerical options only to light coordinates (selected by inertia).

## 8. Reproducing

`build_install.ps1` builds only the JAM targets and installs; `run_case.py` runs an example
(patched for batch use, optional window, muscle disabling and `--reuse-ik`); `compare.py`,
`stats.py`, `sec_jumps.py`, `penalty_diagnosis.py` and `check_constraint_functions.py` compare
and summarize the results.

## Solver, convergence criterion and bounds (windows: walking 1.36-1.48 s, meniscus 1.28-1.40 s, EMG 0-0.3 s)

All runs: reference IK, central-difference Jacobian. One run per setting, other runs shared the CPU, so times are
indicative; iteration counts and convergence are reliable.

| Window | Solver / criterion | Converged | Iterations (per frame) | Notes |
|---|---|---|---|---|
| walking | IPOPT / acceleration | 13/13 | 83 (6.4) | max_change limited per time step |
| walking | IPOPT / force | 13/13 | 30 (2.3) | |
| walking | casadi_qrqp / force | 13/13 | 33 (2.5) | 12 of 33 iterations fell back to IPOPT |
| EMG | IPOPT / force | 29/31 | 141 (4.5) | 53 IPOPT failures (infeasible with the new bounds) |
| EMG | casadi_qrqp / force | 28/31 | 149 (4.8) | 66 fallbacks |
| meniscus | IPOPT / acceleration (old) | 7/12 | 183 (15.2) | before max_change limit |
| meniscus | IPOPT / force | 12/12 | 54 (4.5) | |
| meniscus | casadi_qrqp / force | 12/12 | 45 (3.8) | 31 fallbacks |

Findings
- `convergence_criterion = force` (unbalanced generalized force M*(udot - observed), tolerances 0.1 N*m / 1 N) cuts
  iterations by 60-70% and converges all meniscus frames; solutions differ from the acceleration criterion by at most
  0.4 deg (knee rotation) and 0.005 (activation) on walking. Default stays `acceleration` until the tolerances are
  calibrated on more data.
- `max_change` is now an accumulated limit per time step (it was per COMAK iteration, so a frame could move a
  coordinate several times max_change). It changes the walking solution (knee_add step is capped at 0.01 rad;
  activations differ by up to 0.23 from the old runs). IPOPT fails more often because some subproblems become infeasible.
- `enforce_secondary_coordinate_range` (COMAK and COMAKInverseKinematicsTool) changes nothing on walking (the model
  ranges are wide); results are identical with it on.
- casadi_qrqp: with a free quadratic slack and row scaling it never fails, but the active-set solver can return a point
  outside the bounds and still report success (mostly when the allowed window does not contain zero). Each answer is
  therefore checked and IPOPT is used for that iteration when it is violated. casadi_ipqp never converged and was removed.
- Linearization (model evaluations) is 80-97% of the time. Per walking linearization (0.85 s): muscles 37%,
  reserves 14%, secondary coordinates 39%, secondary speeds 10%. Reusing the Jacobian across iterations
  (jacobian_recompute_interval > 1) diverges: the contact model is too nonlinear after the first step.
  Forward difference needs 42% more iterations than central and is slower overall.

## Full-length confirmation (final defaults, examples run end to end)

`run_case.py` with nothing patched except the visualizer and the logging, install `central1`.
"Original" is the unmodified code (runs `base2`). Times are wall times of the whole MATLAB script
(inverse kinematics, COMAK, joint mechanics).

| Example | Version | Converged frames | COMAK iterations | Solver failures | Wall time |
|---|---|---|---|---|---|
| Walking, 63 frames | original | 46 / 63 | 748 | 713 | 1774 s |
| | final defaults | 63 / 63 | 179 | 0 | 596 s |
| Meniscus, 63 frames | original | 13 / 63 | 1398 | 1233 | 4391 s |
| | final defaults | 63 / 63 | 194 | 0 | 983 s |
| EMG-assisted (bouncing task), 81 frames | original | 81 / 81 | 411 | 399 | 1142 s |
| | final defaults | 81 / 81 | 187 | 0 | 389 s |

Smoothness of the full-length results (RMS second difference over time; activations are all muscle and
reserve activations, secondary coordinates in deg or m):

| Example | Activation roughness, original -> final | Largest activation step | Secondary roughness |
|---|---|---|---|
| Walking | 0.0073 -> 0.0056 | 0.196 -> 0.221 | 0.0140 -> 0.0135 |
| Meniscus | 0.0062 -> 0.0041 | 0.157 -> 0.149 | 0.0188 -> 0.0189 |
| EMG | 0.0153 -> 0.0091 | 0.272 -> 0.275 | 0.0137 -> 0.0144 |

The activations differ from the original by at most 0.045 (walking, meniscus) and 0.13 (EMG);
the secondary coordinates by at most 0.34 / 0.38 / 0.15 (deg or mm).

Meniscus chattering: the 12 meniscus coordinates of the COMAK results are smooth (3-15% direction
changes between frames, noise about 1% of the range) and identical before and after. The settle
simulation shows a broadband zig-zag of the vertical translations (`meniscus_*_ty_r`, about 10 um RMS on a
slow drift of 50-100 um) in the original and in the final code. It is probably the light meniscus (0.1 kg) in
a stiff undamped contact. Not fixed; options: damping in the settle only, smoother contact (triangle
switching, `max_proximity`), smoothing of the output.

Where the time goes (meniscus): inverse kinematics 453 s of which 440 s is the constraint sweep
(depends on the model, not the trial: `perform_secondary_constraint_sim = false` reuses it); COMAK
settle at the first frame about 200 s (91 steps, the change per step is below 1e-3 after 9 steps and
below 3e-4 after 48); COMAK frames about 3 s each, of which 67% is the 23 secondary coordinate columns
(about 18 ms per evaluation, the contact).
