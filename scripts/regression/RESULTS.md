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
`position_extrapolation_factor`. They act on every secondary coordinate, patella and menisci alike.

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
