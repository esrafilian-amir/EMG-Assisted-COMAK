# JAM / COMAK changes in this repository

Changes to the COMAK, COMAK inverse kinematics and contact code relative to the original
`master`. Test results are in `scripts/regression/RESULTS.md`.

## Behaviour changes (defaults)

- `COMAKTool.convergence_criterion` is `force`: a time step has converged when the unbalanced
  generalized force `M (udot - observed)` is below `force_tolerance_rotation` (0.1 N*m) and
  `force_tolerance_translation` (1 N). Use `acceleration` for the previous test.
- `COMAKTool.solver` is `casadi_ipopt` (IPOPT through CasADi with the exact Hessian). `ipopt` is the
  previous solver, `casadi_qrqp` an active-set QP solver. Without CasADi, `ipopt` is used.
- `COMAKSecondaryCoordinate.max_change` is the limit on the total change in a time step (it was
  applied per COMAK iteration). 0 or less means no limit, and 0 is the default.
- `COMAKTool.use_central_difference_jacobian` is `true`.
- `COMAKTool.activation_change_penalty_weight` is 0.1 (penalty on the activation change from the
  previous time step, relative to the activation cost).
- `COMAKTool.relax_acceleration_constraints` is `true`: the acceleration constraints get a penalized
  slack variable (`qp_slack_penalty`), so the optimization always has a solution.
- `secondary_velocity_carryover_to_next_step` is 0.
- `Smith2018ArticularContactForce.max_strain` (new, 0.8): see Fixes. The contact energy of the nonlinear
  variable property model was about 230 times too small, so `contact_energy_weight` 1 of the EMG examples
  acted like 0.004. With the correct energy the EMG examples use 0.1 (weight 1 dominated: peak joint
  contact force -25%, secondary coordinates moved by 4 mm; 0.01 reproduces the old behaviour).

## Fixes

- Reserve actuator bounds and gradient, non-finite results, fallback to the previous time step.
- Settle simulations: plateau stopping, unit handling, bounded loops.
- Secondary constraint functions: spline point spacing, 0.8 patella multiplier removed.
- Contact: OBB ray query returned the wrong triangle, parallel tolerance, force aggregation per body.
- Nonlinear contact (`Smith2018ArticularContactForce`, `elastic_foundation_formulation` nonlinear):
  the potential energy of the variable property model used the stiffness divided by the thickness (energy
  and its COMAK gradient about 230 times too small); the lumped model gave NaN and the variable model a
  wrong, finite pressure when the overlap reached the layer thickness. The pressure is now solved with a
  Newton iteration (a few iterations, no tolerance noise), the energy is the closed form integral of the
  pressure (`dE/dd = A P`), and above the strain `max_strain` (0.8) of a layer the pressure continues
  linearly with the same slope, so pressure and energy are finite and smooth for any overlap (warning once).
  Pressures below the cap are unchanged (1e-10 on the EMG walking frames).
- Invalid contact inputs are errors: Poisson's ratio of 0.5 or more, elastic modulus or thickness of 0 or
  less, an unknown `elastic_foundation_formulation`, `max_strain` outside (0, 1). A COMAK contact energy
  derivative that is not finite is ignored with a warning.
- Input kinematics checks and smoothing spline.

## New options

- `enforce_secondary_coordinate_range` in `COMAKTool` and `COMAKInverseKinematicsTool`.
- `jacobian_recompute_interval` (values above 1 diverge on these models), `ignore_secondary_speed_term`,
  `normalize_constraint_rows`, `secondary_step_penalty_weight`.
- Timing of the linearization columns in the COMAK log.

## Settings, setups and old files

- Setup files and scripts have `setup_version` (0 = not declared). A COMAKTool or COMAKInverseKinematicsTool
  without it is an older setup: when it is loaded from XML (or run from a script) it is upgraded with a
  warning and a notice at the start of the run: every `max_change` limit is removed, the settle threshold
  is set to the default, the plateau check is switched on, and the central difference Jacobian is used.
  `upgrade_old_setup` false keeps the old values. New scripts set `set_setup_version(1)`.
- `saveUpgradedSetup(where)` (MATLAB, Python) and `opensim-cmd run-tool <xml> --save-upgraded[-to <where>]`
  save the upgraded setup: `<name>_v1.xml` next to the file, a path, a folder, or `inplace`.
- Settle: one `settle_threshold` (rotation and translation) with a plateau check
  (`settle_plateau_tolerance` 0.1) on by default, in COMAKTool (default 5e-4) and
  COMAKInverseKinematicsTool (`secondary_constraint_sim_settle_threshold`, default 1e-4: the sweep
  builds the coupling functions from the settled values, and 5e-4 changed them by up to 0.05 rad
  at the patella compared to the original 1e-4).
- The COMAK run only covers the time where the coordinates, the ground reaction data and the EMG (with
  the electromechanical delay) all have data: the start is moved later and the stop earlier, with a
  warning. A GRF file that is set but cannot be read is an error.
- `COMAKTool` can be run again, or cloned after a run: the muscle lists, cost function vectors and result
  tables are reset (a clone used to double the muscles and fail in the second pass).
- A duplicate COMAKCostFunctionParameter for a muscle is an error; muscles without one use the defaults
  with one warning. The MuscleCoordination example had `glmed2_r` pointing to `glmed1_r`.
- `udot_tolerance` and `udot_worse_case_tolerance` are multiples of the force tolerance with the force
  criterion (the tolerance used to cancel out); NaN residuals do not count as converged; the tolerances
  must be positive; the constraint relaxation is logged only above 1e-3; the log has the time split
  per time step and for the run (the linearization is 92-94% of the time).

## EMG

- `emg_file` (.sto or .mot, normalised EMG) is read, with `emg_mapping` per muscle in the
  COMAKCostFunctionParameter: terms separated by semicolons, `label` or `weight*label`
  (`0.5*vasmed; 0.5*vaslat`), labels with spaces, case and `_` ignored. Values above 1 are set to 1.
- `emg_electromechanical_delay` (0.04 s) and `emg_delay` per muscle: the activation at t follows the EMG at
  t - delay.
- `is_emg_assisted` without any EMG is the same as false.
- The EMG-assisted example reads `DM_bouncy1_1_emg.mot`, starts at 0.04 s, and all three muscle force
  switches are off (it used force-length-velocity before).

## Muscle force models (all off by default)

- `use_force_length_velocity` (was `use_muscle_physiology`, still accepted): the force-length-velocity
  relationship and the pennation angle. The old code did not multiply by cos(pennation), so it
  overestimated the tendon force by 1/cos(pennation) (up to 34%).
- `use_passive_muscle_force`: adds the passive fiber force (experimental).
- `use_tendon_compliance`: elastic tendon, solved as the quasi-static equilibrium at each iteration and
  linearized in the activation (experimental). Checked against the Millard muscle: the actual compliant
  force is within 0.1% (median) of the force COMAK used.
- The model muscles get a rigid tendon and no activation dynamics when force-length-velocity or passive
  force is on (compliant fiber lengths are a state that would not be equilibrated).
- `JointMechanicsTool.comak_setup_file`: the muscle force model is read from the COMAK setup so the joint
  mechanics uses the forces COMAK used (the examples set it). With the default model the result is the
  same as before. Walking with the 5 models: knee contact peak 3120 N (rigid), 2613 (force-length-velocity),
  3248 (passive), 2877 (both), 2966 (tendon compliance); EMG trial 2573, 2400, 2697, 2549, 2384.

## Joint mechanics inside COMAK

- `COMAKTool.run_joint_mechanics` (default true) runs the JointMechanicsTool right after COMAK with the
  settings of the `joint_mechanics` section of the COMAK setup, so no extra XML is needed. Settings that
  are left empty come from COMAK: model, input states (the states of this run), start and stop time,
  the muscle force model (force-length-velocity, passive force and scale, tendon compliance), and the
  results file basename (the trial name: results_prefix or the name of the coordinates file).
- Without a results directory the results go to `<folder of the COMAK results>/joint_mechanics/<trial name>`
  (created if needed; existing files in it are not deleted). By default the vtp files and the ForceReporter
  forces .sto file are written, and no h5 file.
- A setup of an older version (no setup_version) does not run it: the upgrade sets it to false and says so.
- The examples use it: the separate joint mechanics code is commented out at the end, and the settings that
  can be changed are listed (commented) before `comak.run()`.

## Inverse kinematics (COMAKInverseKinematicsTool)
- Two-pass smoothed IK (`smooth_ik_second_pass`, on by default): the first solution is low-pass filtered
  (`smooth_ik_lowpass_frequency`, use the `lowpass_filter_frequency` of the COMAK input kinematics; -1 uses
  40 Hz) and the second solution fits the markers with a soft pull (`smooth_ik_weight`, 1) of each free coordinate
  towards it. The first solution is written as `<output>_pass1.mot`. Walking test: the frame-to-frame noise
  (second difference) dropped about 30 times, the marker RMS did not change.
- `secondary_constraint_sim_settle_threshold` default 1e-4 (the COMAK settle threshold stays 5e-4): at 5e-4
  the patella coupling functions differed from the original by up to 0.05 rad.
- The constrained coordinates (e.g. patella flexion) start the sweep from their settled values (they restarted
  from the model defaults).
- A model that cannot be assembled at the first frame is an error (it was ignored).
- A warning if the coupled coordinate (knee flexion) of the trial leaves the sweep range (default 0 to 100
  deg), where the constraint functions are extrapolated.
- `settling_sim_integrator_accuracy` below 1e-9 (the default 1e-10) uses the sweep accuracy; this is now logged
  and the property is no longer overwritten.
- The coordinate file is converted from degrees only if it is in degrees (`inDegrees=yes`).
- The sweep states are only kept in memory when the simulation results are printed.
- Units: JAM works in SI (meters, radians or degrees, Newtons). Marker .trc files are converted from their Units
  header by OpenSim. The .mot/.sto coordinate files have no length unit, so a translational coordinate above 5
  is reported as probably mm or cm (COMAKTool and COMAKInverseKinematicsTool).
- If the model cannot be assembled at the first frame of the IK, the accuracy is relaxed in decades up to 1e-7
  before it is an error (first and second pass).
- The sweep covers [min, max] = `secondary_coupled_coordinate_start_value` (default -10 deg) to
  `secondary_coupled_coordinate_stop_value` (default 110 deg). If min is below 0 and max above, the model is settled at 0
  and two sweeps start from that settled state: one down to min and one up to max; they are joined into one constraint
  function, so a trial that goes slightly below 0 (knee hyperextension) is not extrapolated and the part above 0 is the
  same as before. With min at or above 0 it is the sweep as before (settled at min, swept up). The examples use -10 and 110.
