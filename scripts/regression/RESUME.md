# Where the work stands (written before the machine was switched off, 2026-11-03)

Nothing is pushed. Last local commit: `429f3e349` (settle threshold 1e-3 and example fixes).
Everything after that is UNCOMMITTED in the working tree and NOT yet verified by a full build + tests.

## Uncommitted changes since 429f3e349 (all in `OpenSim/JAM`, the examples, and `scripts/regression`)
- `COMAKTool` / `COMAKInverseKinematicsTool`: `setup_version` (0 = undeclared) and `upgrade_old_setup`;
  upgrade at XML load or at `run()`; old-version notice banner; `saveUpgradedSetup(where)` and
  `opensim-cmd run-tool --save-upgraded[-to]`; settle threshold 5e-4, plateau 0.1 by default; IK stop value 100.
- Review fixes: `udot_tolerance` cancellation bug, NaN residuals, tolerance validation, duplicate cost-function
  entry check, property doc defaults (max_iterations 50, model_assembly_accuracy 1e-12), `emg_file` now read.
- Bug fix: `initialize()` cleared lists/tables so a second run or a clone does not double muscles (MuscleCoordination).
  Result tables (`_result_*`, `_convergence`) are reset too (last edit, not yet built).
- EMG: `emg_file` (.sto/.mot) + per-muscle `emg_mapping` ("0.5*vasmed; 0.5*vaslat", labels with spaces ok),
  `emg_electromechanical_delay` (0.04 s), per-muscle `emg_delay`; run time cut to the range where coordinates,
  GRF and EMG (+delay) all have data; desired activation above 1 set to 1 with a warning;
  `is_emg_assisted` without any EMG behaves as off.
- `use_force_length_velocity` (renamed from `use_muscle_physiology`, which stays as deprecated alias), pennation
  factor fixed (old code overestimated tendon force by 1/cos(pennation)); `use_passive_muscle_force` (off).
- Examples (MATLAB + Python): EMG example reads `DM_bouncy1_1_emg.mot`, start 0.04 s, mapping per muscle;
  MuscleCoordination example `glmed2_r` typo fixed.

## Not done yet
1. Build the latest sources (the last builds were interrupted): `scripts/regression/build_install.ps1 -Prefix ...-install-settleNN -SeedFrom <previous> -Targets osimJAM,_jam,JavaBindings,opensim-cmd`
   (touch the changed headers first; stale Java wrappers gave a heap-corruption crash in the EMG tests).
2. Smoke tests: `python smoke_version.py <install> <old comak_settings.xml> <old ik xml>` (scratchpad) and the EMG tests
   `run_emg_tests.sh` (point `NEW=` at the new install).
3. Walking comparison `run_passive.sh`: force-length-velocity off/on x passive force off/on (needs reuse-IK run `S9_walking`).
4. Tendon compliance (`use_tendon_compliance`, off by default): idea is to linearize the quasi-static tendon force
   about the previous activation each iteration (slope -> optimal force, rest -> force offset). A Python prototype
   `tendon_proto.py` (scratchpad) was written but not run.
5. Final full review of the code, then a local commit (never push).
6. Old runs still pending a rerun on the final install: MuscleCoordination (`S15_...` showed the example typo and the table bug).

## Results so far
- Examples on the settle9 build: walking, EMG, meniscus, Patella_Constrained, passive flexion (both), anterior laxity,
  sensitivity, visualize kinematics: all rc 0, no errors (visualize kinematics logs one harmless mesh warning).
- Old XMLs (walking+IK, EMG, meniscus) and old unmodified scripts (walking, EMG) run on the new build; the old meniscus
  script fails only at its own `jnt_mech.set_verbose(0)`.
- Meniscus settle: 5e-4 threshold, plateau only acts late; damping, mass and max_proximity do not change the zig-zag.
