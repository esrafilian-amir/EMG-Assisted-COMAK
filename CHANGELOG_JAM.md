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

## Fixes

- Reserve actuator bounds and gradient, non-finite results, fallback to the previous time step.
- Settle simulations: plateau stopping, unit handling, bounded loops.
- Secondary constraint functions: spline point spacing, 0.8 patella multiplier removed.
- Contact: OBB ray query returned the wrong triangle, parallel tolerance, force aggregation per body.
- Input kinematics checks and smoothing spline.

## New options

- `enforce_secondary_coordinate_range` in `COMAKTool` and `COMAKInverseKinematicsTool`.
- `jacobian_recompute_interval` (values above 1 diverge on these models), `ignore_secondary_speed_term`,
  `normalize_constraint_rows`, `secondary_step_penalty_weight`.
- Timing of the linearization columns in the COMAK log.
