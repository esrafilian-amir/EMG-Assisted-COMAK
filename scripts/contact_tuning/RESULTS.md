# Contact settings: how far can they be changed, and the Atlas sweep stall

Measured with the scripts in this folder (RelWithDebInfo build, OpenSim 4.6 JAM, Python bindings).

## Recommended contact settings

| model | setting | value | why |
|---|---|---|---|
| any model with variable thickness cartilage | `Smith2018ContactMesh.thickness_smoothing_iterations` | 10 (limit 12) | peak force within 1.6%, peak pressure within 3.9% of the unsmoothed result in the walking trial; IK sweep and COMAK converge (walking and meniscus models) |
| any | `Smith2018ArticularContactForce.max_proximity` | larger than the deepest penetration expected, several times the layer thickness (cartilage 20 mm is fine; a bone backstop contact: 5 mm or more) | a triangle deeper than `max_proximity` gets no contact: the force drops to zero abruptly. This stalled the Atlas sweep (see below) |
| any | `contact_method` | `ray_cast` (default) | `distance_field` differs by about 4% in peak force even with a fine grid, and moves the equilibrium of COMAK enough to leave the 5% (peak pressure 6.5%); it is also 8 to 11% off for the meniscus contacts |
| optional (cartilage pairs only) | `contact_method` `distance_field`, `field_voxel_size` | 0.001 or smaller (limit: 1 mm) | smoother force (derivative disagreement 2.0% against 4.6% at the 95th percentile), 2 to 3 times faster IK sweep |
| optional | `field_edge_taper_width` | 0.003 (limit 0.004) | |

## Models saved with the recommended settings (next to the originals, other file name)

- `models/healthy/lenhart2015/lenhart2015_contact_tuned.osim`: `thickness_smoothing_iterations` 10 on the three cartilage meshes
  (nothing else changed; checked with the walking trial above).
- `models/healthy/smith2019/smith2019_contact_tuned.osim` (menisci): same, on the three variable thickness cartilage meshes
  (the meniscus meshes have a constant thickness). Checked with the meniscus walking trial.
- `OpenSim_Model_Atlas/Unscaled_Model_contact_tuned.osim`: `max_proximity` of `pf_bone_contact` 0.005 instead of 0.0005 (the sweep runs, see below).

## Walking trial (exampleWalking, lenhart2015, 1.28 to 1.91 s, 63 frames)

Reference: default settings (ray cast, no smoothing), states of the COMAK run. Peaks: `tf_contact` 3091 N and 11.05 MPa,
`pf_contact` 586 N and 5.18 MPa. Each setting is replayed on the same states (`replay.py`, `analyze.py`).
A setting passes when the trial peak and the RMS over the trial (relative to the peak) of the contact force and of the
maximum pressure are both within 5% for every contact.

Smoothing of the variable thickness (ray cast), errors in % (peak / RMS):

| iterations | tf force | tf max pressure | pf force | pf max pressure | within 5% |
|---|---|---|---|---|---|
| 5 | 0.5 / 0.4 | 1.0 / 1.2 | 0.8 / 0.4 | 2.4 / 1.5 | yes |
| 10 | 1.0 / 0.8 | 1.9 / 1.8 | 1.6 / 0.7 | 3.9 / 2.3 | yes |
| 12 | 1.2 / 0.9 | 2.3 / 2.0 | 1.9 / 0.9 | 4.5 / 2.6 | yes |
| 15 | 1.4 / 1.1 | 2.8 / 2.3 | 2.4 / 1.1 | 5.3 / 3.1 | no |
| 20 | 1.9 / 1.5 | 3.6 / 2.8 | 3.1 / 1.4 | 6.6 / 3.7 | no |
| 30 | 2.7 / 2.2 | 5.2 / 3.8 | 4.6 / 2.1 | 9.0 / 4.9 | no |
| 50 | 4.4 / 3.6 | 8.3 / 5.8 | 7.4 / 3.3 | 13.2 / 7.0 | no |
| 100 | 8.0 / 6.6 | 15.5 / 10.1 | 13.6 / 5.9 | 21.8 / 10.9 | no |

Distance field (peak force error of `tf_contact` / `pf_contact`; the field itself differs from the ray cast by 3 to 4%):

| change | tf | pf | within 5% |
|---|---|---|---|
| voxel 0.5 mm / 0.75 mm / 1 mm | 3.0 / 3.8 / 3.6 | 3.5 / 3.8 / 4.3 | yes |
| voxel 1.5 mm / 2 mm / 3 mm | 5.2 / 5.9 / 9.4 | 5.6 / 7.4 / 11.5 | no |
| taper 0 / 1 / 3 / 4 mm (voxel 1 mm) | 3.6 / 3.7 / 4.2 / 4.7 | 4.3 | yes |
| taper 5 / 6 / 10 / 20 mm | 5.3 / 6.1 / 13.0 / 55.7 | 4.3 / 4.3 / 4.3 / 7.2 | no |
| smoothing 5 / 10 (taper 0) | 3.2 / 2.7 | 3.6 / 2.8 | yes |
| smoothing 30 (taper 0) | 1.0 | 0.1 | no (max pressure 6.5%) |

End to end (IK sweep + COMAK with the setting, peak force / peak pressure error, `tf` and `pf`):

| setting | IK sweep time | tf | pf |
|---|---|---|---|
| default (reference) | 651 s | | |
| ray cast, smoothing 10 | 443 s | 0.2 / 1.6 | 0.3 / 3.4 |
| distance field, taper 3 mm, smoothing 10 | 189 s | 3.5 / 6.5 | 2.1 / 5.3 |
| distance field, taper 6 mm, smoothing 10 | 136 s | 3.8 / 6.6 | 2.0 / 5.2 |
| distance field, taper 6 mm, smoothing 30 | 137 s | 3.9 / 7.6 | 2.2 / 8.4 |

## Meniscus model (exampleWalkingMeniscus, smith2019, 6 contacts)

The unchanged model converges all 63 frames (IK sweep 22 min, COMAK 7.8 min). With smoothing 10 it also does (22 min, 7.5 min).
Replay on the same states, smoothing 10: all six contacts within 3% (peak force and pressure), end to end within 1.4%.
The distance field is not suitable for the meniscus contacts: 8.6 to 10.9% peak force error.

## Atlas knee sweep stall

The COMAK inverse kinematics sweep of the Atlas model stopped at 1.8 to 1.9 s (knee flexion about 38 to 44 degrees):
"corrector convergence test failed repeatedly", step size 1e-13. A step that small means a jump in a force.
`COMAKInverseKinematicsTool` now writes the last output state and the state at the failure to
`sweep_integrator_failure_states.sto` in the results folder; `lineprobe.py` walks a line between the two states and lists
the force elements with the largest step. Found:

1. `pf_bone_contact` (patella bone against femur cartilage) had `max_proximity` 0.5 mm. A penetration beyond it drops the force
   to zero: a jump of 15 N on 285 N. Raising it to 1, 2, 3, 5 or 10 mm lets the whole sweep run (21 min each, ray cast, nothing else changed).
2. The ligament paths wrap over the ellipsoids `Med_Lig_r` and `Capsule_r` (hybrid method); the length of `MCLp1` jumps by 0.43 mm
   at the same knee angle (a jump of a few N for a loaded ligament). With the wrap objects of the ligaments switched off the sweep also runs.
   This is a model issue, not a contact setting, and is not changed.

Not enough on their own (all failed between 1.78 and 2.34 s): thickness smoothing 10 to 30, the distance field, the edge taper (3 and 6 mm),
raising `max_proximity` of `pf_bone_contact` to 20 mm with the ray cast (the settling simulation did not finish).
`tf_contact` itself is not the cause: with `tf_contact` removed the tibia moves away and the problem is not reached, but the
discontinuities above are.

`Smith2018ArticularContactForce` now warns once when a triangle reaches 80% of `max_proximity`.

## Scripts

- `replay.py <model> <states.sto> <out.json> key=value ...`: contact outputs of a set of states for one set of contact settings.
- `analyze.py ref.json var.json ...`: the table above.
- `runvar.sh variants.txt <model> <states> <outdir> [n]`: many variants in parallel.
- `fdtest.py`, `rough.py`: finite difference smoothness of the contact force along a motion.
- `lineprobe.py`: all force elements along the line between two states (for `sweep_integrator_failure_states.sto`).
