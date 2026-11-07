# Contact settings: how far can they be changed, and the Atlas sweep stall

Measured with the scripts in this folder (RelWithDebInfo build, OpenSim 4.6 JAM, Python bindings).
The ray cast is not the ground truth, so the criterion is 10% of the default result (ray cast, no smoothing); the 5% results are listed too.
Metrics: trial peak and RMS error of the contact force and of the maximum pressure, and the center of pressure (distance to the default
one in the frames above 10% of the peak force, as % of the contact patch diameter, and in mm).

## Recommended contact settings

| setting | value | why |
|---|---|---|
| `Smith2018ContactMesh.thickness_smoothing_iterations` (variable thickness cartilage) | 10 (within 5% up to 12, within 10% up to 30) | removes the triangle to triangle jumps of the measured thickness; peak force within 1.6%, peak pressure within 3.9%, center of pressure within 1.0% of the patch |
| `contact_method` of the cartilage pairs (`tf_contact`, `pf_contact`) | `distance_field`, `field_voxel_size` 0.001 (within 10% up to 2 mm), `field_edge_taper_width` 0.003 (within 10% up to 6 mm) | smoother force, no ray misses, IK sweep 3.7 times faster in the walking model (176 s instead of 651 s; the whole pipeline takes 6 min instead of 16 min), 1.4 to 1.5 times faster in the meniscus models; end to end within 10% (below) |
| `contact_method` of the meniscus contacts | `ray_cast` (the distance field also works end to end within 10% in the nonlinear model, but a frame can have a center of pressure error of 12% of the patch and 8 to 11% force error at the same states) | |
| `Smith2018ArticularContactForce.max_proximity` | larger than the deepest penetration expected, several times the layer thickness (cartilage 20 mm; a bone backstop contact 5 mm or more) | a triangle deeper than `max_proximity` gets no contact: the force drops to zero abruptly. This stalled the Atlas sweep (below) |

## Models saved with the recommended settings (next to the originals, other file name)

- `models/healthy/lenhart2015/lenhart2015_contact_tuned.osim`: smoothing 10 on the three cartilage meshes, `distance_field` (voxel 1 mm, taper 3 mm) in `tf_contact` and `pf_contact`.
- `models/healthy/smith2019/smith2019_contact_tuned.osim` (menisci, linear contact as shipped): the same; the four meniscus contacts stay ray cast.
- `models/healthy/smith2019/smith2019_contact_tuned_nonlinear.osim`: the same with `elastic_foundation_formulation` nonlinear in all six contacts.
- `OpenSim_Model_Atlas/Unscaled_Model_contact_tuned.osim`: `max_proximity` of `pf_bone_contact` 0.005 instead of 0.0005 (the sweep runs, see below).

## End to end: whole pipeline (IK sweep + COMAK) with the tuned settings against the default

Each model is run with the default contact and with the tuned settings; the contact outputs of the tuned run are compared with
the default run (peak / RMS error in %, center of pressure mean as % of the patch). All frames converged in every run.

| model | default | tuned | force | max pressure | center of pressure |
|---|---|---|---|---|---|
| walking, lenhart2015 (nonlinear, 2 contacts) | IK 651 s, COMAK 306 s | IK 176 s, COMAK 187 s | 3.5 / 2.2 (`tf`), 2.1 / 1.0 (`pf`) | 6.5 / 3.8, 5.3 / 2.8 | 0.9%, 1.0% (0.23 mm, 0.14 mm) |
| walking with menisci, smith2019 (linear, 6 contacts) | IK 1325 s, COMAK 467 s | IK 881 s, COMAK 323 s | at most 4.3 / 3.0 (all six) | at most 3.0 / 2.6 | |
| walking with menisci, nonlinear, distance field on all six, smoothing 10 | IK 1164 s, COMAK 372 s | IK 589 s, COMAK 294 s | at most 6.3 / 3.4 | at most 6.5 / 3.6 | mean at most 3.7% (0.52 mm), worst frame 12.2% |
| walking with menisci, nonlinear, distance field on the cartilage pairs only, smoothing 10 | IK 1164 s, COMAK 372 s | IK 863 s, COMAK 329 s | 2.5 / 1.6 (`tf`), 1.3 / 0.6 (`pf`), menisci at most 4.7 | at most 5.6 | mean at most 2.9% |
| walking with menisci, nonlinear, ray cast with smoothing 10 | IK 1164 s, COMAK 372 s | IK 1144 s, COMAK 374 s | at most 0.6 / 0.5 | at most 2.6 | at most 0.5% |

## Nonlinear contact with menisci (smith2019 with `elastic_foundation_formulation` nonlinear in all six contacts)

Same states (default run of the nonlinear model), peak / RMS errors in %:

| setting | tf force | pf force | meniscus forces (max) | max pressure (max over contacts) | center of pressure mean (max over contacts) | within 5% / 10% |
|---|---|---|---|---|---|---|
| smoothing 10 | 0.3 / 0.4 | 2.0 / 0.8 | 0.6 | 3.3 | 0.2% | yes / yes |
| smoothing 20 | 0.7 / 0.7 | 3.7 / 1.4 | 1.0 | 6.3 | 0.4% | no / yes |
| smoothing 30 | 1.0 / 1.1 | 5.2 / 2.0 | 1.4 | 8.8 | 0.6% | no / yes |
| distance field, cartilage pairs only | 4.1 / 2.2 | 4.9 / 2.0 | 0 (ray cast) | 2.0 | 0.6% | yes / yes |
| distance field, cartilage pairs only + smoothing 10 | 3.8 / 1.9 | 3.1 / 1.3 | 0.6 | 2.2 | 0.8% | yes / yes |
| distance field, all six contacts | 4.1 / 2.2 | 4.9 / 2.0 | 4.0 to 10.8 | 2.0 | 10.9% (tibia, medial meniscus), 1.6 mm | no / no |

The distance field of the meniscus contacts has a cache per contact force object: two contacts with the same target mesh and a different
`max_proximity` (`tf_contact` 0.02 and the femur-meniscus contacts 0.01 share the femur cartilage) used to rebuild the field at every call.

## Center of pressure (walking trial, lenhart2015, same states)

The center of pressure of the workable settings stays within 1.1% of the contact patch diameter on average (at most 0.3 mm):
smoothing up to 30: 0.1 to 0.8% (0.05 to 0.10 mm); distance field voxel 1 mm: 0.2% (`tf`), 0.6% (`pf`); voxel 2 mm: 2.4% (`pf`, 0.31 mm);
voxel 3 mm: 4.7% (0.60 mm); taper 10 mm: 1.0% (`tf`); taper 20 mm: 5.3% (1.4 mm).

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
With the distance field in the four meniscus contacts too (replay on the same states, linear model) the peak force is off by 3.8 to 10.9%; end to end (nonlinear model, above) it stays within 10%. The meniscus contacts are therefore left on the ray cast in the saved models.

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
