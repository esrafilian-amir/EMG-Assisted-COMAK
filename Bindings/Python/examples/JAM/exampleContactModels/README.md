# exampleContactModels

Runs the walking models with the default contact settings and with the tuned contact settings, and reports the
contact results: COMAK inverse kinematics and COMAK, the joint mechanics tool (contact vtp files), the contact
force, center of pressure, pressure and contact area of every frame, the cartilage thickness, and one pdf report.

Python: `exampleContactModels.py`. MATLAB: `Bindings/Java/Matlab/examples/JAM/exampleContactModels/exampleContactModels.m`
(same workflow and the same results layout).

```
python exampleContactModels.py                       # all models, 3 in parallel (--jobs)
python exampleContactModels.py --models lenhart2015,lenhart2015_contact_tuned
python exampleContactModels.py --stage report        # only the report, from the results of an earlier run
```

Models (the tuned ones are next to the originals in `Bindings/Java/Matlab/examples/JAM/models/healthy`):

| name | model | contact |
|---|---|---|
| `lenhart2015` | lenhart2015 | original |
| `lenhart2015_contact_tuned` | lenhart2015_contact_tuned | `distance_field` in `tf_contact` and `pf_contact`, thickness smoothing 10 |
| `smith2019` | smith2019 (menisci) | original (linear) |
| `smith2019_contact_tuned` | smith2019_contact_tuned | same as above, menisci contacts ray cast |
| `smith2019_contact_tuned_nonlinear` | smith2019_contact_tuned_nonlinear | same, nonlinear in all six contacts |

The workflow reuses the example scripts `exampleWalking` and `exampleWalkingMeniscus` (the model and the paths are
replaced, the visualizer is off, and the joint mechanics settings are added), so it stays identical to them.

## Results (written next to this script, in `results`)

```
results/
├── contact_report.pdf                       the report
├── summary.csv                              run times and peak contact results of every model
└── <model>/                                 one folder per model
    ├── inputs/                              COMAK inverse kinematics and COMAK setup files (xml)
    ├── comak-inverse-kinematics/            sweep simulation, constraint functions, constrained model, IK motion
    ├── comak/                               COMAK results: states, activations, forces, convergence
    ├── joint-mechanics/                     joint mechanics: contact vtp files (pressure, thickness, ...) for ParaView, forces .sto
    ├── contact-analysis/                    contact_results.csv (per frame), thickness_<mesh>.npz, model_settings.json
    └── log/                                 pipeline.log, pipeline_info.json, the script that was run
```

How the contact settings were chosen: `scripts/contact_tuning/RESULTS.md`.
