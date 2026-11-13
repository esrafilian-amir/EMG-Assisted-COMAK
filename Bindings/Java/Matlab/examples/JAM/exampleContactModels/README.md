# exampleContactModels

Runs the walking models with the default contact settings and with the tuned contact settings, and reports the
contact results: COMAK inverse kinematics and COMAK, the joint mechanics tool (contact vtp files), the contact
force, center of pressure, pressure and contact area of every frame, the cartilage thickness, and one pdf report.

MATLAB: `exampleContactModels.m`. Python: `Bindings/Python/examples/JAM/exampleContactModels/exampleContactModels.py`
(same workflow and the same results layout).

Run `exampleContactModels.m` (OpenSim configured for MATLAB, as for the other examples). Options at the top of the
script, or set the variable before running it:

```
models_to_run = {'lenhart2015','lenhart2015_SDF'};   % default {}: all five models
stages = 'report';        % 'all' (default), 'pipeline', 'analyze' or 'report'
skip_existing = true;     % do not run again a model that already has a COMAK states file
results_dir = 'C:/my/results';   % default: the results folder next to the script
exampleContactModels
```

Models: `lenhart2015`, `lenhart2015_SDF` (distance field in `tf_contact` and `pf_contact`, thickness smoothing 10),
`smith2019`, `smith2019_SDF`, `smith2019_SDF_nonlinear` (the tuned models are next to the originals in
`../models/healthy`). The workflow reuses the scripts `exampleWalking.m` and `exampleWalkingMeniscus.m` (the model and the paths are
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
    ├── contact-analysis/                    contact_results.csv (per frame), thickness_<mesh>.mat, model_settings.json
    └── log/                                 pipeline.log, pipeline_info.json, the script that was run
```

How the contact settings were chosen: `scripts/contact_tuning/RESULTS.md`.
