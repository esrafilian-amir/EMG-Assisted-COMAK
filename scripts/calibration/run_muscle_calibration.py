"""EMG muscle calibration driven by a setup file.

    python run_muscle_calibration.py <muscle_calibration_setup.xml> --install <opensim install>

Inputs: the setup file (see muscle_calibration_setup.xml) and a finished COMAK run,
whose COMAK setup gives the model, the EMG file, its mapping and delay, the external
loads and the results (the states, which hold the secondary coordinates found by the
COMAK sweep, and the muscle forces of the muscles without EMG).

Outputs, in the results directory of the setup:
  <output_model_file>   the model with the calibrated max_isometric_force,
                        optimal_fiber_length and tendon_slack_length; a relative path
                        is relative to the folder of the original model (the contact
                        meshes and geometry are found next to it)
  <output_emg_file>     the EMG with the calibrated shape, to use as emg_file of COMAK
                        (the electromechanical delay is still applied by COMAK)
  calibration_report.json, geometry.npz (the cache, reused when it exists)
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate  # noqa: E402
import extract_geometry  # noqa: E402


def text(node, tag, default=None):
    child = node.find(tag)
    return child.text.strip() if child is not None and child.text else default


def pair(node, tag, default):
    s = text(node, tag)
    return tuple(float(v) for v in s.split()) if s else default


def resolve(path, base):
    return path if os.path.isabs(path) else os.path.normpath(os.path.join(base, path))


def parse_mapping(s):
    terms = []
    for term in s.split(';'):
        term = term.strip()
        if not term:
            continue
        w, ch = (term.split('*', 1) if '*' in term else ('1', term))
        terms.append((float(w), ch.strip()))
    return terms


def read_setup(setup_file):
    base = os.path.dirname(os.path.abspath(setup_file))
    tool = ET.parse(setup_file).getroot().find('MuscleCalibrationTool')
    cfg = {'base': base, 'name': tool.get('name', 'muscle_calibration')}
    cfg['comak_setup'] = resolve(text(tool, 'comak_setup_file'), base)
    # relative paths inside the COMAK setup are relative to the folder COMAK was run from
    cfg['comak_cwd'] = resolve(text(tool, 'comak_working_directory',
                                    os.path.join(os.path.dirname(cfg['comak_setup']), '..')), base)
    cfg['results'] = resolve(text(tool, 'results_directory', './muscle-calibration'), base)
    cfg['out_model'] = text(tool, 'output_model_file', 'calibrated_model.osim')
    cfg['out_emg'] = text(tool, 'output_emg_file', 'calibrated_emg.sto')
    cfg['coordinates'] = text(tool, 'coordinates', 'hip_flex_r knee_flex_r ankle_flex_r').split()
    cfg['suffix'] = text(tool, 'muscle_side_suffix', '_r')
    cfg['bounds'] = {k: pair(tool, k + '_bounds', calibrate.DEFAULT_BOUNDS[k])
                     for k in calibrate.PARAMS}
    cfg['groups'] = []
    for g in tool.find('MuscleCalibrationGroups').findall('MuscleCalibrationGroup'):
        # bounds written inside a group replace the global ones for that group
        own = {k: pair(g, k + '_bounds', None) for k in calibrate.PARAMS}
        cfg['groups'].append({'name': g.get('name'), 'muscles': text(g, 'muscles', '').split(),
                              'fit': text(g, 'calibrate', '').split(),
                              'bounds': {k: v for k, v in own.items() if v}})
        for p in cfg['groups'][-1]['fit']:
            if p not in calibrate.PARAMS:
                raise SystemExit(f"group {g.get('name')}: unknown parameter '{p}' "
                                 f"(use {calibrate.PARAMS})")
    return cfg


def read_comak_setup(cfg):
    root = ET.parse(cfg['comak_setup']).getroot()
    tool = root.find('COMAKTool')
    cwd = cfg['comak_cwd']
    c = {'model': resolve(text(tool, 'model_file'), cwd),
         'grf': resolve(text(tool, 'external_loads_file'), cwd),
         'emg': resolve(text(tool, 'emg_file'), cwd),
         'delay': float(text(tool, 'emg_electromechanical_delay', '0.04')),
         'results': resolve(text(tool, 'results_directory'), cwd),
         'prefix': text(tool, 'results_prefix', 'comak'), 'map': {}}
    for p in tool.iter('COMAKCostFunctionParameter'):
        mapping = text(p, 'emg_mapping', '')
        if mapping and float(text(p, 'emg_gamma_weight', '0')) > 0:
            c['map'][text(p, 'actuator').split('/')[-1]] = parse_mapping(mapping)
    return c


def write_outputs(cfg, cm, pr, x):
    import opensim as osim
    q = pr.unpack(x)
    new = {}
    model = osim.Model(cm['model'])
    muscles = model.updMuscles()
    for j, m in enumerate(pr.emg_muscles):
        k = pr.names.index(m)
        mus = muscles.get(m)
        old = (mus.get_max_isometric_force(), mus.get_optimal_fiber_length(),
               mus.get_tendon_slack_length())
        mus.set_max_isometric_force(old[0] * q['strength'][j])
        mus.set_optimal_fiber_length(old[1] * q['fiber_length'][j])
        mus.set_tendon_slack_length(old[2] * q['tendon_slack'][j])
        new[m] = {'max_isometric_force': [old[0], mus.get_max_isometric_force()],
                  'optimal_fiber_length': [old[1], mus.get_optimal_fiber_length()],
                  'tendon_slack_length': [old[2], mus.get_tendon_slack_length()]}
    model.finalizeConnections()
    os.makedirs(cfg['results'], exist_ok=True)
    # The contact meshes and the geometry are found relative to the model file, so by
    # default the new model goes into the folder of the original one.
    model_path = resolve(cfg['out_model'], os.path.dirname(cm['model']))
    model.printToXML(model_path)

    # EMG with the calibrated shape: one A per channel (the mean over the muscles it drives)
    table = osim.TimeSeriesTable(cm['emg'])
    labels = [str(l) for l in table.getColumnLabels()]
    data = table.getMatrix().to_numpy().copy()
    shape_of = {}
    for j, m in enumerate(pr.emg_muscles):
        for _, ch in cm['map'][m]:
            shape_of.setdefault(ch.lower(), []).append(q['emg_shape'][j])
    for ch, vals in shape_of.items():
        col = [l.lower() for l in labels].index(ch)
        if np.ptp(vals) > 1e-6:
            print(f'warning: channel {ch} drives muscles with different shapes, using the mean')
        data[:, col] = calibrate.shape_emg(np.clip(data[:, col], 0, 1), float(np.mean(vals)))
    out = osim.TimeSeriesTable()
    times = list(table.getIndependentColumn())
    for k, t in enumerate(times):
        out.appendRow(float(t), osim.RowVector(data[k, :].tolist()))
    out.setColumnLabels(labels)
    emg_path = os.path.join(cfg['results'], cfg['out_emg'])
    osim.STOFileAdapter.write(out, emg_path)
    return model_path, emg_path, new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('setup')
    ap.add_argument('--install', required=True, help='OpenSim install with the Python bindings')
    ap.add_argument('--recompute-geometry', action='store_true')
    a = ap.parse_args()

    cfg = read_setup(a.setup)
    cm = read_comak_setup(cfg)
    os.makedirs(cfg['results'], exist_ok=True)
    cache = os.path.join(cfg['results'], 'geometry.npz')
    states = os.path.join(cm['results'], cm['prefix'] + '_states.sto')
    force = os.path.join(cm['results'], cm['prefix'] + '_force.sto')

    if a.recompute_geometry or not os.path.exists(cache):
        print('Stage 1: geometry and inverse dynamics from', states)
        extract_geometry.main(argparse.Namespace(
            install=a.install, model=cm['model'], states=states, grf_setup=cm['grf'],
            out=cache, suffix=cfg['suffix'], exclude='Muscles'))
    else:
        print('Stage 1: using the cache', cache)

    print('Stage 2: calibration')
    pr = calibrate.Problem(cache, cm['emg'], force, a.install, cm['delay'], cfg['coordinates'],
                           cm['map'], cfg['groups'], cfg['bounds'])
    before = pr.report(pr.x0)
    x, sol = pr.fit()
    after = pr.report(x)

    print('%-14s %22s %22s' % ('', 'before (rmse Nm, R2)', 'after (rmse Nm, R2)'))
    for n in pr.dofs:
        print('%-14s %10.1f  %8.2f %12.1f  %8.2f' % (n, before[n]['rmse_Nm'], before[n]['r2'],
                                                     after[n]['rmse_Nm'], after[n]['r2']))
    print('%-24s %s' % ('group', ' '.join('%12s' % p for p in calibrate.PARAMS)))
    for name, v in pr.group_values(x).items():
        print('%-24s %s' % (name, ' '.join('%12.3f' % v[p] for p in calibrate.PARAMS)))

    print('Stage 3: writing the calibrated model and EMG')
    model_path, emg_path, new = write_outputs(cfg, cm, pr, x)
    print('  model:', model_path)
    print('  EMG  :', emg_path)
    json.dump({'setup': os.path.abspath(a.setup), 'coordinates': pr.dofs, 'before': before,
               'after': after, 'groups': pr.group_values(x), 'muscles': new,
               'bounds_hit': [f"{g['name']}.{p}" for gi, g in enumerate(pr.groups)
                              for pi, p in enumerate(calibrate.PARAMS)
                              if pr.free[gi * len(calibrate.PARAMS) + pi] and
                              (abs(x[gi * len(calibrate.PARAMS) + pi] - pr.lb[gi * len(calibrate.PARAMS) + pi]) < 1e-3 or
                               abs(x[gi * len(calibrate.PARAMS) + pi] - pr.ub[gi * len(calibrate.PARAMS) + pi]) < 1e-3)]},
              open(os.path.join(cfg['results'], 'calibration_report.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
