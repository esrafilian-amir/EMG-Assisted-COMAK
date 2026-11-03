"""Stage 1 of the EMG muscle calibration: geometry and inverse dynamics cache.

Reads a finished COMAK run (its states file holds every coordinate, so the
secondary knee and patella coordinates are the ones found by the COMAK sweep)
and writes one .npz with, per frame and right-leg muscle:
  length, lengthening speed, moment arm about each primary coordinate,
and the inverse dynamics moments of the primary coordinates.
Nothing here depends on the muscle parameters, so it is done once.

    python extract_geometry.py --install <install> --model <osim> --states <states.sto>
        --grf-setup <grf_Setup.xml> --out geometry.npz
"""
import argparse
import os
import sys

import numpy as np

PRIMARY = ['hip_flex_r', 'hip_add_r', 'hip_rot_r', 'knee_flex_r', 'ankle_flex_r']


def parse():
    p = argparse.ArgumentParser()
    p.add_argument('--install', required=True)
    p.add_argument('--model', required=True)
    p.add_argument('--states', required=True)
    p.add_argument('--grf-setup', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--suffix', default='_r', help='side suffix of the muscles to keep')
    p.add_argument('--exclude', default='Muscles',
                   help='forces left out of the inverse dynamics (Muscles, or ALL)')
    return p.parse_args()


def main(a=None):
    a = a or parse()
    sys.path.insert(0, os.path.join(a.install, 'sdk', 'python'))
    os.environ['PATH'] = os.path.join(a.install, 'bin') + os.pathsep + os.environ['PATH']
    import opensim as osim

    osim.Logger.setLevelString('Error')
    model = osim.Model(a.model)
    state = model.initSystem()

    states = osim.TimeSeriesTable(a.states)
    labels = list(states.getColumnLabels())
    time = np.array(states.getIndependentColumn())
    data = states.getMatrix().to_numpy()
    names_arr = model.getStateVariableNames()
    model_vars = {names_arr.get(i) for i in range(names_arr.getSize())}
    use_cols = [(i, l) for i, l in enumerate(labels) if l in model_vars]

    muscles = [m for m in model.getMuscles() if m.getName().endswith(a.suffix)]
    names = [m.getName() for m in muscles]
    coords = [model.getCoordinateSet().get(c) for c in PRIMARY]
    n_t, n_m, n_d = len(time), len(muscles), len(coords)

    length = np.zeros((n_t, n_m))
    speed = np.zeros((n_t, n_m))
    arm = np.zeros((n_t, n_m, n_d))
    for k in range(n_t):
        for i, l in use_cols:
            model.setStateVariableValue(state, l, data[k, i])
        model.realizeVelocity(state)
        for j, m in enumerate(muscles):
            length[k, j] = m.getLength(state)
            speed[k, j] = m.getLengtheningSpeed(state)
            for d, c in enumerate(coords):
                arm[k, j, d] = m.computeMomentArm(state, c)

    # parameters of the muscles in the model (the starting point of the calibration)
    par = {key: np.array([getattr(m, f'get_{key}')() for m in muscles]) for key in
           ['max_isometric_force', 'optimal_fiber_length', 'tendon_slack_length',
            'pennation_angle_at_optimal', 'max_contraction_velocity']}

    # inverse dynamics of all coordinates in the states file
    work = os.path.dirname(os.path.abspath(a.out))
    vcols = [l for l in labels if l.endswith('/value')]
    mat = np.stack([data[:, labels.index(l)] for l in vcols], axis=1)
    values = osim.TimeSeriesTable()
    for k in range(n_t):
        values.appendRow(float(time[k]), osim.RowVector(mat[k, :].tolist()))
    values.setColumnLabels([l.split('/')[-2] for l in vcols])
    values.addTableMetaDataString('inDegrees', 'no')
    coord_file = os.path.join(work, 'id_coordinates.sto')
    osim.STOFileAdapter.write(values, coord_file)

    tool = osim.InverseDynamicsTool()
    tool.setModel(model)
    tool.setCoordinatesFileName(coord_file)
    tool.setLowpassCutoffFrequency(-1)
    tool.setStartTime(float(time[0]))
    tool.setEndTime(float(time[-1]))
    tool.setExternalLoadsFileName(a.grf_setup)
    ex = osim.ArrayStr()
    # Only the muscles are excluded: the ligaments, the cartilage contact and the
    # coordinate limits stay, as in COMAK, so what is left is what the muscles must
    # supply. Excluding everything wrongly gives the knee flexion moment a large
    # error (the contact and ligaments act on it in the 6-DOF knee).
    ex.append(a.exclude)
    tool.setExcludedForces(ex)
    tool.setResultsDir(work)
    tool.setOutputGenForceFileName('id_moments.sto')
    tool.run()
    idt = osim.TimeSeriesTable(os.path.join(work, 'id_moments.sto'))
    id_labels = list(idt.getColumnLabels())
    id_mat = idt.getMatrix().to_numpy()
    id_time = np.array(idt.getIndependentColumn())
    moments = np.zeros((n_t, n_d))
    for d, c in enumerate(PRIMARY):
        col = [i for i, l in enumerate(id_labels) if l in (c, c + '_moment', c + '_force')][0]
        moments[:, d] = np.interp(time, id_time, id_mat[:, col])

    np.savez(a.out, time=time, names=np.array(names), primary=np.array(PRIMARY),
             length=length, speed=speed, arm=arm, id_moments=moments,
             **{k: v for k, v in par.items()})
    print('muscles', n_m, 'frames', n_t, 'wrote', a.out)


if __name__ == '__main__':
    main()
