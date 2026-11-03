"""Compare the Hill model of calibrate.py with the Millard muscles of OpenSim.

Isometric (zero velocity) tendon force at the same activation and the same pose, for
the muscles with an EMG channel, over a subset of frames. If the two disagree at the
unchanged parameters, the calibration fits the wrong model.

    python validate_hill.py --geometry g.npz --states s.sto --model m.osim --emg e.mot
        --comak-force f.sto --install <install>
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate import Problem, hill_force  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--geometry', required=True)
    p.add_argument('--states', required=True)
    p.add_argument('--model', required=True)
    p.add_argument('--emg', required=True)
    p.add_argument('--comak-force', required=True)
    p.add_argument('--install', required=True)
    p.add_argument('--every', type=int, default=6)
    a = p.parse_args()

    pr = Problem(a.geometry, a.emg, a.comak_force, a.install, 0.04,
                 ['hip_flex_r', 'knee_flex_r', 'ankle_flex_r'])
    import opensim as osim
    model = osim.Model(a.model)
    for m in model.getMuscles():
        m.set_ignore_tendon_compliance(False)
    state = model.initSystem()
    st = osim.TimeSeriesTable(a.states)
    labels = list(st.getColumnLabels())
    data = st.getMatrix().to_numpy()
    names = model.getStateVariableNames()
    model_vars = {names.get(i) for i in range(names.getSize())}
    cols = [(i, l) for i, l in enumerate(labels) if l in model_vars and 'activation' not in l
            and 'fiber_length' not in l]

    frames = list(range(0, len(pr.t), a.every))
    s = pr.m_emg
    act = pr.emg[frames]                      # linear EMG, as the starting point
    mine = hill_force(act, pr.lmt[frames][:, s], np.zeros((len(frames), len(s))),
                      pr.fmax[s], pr.l0[s], pr.lts[s], pr.alpha0[s], pr.vmax[s])
    ref = np.zeros_like(mine)
    for k, fr in enumerate(frames):
        for i, l in cols:
            model.setStateVariableValue(state, l, data[fr, i])
        model.realizePosition(state)
        for j, mi in enumerate(s):
            m = osim.Millard2012EquilibriumMuscle.safeDownCast(model.getMuscles().get(pr.names[mi]))
            m.setActivation(state, float(act[k, j]))
        model.equilibrateMuscles(state)
        model.realizeDynamics(state)
        for j, mi in enumerate(s):
            ref[k, j] = model.getMuscles().get(pr.names[mi]).getTendonForce(state)

    print('%-14s %9s %9s %7s' % ('muscle', 'OpenSim', 'mine', 'ratio'))
    for j, mi in enumerate(s):
        o, h = ref[:, j].mean(), mine[:, j].mean()
        print('%-14s %9.1f %9.1f %7.2f   corr %.2f' % (
            pr.names[mi], o, h, h / max(o, 1e-6),
            np.corrcoef(ref[:, j], mine[:, j])[0, 1] if ref[:, j].std() > 0 else float('nan')))
    r = mine.sum(1) / np.maximum(ref.sum(1), 1e-6)
    print('total force ratio (mine / OpenSim) over frames: mean %.2f  min %.2f  max %.2f'
          % (r.mean(), r.min(), r.max()))


if __name__ == '__main__':
    main()
