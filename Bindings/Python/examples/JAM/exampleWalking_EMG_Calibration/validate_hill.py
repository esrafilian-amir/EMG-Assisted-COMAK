"""Check the fast Hill model of calibrate.py against OpenSim and against COMAK.

  A. OpenSim: the tendon force at zero fiber velocity (equilibrateMuscles, as COMAK does
     with use_tendon_compliance) for the same activations, on a subset of frames.
  B. COMAK: the muscle forces COMAK wrote (<prefix>_force.sto) for the activations COMAK
     found (<prefix>_activations.sto), all frames, in a run with use_tendon_compliance.

    python validate_hill.py --install <install> --comak-setup <comak_settings.xml>
        --comak-results <folder of that run's results> --geometry <geometry.npz>
        [--comak-cwd <folder COMAK was run from>] [--prefix walking]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calibrate  # noqa: E402
import run_muscle_calibration as rmc  # noqa: E402


def compare(title, names, ref, mine):
    print(title)
    print('%-14s %9s %9s %7s %6s %9s' % ('muscle', 'reference', 'mine', 'ratio', 'corr', 'rmse %peak'))
    bad = 0
    for j, n in enumerate(names):
        o, h = ref[:, j], mine[:, j]
        if o.max() < 1.0 and h.max() < 1.0:
            continue
        corr = np.corrcoef(o, h)[0, 1] if o.std() > 0 and h.std() > 0 else float('nan')
        rm = 100 * np.sqrt(np.mean((o - h) ** 2)) / max(o.max(), 1e-6)
        bad += rm > 5
        print('%-14s %9.1f %9.1f %7.2f %6.2f %9.1f' % (n, o.mean(), h.mean(),
                                                       h.mean() / max(o.mean(), 1e-6), corr, rm))
    tot = np.sqrt(np.mean((ref.sum(1) - mine.sum(1)) ** 2)) / ref.sum(1).max() * 100
    print('total force rmse: %.1f %% of its peak;  muscles above 5 %% rmse: %d\n' % (tot, bad))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--install', default=None)
    p.add_argument('--comak-setup', required=True)
    p.add_argument('--comak-results', required=True)
    p.add_argument('--geometry', required=True)
    p.add_argument('--comak-cwd', default=None)
    p.add_argument('--prefix', default='walking')
    p.add_argument('--every', type=int, default=6)
    a = p.parse_args()

    setup = os.path.abspath(a.comak_setup)
    trial = {'comak_setup': setup,
             'comak_cwd': a.comak_cwd or os.path.join(os.path.dirname(setup), '..')}
    cm = rmc.read_comak_setup(trial)
    force_file = os.path.join(a.comak_results, a.prefix + '_force.sto')
    pr = calibrate.Problem(a.geometry, cm['emg'], force_file, a.install, cm['delay'],
                           ['knee_flex_r'], cm['map'], [])
    import opensim as osim
    allm = np.arange(len(pr.names))

    # B. COMAK
    t_act = osim.TimeSeriesTable(os.path.join(a.comak_results, a.prefix + '_activations.sto'))
    lab = [str(l) for l in t_act.getColumnLabels()]
    times = np.array(t_act.getIndependentColumn())
    ad = t_act.getMatrix().to_numpy()
    act = np.stack([np.interp(pr.t, times, ad[:, lab.index('/forceset/' + n)]) for n in pr.names], 1)
    mine = calibrate.hill_force(act, pr.lmt, pr.fmax, pr.l0, pr.lts, pr.alpha0, pr.curves)
    t_frc = osim.TimeSeriesTable(force_file)
    flab = [str(l) for l in t_frc.getColumnLabels()]
    ftimes = np.array(t_frc.getIndependentColumn())
    fd = t_frc.getMatrix().to_numpy()
    ref = np.stack([np.interp(pr.t, ftimes, fd[:, flab.index('/forceset/' + n)])
                    for n in pr.names], 1)
    compare('B. against the muscle forces COMAK wrote (COMAK activations)', pr.names, ref, mine)

    # A. OpenSim equilibrium
    model = osim.Model(cm['model'])
    for m in model.getMuscles():
        m.set_ignore_tendon_compliance(False)
    state = model.initSystem()
    st = osim.TimeSeriesTable(os.path.join(a.comak_results, a.prefix + '_states.sto'))
    labels = [str(l) for l in st.getColumnLabels()]
    data = st.getMatrix().to_numpy()
    arr = model.getStateVariableNames()
    model_vars = {arr.get(i) for i in range(arr.getSize())}
    cols = [(i, l) for i, l in enumerate(labels) if l in model_vars and 'activation' not in l
            and 'fiber_length' not in l]
    frames = list(range(0, len(pr.t), a.every))
    s = pr.m_emg
    act_a = pr.emg[frames]
    # the equilibrium alone: COMAK's slack-tendon fallback is not part of OpenSim (test B)
    mine_a = calibrate._equilibrium_force(act_a, pr.lmt[frames][:, s], pr.fmax[s], pr.l0[s],
                                          pr.lts[s], pr.alpha0[s], tuple(c[s] for c in pr.curves))
    ref_a = np.zeros_like(mine_a)
    for k, fr in enumerate(frames):
        for i, l in cols:
            model.setStateVariableValue(state, l, data[fr, i])
        model.realizePosition(state)
        for j, mi in enumerate(s):
            osim.Millard2012EquilibriumMuscle.safeDownCast(
                model.getMuscles().get(pr.names[mi])).setActivation(state, float(act_a[k, j]))
        model.equilibrateMuscles(state)
        model.realizeDynamics(state)
        for j, mi in enumerate(s):
            ref_a[k, j] = model.getMuscles().get(pr.names[mi]).getTendonForce(state)
    compare('A. against the OpenSim equilibrium (EMG activations)', [pr.names[i] for i in s],
            ref_a, mine_a)


if __name__ == '__main__':
    main()
