"""Compare a baseline COMAK run with the run on the calibrated model.

    python compare_runs.py --install <install> --base <comak results> --cal <comak results>
        --comak-setup <comak_settings.xml> --emg <the EMG file used by that run>
Prints, over the common time range: how far the optimised activations are from the EMG
(the less COMAK has to leave the EMG, the better the model explains the motion), the
reserve actuator use, the secondary coordinates and the iterations.
"""
import argparse
import os
import sys

import numpy as np


def table(osim, path):
    t = osim.TimeSeriesTable(path)
    return (np.array(t.getIndependentColumn()), [str(l) for l in t.getColumnLabels()],
            t.getMatrix().to_numpy())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--install', required=True)
    p.add_argument('--base', required=True)
    p.add_argument('--cal', required=True)
    p.add_argument('--prefix', default='walking')
    p.add_argument('--emg-base', required=True, help='EMG file of the baseline run')
    p.add_argument('--emg-cal', required=True, help='EMG file of the calibrated run')
    p.add_argument('--delay', type=float, default=0.04)
    p.add_argument('--map', required=True, help='comak_settings.xml for the EMG mapping')
    a = p.parse_args()
    sys.path.insert(0, os.path.join(a.install, 'sdk', 'python'))
    os.environ['PATH'] = os.path.join(a.install, 'bin') + os.pathsep + os.environ['PATH']
    import opensim as osim
    osim.Logger.setLevelString('Error')
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import run_muscle_calibration as rmc

    root = rmc.ET.parse(a.map).getroot().find('COMAKTool')
    emg_map = {}
    for q in root.iter('COMAKCostFunctionParameter'):
        mp = rmc.text(q, 'emg_mapping', '')
        if mp and float(rmc.text(q, 'emg_gamma_weight', '0')) > 0:
            emg_map[rmc.text(q, 'actuator').split('/')[-1]] = rmc.parse_mapping(mp)

    def emg_of(path, t):
        et, el, ed = table(osim, path)
        el = [l.lower() for l in el]
        return {m: sum(w * np.interp(t - a.delay, et, ed[:, el.index(ch.lower())]) for w, ch in terms)
                for m, terms in emg_map.items()}

    res = {}
    for tag, d, emgf in [('baseline', a.base, a.emg_base), ('calibrated', a.cal, a.emg_cal)]:
        t, lab, act = table(osim, os.path.join(d, a.prefix + '_activations.sto'))
        _, flab, frc = table(osim, os.path.join(d, a.prefix + '_force.sto'))
        e = emg_of(emgf, t)
        dev = {m: np.mean(np.abs(act[:, lab.index('/forceset/' + m)] - np.clip(e[m], 0, 1)))
               for m in e}
        reserves = [i for i, l in enumerate(flab) if 'reserve' in l.lower() or 'residual' in l.lower()
                    or 'lumbar' in l.lower() or 'pelvis' in l.lower()]
        res[tag] = {'t': t, 'act': act, 'lab': lab, 'dev': dev, 'frc': frc, 'flab': flab,
                    'reserve_rms': np.sqrt(np.mean(frc[:, reserves] ** 2)) if reserves else float('nan')}

    print('%-14s %10s %10s' % ('muscle', 'baseline', 'calibrated'), '   mean |activation - EMG|')
    for m in res['baseline']['dev']:
        print('%-14s %10.3f %10.3f' % (m, res['baseline']['dev'][m], res['calibrated']['dev'][m]))
    b = np.mean(list(res['baseline']['dev'].values()))
    c = np.mean(list(res['calibrated']['dev'].values()))
    print('%-14s %10.3f %10.3f' % ('MEAN', b, c))
    print('reserve/residual force rms: baseline %.3f  calibrated %.3f' % (
        res['baseline']['reserve_rms'], res['calibrated']['reserve_rms']))

    # secondary coordinates
    tb, lb, vb = table(osim, os.path.join(a.base, a.prefix + '_values.sto'))
    tc, lc, vc = table(osim, os.path.join(a.cal, a.prefix + '_values.sto'))
    n = min(len(tb), len(tc))
    print('secondary coordinates, max |difference| (deg or m):')
    for c_ in ['knee_add_r', 'knee_rot_r', 'knee_tx_r', 'knee_ty_r', 'knee_tz_r', 'pf_flex_r']:
        print('  %-12s %.4f' % (c_, np.abs(vb[:n, lb.index(c_)] - vc[:n, lc.index(c_)]).max()))

    for tag, d in [('baseline', a.base), ('calibrated', a.cal)]:
        _, cl, cv = table(osim, os.path.join(d, a.prefix + '_convergence.sto'))
        print(tag, 'convergence columns:', cl[:6])


if __name__ == '__main__':
    main()
