"""Compare COMAK runs of differently calibrated models against the EMG and by knee contact force.

  python analyze_runs.py --comak-setup <comak_settings.xml> --comak-cwd <folder> --emg <raw emg>
      --run "name=<results folder>[=<emg file given to that run>]" ... --out <folder>

Writes into --out:
  emg_vs_activation.png    one subplot per EMG channel: the EMG (delayed as COMAK does) and the
                           activation of the muscles it drives in every run
  knee_contact_force.png   net tibiofemoral contact force (medial + lateral) of every run
  summary.txt              mean |activation - EMG| and correlation per channel and run
The activation of a channel that drives several muscles is their mean. The knee contact force is
read from <results>/joint_mechanics/<prefix>_ForceReporter_forces.sto.
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_muscle_calibration as rmc  # noqa: E402


def read_sto(path):
    """(time, {label: column}) of an OpenSim .sto/.mot file, read as plain text."""
    with open(path) as f:
        lines = f.read().splitlines()
    head = next(i for i, l in enumerate(lines) if l.strip().lower() == 'endheader')
    labels = lines[head + 1].split('\t')
    data = np.array([[float(v) for v in l.split('\t')] for l in lines[head + 2:] if l.strip()])
    return data[:, 0], {lab: data[:, i] for i, lab in enumerate(labels) if i > 0}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--comak-setup', required=True)
    p.add_argument('--comak-cwd', required=True)
    p.add_argument('--emg', required=True, help='the raw EMG file')
    p.add_argument('--run', action='append', required=True,
                   help='name=results folder[=EMG file the run was given]')
    p.add_argument('--prefix', default='walking')
    p.add_argument('--out', required=True)
    a = p.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    cm = rmc.read_comak_setup({'comak_setup': os.path.abspath(a.comak_setup),
                               'comak_cwd': a.comak_cwd})
    delay = cm['delay']
    channels = {}
    for m, terms in cm['map'].items():
        for _, ch in terms:
            channels.setdefault(ch, []).append(m)

    def emg_of(path, t):
        et, ed = read_sto(path)
        low = {k.lower(): v for k, v in ed.items()}
        return {ch: np.interp(t - delay, et, low[ch.lower()]) for ch in channels}

    runs = []
    for spec in a.run:
        parts = spec.split('=')
        runs.append({'name': parts[0], 'dir': parts[1], 'emg': parts[2] if len(parts) > 2 else a.emg})
    for r in runs:
        t, act = read_sto(os.path.join(r['dir'], a.prefix + '_activations.sto'))
        r['t'] = t
        r['act'] = {ch: np.mean([act['/forceset/' + m] for m in ms], axis=0) for ch, ms in channels.items()}
        r['given'] = emg_of(r['emg'], t)
    t0 = runs[0]['t']
    raw = emg_of(a.emg, t0)

    # ---- table
    lines = []
    for ref_name, ref in (('the raw EMG', None), ('the EMG each run was given', 'given')):
        lines.append('mean |activation - EMG| (corr) against %s' % ref_name)
        lines.append('%-10s' % 'channel' + ''.join('%22s' % r['name'] for r in runs))
        tot = np.zeros(len(runs))
        for ch in channels:
            row = '%-10s' % ch
            for k, r in enumerate(runs):
                e = raw[ch] if ref is None else r['given'][ch]
                tt = r['t']
                e = np.interp(tt, t0, e) if ref is None else e
                d = np.mean(np.abs(r['act'][ch] - e))
                c = np.corrcoef(r['act'][ch], e)[0, 1] if e.std() > 0 and r['act'][ch].std() > 0 else float('nan')
                tot[k] += d
                row += '%14.3f (%5.2f)' % (d, c)
            lines.append(row)
        lines.append('%-10s' % 'MEAN' + ''.join('%22.3f' % (v / len(channels)) for v in tot) + '\n')
    for r in runs:
        # convergence
        try:
            tc, conv = read_sto(os.path.join(r['dir'], a.prefix + '_convergence.sto'))
            key = next((k for k in conv if 'converge' in k.lower()), None)
            if key:
                lines.append('%s: %d of %d frames converged' % (r['name'], int(np.sum(conv[key] > 0.5)), len(tc)))
        except Exception as ex:  # noqa: BLE001
            lines.append('%s: convergence file not read (%s)' % (r['name'], ex))

    # ---- knee contact force
    fig2, ax2 = plt.subplots(1, 2, figsize=(11, 4), sharex=True)
    for r in runs:
        f = os.path.join(r['dir'], 'joint_mechanics', a.prefix + '_ForceReporter_forces.sto')
        if not os.path.exists(f):
            lines.append('%s: no joint mechanics output (%s)' % (r['name'], f))
            continue
        tf, d = read_sto(f)
        tot = np.zeros((len(tf), 3))
        for side in ('Lat', 'Med'):
            for i, ax in enumerate('xyz'):
                tot[:, i] += d['tf_contact_%s.tibia_cartilage_%s.total.contact_force_%s' % (side, side, ax)]
        mag = np.linalg.norm(tot, axis=1)
        r['kcf'] = (tf, mag, tot)
        ax2[0].plot(tf, mag, label=r['name'])
        ax2[1].plot(tf, tot[:, 1], label=r['name'])
        lines.append('%s: knee contact force (N) peak %.0f, mean %.0f' % (r['name'], mag.max(), mag.mean()))
    ax2[0].set_title('net tibiofemoral contact force (magnitude)')
    ax2[1].set_title('y component (tibia frame)')
    for x in ax2:
        x.set_xlabel('time (s)')
        x.set_ylabel('N')
        x.grid(alpha=.3)
    ax2[0].legend()
    fig2.tight_layout()
    fig2.savefig(os.path.join(a.out, 'knee_contact_force.png'), dpi=150)

    # ---- EMG vs activation
    n = len(channels)
    cols = 4
    rows = int(np.ceil(n / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(3.6 * cols, 2.3 * rows), sharex=True, sharey=True)
    axs = np.atleast_1d(axs).ravel()
    for i, ch in enumerate(channels):
        ax = axs[i]
        ax.plot(t0, raw[ch], 'k', lw=2, label='EMG (raw)')
        for r in runs:
            if r['emg'] != a.emg:
                ax.plot(r['t'], r['given'][ch], '--', lw=1, label='EMG given to ' + r['name'])
            ax.plot(r['t'], r['act'][ch], lw=1.3, label='activation ' + r['name'])
        ax.set_title(ch, fontsize=9)
        ax.grid(alpha=.3)
    for ax in axs[n:]:
        ax.axis('off')
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='lower right', fontsize=8)
    fig.supxlabel('time (s)')
    fig.supylabel('activation / EMG')
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, 'emg_vs_activation.png'), dpi=150)

    open(os.path.join(a.out, 'summary.txt'), 'w').write('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
