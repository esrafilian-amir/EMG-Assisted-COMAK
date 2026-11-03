"""Stage 2 of the EMG muscle calibration: fit musculotendon parameters to the ID moments.

Works on the cache of extract_geometry.py, so one evaluation is a few array
operations (no OpenSim). Per muscle group it fits
  strength scale (max_isometric_force), optimal fiber length scale,
  tendon slack length scale, and the EMG shape A of a = (exp(A e) - 1) / (exp(A) - 1).
The EMG is delayed by the electromechanical delay. Muscles without an EMG channel keep
the force COMAK found (a fixed background). The objective is the CEINMS one: the
squared error of the primary coordinate moments, normalised by the ID variance, plus
small priors that keep the scales near 1.

    python calibrate.py --geometry geometry.npz --emg <emg.mot> --comak-force <walking_force.sto>
        --install <install> --out result.json
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares

# (EMG channel, muscle) as in the EMG example
EMG_MAP = [('semimem', 'semimem_r'), ('bifem', 'bfsh_r'), ('bifem', 'bflh_r'),
           ('vasmed', 'vasmed_r'), ('vaslat', 'vaslat_r'), ('rf', 'recfem_r'),
           ('medgas', 'gasmed_r'), ('latgas', 'gaslat_r'), ('tfl', 'tfl_r'),
           ('tibant', 'tibant_r'), ('peronl', 'perlong_r'), ('soleus', 'soleus_r'),
           ('addmagnus', 'addmagProx_r'), ('addmagnus', 'addmagMid_r'),
           ('addmagnus', 'addmagDist_r'), ('addmagnus', 'addmagIsch_r'),
           ('gmax', 'glmax1_r'), ('gmax', 'glmax2_r'), ('gmax', 'glmax3_r'),
           ('gmed', 'glmed1_r'), ('gmed', 'glmed2_r'), ('gmed', 'glmed3_r'),
           ('sartorius', 'sart_r')]

GROUPS = {
    'quadriceps': ['vasmed_r', 'vaslat_r', 'recfem_r'],
    'hamstrings': ['semimem_r', 'bfsh_r', 'bflh_r'],
    'plantarflexors': ['gasmed_r', 'gaslat_r', 'soleus_r'],
    'dorsiflexors_peroneals': ['tibant_r', 'perlong_r'],
    'hip_extensors': ['glmax1_r', 'glmax2_r', 'glmax3_r', 'addmagProx_r', 'addmagMid_r',
                      'addmagDist_r', 'addmagIsch_r'],
    'hip_abductors_flexors': ['glmed1_r', 'glmed2_r', 'glmed3_r', 'tfl_r', 'sart_r'],
}
PER_GROUP = ['strength', 'fiber_length', 'tendon_slack', 'emg_shape']
A0 = -1e-3          # nearly linear EMG to activation
BOUNDS = {'strength': (0.5, 2.0), 'fiber_length': (0.95, 1.05),
          'tendon_slack': (0.95, 1.05), 'emg_shape': (-3.0, A0)}
START = {'strength': 1.0, 'fiber_length': 1.0, 'tendon_slack': 1.0, 'emg_shape': A0}


# ---------------------------------------------------------------- Hill model (Thelen-like)
def tendon_force(strain):
    e = np.clip(strain, 0.0, 0.25)
    return np.where(strain < 0, 0.0, (np.exp(35.0 * e) - 1.0) / (np.exp(35.0 * 0.033) - 1.0))


def hill_force(act, lmt, vmt, fmax, l0, lts, alpha0, vmax):
    """Force along the tendon (N). All arrays are frames x muscles or muscles."""
    lo = np.full(act.shape, 0.3) * l0
    hi = np.full(act.shape, 1.8) * l0

    def residual(lm):
        sa = np.clip(l0 * np.sin(alpha0) / lm, 0.0, 0.95)
        ca = np.sqrt(1.0 - sa * sa)
        lt = lmt - lm * ca
        ln = lm / l0
        vn = np.clip(vmt * ca / (vmax * l0), -1.0, 1.0)
        fl = np.exp(-((ln - 1.0) ** 2) / 0.45)
        fv = np.where(vn < 0, (1.0 + vn) / (1.0 - 4.0 * vn), 1.0 + 0.4 * np.tanh(4.0 * vn))
        fpe = np.where(ln > 1.0, (np.exp(4.0 * (ln - 1.0) / 0.6) - 1.0) / (np.exp(4.0) - 1.0), 0.0)
        fm = (act * fl * fv + fpe) * ca
        return tendon_force(lt / lts - 1.0) - fm, lt

    for _ in range(40):
        mid = 0.5 * (lo + hi)
        r, _ = residual(mid)
        lo = np.where(r > 0, mid, lo)   # tendon pulls harder than the fiber: lengthen the fiber
        hi = np.where(r > 0, hi, mid)
    _, lt = residual(0.5 * (lo + hi))
    return fmax * tendon_force(lt / lts - 1.0)


# ---------------------------------------------------------------- data
class Problem:
    def __init__(self, geo, emg_file, force_file, install, delay, dofs):
        sys.path.insert(0, os.path.join(install, 'sdk', 'python'))
        os.environ['PATH'] = os.path.join(install, 'bin') + os.pathsep + os.environ['PATH']
        import opensim as osim
        osim.Logger.setLevelString('Error')
        g = np.load(geo)
        self.t = g['time']
        self.names = [str(n) for n in g['names']]
        self.primary = [str(n) for n in g['primary']]
        self.dof_idx = [self.primary.index(d) for d in dofs]
        self.dofs = dofs
        self.lmt, self.vmt, self.arm = g['length'], g['speed'], g['arm']
        self.fmax, self.l0, self.lts = (g['max_isometric_force'], g['optimal_fiber_length'],
                                        g['tendon_slack_length'])
        self.alpha0, self.vmax = g['pennation_angle_at_optimal'], g['max_contraction_velocity']
        self.id = g['id_moments'][:, self.dof_idx]

        emg = osim.TimeSeriesTable(emg_file)
        el = list(emg.getColumnLabels())
        et = np.array(emg.getIndependentColumn())
        ed = emg.getMatrix().to_numpy()
        self.emg_muscles = [m for _, m in EMG_MAP if m in self.names]
        self.m_emg = np.array([self.names.index(m) for m in self.emg_muscles])
        e = np.zeros((len(self.t), len(self.emg_muscles)))
        for j, (ch, m) in enumerate([x for x in EMG_MAP if x[1] in self.names]):
            e[:, j] = np.interp(self.t - delay, et, ed[:, el.index(ch)])
        self.emg = np.clip(e, 0.0, 1.0)

        f = osim.TimeSeriesTable(force_file)
        fl = list(f.getColumnLabels())
        ft = np.array(f.getIndependentColumn())
        fd = f.getMatrix().to_numpy()
        self.f_comak = np.stack([np.interp(self.t, ft, fd[:, fl.index('/forceset/' + n)])
                                 for n in self.names], axis=1)
        bg = np.ones(len(self.names), bool)
        bg[self.m_emg] = False
        self.f_bg = self.f_comak.copy()
        self.f_bg[:, ~bg] = 0.0                      # only the non-EMG muscles stay
        self.var = np.maximum(self.id.var(axis=0), 1.0)

        # parameter vector: group-wise
        self.group_of = np.zeros(len(self.emg_muscles), int)
        for gi, (gn, members) in enumerate(GROUPS.items()):
            for m in members:
                if m in self.emg_muscles:
                    self.group_of[self.emg_muscles.index(m)] = gi
        self.gnames = list(GROUPS)
        self.x0 = np.array([START[p] for _ in self.gnames for p in PER_GROUP])
        self.lb = np.array([BOUNDS[p][0] for _ in self.gnames for p in PER_GROUP])
        self.ub = np.array([BOUNDS[p][1] for _ in self.gnames for p in PER_GROUP])

    def unpack(self, x):
        p = x.reshape(len(self.gnames), len(PER_GROUP))
        return {k: p[:, i][self.group_of] for i, k in enumerate(PER_GROUP)}

    def forces(self, x):
        q = self.unpack(x)
        s = self.m_emg
        A = q['emg_shape']
        act = (np.exp(A * self.emg) - 1.0) / (np.exp(A) - 1.0)
        f = self.f_bg.copy()
        f[:, s] = hill_force(act, self.lmt[:, s], self.vmt[:, s],
                             self.fmax[s] * q['strength'], self.l0[s] * q['fiber_length'],
                             self.lts[s] * q['tendon_slack'], self.alpha0[s], self.vmax[s])
        return f, act

    def moments(self, x):
        f, _ = self.forces(x)
        return np.einsum('tm,tmd->td', f, self.arm[:, :, self.dof_idx])

    def residuals(self, x, w_prior=0.05):
        e = (self.id - self.moments(x)) / np.sqrt(self.var) / np.sqrt(len(self.t))
        prior = (x - self.x0) / (self.ub - self.lb) * w_prior
        return np.concatenate([e.ravel(), prior])

    def report(self, x):
        m = self.moments(x)
        out = {}
        for d, n in enumerate(self.dofs):
            err = self.id[:, d] - m[:, d]
            out[n] = {'rmse_Nm': float(np.sqrt(np.mean(err ** 2))),
                      'r2': float(1 - err.var() / self.id[:, d].var())}
        return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--geometry', required=True)
    p.add_argument('--emg', required=True)
    p.add_argument('--comak-force', required=True)
    p.add_argument('--install', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--delay', type=float, default=0.04)
    p.add_argument('--dofs', default='hip_flex_r,knee_flex_r,ankle_flex_r')
    a = p.parse_args()

    pr = Problem(a.geometry, a.emg, a.comak_force, a.install, a.delay, a.dofs.split(','))
    before = pr.report(pr.x0)
    sol = least_squares(pr.residuals, pr.x0, bounds=(pr.lb, pr.ub), diff_step=1e-3,
                        x_scale='jac', max_nfev=300)
    after = pr.report(sol.x)

    print('%-10s %22s %22s' % ('', 'before (rmse, R2)', 'after (rmse, R2)'))
    for n in pr.dofs:
        print('%-10s %10.1f  %8.2f %12.1f  %8.2f' % (n, before[n]['rmse_Nm'], before[n]['r2'],
                                                     after[n]['rmse_Nm'], after[n]['r2']))
    print('cost', 0.5 * np.sum(pr.residuals(pr.x0) ** 2), '->', sol.cost, ' nfev', sol.nfev)
    print('%-24s %9s %9s %9s %9s' % ('group', *PER_GROUP))
    for gi, gn in enumerate(pr.gnames):
        print('%-24s' % gn, ' '.join('%9.3f' % v for v in sol.x.reshape(-1, len(PER_GROUP))[gi]))

    json.dump({'dofs': pr.dofs, 'before': before, 'after': after, 'cost': float(sol.cost),
               'groups': {gn: dict(zip(PER_GROUP, map(float, sol.x.reshape(-1, len(PER_GROUP))[gi])))
                          for gi, gn in enumerate(pr.gnames)}}, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
