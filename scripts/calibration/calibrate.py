"""Fitting module of the EMG muscle calibration (used by run_muscle_calibration.py).

Works on the cache of extract_geometry.py, so one evaluation is a few array
operations (no OpenSim). Per muscle group it can fit
  strength            scale of max_isometric_force
  fiber_length        scale of optimal_fiber_length
  tendon_slack        scale of tendon_slack_length
  emg_shape           A of a = (exp(A e) - 1) / (exp(A) - 1)
The EMG is delayed by the electromechanical delay. Muscles without an EMG channel keep
the force COMAK found (a fixed background). The objective is the CEINMS one: the
squared error of the coordinate moments, normalised by the ID variance, plus small
priors that keep the parameters near their starting values.
"""
import os
import sys

import numpy as np
from scipy.optimize import least_squares

PARAMS = ['strength', 'fiber_length', 'tendon_slack', 'emg_shape']
A0 = -1e-3          # nearly linear EMG to activation
START = {'strength': 1.0, 'fiber_length': 1.0, 'tendon_slack': 1.0, 'emg_shape': A0}
DEFAULT_BOUNDS = {'strength': (0.5, 2.0), 'fiber_length': (0.95, 1.05),
                  'tendon_slack': (0.95, 1.05), 'emg_shape': (-3.0, A0)}


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


def shape_emg(e, a):
    return (np.exp(a * e) - 1.0) / (np.exp(a) - 1.0)


# ---------------------------------------------------------------- problem
class Problem:
    """emg_map: {muscle: [(weight, channel), ...]}; groups: [{'name', 'muscles', 'fit', 'bounds'}]"""

    def __init__(self, geo, emg_file, force_file, install, delay, dofs, emg_map, groups,
                 bounds=None):
        sys.path.insert(0, os.path.join(install, 'sdk', 'python'))
        os.environ['PATH'] = os.path.join(install, 'bin') + os.pathsep + os.environ['PATH']
        import opensim as osim
        osim.Logger.setLevelString('Error')
        g = np.load(geo)
        self.t = g['time']
        self.names = [str(n) for n in g['names']]
        self.primary = [str(n) for n in g['primary']]
        self.dofs = dofs
        self.dof_idx = [self.primary.index(d) for d in dofs]
        self.lmt, self.vmt, self.arm = g['length'], g['speed'], g['arm']
        self.fmax, self.l0, self.lts = (g['max_isometric_force'], g['optimal_fiber_length'],
                                        g['tendon_slack_length'])
        self.alpha0, self.vmax = g['pennation_angle_at_optimal'], g['max_contraction_velocity']
        self.id = g['id_moments'][:, self.dof_idx]

        # EMG of each muscle: weighted sum of its channels, delayed
        emg = osim.TimeSeriesTable(emg_file)
        el = [str(l).lower() for l in emg.getColumnLabels()]
        et = np.array(emg.getIndependentColumn())
        ed = emg.getMatrix().to_numpy()
        self.emg_muscles = [m for m in emg_map if m in self.names]
        self.m_emg = np.array([self.names.index(m) for m in self.emg_muscles])
        e = np.zeros((len(self.t), len(self.emg_muscles)))
        for j, m in enumerate(self.emg_muscles):
            for w, ch in emg_map[m]:
                e[:, j] += w * np.interp(self.t - delay, et, ed[:, el.index(ch.lower())])
        self.emg = np.clip(e, 0.0, 1.0)

        # forces COMAK found for the muscles without EMG: a fixed background
        f = osim.TimeSeriesTable(force_file)
        fl = [str(l) for l in f.getColumnLabels()]
        ft = np.array(f.getIndependentColumn())
        fd = f.getMatrix().to_numpy()
        self.f_bg = np.stack([np.interp(self.t, ft, fd[:, fl.index('/forceset/' + n)])
                              for n in self.names], axis=1)
        self.f_bg[:, self.m_emg] = 0.0
        self.var = np.maximum(self.id.var(axis=0), 1.0)

        # groups; EMG muscles that are in no group are modelled but not fitted
        bounds = {**DEFAULT_BOUNDS, **(bounds or {})}
        groups = [dict(g) for g in groups]
        used = {m for g in groups for m in g['muscles']}
        rest = [m for m in self.emg_muscles if m not in used]
        if rest:
            groups.append({'name': 'not_calibrated', 'muscles': rest, 'fit': []})
        self.groups = groups
        self.group_of = np.zeros(len(self.emg_muscles), int)
        for gi, gr in enumerate(groups):
            for m in gr['muscles']:
                if m in self.emg_muscles:
                    self.group_of[self.emg_muscles.index(m)] = gi
        self.x0 = np.array([START[p] for _ in groups for p in PARAMS])
        self.lb = np.array([gr.get('bounds', {}).get(p, bounds[p])[0] for gr in groups for p in PARAMS])
        self.ub = np.array([gr.get('bounds', {}).get(p, bounds[p])[1] for gr in groups for p in PARAMS])
        self.free = np.array([p in gr['fit'] for gr in groups for p in PARAMS])

    def unpack(self, x):
        p = x.reshape(len(self.groups), len(PARAMS))
        return {k: p[:, i][self.group_of] for i, k in enumerate(PARAMS)}

    def forces(self, x):
        q = self.unpack(x)
        s = self.m_emg
        act = shape_emg(self.emg, q['emg_shape'])
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

    def fit(self, max_nfev=300):
        free = np.flatnonzero(self.free)

        def full(xf):
            x = self.x0.copy()
            x[free] = xf
            return x
        sol = least_squares(lambda xf: self.residuals(full(xf)), self.x0[free],
                            bounds=(self.lb[free], self.ub[free]), diff_step=1e-3,
                            x_scale='jac', max_nfev=max_nfev)
        return full(sol.x), sol

    def group_values(self, x):
        p = x.reshape(len(self.groups), len(PARAMS))
        return {gr['name']: dict(zip(PARAMS, map(float, p[gi]))) for gi, gr in enumerate(self.groups)}
