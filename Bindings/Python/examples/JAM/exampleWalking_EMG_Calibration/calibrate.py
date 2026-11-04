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
priors that keep the parameters near their starting values. Optionally also the knee
contact force the muscles cause (contact_weight): the sum over the muscles of force times the
compression that one newton of the muscle adds to the knee (contact_gain of extract_geometry),
normalised by its peak with the starting parameters.
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


# ---------------------------------------------------------------- Hill model
# The curves are the ones of the model's Millard muscles, sampled by extract_geometry.py
# into tables (the curves depend on the normalised length only, so the parameters do not
# change them). The force is the one COMAK uses with use_tendon_compliance: the tendon force
# of the equilibrium at zero fiber velocity, so there is no force-velocity term.
LN_GRID = np.linspace(0.0, 2.5, 1251)      # fiber length / optimal fiber length
TN_GRID = np.linspace(0.9, 1.25, 1401)     # tendon length / tendon slack length


def table_lookup(tab, x, grid):
    """tab: muscles x samples; x: frames x muscles (or muscles); linear interpolation."""
    pos = np.clip((x - grid[0]) / (grid[1] - grid[0]), 0.0, len(grid) - 1.000001)
    i = pos.astype(int)
    f = pos - i
    cols = np.arange(tab.shape[0])
    return tab[cols, i] * (1.0 - f) + tab[cols, i + 1] * f


def hill_force(act, lmt, fmax, l0, lts, alpha0, curves):
    """Muscle force (N) as COMAK computes it with use_tendon_compliance.

    The tendon force of the equilibrium at zero fiber velocity (equilibrateMuscles) is
    linearised in the activation from two points, a and a + 0.1 (a clipped to 0.02..0.9);
    a muscle whose slope is below 5 % of its maximum force (a slack tendon) keeps the rigid
    tendon force, activation * maximum force (COMAKTool::updateTendonComplianceForces).
    act, lmt: frames x muscles; the rest muscles.
    curves = (active force-length, passive force-length, tendon force-length) tables."""
    a1 = np.clip(act, 0.02, 0.9)
    f1 = _equilibrium_force(a1, lmt, fmax, l0, lts, alpha0, curves)
    f2 = _equilibrium_force(a1 + 0.1, lmt, fmax, l0, lts, alpha0, curves)
    slope = (f2 - f1) / 0.1
    elastic = slope >= 0.05 * fmax
    return np.where(elastic, slope * act + (f1 - slope * a1), fmax * act)


def _equilibrium_force(act, lmt, fmax, l0, lts, alpha0, curves):
    fal, fpe, ftn = curves
    lo = np.full(act.shape, 0.3) * l0
    hi = np.full(act.shape, 1.8) * l0

    def residual(lm):
        sa = np.clip(l0 * np.sin(alpha0) / lm, 0.0, 0.95)
        ca = np.sqrt(1.0 - sa * sa)
        lt = lmt - lm * ca
        ln = lm / l0
        fm = (act * table_lookup(fal, ln, LN_GRID) + table_lookup(fpe, ln, LN_GRID)) * ca
        return table_lookup(ftn, lt / lts, TN_GRID) - fm, lt

    for _ in range(40):
        mid = 0.5 * (lo + hi)
        r, _ = residual(mid)
        lo = np.where(r > 0, mid, lo)   # tendon pulls harder than the fiber: lengthen the fiber
        hi = np.where(r > 0, hi, mid)
    _, lt = residual(0.5 * (lo + hi))
    return fmax * table_lookup(ftn, lt / lts, TN_GRID)


def shape_emg(e, a):
    return (np.exp(a * e) - 1.0) / (np.exp(a) - 1.0)


# ---------------------------------------------------------------- problem
class Problem:
    """emg_map: {muscle: [(weight, channel), ...]}; groups: [{'name', 'muscles', 'fit', 'bounds'}]"""

    def __init__(self, geo, emg_file, force_file, install, delay, dofs, emg_map, groups,
                 bounds=None, contact_weight=0.0):
        if install:
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
        self.alpha0 = g['pennation_angle_at_optimal']
        if 'curve_active_force_length' not in g.files:
            raise SystemExit(f'{geo} has no muscle curves: recompute the geometry cache '
                             f'(--recompute-geometry)')
        self.curves = (g['curve_active_force_length'], g['curve_passive_force_length'],
                       g['curve_tendon_force_length'])
        self.id = g['id_moments'][:, self.dof_idx]
        self.contact_weight = contact_weight
        if contact_weight > 0 and 'contact_gain' not in g.files:
            raise SystemExit(f'{geo} has no contact gains: recompute the geometry cache '
                             f'(--recompute-geometry)')
        self.gain = g['contact_gain'] if 'contact_gain' in g.files else None

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
        # always present, so the parameter vector has the same layout in every trial
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
        self.cf_scale = 1.0
        if self.gain is not None:
            self.cf_scale = max(self.contact(self.x0).max(), 1.0)

    def contact(self, x):
        """Knee compression (N) that the muscle forces cause, per frame."""
        f, _ = self.forces(x)
        return (f * self.gain).sum(axis=1)

    def unpack(self, x):
        p = x.reshape(len(self.groups), len(PARAMS))
        return {k: p[:, i][self.group_of] for i, k in enumerate(PARAMS)}

    def forces(self, x):
        q = self.unpack(x)
        s = self.m_emg
        act = shape_emg(self.emg, q['emg_shape'])
        f = self.f_bg.copy()
        f[:, s] = hill_force(act, self.lmt[:, s], self.fmax[s] * q['strength'],
                             self.l0[s] * q['fiber_length'], self.lts[s] * q['tendon_slack'],
                             self.alpha0[s], tuple(c[s] for c in self.curves))
        return f, act

    def moments(self, x):
        f, _ = self.forces(x)
        return np.einsum('tm,tmd->td', f, self.arm[:, :, self.dof_idx])

    def errors(self, x):
        """Moment errors normalised by the ID variance; sum of squares = CEINMS error."""
        e = ((self.id - self.moments(x)) / np.sqrt(self.var) / np.sqrt(len(self.t))).ravel()
        if self.contact_weight > 0:
            e = np.concatenate([e, self.contact_weight * self.contact(x) / self.cf_scale
                                / np.sqrt(len(self.t))])
        return e

    def prior(self, x, w_prior=0.05):
        return (x - self.x0) / (self.ub - self.lb) * w_prior

    def residuals(self, x):
        return np.concatenate([self.errors(x), self.prior(x)])

    def report(self, x):
        m = self.moments(x)
        out = {}
        for d, n in enumerate(self.dofs):
            err = self.id[:, d] - m[:, d]
            out[n] = {'rmse_Nm': float(np.sqrt(np.mean(err ** 2))),
                      'r2': float(1 - err.var() / self.id[:, d].var())}
        if self.gain is not None:
            c = self.contact(x)
            out['knee_compression_N'] = {'peak': float(c.max()), 'mean': float(c.mean())}
        return out

    def fit(self, max_nfev=300):
        return fit_problem(self, max_nfev)

    def group_values(self, x):
        p = x.reshape(len(self.groups), len(PARAMS))
        return {gr['name']: dict(zip(PARAMS, map(float, p[gi]))) for gi, gr in enumerate(self.groups)}


def fit_problem(pr, max_nfev=300):
    free = np.flatnonzero(pr.free)

    def full(xf):
        x = pr.x0.copy()
        x[free] = xf
        return x
    sol = least_squares(lambda xf: pr.residuals(full(xf)), pr.x0[free],
                        bounds=(pr.lb[free], pr.ub[free]), diff_step=1e-3,
                        x_scale='jac', max_nfev=max_nfev)
    return full(sol.x), sol


class MultiProblem:
    """Several trials, one shared parameter set.

    The objective is the mean over the trials of each trial's normalised moment error, so
    every trial counts the same whatever its length or moment size, plus the prior. A
    parameter set that is good for one trial only is penalised by the others."""

    def __init__(self, problems, names):
        self.problems, self.trial_names = problems, names
        p0 = problems[0]
        for p in problems[1:]:
            if [g['name'] for g in p.groups] != [g['name'] for g in p0.groups]:
                raise ValueError('the trials must use the same muscle groups')
        self.groups, self.dofs = p0.groups, p0.dofs
        self.x0, self.lb, self.ub, self.free = p0.x0, p0.lb, p0.ub, p0.free
        # one entry per muscle that has EMG in any trial, with its group
        self.emg_muscles = []
        for p in problems:
            self.emg_muscles += [m for m in p.emg_muscles if m not in self.emg_muscles]
        self.group_of = np.zeros(len(self.emg_muscles), int)
        for gi, gr in enumerate(self.groups):
            for m in gr['muscles']:
                if m in self.emg_muscles:
                    self.group_of[self.emg_muscles.index(m)] = gi
        self.names = p0.names

    unpack = Problem.unpack
    group_values = Problem.group_values

    def residuals(self, x):
        n = np.sqrt(len(self.problems))
        return np.concatenate([p.errors(x) / n for p in self.problems] + [self.problems[0].prior(x)])

    def report(self, x):
        return {t: p.report(x) for t, p in zip(self.trial_names, self.problems)}

    def fit(self, max_nfev=300):
        return fit_problem(self, max_nfev)
