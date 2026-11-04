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
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PRIMARY = ['hip_flex_r', 'hip_add_r', 'hip_rot_r', 'knee_flex_r', 'ankle_flex_r']
# Secondary coordinates that move with a primary one (the sweep found how). OpenSim gives a
# muscle a zero moment arm about a coordinate it does not cross, so the quadriceps, which end
# on the patella, have none about knee_flex_r. Their effective arm follows from the chain rule
# along the motion, arm_eff = arm_direct + sum_s arm_s * dq_s/dtheta (virtual work).
FOLLOW = 'knee_flex_r'
# Bodies below the knee (the tibiofemoral joint), the tibia that gives the axis of the
# compressive force, the patella and the muscles that end on it (see contact_gain below).
DISTAL = ['tibia_proximal', 'tibia', 'talus', 'calcn', 'toes']
TIBIA = 'tibia'
PATELLA = 'patella'
SECONDARY = ['knee_add_r', 'knee_rot_r', 'knee_tx_r', 'knee_ty_r', 'knee_tz_r',
             'pf_flex_r', 'pf_rot_r', 'pf_tilt_r', 'pf_tx_r', 'pf_ty_r', 'pf_tz_r']


def parse():
    p = argparse.ArgumentParser()
    p.add_argument('--install', default=None, help='OpenSim install, if the bindings are not importable')
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
    if a.install:    # not needed when the OpenSim Python bindings can already be imported
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
    distal = {n + a.suffix for n in DISTAL}
    tibia = model.getBodySet().get(TIBIA + a.suffix)
    fset0 = model.getForceSet()
    pt_ligaments = [osim.Blankevoort1991Ligament.safeDownCast(fset0.get(i))
                    for i in range(fset0.getSize()) if re.fullmatch(r'PT\d+', fset0.get(i).getName())]
    names = [m.getName() for m in muscles]
    coords = [model.getCoordinateSet().get(c) for c in PRIMARY]
    n_t, n_m, n_d = len(time), len(muscles), len(coords)

    cset = model.getCoordinateSet()
    sec_names = [c for c in SECONDARY if cset.contains(c)]
    sec = [cset.get(c) for c in sec_names]
    follow = cset.get(FOLLOW)
    n_s = len(sec)

    length = np.zeros((n_t, n_m))
    speed = np.zeros((n_t, n_m))
    arm = np.zeros((n_t, n_m, n_d))
    arm_sec = np.zeros((n_t, n_m, n_s))
    gain = np.zeros((n_t, n_m))
    q_sec = np.zeros((n_t, n_s))
    theta = np.zeros(n_t)
    def point_forces(path):
        # (body name, direction in ground of the force the path applies to it, per unit tension)
        pf = osim.ArrayPointForceDirection()
        path.getPointForceDirections(state, pf)
        return [(pf.get(i).frame().getName(),
                 np.array([pf.get(i).direction().get(c) for c in range(3)])) for i in range(pf.getSize())]

    for k in range(n_t):
        for i, l in use_cols:
            model.setStateVariableValue(state, l, data[k, i])
        model.realizeVelocity(state)
        # The compression of the knee that one newton of a muscle adds (statics of the shank and
        # foot: the net force a muscle puts on them, along the long axis of the tibia, pointing
        # to the knee). The quadriceps end on the patella: its force reaches the tibia through
        # the patellar ligament (equal tension), in the direction of that ligament.
        y_prox = np.array([tibia.expressVectorInGround(state, osim.Vec3(0, 1, 0)).get(c)
                           for c in range(3)])
        d_pt = np.zeros(3)
        for lig in pt_ligaments:
            for body, d in point_forces(lig.getGeometryPath()):
                if body in distal:
                    d_pt += d
        d_pt = d_pt / max(np.linalg.norm(d_pt), 1e-9)
        for j, m in enumerate(muscles):
            f_dist, f_pat = np.zeros(3), np.zeros(3)
            for body, d in point_forces(m.getGeometryPath()):
                if body in distal:
                    f_dist += d
                elif body == PATELLA + a.suffix:
                    f_pat += d
            gain[k, j] = f_dist @ y_prox + np.linalg.norm(f_pat) * (d_pt @ y_prox)
        theta[k] = follow.getValue(state)
        for s, c in enumerate(sec):
            q_sec[k, s] = c.getValue(state)
        for j, m in enumerate(muscles):
            length[k, j] = m.getLength(state)
            speed[k, j] = m.getLengtheningSpeed(state)
            for d, c in enumerate(coords):
                arm[k, j, d] = m.computeMomentArm(state, c)
            for s, c in enumerate(sec):
                arm_sec[k, j, s] = m.computeMomentArm(state, c)

    # How the secondary coordinates follow the knee flexion in this motion: a smooth fit of
    # each one against the flexion angle, and its slope at every frame.
    arm_direct = arm.copy()
    dq = np.zeros((n_t, n_s))
    for s in range(n_s):
        coef = np.polyfit(theta, q_sec[:, s], min(4, n_t - 1))
        dq[:, s] = np.polyval(np.polyder(coef), theta)
        res = q_sec[:, s] - np.polyval(coef, theta)
        r2 = 1.0 - res.var() / max(q_sec[:, s].var(), 1e-30)
        print('  %-10s follows %s: fit R2 %.3f, slope %.4f' % (sec_names[s], FOLLOW, r2, dq[:, s].mean()))
    arm[:, :, PRIMARY.index(FOLLOW)] += np.einsum('tms,ts->tm', arm_sec, dq)

    # parameters of the muscles in the model (the starting point of the calibration)
    par = {key: np.array([getattr(m, f'get_{key}')() for m in muscles]) for key in
           ['max_isometric_force', 'optimal_fiber_length', 'tendon_slack_length',
            'pennation_angle_at_optimal', 'max_contraction_velocity']}

    # The force-length curves of every muscle, sampled on the grids of calibrate.py. They
    # depend on the normalised length only, so they do not change with the parameters.
    from calibrate import LN_GRID, TN_GRID
    curves = {'curve_active_force_length': [], 'curve_passive_force_length': [],
              'curve_tendon_force_length': []}
    for m in muscles:
        mus = osim.Millard2012EquilibriumMuscle.safeDownCast(m)
        if mus is None:
            raise SystemExit(f'{m.getName()} is not a Millard2012EquilibriumMuscle: the '
                             f'calibration reads the curves of that muscle model')
        curves['curve_active_force_length'].append(
            [mus.getActiveForceLengthCurve().calcValue(float(x)) for x in LN_GRID])
        curves['curve_passive_force_length'].append(
            [mus.getFiberForceLengthCurve().calcValue(float(x)) for x in LN_GRID])
        curves['curve_tendon_force_length'].append(
            [mus.getTendonForceLengthCurve().calcValue(float(x)) for x in TN_GRID])
    curves = {k: np.array(v) for k, v in curves.items()}

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
    # The patellar ligament is also left out: the quadriceps force reaches the tibia through
    # it, and the effective knee arm above credits that to the quadriceps. With the ligament
    # in the target, its force (set by the pose) would count twice.
    # The JAM ligaments ignore the appliesForce flag the tool uses to exclude a force (the ID
    # was identical with and without them), so they are removed from a copy of the model.
    fset = model.getForceSet()
    ligaments = [fset.get(i).getName() for i in range(fset.getSize())
                 if re.fullmatch(r'PT\d+', fset.get(i).getName())
                 or 'patellar' in fset.get(i).getName().lower()]
    if ligaments and not getattr(a, 'keep_patellar_ligament', False):
        id_model = osim.Model(a.model)
        for n in ligaments:
            id_model.updForceSet().remove(id_model.getForceSet().getIndex(n))
        id_model.finalizeConnections()
        id_model.initSystem()
        tool.setModel(id_model)
        print('  inverse dynamics without:', a.exclude, 'and', ligaments, '(removed from the model)')
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
             length=length, speed=speed, arm=arm, arm_direct=arm_direct, id_moments=moments,
             contact_gain=gain,
             **curves, **{k: v for k, v in par.items()})
    print('muscles', n_m, 'frames', n_t, 'wrote', a.out)


if __name__ == '__main__':
    main()
