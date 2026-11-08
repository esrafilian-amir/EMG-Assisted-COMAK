"""Right knee of lenhart2015.osim versus left knee of lenhart2015_Left_fixed(.osim) symmetry check.

Usage: python check_symmetry_lenhart2015_Left_fixed.py [left_model.osim] [constraint_functions.xml] [right_model.osim]

For knee flexion 0, 30, 60, 90, 120 deg (and the default pose) the other 10 knee/patella coordinates of the
right knee are taken from the single knee constraint functions; the left knee gets the same values with the
sign of the coordinates that change under a reflection about the sagittal plane (rotations about x and y,
translation along z: knee_add, knee_rot, knee_tz, pf_rot, pf_tilt, pf_tz). Then every quantity of the right
knee (in the right model) is compared with the same quantity of the left knee (in the left model); positions
are compared after mirroring z.
"""
import os
import sys
import numpy as np
import opensim as osim

osim.Logger.setLevelString('Error')
here = os.path.dirname(os.path.abspath(__file__))
left_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'lenhart2015_Left_fixed.osim')
func_file = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\amires\Desktop\Subject03_JAM\Model\secondary_coordinate_constraint_functions.xml'
right_file = sys.argv[3] if len(sys.argv) > 3 else os.path.join(os.path.dirname(here), 'lenhart2015', 'lenhart2015.osim')

FLIP = {'knee_flex': 1, 'knee_add': -1, 'knee_rot': -1, 'knee_tx': 1, 'knee_ty': 1, 'knee_tz': -1,
        'pf_flex': 1, 'pf_rot': -1, 'pf_tilt': -1, 'pf_tx': 1, 'pf_ty': 1, 'pf_tz': -1}
M = np.diag([1.0, 1.0, -1.0])

mR, mL = osim.Model(right_file), osim.Model(left_file)
sR, sL = mR.initSystem(), mL.initSystem()
funcs = osim.FunctionSet(func_file)
fdict = {funcs.get(i).getName().split('/')[-1]: funcs.get(i) for i in range(funcs.getSize())}


def val(fn, x):
    return fn.calcValue(osim.Vector(1, x))


def tmat(t):
    R = t.R()
    p = t.p()
    return np.array([[R.get(i, j) for j in range(3)] for i in range(3)]), np.array([p.get(0), p.get(1), p.get(2)])


def rel(m, s, a, b):
    return tmat(m.getBodySet().get(b).findTransformBetween(s, m.getBodySet().get(a)))


def set_pose(flex_deg, use_default=False):
    for m, s, side in ((mR, sR, 'r'), (mL, sL, 'l')):
        for base, sgn in FLIP.items():
            c = m.getCoordinateSet().get('%s_%s' % (base, side))
            if use_default:
                v = c.getDefaultValue()
            else:
                f = np.deg2rad(flex_deg)
                v = f if base == 'knee_flex' else val(fdict[base + '_r'], f)
                if side == 'l' and base != 'knee_flex':
                    v *= sgn
            c.setValue(s, v, False)
        m.realizeReport(s)


def od(f, s, o):
    return osim.OutputDouble.safeDownCast(f.getOutput(o)).getValue(s)


def v3(f, s, o):
    a = osim.OutputVec3.safeDownCast(f.getOutput(o)).getValue(s)
    return np.array([a.get(0), a.get(1), a.get(2)])


def pct(a, b):
    return 100 * abs(a - b) / max(abs(a), abs(b), 1e-12)


def run(flex_deg, use_default=False):
    set_pose(flex_deg, use_default)
    out = {}
    for name, (a, b) in {'tibia in femur_distal': ('femur_distal', 'tibia_proximal'),
                         'patella in femur_distal': ('femur_distal', 'patella')}.items():
        Rr, pr = rel(mR, sR, a + '_r', b + '_r')
        Rl, pl = rel(mL, sL, a + '_l', b + '_l')
        out[name] = (np.linalg.norm(M @ pr - pl) * 1000.0, np.abs(M @ Rr @ M - Rl).max())
    fR, fL = mR.getForceSet(), mL.getForceSet()
    lig = {}
    for i in range(fR.getSize()):
        f = fR.get(i)
        if f.getConcreteClassName() == 'Blankevoort1991Ligament':
            g = fL.get(f.getName())
            for o in ('length', 'strain', 'total_force'):
                lig[(f.getName(), o)] = (od(f, sR, o), od(g, sL, o))
    con = {}
    for c in ('tf_contact', 'pf_contact'):
        fr, fl = fR.get(c), fL.get(c)
        con[(c, 'casting_total_contact_force |F|')] = (np.linalg.norm(v3(fr, sR, 'casting_total_contact_force')),
                                                       np.linalg.norm(v3(fl, sL, 'casting_total_contact_force')))
        for o in ('casting_total_max_proximity', 'casting_total_contact_area', 'casting_total_max_pressure'):
            con[(c, o)] = (od(fr, sR, o), od(fl, sL, o))
    return out, lig, con


worst = {}
cases = [('default pose', 0, True)] + [('flexion %d deg' % d, d, False) for d in (0, 30, 60, 90, 120)]
for label, flex, dflt in cases:
    fr, lig, con = run(flex, dflt)
    print('=== %s' % label)
    for k, (dp, dR) in fr.items():
        print('  %-26s position difference %.4f mm, rotation difference %.2e' % (k, dp, dR))
        worst['frame position mm'] = max(worst.get('frame position mm', 0), dp)
        worst['frame rotation'] = max(worst.get('frame rotation', 0), dR)
    for o in ('length', 'strain', 'total_force'):
        d, n, a, b = max((abs(a - b), n, a, b) for (n, oo), (a, b) in lig.items() if oo == o)
        unit = 1000.0 if o == 'length' else 1.0
        print('  ligament %-11s max |R-L| = %.5g %s (%s: R %.5g L %.5g)' % (
            o, d * unit, {'length': 'mm', 'strain': '', 'total_force': 'N'}[o], n, a, b))
        worst['ligament ' + o] = max(worst.get('ligament ' + o, 0), d * unit)
    for (c, o), (a, b) in con.items():
        print('  %-11s %-40s R %.5g  L %.5g  diff %.3f %%' % (c, o, a, b, pct(a, b)))
        if ('force' in o or 'proximity' in o) and max(a, b) > 1e-3:
            k = 'contact %s %s (percent)' % (c, o.split('_')[0] + o.split('total')[-1])
            worst[k] = max(worst.get(k, 0), pct(a, b))
print('=== worst case over the poses')
for k, v in worst.items():
    print('  %s: %.5g' % (k, v))

print('=== coordinate ranges and defaults (left expected = sign * right)')
for base, sgn in FLIP.items():
    cr, cl = mR.getCoordinateSet().get(base + '_r'), mL.getCoordinateSet().get(base + '_l')
    exp = sorted([sgn * cr.getRangeMin(), sgn * cr.getRangeMax()])
    ok = np.allclose(exp, [cl.getRangeMin(), cl.getRangeMax()], atol=1e-9) and abs(sgn * cr.getDefaultValue() - cl.getDefaultValue()) < 1e-9
    print('  %-10s R [%.5f, %.5f] def %.5f | L [%.5f, %.5f] def %.5f | sign %+d %s' % (
        base, cr.getRangeMin(), cr.getRangeMax(), cr.getDefaultValue(), cl.getRangeMin(), cl.getRangeMax(),
        cl.getDefaultValue(), sgn, 'OK' if ok else 'DIFFERENT'))
