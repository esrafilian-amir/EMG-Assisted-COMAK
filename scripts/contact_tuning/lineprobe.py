"""Probe all force elements along the straight line between two states.

    python lineprobe.py <model.osim> <two_states.sto> [n=3000] [extend=0.0]

Each force element's magnitude (contact: total contact force of the casting mesh,
ligament: total_force, others: the sum of the absolute generalized forces they apply, via
getRecordValues) is recorded; the largest relative step between neighbours is reported
per element, which shows which force has a jump.
"""
import sys

import numpy as np
import opensim as osim

osim.Logger.setLevelString("Error")
model_file, states_file = sys.argv[1:3]
opts = dict(a.split("=", 1) for a in sys.argv[3:])
n = int(opts.get("n", 3000))
extend = float(opts.get("extend", 0.0))

model = osim.Model(model_file)
for m_ in model.updMuscles():
    if m_.getConcreteClassName() == "Millard2012EquilibriumMuscle":
        mm_ = osim.Millard2012EquilibriumMuscle.safeDownCast(m_)
        mm_.set_ignore_activation_dynamics(True)
        mm_.set_ignore_tendon_compliance(True)
model.initSystem()
tbl = osim.TimeSeriesTable(states_file)
tbl.addTableMetaDataString("inDegrees", "no")
labels = list(tbl.getColumnLabels())
A = np.array([tbl.getRowAtIndex(0)[j] for j in range(len(labels))])
B = np.array([tbl.getRowAtIndex(1)[j] for j in range(len(labels))])
tA, tB = tbl.getIndependentColumn()[0], tbl.getIndependentColumn()[1]
print("states at t =", tA, tB)

ftbl = osim.TimeSeriesTable()
ftbl.setColumnLabels(labels)
ftbl.addTableMetaDataString("inDegrees", "no")
ss = np.linspace(-extend, 1 + extend, n)
for s in ss:
    ftbl.appendRow(float(tA + s * (tB - tA)), osim.RowVector((A + s * (B - A)).tolist()))
traj = osim.StatesTrajectory.createFromStatesTable(model, ftbl, True, True, False)

fs = model.getForceSet()
series = {}
for k in range(traj.getSize()):
    st = traj.get(k)
    for m_ in model.getMuscles():
        m_.overrideActuation(st, True)
        m_.setOverrideActuation(st, m_.getMaxIsometricForce() * 0.02)
    model.realizeDynamics(st)
    for i in range(fs.getSize()):
        f = fs.get(i)
        name = f.getName()
        cname = f.getConcreteClassName()
        try:
            if cname == "Smith2018ArticularContactForce":
                c = osim.Smith2018ArticularContactForce.safeDownCast(f)
                v = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_contact_force")).getValue(st)
                val = float(np.linalg.norm([v.get(0), v.get(1), v.get(2)]))
            elif cname == "Blankevoort1991Ligament":
                val = float(osim.OutputDouble.safeDownCast(f.getOutput("total_force")).getValue(st))
            else:
                rv = f.getRecordValues(st)
                val = float(sum(abs(rv.get(j)) for j in range(rv.size())))
        except Exception as e:
            continue
        series.setdefault(name, []).append(val)

print("%-34s %12s %12s %10s  at s" % ("force", "max", "max step", "step/max"))
rows = []
for name, v in series.items():
    v = np.array(v)
    if len(v) < 3 or v.max() == 0:
        continue
    d = np.abs(np.diff(v))
    rows.append((d.max() / v.max(), name, v.max(), d.max(), ss[d.argmax()]))
for r in sorted(rows, reverse=True)[:12]:
    print("%-34s %12.4g %12.4g %9.3f%%  s=%.4f" % (r[1], r[2], r[3], 100 * r[0], r[4]))
