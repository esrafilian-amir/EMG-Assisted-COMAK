"""Smoothness of the contact force along a motion path (finely resampled).

    python rough.py <model.osim> <states.sto> <out.json> t0=1.7 t1=2.0 dt=0.0001 [method=.. voxel=.. taper=.. smooth=.. only=tf_contact]

The states of the .sto are resampled with a cubic spline at a step dt much smaller than
the output interval, and the contact force is recorded. A jump of the force between two
fine steps (relative to the peak force) shows a discontinuity of the contact model, which
is what makes a variable step integrator take steps of 1e-13.
"""
import json
import sys
import time

import numpy as np
import opensim as osim
from scipy.interpolate import CubicSpline

osim.Logger.setLevelString("Error")
model_file, states_file, out_file = sys.argv[1:4]
opts = dict(a.split("=", 1) for a in sys.argv[4:])
t0, t1, dt = float(opts.get("t0", 1.7)), float(opts.get("t1", 2.0)), float(opts.get("dt", 1e-4))
method = opts.get("method", "ray_cast")
voxel = float(opts.get("voxel", 0.001))
taper = float(opts.get("taper", 0.003))
smooth = int(opts.get("smooth", 0))
only = opts.get("only", "tf_contact").split(",")

model = osim.Model(model_file)
cg = model.getContactGeometrySet()
for i in range(cg.getSize()):
    g = cg.get(i)
    if g.getConcreteClassName() == "Smith2018ContactMesh":
        mm = osim.Smith2018ContactMesh.safeDownCast(g)
        if mm.get_use_variable_thickness():
            mm.set_thickness_smoothing_iterations(smooth)
contacts = []
fs = model.updForceSet()
for i in range(fs.getSize()):
    f = fs.get(i)
    if f.getConcreteClassName() == "Smith2018ArticularContactForce" and f.getName() in only:
        c = osim.Smith2018ArticularContactForce.safeDownCast(f)
        c.set_contact_method(method)
        c.set_field_voxel_size(voxel)
        c.set_field_edge_taper_width(taper)
        contacts.append(c)
# only the contact forces are needed: remove all other forces (muscles etc. are not
# in equilibrium in a resampled state and are not needed for the contact output)
fs2 = model.updForceSet()
for i in range(fs2.getSize() - 1, -1, -1):
    if fs2.get(i).getName() not in only:
        fs2.remove(i)
model.initSystem()

# read the states table and resample
tbl = osim.TimeSeriesTable(states_file)
labels = list(tbl.getColumnLabels())
times = np.array(tbl.getIndependentColumn())
data = np.array([[tbl.getRowAtIndex(k)[j] for j in range(len(labels))]
                 for k in range(tbl.getNumRows())])
cs = CubicSpline(times, data, axis=0)
tf = np.arange(t0, t1, dt)
fine = cs(tf)
ftbl = osim.TimeSeriesTable()
ftbl.setColumnLabels(labels)
ftbl.addTableMetaDataString('inDegrees', 'no')
for k in range(len(tf)):
    ftbl.appendRow(float(tf[k]), osim.RowVector(fine[k].tolist()))
traj = osim.StatesTrajectory.createFromStatesTable(model, ftbl, True, True, False)

t_start = time.time()
res = {"opts": opts, "t": tf.tolist(), "force": {c.getName(): [] for c in contacts},
       "max_pressure": {c.getName(): [] for c in contacts}}
for k in range(traj.getSize()):
    s = traj.get(k)
    model.realizeDynamics(s)
    for c in contacts:
        fv = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_contact_force")).getValue(s)
        res["force"][c.getName()].append([fv.get(0), fv.get(1), fv.get(2)])
        res["max_pressure"][c.getName()].append(osim.OutputDouble.safeDownCast(
            c.getOutput("casting_total_max_pressure")).getValue(s))
res["seconds"] = time.time() - t_start
json.dump(res, open(out_file, "w"))

for name, F in res["force"].items():
    F = np.array(F)
    mag = np.linalg.norm(F, axis=1)
    d = np.linalg.norm(np.diff(F, axis=0), axis=1)
    pk = mag.max() if mag.max() > 0 else 1
    big = np.where(d > 0.005 * pk)[0]
    print("%s %s: peak %.0f N, max step %.3f%% of peak (t=%.5f), %d steps above 0.5%%, "
          "99.9pct %.4f%%, %.0fs" % (opts.get("tag", ""), name, pk, 100 * d.max() / pk,
                                      tf[d.argmax()], len(big),
                                      100 * np.percentile(d, 99.9) / pk, res["seconds"]))
