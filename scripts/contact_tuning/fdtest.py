"""Finite difference smoothness test of the contact force.

    python fdtest.py <model.osim> <states.sto> <out.json> [t0=1.7 t1=2.0 every=2 method=.. voxel=.. taper=.. smooth=.. only=tf_contact tag=..]

At states of the file, every knee and patellofemoral coordinate is perturbed by
+-h for h = 1e-5, 1e-7 and 1e-9 (m or rad) and the derivative of the contact force vector
is formed by central differences. If the force is smooth, the three derivatives agree;
a jump or a rough force (noise, a triangle entering or leaving) makes them differ.
"""
import json
import sys
import time

import numpy as np
import opensim as osim

osim.Logger.setLevelString("Error")
model_file, states_file, out_file = sys.argv[1:4]
opts = dict(a.split("=", 1) for a in sys.argv[4:])
t0, t1 = float(opts.get("t0", 1.7)), float(opts.get("t1", 2.0))
every = int(opts.get("every", 2))
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
fs = model.updForceSet()
for i in range(fs.getSize() - 1, -1, -1):
    f = fs.get(i)
    if f.getName() in only:
        c = osim.Smith2018ArticularContactForce.safeDownCast(f)
        c.set_contact_method(method)
        c.set_field_voxel_size(voxel)
        c.set_field_edge_taper_width(taper)
    else:
        fs.remove(i)
contacts = [osim.Smith2018ArticularContactForce.safeDownCast(fs.get(n)) for n in only]
model.initSystem()

tbl = osim.TimeSeriesTable(states_file)
tbl.addTableMetaDataString("inDegrees", "no")
traj = osim.StatesTrajectory.createFromStatesTable(model, tbl, True, True, False)
sv = model.getStateVariableNames()
names = [sv.get(i) for i in range(sv.getSize())]
dofs = [n for n in names if n.endswith("/value") and
        ("/knee_" in n or "/pf_" in n or "/knee_trans_r/" in n or "/knee_rots_r/" in n)
        and "knee_flex_r" not in n]
hs = [1e-5, 1e-7, 1e-9]


def force(s):
    model.realizeDynamics(s)
    fv = osim.OutputVec3.safeDownCast(contacts[0].getOutput("casting_total_contact_force")).getValue(s)
    return np.array([fv.get(0), fv.get(1), fv.get(2)])


t_start = time.time()
rows = []
for k in range(traj.getSize()):
    s = traj.get(k)
    if not (t0 <= s.getTime() <= t1) or k % every:
        continue
    F0 = force(s)
    for n in dofs:
        v0 = model.getStateVariableValue(s, n)
        J = []
        for h in hs:
            model.setStateVariableValue(s, n, v0 + h)
            fp = force(s)
            model.setStateVariableValue(s, n, v0 - h)
            fm = force(s)
            J.append((fp - fm) / (2 * h))
        model.setStateVariableValue(s, n, v0)
        J = np.array(J)
        scale = max(np.linalg.norm(J[0]), np.linalg.norm(J[1]), 1e-3)
        rows.append({"t": s.getTime(), "dof": n, "F": np.linalg.norm(F0),
                     "J": np.linalg.norm(J[0]), "err_1e7": np.linalg.norm(J[1] - J[0]) / scale,
                     "err_1e9": np.linalg.norm(J[2] - J[0]) / scale})
json.dump({"opts": opts, "rows": rows, "seconds": time.time() - t_start}, open(out_file, "w"))
e7 = np.array([r["err_1e7"] for r in rows])
e9 = np.array([r["err_1e9"] for r in rows])
print("%s: %d derivatives, median err(1e-7) %.2e, 95pct %.2e, max %.2e | err(1e-9) median %.2e 95pct %.2e max %.2e | %.0fs"
      % (opts.get("tag", ""), len(rows), np.median(e7), np.percentile(e7, 95), e7.max(),
         np.median(e9), np.percentile(e9, 95), e9.max(), time.time() - t_start))
