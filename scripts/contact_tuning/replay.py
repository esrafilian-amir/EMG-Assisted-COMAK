"""Replay saved states of a walking trial and record the contact outputs for one
set of contact settings.

    python replay.py <model.osim> <states.sto> <out.json> [key=value ...]

keys: method (ray_cast|distance_field), voxel, taper, smooth, formulation, tag
Contact settings are applied to every Smith2018ArticularContactForce in the model
(smooth to every Smith2018ContactMesh with variable thickness).
For each contact and each state: force vector (casting_total_contact_force),
max pressure, mean pressure and contact area.
"""
import json
import sys
import time

import numpy as np
import opensim as osim

osim.Logger.setLevelString("Error")

model_file, states_file, out_file = sys.argv[1:4]
opts = dict(a.split("=", 1) for a in sys.argv[4:])
method = opts.get("method", "ray_cast")
voxel = float(opts.get("voxel", 0.001))
taper = float(opts.get("taper", 0.003))
smooth = int(opts.get("smooth", 0))
only = opts.get("only", "")  # comma separated contact names, default all cartilage ones

t0 = time.time()
model = osim.Model(model_file)
meshes = model.getContactGeometrySet()
for i in range(meshes.getSize()):
    g = meshes.get(i)
    if g.getConcreteClassName() == "Smith2018ContactMesh":
        mm = osim.Smith2018ContactMesh.safeDownCast(g)
        if mm.get_use_variable_thickness():
            mm.set_thickness_smoothing_iterations(smooth)

contacts = []
fs = model.updForceSet()
for i in range(fs.getSize()):
    f = fs.get(i)
    if f.getConcreteClassName() == "Smith2018ArticularContactForce":
        c = osim.Smith2018ArticularContactForce.safeDownCast(f)
        c.set_contact_method(method if (not opts.get("sdfonly") or c.getName() in opts["sdfonly"].split(",")) else "ray_cast")
        c.set_field_voxel_size(voxel)
        c.set_field_edge_taper_width(taper)
        if "formulation" in opts:
            c.set_elastic_foundation_formulation(opts["formulation"])
        contacts.append(c)
if only:
    keep = only.split(",")
    contacts = [c for c in contacts if c.getName() in keep]

state0 = model.initSystem()
sto = osim.Storage(states_file)
traj = osim.StatesTrajectory.createFromStatesStorage(model, sto, True, True)
n = traj.getSize()

res = {"opts": opts, "time": [], "contacts": {}}
for c in contacts:
    res["contacts"][c.getName()] = {"force": [], "cop": [], "max_pressure": [],
                                    "mean_pressure": [], "area": []}
for k in range(n):
    s = traj.get(k)
    model.realizeReport(s)
    res["time"].append(s.getTime())
    for c in contacts:
        d = res["contacts"][c.getName()]
        fv = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_contact_force")).getValue(s)
        d["force"].append([fv.get(0), fv.get(1), fv.get(2)])
        cv = osim.OutputVec3.safeDownCast(c.getOutput("casting_total_center_of_pressure")).getValue(s)
        d["cop"].append([cv.get(0), cv.get(1), cv.get(2)])
        d["max_pressure"].append(osim.OutputDouble.safeDownCast(c.getOutput("casting_total_max_pressure")).getValue(s))
        d["mean_pressure"].append(osim.OutputDouble.safeDownCast(c.getOutput("casting_total_mean_pressure")).getValue(s))
        d["area"].append(osim.OutputDouble.safeDownCast(c.getOutput("casting_total_contact_area")).getValue(s))
res["seconds"] = time.time() - t0
json.dump(res, open(out_file, "w"))
print("done", out_file, n, "states,", round(res["seconds"], 1), "s")
