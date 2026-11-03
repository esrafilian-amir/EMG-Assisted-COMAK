"""Record model accelerations at fixed poses so two installs can be compared.

    python contact_force_equivalence.py <install_dir> <out.npz>
    python contact_force_equivalence.py --compare a.npz b.npz

The accelerations of all coordinates of the smith2019 model depend on every applied
force, including the Smith2018 contact forces, so equal accelerations (to round-off)
mean the contact forces applied to the bodies are equal. Poses: random knee flexion
(0 to 60 deg) and small random offsets of the other knee, patella and meniscus coordinates.
"""
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL_DIR = os.path.join(REPO, "Bindings", "Java", "Matlab", "examples", "JAM",
                         "exampleWalkingMeniscus")


def record(install, out):
    os.environ["PATH"] = os.path.join(install, "bin") + os.pathsep + os.environ["PATH"]
    sys.path.insert(0, os.path.join(install, "sdk", "Python"))
    import opensim as osim

    os.chdir(MODEL_DIR)
    model = osim.Model("../models/healthy/smith2019/smith2019.osim")
    state = model.initSystem()
    rng = np.random.default_rng(7)
    coords = model.getCoordinateSet()
    names = [coords.get(i).getName() for i in range(coords.getSize())]
    secondary = [n for n in names if n.startswith(("knee_", "pf_", "meniscus_"))
                 and n != "knee_flex_r"]
    acc = []
    for pose in range(12):
        state = model.initSystem()
        coords.get("knee_flex_r").setValue(state, np.radians(rng.uniform(0, 60)), False)
        for n in secondary:
            c = coords.get(n)
            span = 0.0005 if c.getMotionType() == osim.Coordinate.Translational else np.radians(0.5)
            c.setValue(state, c.getValue(state) + rng.uniform(-span, span), False)
        model.assemble(state)
        model.realizeAcceleration(state)
        acc.append([coords.get(n).getAccelerationValue(state) for n in names])
    np.savez(out, names=np.array(names), acc=np.array(acc))
    print("recorded", len(acc), "poses,", len(names), "coordinates ->", out)


def compare(a, b):
    A, B = np.load(a), np.load(b)
    d = np.abs(A["acc"] - B["acc"])
    scale = np.maximum(np.abs(A["acc"]), 1.0)
    rel = (d / scale).max()
    print("max |acc difference| %.3g, max relative to max(|acc|,1) %.3g" % (d.max(), rel))
    worst = np.unravel_index(np.argmax(d / scale), d.shape)
    print("worst: pose %d coordinate %s" % (worst[0], A["names"][worst[1]]))
    sys.exit(0 if rel < 1e-8 else 1)


if __name__ == "__main__":
    if sys.argv[1] == "--compare":
        compare(sys.argv[2], sys.argv[3])
    else:
        record(sys.argv[1], sys.argv[2])
