"""Check the Smith2018ContactMesh ray query against an exhaustive search.

    python test_contact_mesh.py <install_dir> [--rays 2000] [--old]

For every contact mesh of the smith2019 model (cartilage and menisci), rays are cast
from random points near the surface, in directions up to 60 deg from the inward normal,
with the same limits as the contact model (0 < distance < 0.01 m). The result of
rayIntersectionTest is compared with the nearest forward hit found by testing every
triangle (Moller-Trumbore, as in the C++ code). --old uses the absolute parallel
tolerance (1e-8) of the old code instead of the relative one (1e-6 * |e1| * |e2|).

Reported per mesh: rays where the hit/no-hit decision differs, where the distance
differs, and where the triangle index differs although the distance agrees
(the symptom of the old OBB bug: right distance, wrong triangle).
"""
import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL_DIR = os.path.join(REPO, "Bindings", "Java", "Matlab", "examples", "JAM",
                         "exampleWalkingMeniscus")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("install")
    ap.add_argument("--rays", type=int, default=2000)
    ap.add_argument("--old", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    os.environ["PATH"] = os.path.join(a.install, "bin") + os.pathsep + os.environ["PATH"]
    sys.path.insert(0, os.path.join(a.install, "sdk", "Python"))
    import opensim as osim

    os.chdir(MODEL_DIR)
    model = osim.Model("../models/healthy/smith2019/smith2019.osim")
    model.initSystem()
    rng = np.random.default_rng(a.seed)

    total_bad = 0
    for comp in model.getComponentsList():
        if comp.getConcreteClassName() != "Smith2018ContactMesh":
            continue
        mesh = osim.Smith2018ContactMesh.safeDownCast(comp)
        nf = mesh.getNumFaces()
        fv = mesh.getFaceVertexLocations()
        v = np.array([[[fv.get(i, j).get(k) for k in range(3)] for j in range(3)]
                      for i in range(nf)])
        v0, e1, e2 = v[:, 0], v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]
        e1n, e2n = np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1)
        centers_all = v.mean(axis=1)
        normals_all = np.cross(e1, e2)
        normals_all /= np.linalg.norm(normals_all, axis=1)[:, None]

        mismatch_hit = mismatch_dist = wrong_tri = 0
        n_hits = 0
        for _ in range(a.rays):
            i = int(rng.integers(nf))
            c = centers_all[i]
            n = normals_all[i]
            origin = c + n * rng.uniform(-0.002, 0.008)
            # direction: inward normal, tilted by up to 60 degrees
            t = rng.normal(size=3)
            t -= t.dot(n) * n
            t /= np.linalg.norm(t)
            ang = np.radians(rng.uniform(0, 60))
            d = -n * np.cos(ang) + t * np.sin(ang)
            d /= np.linalg.norm(d)

            ok = mesh.rayIntersectionTest(osim.Vec3(*origin), osim.UnitVec3(osim.Vec3(*d)), 0.0, 0.01)
            tri = mesh.getRayIntersectionTriangle()
            dist = mesh.getRayIntersectionDistance()

            h = np.cross(d, e2)
            det = np.einsum("ij,ij->i", e1, h)
            tol = 1e-8 if a.old else 1e-6 * e1n * e2n
            valid = np.abs(det) >= tol
            f = np.where(valid, 1.0 / np.where(valid, det, 1.0), 0.0)
            s = origin - v0
            u = f * np.einsum("ij,ij->i", s, h)
            q = np.cross(s, e1)
            w = f * (q @ d)
            tt = f * np.einsum("ij,ij->i", e2, q)
            hit = valid & (u >= 0) & (u <= 1) & (w >= 0) & (u + w <= 1) & (tt >= 0)
            if hit.any():
                best = int(np.argmin(np.where(hit, tt, np.inf)))
                bd = tt[best]
                ref_ok = 0.0 < bd < 0.01
            else:
                best, bd, ref_ok = -1, np.inf, False

            if ok != ref_ok:
                mismatch_hit += 1
            elif ok:
                n_hits += 1
                if abs(dist - bd) > 1e-9:
                    mismatch_dist += 1
                elif tri != best and not np.isclose(tt[tri], bd, atol=1e-12):
                    wrong_tri += 1
        bad = mismatch_hit + mismatch_dist + wrong_tri
        total_bad += bad
        print("%-26s faces %6d rays %d hits %4d | hit/no-hit differs %3d | "
              "distance differs %3d | wrong triangle %3d" %
              (mesh.getName(), nf, a.rays, n_hits, mismatch_hit, mismatch_dist, wrong_tri))
    print("TOTAL mismatches:", total_bad)
    sys.exit(1 if total_bad else 0)


if __name__ == "__main__":
    main()
