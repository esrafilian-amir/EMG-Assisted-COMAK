"""Minimal reader for OpenSim .sto/.mot files."""
import numpy as np


def read_sto(path):
    """Return (labels, data) where labels[0] is 'time' and data is (rows, cols)."""
    with open(path) as f:
        lines = f.read().splitlines()
    start = next(i for i, l in enumerate(lines) if l.strip().lower() == "endheader")
    labels = lines[start + 1].split()
    rows = [[float(x) for x in l.split()] for l in lines[start + 2:] if l.strip()]
    return labels, np.array(rows, dtype=float).reshape(len(rows), len(labels))
