"""Trajectory utilities: simple linear interpolation and smoothing.
"""
import numpy as np

def lerp(a, b, t):
    return a + (b - a) * t

def interpolate_angles(start_angles, end_angles, steps=20):
    """Linearly interpolate between two angle tuples.

    start_angles, end_angles: iterable of same length
    returns: list of tuples length=steps
    """
    start = np.array(start_angles, dtype=float)
    end = np.array(end_angles, dtype=float)
    out = []
    for i in range(1, steps+1):
        t = i / float(steps)
        out.append(tuple((start * (1-t) + end * t).tolist()))
    return out
