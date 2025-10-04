"""Tracker/handoff logic between Camera A (global) and Camera B (close-up).

This file contains helpers to compute arm-centric coordinates and decide when to switch cameras.
"""
import math

def image_to_arm_coords(cx, cy, z, cam_params, arm_offset=(0,0,0)):
    """Convert image pixel coordinates + depth into approximate arm-base coordinates.

    cam_params should contain camera intrinsics and an extrinsic transform to arm base if available.
    This function currently assumes camera is mounted at the arm base origin and aligned.
    Returns (x_cm, y_cm, z_cm) in arm-base frame.
    TODO: Replace with calibrated extrinsics for accuracy.
    """
    fx = cam_params.get("fx")
    fy = cam_params.get("fy", fx)
    cx0 = cam_params.get("cx")
    cy0 = cam_params.get("cy")

    # simple pinhole back-projection: X = (u-cx)*Z/fx, Y = (v-cy)*Z/fy
    x = (cx - cx0) * z / fx
    y = (cy - cy0) * z / fy
    z = z

    # apply arm offset
    ox, oy, oz = arm_offset
    return x + ox, y + oy, z + oz

def should_switch_to_closeup(distance_cm, threshold=10.0):
    """Decide when to switch to Camera B. Default: when target closer than threshold cm."""
    return distance_cm is not None and distance_cm <= threshold
