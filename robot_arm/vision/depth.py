"""Depth estimation utilities.

Provides two simple strategies:
- size_based_depth: assumes known object width in cm and focal length in pixels
- stereo_triangulation: placeholder for stereo pairs (not implemented)
"""
import math

def size_based_depth(observed_px_width, known_width_cm, focal_length_px):
    """Estimate distance (cm) from camera by similar triangles: Z = (f * W) / w

    observed_px_width: width in pixels
    known_width_cm: real world width
    focal_length_px: camera focal length in pixels
    Returns distance in same units as known_width_cm (cm)
    """
    if observed_px_width <= 0:
        return None
    z = (focal_length_px * known_width_cm) / observed_px_width
    return z

def stereo_triangulation(pt_left, pt_right, cam_left_params, cam_right_params):
    """Placeholder: triangulate points given calibrated stereo cameras.
    Returns (X,Y,Z) in camera coordinate frame.
    """
    raise NotImplementedError("Stereo triangulation not implemented. Provide camera calibration and implement triangulation.")
