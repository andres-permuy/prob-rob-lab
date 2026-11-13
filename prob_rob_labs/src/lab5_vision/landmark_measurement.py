"""
Making this code to compete the algorithm needed for Asssignment #1
The goal is to estimate the landmark (lab cylinders) geometry from detected image points.
"""

import numpy as np


def landmark_pixel_geometry(points):
    """Estimate landmark pixel height and symmetry-axis position.

    Args:
        points: Detected [x, y] pixel coordinates, shape (N, 2).

    Returns:
        Tuple (delta_y_p, x_p), in pixels, or None for invalid input.
        Assumes the points capture the fully visible landmark's bounds.
    """
    pixels = np.asarray(points, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) < 2:
        return None
    if not np.isfinite(pixels).all():
        return None

    # Bounding coordinates of the detected landmark.
    x_min, y_min = pixels.min(axis=0)
    x_max, y_max = pixels.max(axis=0)
    delta_y_p = y_max - y_min
    if delta_y_p <= 0.0 or x_max <= x_min:
        return None

    # Using horizontal midpoint to define vertical symmetry axis.
    x_p = (x_min + x_max) / 2.0
    return float(delta_y_p), float(x_p)