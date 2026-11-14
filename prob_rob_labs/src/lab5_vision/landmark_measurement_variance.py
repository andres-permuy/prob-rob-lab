"""Assignment 4: empirical measurement variances from the cyan rollout."""

import math


def landmark_measurement_variance(distance):
    """Estimate distance and bearing variances from measured distance.

    Args:
        distance: Positive, finite measured camera-to-landmark distance in metres.

    Returns:
        Tuple (distance variance in m^2, bearing variance in rad^2).

    The cyan-rollout fit covers measured distances of 0.95--10.55 m and
    bearings near +/-0.52 rad. Behavior outside that coverage is unvalidated.
    """
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError('distance must be positive and finite.')

    # Rounded fit to centered distance-error variances in distance intervals.
    distance_variance = 1.6e-4 * distance**2
    
    # Rounded centered bearing-error variance over the automated rollout.
    bearing_variance = 1.0e-6
    return distance_variance, bearing_variance
