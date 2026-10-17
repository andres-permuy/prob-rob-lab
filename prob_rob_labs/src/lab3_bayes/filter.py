"""Binary Bayes filter for the open/closed door state."""

import math


def feature_says_open(feature_mean, threshold):
    """Return whether a finite feature mean is on the open side of threshold."""
    return math.isfinite(feature_mean) and feature_mean < threshold


class DoorBayesFilter:
    """Track the probability that the door is open."""

    def __init__(self, prior_open, p_z_open_given_open,
                 p_z_open_given_closed):
        self._validate_probability('prior_open', prior_open)
        self._validate_likelihood(
            'p_z_open_given_open', p_z_open_given_open)
        self._validate_likelihood(
            'p_z_open_given_closed', p_z_open_given_closed)
        self.belief_open = prior_open
        self.p_z_open_given_open = p_z_open_given_open
        self.p_z_open_given_closed = p_z_open_given_closed

    @staticmethod
    def _validate_probability(name, value):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f'{name} must be a finite probability in [0, 1]')

    @staticmethod
    def _validate_likelihood(name, value):
        if not math.isfinite(value) or not 0.0 < value < 1.0:
            raise ValueError(f'{name} must be a finite probability in (0, 1)')

    def predict_open_command(self, p_open_given_closed,
                             p_open_given_open=1.0):
        """Apply the door-opening command transition model."""
        self._validate_probability(
            'p_open_given_closed', p_open_given_closed)
        self._validate_probability('p_open_given_open', p_open_given_open)
        prior = self.belief_open
        self.belief_open = (
            p_open_given_open * prior
            + p_open_given_closed * (1.0 - prior)
        )
        return prior, self.belief_open

    def update(self, observed_open):
        """Apply the camera measurement model and normalize the belief."""
        prior = self.belief_open
        if observed_open:
            likelihood_open = self.p_z_open_given_open
            likelihood_closed = self.p_z_open_given_closed
        else:
            likelihood_open = 1.0 - self.p_z_open_given_open
            likelihood_closed = 1.0 - self.p_z_open_given_closed

        open_weight = likelihood_open * prior
        closed_weight = likelihood_closed * (1.0 - prior)
        self.belief_open = open_weight / (open_weight + closed_weight)
        return prior, self.belief_open
