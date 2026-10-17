"""Unit tests for the Lab 3 binary Bayes filter."""

from lab3_bayes.filter import DoorBayesFilter, feature_says_open
import pytest


def test_feature_threshold_matches_report_convention():
    assert feature_says_open(262.5, 287.5)
    assert not feature_says_open(312.5, 287.5)
    assert not feature_says_open(float('nan'), 287.5)


def test_open_measurement_raises_belief_using_measurement_model():
    door_filter = DoorBayesFilter(0.5, 0.9, 0.2)
    prior, posterior = door_filter.update(observed_open=True)
    assert prior == 0.5
    assert posterior == pytest.approx(0.9 / 1.1)


def test_closed_measurement_lowers_belief_using_measurement_model():
    door_filter = DoorBayesFilter(0.5, 0.9, 0.2)
    _, posterior = door_filter.update(observed_open=False)
    assert posterior == pytest.approx(0.1 / 0.9)


def test_repeated_flaky_open_commands_accumulate_probability():
    door_filter = DoorBayesFilter(0.0, 0.9, 0.2)
    door_filter.predict_open_command(0.2)
    assert door_filter.belief_open == pytest.approx(0.2)
    door_filter.predict_open_command(0.2)
    assert door_filter.belief_open == pytest.approx(0.36)


@pytest.mark.parametrize('value', [-0.1, 1.1, float('nan')])
def test_invalid_prior_is_rejected(value):
    with pytest.raises(ValueError, match='prior_open'):
        DoorBayesFilter(value, 0.9, 0.2)
