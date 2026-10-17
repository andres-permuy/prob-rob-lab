"""Verify command ordering and elapsed-time behavior without running Gazebo."""

import math

from door_controller.sequence import PARAMETERS, Sequence
import pytest


def settings(**changes):
    values = {name: default for name, (default, _) in PARAMETERS.items()}
    values.update(changes)
    return values


def test_complete_traversal_and_parameterized_speed():
    sequence = Sequence(settings(forward_speed=0.35))
    expected = [
        (100.0, 'STARTUP', 0.0, 0.0),
        (102.9, 'STARTUP', 0.0, 0.0),
        (103.0, 'OPENING', 0.0, 5.0),
        (110.9, 'OPENING', 0.0, 5.0),
        (111.0, 'DRIVING', 0.35, 5.0),
        (132.9, 'DRIVING', 0.35, 5.0),
        (133.0, 'STOPPING', 0.0, 5.0),
        (134.0, 'CLOSING', 0.0, -5.0),
        (142.0, 'DONE', 0.0, 0.0),
        (1000.0, 'DONE', 0.0, 0.0),
    ]
    for now, name, speed, torque in expected:
        feature_mean = 260.0 if now == 111.0 else None
        stage = sequence.tick(now, feature_mean=feature_mean)
        assert (stage.name, stage.speed, stage.torque) == (name, speed, torque)


def test_paused_simulation_does_not_advance():
    sequence = Sequence(settings())
    sequence.tick(0.0)
    for _ in range(100):
        assert sequence.tick(0.0).name == 'STARTUP'
    assert sequence.tick(3.0).name == 'OPENING'


def test_opening_waits_for_open_measurement_without_a_timeout():
    sequence = Sequence(settings())
    sequence.tick(0.0)
    assert sequence.tick(100.0).name == 'OPENING'
    assert sequence.tick(107.9).name == 'OPENING'
    assert sequence.tick(1008.0, feature_mean=312.5).name == 'OPENING'
    assert sequence.tick(1008.1, feature_mean=262.5).name == 'DRIVING'


def test_clock_reset_aborts_instead_of_repeating_motion():
    sequence = Sequence(settings())
    sequence.tick(0.0)
    sequence.tick(3.0)
    assert sequence.tick(11.0, feature_mean=262.5).name == 'DRIVING'
    with pytest.raises(ValueError, match='clock moved backward'):
        sequence.tick(0.0)
    stopped = sequence.tick(1.0)
    assert (stopped.name, stopped.speed, stopped.torque) == ('DONE', 0.0, 0.0)


@pytest.mark.parametrize('name,value', [
    ('forward_speed', 0.0), ('forward_speed', -0.2),
    ('forward_speed', math.nan), ('forward_speed', math.inf),
    ('startup_delay', -1.0), ('feature_threshold', 0.0),
    ('drive_duration', -1.0), ('stop_duration', 0.0),
    ('close_duration', 0.0), ('open_torque', 0.0),
    ('open_torque', 5.1), ('close_torque', 1.0), ('close_torque', -5.1),
])
def test_invalid_parameters_are_rejected(name, value):
    with pytest.raises(ValueError, match=name):
        Sequence(settings(**{name: value}))
