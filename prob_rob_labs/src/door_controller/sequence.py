"""Open-loop stages, driven exclusively by elapsed simulation time."""

from dataclasses import dataclass
import math


# name: (default, explanation); these are startup parameters, not live tuning.
PARAMETERS = {
    'forward_speed': (0.2, 'Commanded forward speed in m/s'),
    'startup_delay': (3.0, 'Delay after command subscribers appear, in seconds'),
    'open_duration': (8.0, 'Time allowed for opening the door, in seconds'),
    'drive_duration': (22.0, 'Time driving straight through the doorway, in seconds'),
    'stop_duration': (1.0, 'Time stopped before closing the door, in seconds'),
    'close_duration': (8.0, 'Time allowed for closing the door, in seconds'),
    'open_torque': (5.0, 'Opening torque in N m (positive, at most 5)'),
    'close_torque': (-5.0, 'Closing torque in N m (negative, at least -5)'),
}


@dataclass(frozen=True)
class Stage:
    name: str
    duration: float
    speed: float
    torque: float


class Sequence:
    """Advance at most one stage per tick so no actuator stage gets skipped."""

    def __init__(self, values):
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('forward_speed', 'open_duration', 'drive_duration',
                     'stop_duration', 'close_duration'):
            if values[name] <= 0:
                raise ValueError(f'{name} must be positive')
        if values['startup_delay'] < 0:
            raise ValueError('startup_delay must be nonnegative')
        if not 0 < values['open_torque'] <= 5:
            raise ValueError('open_torque must be in (0, 5] N m')
        if not -5 <= values['close_torque'] < 0:
            raise ValueError('close_torque must be in [-5, 0) N m')

        self.stages = [
            Stage('STARTUP', values['startup_delay'], 0.0, 0.0),
            Stage('OPENING', values['open_duration'], 0.0, values['open_torque']),
            Stage('DRIVING', values['drive_duration'], values['forward_speed'],
                  values['open_torque']),
            Stage('STOPPING', values['stop_duration'], 0.0, values['open_torque']),
            Stage('CLOSING', values['close_duration'], 0.0, values['close_torque']),
            Stage('DONE', math.inf, 0.0, 0.0),
        ]
        self.index = 0
        self.started_at = None
        self.last_time = None

    def tick(self, now):
        if self.last_time is not None and now < self.last_time:
            self.index = len(self.stages) - 1
            self.last_time = now
            raise ValueError('Simulation clock moved backward; restart the controller')
        self.last_time = now
        if self.started_at is None:
            self.started_at = now
        if now - self.started_at >= self.stages[self.index].duration:
            self.index += 1
            self.started_at = now
        return self.stages[self.index]
