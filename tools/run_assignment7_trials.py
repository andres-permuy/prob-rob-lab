#!/usr/bin/env python3
"""Automate Lab 3 Assignment 7 flaky-door-opener measurements."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


class Assignment7Trials(Node):
    """Command the flaky opener repeatedly and close the door after successes."""

    def __init__(self):
        super().__init__('assignment7_trials')
        defaults = {
            'trials': 25,
            'initial_close_seconds': 5.0,
            'result_timeout_seconds': 1.0,
            'open_wait_seconds': 4.0,
            'close_seconds': 8.0,
            'inter_trial_seconds': 1.0,
            'close_torque': -5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        self._validate_settings()

        self.open_command_pub = self.create_publisher(Empty, '/door_open', 10)
        self.torque_pub = self.create_publisher(
            Float64, '/hinged_glass_door/torque', 10)
        self.torque_subscription = self.create_subscription(
            Float64, '/hinged_glass_door/torque', self.handle_torque, 10)

        self.trial_number = 0
        self.successes = 0
        self.failures = 0
        self.finished = False
        self.state = 'INITIAL_CLOSING'
        self.state_started_at = time.monotonic()
        self.timer = self.create_timer(0.05, self.heartbeat)
        self.get_logger().info(
            'ASSIGNMENT7_START '
            f'trials={self.settings["trials"]}; first ensuring door is closed.')

    def _validate_settings(self):
        trials = self.settings['trials']
        if not isinstance(trials, int) or isinstance(trials, bool) or trials <= 0:
            raise ValueError('trials must be a positive integer')
        for name in (
                'initial_close_seconds', 'result_timeout_seconds',
                'open_wait_seconds', 'close_seconds',
                'inter_trial_seconds'):
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
        close_torque = self.settings['close_torque']
        if not math.isfinite(close_torque) or not -5.0 <= close_torque < 0.0:
            raise ValueError('close_torque must be in [-5, 0)')

    def set_state(self, state):
        self.state = state
        self.state_started_at = time.monotonic()

    def publish_torque(self, torque):
        self.torque_pub.publish(Float64(data=float(torque)))

    def start_trial(self):
        self.trial_number += 1
        self.set_state('WAITING_FOR_RESULT')
        self.open_command_pub.publish(Empty())
        self.get_logger().info(
            f'ASSIGNMENT7_COMMAND trial={self.trial_number}')

    def handle_torque(self, message):
        # The flaky opener publishes +5.0 only when an open command succeeds.
        # This node publishes zero or a negative torque, so a positive sample
        # received during the result window unambiguously indicates success.
        if self.state != 'WAITING_FOR_RESULT' or message.data <= 0.0:
            return
        self.successes += 1
        self.get_logger().info(
            f'ASSIGNMENT7_TRIAL trial={self.trial_number} outcome=success '
            f'successes={self.successes} failures={self.failures}')
        self.set_state('WAITING_FOR_DOOR_TO_OPEN')

    def record_failure(self):
        self.failures += 1
        self.get_logger().info(
            f'ASSIGNMENT7_TRIAL trial={self.trial_number} outcome=failure '
            f'successes={self.successes} failures={self.failures}')
        if self.trial_number >= self.settings['trials']:
            self.finish()
        else:
            self.set_state('BETWEEN_TRIALS')

    def finish(self):
        self.publish_torque(0.0)
        total = self.successes + self.failures
        measured = self.successes / total
        self.get_logger().info(
            'ASSIGNMENT7_SUMMARY '
            f'total={total} successes={self.successes} failures={self.failures} '
            f'measured_probability={measured:.6f} '
            'actual_probability_from_code=0.200000')
        self.finished = True
        self.timer.cancel()

    def heartbeat(self):
        elapsed = time.monotonic() - self.state_started_at

        if self.state == 'INITIAL_CLOSING':
            self.publish_torque(self.settings['close_torque'])
            if elapsed >= self.settings['initial_close_seconds']:
                self.publish_torque(0.0)
                self.set_state('BETWEEN_TRIALS')
            return

        if self.state == 'BETWEEN_TRIALS':
            self.publish_torque(0.0)
            if elapsed >= self.settings['inter_trial_seconds']:
                self.start_trial()
            return

        if self.state == 'WAITING_FOR_RESULT':
            if elapsed >= self.settings['result_timeout_seconds']:
                self.record_failure()
            return

        if self.state == 'WAITING_FOR_DOOR_TO_OPEN':
            if elapsed >= self.settings['open_wait_seconds']:
                self.set_state('CLOSING')
            return

        if self.state == 'CLOSING':
            self.publish_torque(self.settings['close_torque'])
            if elapsed >= self.settings['close_seconds']:
                self.publish_torque(0.0)
                if self.trial_number >= self.settings['trials']:
                    self.finish()
                else:
                    self.set_state('BETWEEN_TRIALS')


def main(args=None):
    rclpy.init(args=args)
    node = Assignment7Trials()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_torque(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
