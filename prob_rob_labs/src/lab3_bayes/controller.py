"""Lab 3 threshold, Bayesian-inference, and full Bayes-filter controller."""

import math

from geometry_msgs.msg import TwistStamped
from lab3_bayes.filter import DoorBayesFilter, feature_says_open
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


class Lab3Controller(Node):
    def __init__(self):
        super().__init__('lab3_controller')
        defaults = {
            'mode': 'threshold',
            'feature_threshold': 287.5,
            'prior_open': 0.5,
            'p_z_open_given_open': 0.95,
            'p_z_open_given_closed': 0.05,
            'open_command_success': 0.2,
            'decision_probability': 0.99,
            'measurement_period': 0.5,
            'command_period': 3.5,
            'startup_delay': 1.0,
            'forward_speed': 0.2,
            'drive_duration': 22.0,
            'stop_duration': 1.0,
            'close_duration': 8.0,
            'open_torque': 5.0,
            'close_torque': -5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {
            name: self.get_parameter(name).value for name in defaults
        }
        self._validate_settings()

        self.mode = self.settings['mode'].lower()
        self.filter = DoorBayesFilter(
            self.settings['prior_open'],
            self.settings['p_z_open_given_open'],
            self.settings['p_z_open_given_closed'])
        self.feature_subscription = self.create_subscription(
            Float64, '/feature_mean', self.handle_feature, 10)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        self.torque_pub = self.create_publisher(
            Float64, '/hinged_glass_door/torque', 10)
        self.open_command_pub = self.create_publisher(Empty, '/door_open', 10)

        self.state = 'STARTUP'
        self.state_started_at = None
        self.last_measurement_at = None
        self.last_command_at = None
        self.open_is_confident = False
        self.last_feature = None
        self.timer = self.create_timer(0.05, self.heartbeat)
        self.get_logger().info(
            f'Started in mode={self.mode}; feature threshold='
            f'{self.settings["feature_threshold"]:.3f}; decision probability='
            f'{self.settings["decision_probability"]:.6f}.')
        if self.mode != 'threshold':
            self.get_logger().info(
                'Measurement model: '
                f'P(z=open|x=open)={self.settings["p_z_open_given_open"]:.6f}, '
                f'P(z=open|x=closed)='
                f'{self.settings["p_z_open_given_closed"]:.6f}.')

    def _validate_settings(self):
        mode = self.settings['mode'].lower()
        if mode not in ('threshold', 'bayes', 'full'):
            raise ValueError('mode must be threshold, bayes, or full')
        positive = (
            'feature_threshold', 'decision_probability',
            'measurement_period', 'command_period', 'forward_speed',
            'drive_duration', 'stop_duration', 'close_duration',
            'open_torque')
        for name in positive:
            value = self.settings[name]
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
        if not 0.0 < self.settings['decision_probability'] < 1.0:
            raise ValueError('decision_probability must be in (0, 1)')
        for name in ('prior_open', 'open_command_success'):
            value = self.settings[name]
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be in [0, 1]')
        for name in ('p_z_open_given_open', 'p_z_open_given_closed'):
            value = self.settings[name]
            if not math.isfinite(value) or not 0.0 < value < 1.0:
                raise ValueError(f'{name} must be in (0, 1)')
        if self.settings['startup_delay'] < 0.0:
            raise ValueError('startup_delay must be nonnegative')
        if not -5.0 <= self.settings['close_torque'] < 0.0:
            raise ValueError('close_torque must be in [-5, 0)')
        if not 0.0 < self.settings['open_torque'] <= 5.0:
            raise ValueError('open_torque must be in (0, 5]')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def handle_feature(self, message):
        if self.state != 'OPENING' or not math.isfinite(message.data):
            return
        now = self.now_seconds()
        if (self.last_measurement_at is not None
                and now - self.last_measurement_at
                < self.settings['measurement_period']):
            return
        self.last_measurement_at = now
        self.last_feature = message.data
        observed_open = feature_says_open(
            message.data, self.settings['feature_threshold'])

        if self.mode == 'threshold':
            self.open_is_confident = observed_open
            self.get_logger().info(
                f'MEASUREMENT z={"open" if observed_open else "closed"} '
                f'feature_mean={message.data:.3f}')
            return

        prior, posterior = self.filter.update(observed_open)
        self.get_logger().info(
            f'MEASUREMENT z={"open" if observed_open else "closed"} '
            f'feature_mean={message.data:.3f} prior={prior:.9f} '
            f'posterior={posterior:.9f}')
        self.open_is_confident = (
            posterior >= self.settings['decision_probability'])

    def publish_commands(self, speed=0.0, torque=None):
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = 'base_footprint'
        velocity.twist.linear.x = speed
        self.velocity_pub.publish(velocity)
        if torque is not None:
            self.torque_pub.publish(Float64(data=torque))

    def enter_state(self, state, now):
        self.state = state
        self.state_started_at = now
        self.get_logger().info(
            f'STATE {state}; belief_open={self.filter.belief_open:.9f}')

    def send_open_command(self, now):
        prior, predicted = self.filter.predict_open_command(
            self.settings['open_command_success'])
        self.open_command_pub.publish(Empty())
        self.last_command_at = now
        self.get_logger().info(
            f'PREDICTION open_command prior={prior:.9f} '
            f'posterior={predicted:.9f} success_probability='
            f'{self.settings["open_command_success"]:.6f}')

    def heartbeat(self):
        now = self.now_seconds()
        if self.state_started_at is None:
            self.state_started_at = now

        if self.state == 'STARTUP':
            self.publish_commands(torque=0.0)
            if now - self.state_started_at >= self.settings['startup_delay']:
                self.enter_state('OPENING', now)
            return

        if self.state == 'OPENING':
            if self.mode == 'full':
                self.publish_commands()
                if (self.last_command_at is None
                        or now - self.last_command_at
                        >= self.settings['command_period']):
                    self.send_open_command(now)
            else:
                self.publish_commands(torque=self.settings['open_torque'])

            if self.open_is_confident:
                self.enter_state('DRIVING', now)
            return

        if self.state == 'DRIVING':
            torque = None if self.mode == 'full' else self.settings['open_torque']
            self.publish_commands(self.settings['forward_speed'], torque)
            if now - self.state_started_at >= self.settings['drive_duration']:
                self.enter_state('STOPPING', now)
            return

        if self.state == 'STOPPING':
            torque = None if self.mode == 'full' else self.settings['open_torque']
            self.publish_commands(torque=torque)
            if now - self.state_started_at >= self.settings['stop_duration']:
                self.enter_state('CLOSING', now)
            return

        if self.state == 'CLOSING':
            self.publish_commands(torque=self.settings['close_torque'])
            if now - self.state_started_at >= self.settings['close_duration']:
                self.enter_state('DONE', now)
            return

        self.publish_commands(torque=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Lab3Controller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_commands(torque=0.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
