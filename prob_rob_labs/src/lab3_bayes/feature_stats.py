"""Lab 3 Assignment 5: count threshold classifications in a time window."""

import math

from lab3_bayes.filter import feature_says_open
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class FeatureStats(Node):
    def __init__(self):
        super().__init__('feature_stats')
        self.declare_parameter('threshold', 287.5)
        self.declare_parameter('window_seconds', 30.0)
        self.declare_parameter('actual_state', 'closed')

        self.threshold = self.get_parameter('threshold').value
        self.window_seconds = self.get_parameter('window_seconds').value
        self.actual_state = self.get_parameter('actual_state').value.lower()
        if not math.isfinite(self.threshold):
            raise ValueError('threshold must be finite')
        if not math.isfinite(self.window_seconds) or self.window_seconds <= 0.0:
            raise ValueError('window_seconds must be positive and finite')
        if self.actual_state not in ('open', 'closed'):
            raise ValueError('actual_state must be "open" or "closed"')

        self.started_at = None
        self.open_count = 0
        self.closed_count = 0
        self.finished = False
        self.subscription = self.create_subscription(
            Float64, '/feature_mean', self.handle_measurement, 10)
        self.get_logger().info(
            f'Collecting {self.window_seconds:.1f} s with actual_state='
            f'{self.actual_state}, threshold={self.threshold:.3f}.')

    def handle_measurement(self, message):
        if self.finished or not math.isfinite(message.data):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started_at is None:
            self.started_at = now

        if feature_says_open(message.data, self.threshold):
            self.open_count += 1
        else:
            self.closed_count += 1

        if now - self.started_at >= self.window_seconds:
            self.report()

    def report(self):
        self.finished = True
        total = self.open_count + self.closed_count
        p_open = self.open_count / total if total else float('nan')
        p_closed = self.closed_count / total if total else float('nan')
        # Add-one (Laplace) estimates avoid exact 0/1 likelihoods when the
        # finite experiment happens not to contain a rare error.
        laplace_open = (self.open_count + 1) / (total + 2)
        laplace_closed = (self.closed_count + 1) / (total + 2)
        self.get_logger().info(
            'LAB3_STATS '
            f'actual_state={self.actual_state} total={total} '
            f'z_open={self.open_count} z_closed={self.closed_count} '
            f'P(z=open|x={self.actual_state})={p_open:.6f} '
            f'P(z=closed|x={self.actual_state})={p_closed:.6f} '
            f'laplace_P(z=open|x={self.actual_state})={laplace_open:.6f} '
            f'laplace_P(z=closed|x={self.actual_state})='
            f'{laplace_closed:.6f}')
        self.get_logger().info('Window complete; save this output and press Ctrl+C.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FeatureStats()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
