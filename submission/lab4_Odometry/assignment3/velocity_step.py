#!/usr/bin/env python3
"""Publish linear and angular velocity steps"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class StepCommands(Node):
    """Send a timed staircase of velocity commands to the TurtleBot."""

    def __init__(self):
        """Initialize the command sequence and ROS publisher."""
        super().__init__('lab4_step_commands')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # linear x [m/s], angular z [rad/s]
        self.commands = [
            (0.00, 0.00),
            (0.03, 0.00),
            (0.06, 0.00),
            (0.09, 0.00),
            (0.12, 0.00),
            (0.15, 0.00),
            (0.00, 0.00),
            (0.00, 0.10),
            (0.00, 0.20),
            (0.00, 0.30),
            (0.00, 0.40),
            (0.00, 0.00),
        ]
        self.hold_seconds = 2.0
        self.start_ns = None
        self.last_step = -1
        self.complete = False

        # Continue publishing to avoid command timeout.
        self.timer = self.create_timer(0.02, self.publish_step)

    def publish_step(self):
        """Publish the command selected by elapsed simulation time."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # Waiting for the sim to publish

        if self.start_ns is None:
            self.start_ns = now.nanoseconds

        elapsed = (now.nanoseconds - self.start_ns) / 1e9
        step = int(elapsed / self.hold_seconds)
        if step >= len(self.commands):
            self.get_logger().info('Test complete. Stopping the bag.')
            self.timer.cancel()
            self.complete = True
            return

        v, omega = self.commands[step]
        if step != self.last_step:
            self.get_logger().info(f'Step {step}: u_v={v:.2f} m/s, u_omega={omega:.2f} rad/s')
            self.last_step = step

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = StepCommands()
    try:
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()