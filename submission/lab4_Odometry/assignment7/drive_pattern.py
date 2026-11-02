#!/usr/bin/env python3
"""Drive a fixed ~70 s pattern of straight segments, arcs, a spin, and a stop.

Used for the Assignment 7-9 evaluation run. Requires the simulation to be
running; timing follows simulation time. Run from any directory:
    python3 drive_pattern.py
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

# (duration [s], linear x [m/s], angular z [rad/s], label)
PATTERN = [
    (5.0, 0.00, 0.0, 'parked'),
    (10.0, 0.15, 0.0, 'straight'),
    (10.0, 0.15, 0.4, 'arc left'),
    (10.0, 0.15, 0.0, 'straight'),
    (10.0, 0.15, -0.4, 'arc right'),
    (6.0, 0.00, 0.5, 'spin in place'),
    (10.0, 0.15, 0.0, 'straight'),
    (10.0, 0.00, 0.0, 'parked'),
]


class DrivePattern(Node):
    """Publish the pattern on /cmd_vel at 10 Hz, switching segments on sim time."""

    def __init__(self):
        super().__init__('drive_pattern', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.start_ns = None
        self.segment = -1
        self.complete = False
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        """Publish the command for the current segment of the pattern."""
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # waiting for /clock from the simulator
        if self.start_ns is None:
            self.start_ns = now.nanoseconds

        elapsed = (now.nanoseconds - self.start_ns) / 1e9
        end = 0.0
        for index, (duration, v, omega, label) in enumerate(PATTERN):
            end += duration
            if elapsed < end:
                break
        else:
            self.get_logger().info('Pattern complete.')
            self.complete = True
            return

        if index != self.segment:
            self.segment = index
            self.get_logger().info(
                f't={elapsed:5.1f} s: {label} (v={v:.2f} m/s, w={omega:.2f} rad/s)')

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DrivePattern()
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
