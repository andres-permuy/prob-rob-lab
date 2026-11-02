#!/usr/bin/env python3
"""Lab 4, Assignment 8: error between the EKF odometry and ground truth."""

import math

import message_filters
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


def yaw_from_quaternion(q):
    """Return the rotation about z (rad) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi] so errors across +/-180 deg stay small."""
    return math.atan2(math.sin(angle), math.cos(angle))


class EkfError(Node):
    """Publish position and heading error of /ekf_odom against ground truth.

    The odom frame coincides with world because the robot spawns at the
    world origin with zero yaw and the EKF is started before the robot moves.
    """

    def __init__(self):
        super().__init__('ekf_error')
        self.truth_sub = message_filters.Subscriber(
            self, PoseStamped, '/tb3/ground_truth/pose')
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/ekf_odom')
        # Both topics carry simulation-time stamps from the sensors/simulator.
        # /ekf_odom runs at ~200 Hz, so any ground-truth pose has an estimate
        # within ~2.5 ms. The queue holds 1 s of estimates because ground
        # truth arrives through the gz CLI with extra latency.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.truth_sub, self.odom_sub], queue_size=200, slop=0.01)
        self.sync.registerCallback(self.handle_pair)
        self.position_pub = self.create_publisher(Float64, '/ekf_error/position', 10)
        self.yaw_pub = self.create_publisher(Float64, '/ekf_error/yaw', 10)

    def handle_pair(self, truth, odom):
        """Compute and publish the errors for one time-aligned pair.

        Args:
            truth: Ground-truth pose (PoseStamped, world frame).
            odom: EKF estimate (Odometry, odom frame).
        """
        est = odom.pose.pose
        dx = est.position.x - truth.pose.position.x
        dy = est.position.y - truth.pose.position.y
        dyaw = wrap_angle(yaw_from_quaternion(est.orientation)
                          - yaw_from_quaternion(truth.pose.orientation))

        self.position_pub.publish(Float64(data=math.hypot(dx, dy)))
        self.yaw_pub.publish(Float64(data=abs(math.degrees(dyaw))))


def main():
    """Run the error node until interrupted."""
    rclpy.init()
    node = EkfError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
