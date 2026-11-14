#!/usr/bin/env python3

# IMPORTANT: THIS CODE WAS MADE WITH A LOT OF HELP FROM CHATGPT TO MAKE A CLEAN TRAJECTORY
#            FOR DATA COLLECTION!!!!!!!

"""
Drive a cyan-landmark range/bearing sweep for Lab 5 Assignment 4.

Run in the supplied landmarks world with Assignments 2 and 3 and rosbag
already running. Ground truth controls this experiment only; the existing
Assignment 3 node remains responsible for the camera-frame error calculation.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


heartbeat_period = 0.1


class VarianceCollection(Node):
    """Move between ranges and sweep the bearing at each station."""

    def __init__(self):
        """Initialize the experiment, pose subscriber, and command publisher."""
        super().__init__('lab5_variance_collection')
        self.log = self.get_logger()
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/tb3/ground_truth/pose', self.handle_pose,
            qos_profile_sensor_data)

        # Robot-origin ranges from cyan at (0, 0), in metres. The camera is
        # forward of the origin; these span approximately 1 to 10 m at the camera.
        self.stations = [1.1, 2.1, 3.1, 4.1, 6.1, 8.1, 10.2]
        self.station = 0
        self.phase = 'move'
        self.pose = None
        self.pose_time_ns = None
        self.hold_until_ns = 0

        # Sweep beyond visibility limits so the valid data cover the full view.
        self.bearings = {'left': -0.60, 'right': 0.60, 'center': 0.0}
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.log.info('Starting cyan range sweep. Keep other command publishers stopped.')

    def handle_pose(self, msg):
        """Store a finite world pose and its receipt time for motion feedback.

        Args:
            msg: Ground-truth PoseStamped from the course simulator.
        """
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if (msg.header.frame_id.lstrip('/') != 'world'
                or not all(math.isfinite(value) for value in values)
                or q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == 0.0):
            return

        # atan2 is unchanged by a common quaternion normalization factor.
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
        self.pose = (p.x, p.y, yaw)
        self.pose_time_ns = self.get_clock().now().nanoseconds

    def heartbeat(self):
        """Refresh bounded velocity commands and advance the experiment."""
        now_ns = self.get_clock().now().nanoseconds
        v, omega = 0.0, 0.0

        # Stop while waiting for pose feedback, or if feedback is interrupted.
        if (self.pose is not None
                and 0 <= now_ns - self.pose_time_ns <= 500_000_000):
            x, y, yaw = self.pose
            distance = math.hypot(x, y)
            angle = math.atan2(-y, -x) - yaw
            bearing = math.atan2(math.sin(angle), math.cos(angle))

            if self.phase == 'move':
                error = distance - self.stations[self.station]
                omega = max(-0.25, min(0.25, 2.0*bearing))
                # Face cyan before driving toward or away from it.
                if abs(bearing) < 0.10:
                    v = max(-0.20, min(0.20, 0.8*error))
                if abs(error) < 0.03 and abs(bearing) < 0.03:
                    v, omega = 0.0, 0.0
                    self.phase = 'hold'
                    self.hold_until_ns = now_ns + 2_000_000_000
                    self.log.info(f'Sweeping at robot range {distance:.2f} m.')

            elif self.phase == 'hold':
                if now_ns >= self.hold_until_ns:
                    self.phase = 'left'

            elif self.phase in self.bearings:
                error = bearing - self.bearings[self.phase]
                omega = max(-0.25, min(0.25, 2.0*error))
                if abs(error) < 0.02:
                    omega = 0.0
                    if self.phase == 'left':
                        self.phase = 'right'
                    elif self.phase == 'right':
                        self.phase = 'center'
                    else:
                        self.station += 1
                        if self.station < len(self.stations):
                            self.phase = 'move'
                        else:
                            self.phase = 'done'
                            self.log.info('Collection complete. Zero commands continue; stop the bag.')

        # /cmd_vel is stamped in this repository. Refresh it to avoid timeout.
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_footprint'
        command.twist.linear.x = v
        command.twist.angular.z = omega
        self.publisher.publish(command)

    def spin(self):
        """Process callbacks following the professor's node-template pattern."""
        rclpy.spin(self)


def main():
    """Run the standalone collection experiment in the sourced ROS workspace."""
    rclpy.init()
    node = VarianceCollection()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()