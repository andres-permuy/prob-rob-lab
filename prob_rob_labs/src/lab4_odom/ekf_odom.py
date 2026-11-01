#!/usr/bin/env python3
"""Lab 4: estimate planar odometry using wheel encoders and the IMU."""

import math

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class EkfOdom(Node):
    """Track [theta, x, y, v, omega] using the Lab 4 EKF equations."""

    def __init__(self):
        """Initialize the state, covariances, and ROS interfaces."""
        super().__init__('ekf_odom')

        # State [theta, x, y, v, omega] and 5x5 covariance.
        self.x = np.zeros((5, 1))
        self.Sigma_x = np.diag([0.0, 0.0, 0.0, 0.01, 0.01])
        self.last_stamp = None
        self.cmd_stamp = None
        self.u = np.zeros((2, 1))  # [u_v, u_omega]

        # Values from assignment 3
        self.G_v = 0.9965
        self.G_omega = 1.0085
        self.tau_v = 0.0367
        self.tau_omega = 0.0212

        # Input noise approximated from assignment 1 zero-command data
        self.Sigma_u = np.diag([0.004448870**2, 0.004391655**2])

        # assignment 5 wheel geometry and measurement model
        r_w = 0.033
        R = 0.1435 / 2.0
        self.C = np.array([
            [0, 0, 0, 1 / r_w, R / r_w],
            [0, 0, 0, 1 / r_w, -R / r_w],
            [0, 0, 0, 0, 1],
        ])
        self.encoder_var = 0.1**2  # Assumed encoder standard deviation: 0.1 rad/s
        self.gyro_var = 0.01**2    # Fallback if the IMU has no valid variance
        self.missing_wheels_logged = False

        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self.handle_command, 10)
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu')
        self.joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.joint_sub], queue_size=20, slop=0.05)
        self.sync.registerCallback(self.handle_measurements)
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

    def handle_command(self, msg):
        """Store the commanded body velocities and their reception time.

        Args:
            msg: Commanded velocity (TwistStamped).
        """
        self.u[:, 0] = [msg.twist.linear.x, msg.twist.angular.z]
        # CLI commands may have a zero header stamp; receipt uses simulation time

        self.cmd_stamp = self.get_clock().now().nanoseconds * 1e-9

    def predict(self, dt, u):
        """Predict the state and covariance using Assignments 3 and 4.

        Args:
            dt: Sensor-to-sensor elapsed simulation time in seconds.
            u: Current [u_v, u_omega] command (2x1).
        """
        theta, px, py, v, omega = self.x[:, 0]
        a_v = 0.1 ** (dt / self.tau_v)
        a_omega = 0.1 ** (dt / self.tau_omega)

        # Nonlinear state update from Assignment 4
        f = np.array([
            [theta + omega * dt],
            [px + v * dt * math.cos(theta)],
            [py + v * dt * math.sin(theta)],
            [a_v * v],
            [a_omega * omega],
        ])
        B = np.array([
            [0, 0], [0, 0], [0, 0],
            [self.G_v * (1 - a_v), 0],
            [0, self.G_omega * (1 - a_omega)],
        ])
        G_x = np.array([
            [1, 0, 0, 0, dt],
            [-v * dt * math.sin(theta), 1, 0, dt * math.cos(theta), 0],
            [v * dt * math.cos(theta), 0, 1, dt * math.sin(theta), 0],
            [0, 0, 0, a_v, 0],
            [0, 0, 0, 0, a_omega],
        ])

        self.x = f + B @ u
        self.Sigma_x = (G_x @ self.Sigma_x @ G_x.T
                        + B @ self.Sigma_u @ B.T)

    def correct(self, z, gyro_var):
        """Correct the predicted state using wheel and gyroscope readings.

        Args:
            z: [omega_r, omega_l, omega_g] measurement column (3x1).
            gyro_var: Gyroscope z-axis measurement variance.
        """
        Sigma_z = np.diag([self.encoder_var, self.encoder_var, gyro_var])
        S = self.C @ self.Sigma_x @ self.C.T + Sigma_z
        K = self.Sigma_x @ self.C.T @ np.linalg.inv(S)

        # Measurement model h(x)=Cx and standard EKF innovation
        self.x = self.x + K @ (z - self.C @ self.x)
        self.Sigma_x = (np.eye(5) - K @ self.C) @ self.Sigma_x
        self.Sigma_x = (self.Sigma_x + self.Sigma_x.T) / 2.0

    def handle_measurements(self, imu, joints):
        """Process time-aligned sensor readings and publish an odometry estimate.

        Args:
            imu: IMU message containing gyroscope z reading and variance.
            joints: JointState message containing left/right wheel velocities.
        """
        # JointState entries must be identified by name, not array order
        if 'wheel_right_joint' not in joints.name or 'wheel_left_joint' not in joints.name:
            if not self.missing_wheels_logged:
                self.get_logger().warning('Wheel joints not found in /joint_states.')
                self.missing_wheels_logged = True
            return

        # Use the later of the two synchronized measurement timestamps
        imu_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        joint_time = joints.header.stamp.sec + joints.header.stamp.nanosec * 1e-9
        stamp = max(imu_time, joint_time)
        if stamp <= 0.0 or (self.last_stamp is not None and stamp <= self.last_stamp):
            return

        # Ignore commands older than the simu timeout
        age = None if self.cmd_stamp is None else stamp - self.cmd_stamp
        u = self.u if age is not None and 0.0 <= age < 1.0 else np.zeros((2, 1))
        if self.last_stamp is not None:
            self.predict(stamp - self.last_stamp, u)

        right = joints.name.index('wheel_right_joint')
        left = joints.name.index('wheel_left_joint')
        if len(joints.velocity) <= max(right, left):
            return
        z = np.array([[joints.velocity[right]],
                      [joints.velocity[left]],
                      [imu.angular_velocity.z]])

        # IMU supplies covariance, but use fallback if unknown
        gyro_var = imu.angular_velocity_covariance[8]
        if gyro_var <= 0.0:
            gyro_var = self.gyro_var
        self.correct(z, gyro_var)
        self.last_stamp = stamp
        self.publish_odometry(imu.header.stamp if imu_time >= joint_time
                              else joints.header.stamp)

    def publish_odometry(self, stamp):
        """Publish the state and covariance in a ROS Odometry message.

        Args:
            stamp: Timestamp of the latest synchronized sensor measurement.
        """
        theta, px, py, v, omega = self.x[:, 0]
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(omega)

        # ROS pose order: [x, y, z, roll, pitch, yaw].
        pose_indices = [0, 1, 5]  # ROS covariance indices for x, y, yaw.
        state_pose_indices = [1, 2, 0]
        for i, pi in enumerate(pose_indices):
            for j, pj in enumerate(pose_indices):
                odom.pose.covariance[6 * pi + pj] = float(
                    self.Sigma_x[state_pose_indices[i], state_pose_indices[j]])

        # ROS twist order: [linear x, y, z, angular x, y, z].
        odom.twist.covariance[0] = float(self.Sigma_x[3, 3])
        odom.twist.covariance[5] = float(self.Sigma_x[3, 4])
        odom.twist.covariance[30] = float(self.Sigma_x[4, 3])
        odom.twist.covariance[35] = float(self.Sigma_x[4, 4])
        self.odom_pub.publish(odom)


def main():
    """Run the EKF node until interrupted."""
    rclpy.init()
    node = EkfOdom()
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
