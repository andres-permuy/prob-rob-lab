"""Assignment 3: compare camera measurements with landmark ground truth."""

import math

import message_filters
import numpy
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def rotation_matrix(q):
    """Convert a ROS quaternion into a 3x3 rotation matrix.

    Args:
        q: Quaternion with x, y, z, and w components.

    Returns:
        Rotation matrix, or None for a nonfinite or zero quaternion.
    """
    values = numpy.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = numpy.linalg.norm(values)
    if not numpy.isfinite(norm) or norm == 0.0:
        return None
    x, y, z, w = values / norm
    return numpy.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class LandmarkMeasurementError(Node):
    """Publish time-aligned measurement/error pairs for one landmark color.

    Both outputs use Vector3Stamped as a scalar-pair encoding:
    measurement.vector = [distance (m), bearing (rad), 0];
    error.vector = [signed distance error (m), signed bearing error (rad), 0].
    They share the source measurement header and are not Cartesian vectors.
    """

    def __init__(self):
        super().__init__('landmark_measurement_error')
        self.log = self.get_logger()
        self.landmark_color = self.declare_parameter(
            'landmark_color', 'cyan').value

        # Fixed landmark centers in world coordinates, from landmarks.world.
        landmarks = {
            'red': (8.5, -5.0),
            'green': (8.5, 5.0),
            'yellow': (-11.5, 5.0),
            'magenta': (-11.5, -5.0),
            'cyan': (0.0, 0.0),
        }
        if self.landmark_color not in landmarks:
            raise ValueError('landmark_color must match a supplied landmark color.')
        self.landmark = numpy.array(landmarks[self.landmark_color])

        # The course simulator reports the robot model origin as ground truth.
        # Its planar body reference is base_footprint. camera_rgb_frame has forward/left/up axes
        self.robot_frame = 'base_footprint'
        self.camera_frame = 'camera_rgb_frame'
        self.R_BC = None
        self.t_BC = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        prefix = f'/vision_{self.landmark_color}'
        self.sub_truth = message_filters.Subscriber(
            self, PoseStamped, '/tb3/ground_truth/pose',
            qos_profile=qos_profile_sensor_data)
        self.sub_measurement = message_filters.Subscriber(
            self, Vector3Stamped, prefix + '/measurement',
            qos_profile=qos_profile_sensor_data)
        
        # Reuse Lab 4's header-time synchronization pattern and 10 ms slop.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_truth, self.sub_measurement], queue_size=200, slop=0.01)
        self.sync.registerCallback(self.handle_pair)
        self.pub_measurement = self.create_publisher(
            Vector3Stamped, prefix + '/evaluation/measurement', 1)
        self.pub_error = self.create_publisher(
            Vector3Stamped, prefix + '/evaluation/error', 1)
        self.log.info(
            f'Comparing {self.landmark_color} measurements with world ground truth.')

    def spin(self):
        """Process callbacks using the professor's node-template pattern."""
        rclpy.spin(self)

    def handle_pair(self, truth, measurement):
        """Calculate signed errors for one synchronized observation.

        Args:
            truth: Robot PoseStamped in world coordinates.
            measurement: Assignment 2 distance/bearing at the camera origin.
        """
        if (truth.header.frame_id.lstrip('/') != 'world'
                or measurement.header.frame_id.lstrip('/') != self.camera_frame):
            self.log.warning(
                'Expected world ground truth and camera_rgb_frame measurements.',
                throttle_duration_sec=5.0)
            return
        if not all(math.isfinite(value) for value in (
                measurement.vector.x, measurement.vector.y)):
            return

        # The rigid mounting transform is constant: look it up and cache it.
        if self.R_BC is None:
            try:
                mount = self.tf_buffer.lookup_transform(
                    self.robot_frame, self.camera_frame, Time()).transform
            except TransformException:
                self.log.warning(
                    'Waiting for the base_footprint to camera_rgb_frame transform.',
                    throttle_duration_sec=5.0)
                return
            R_BC = rotation_matrix(mount.rotation)
            t_BC = numpy.array([
                mount.translation.x, mount.translation.y, mount.translation.z])
            if R_BC is None or not numpy.isfinite(t_BC).all():
                return
            self.R_BC, self.t_BC = R_BC, t_BC

        # Compose world-to-body ground truth with the body-to-camera mount.
        R_WB = rotation_matrix(truth.pose.orientation)
        p_WB = numpy.array([
            truth.pose.position.x, truth.pose.position.y, truth.pose.position.z])
        if R_WB is None or not numpy.isfinite(p_WB).all():
            return
        p_WC = p_WB + R_WB @ self.t_BC
        R_WC = R_WB @ self.R_BC
        camera_yaw = math.atan2(R_WC[1, 0], R_WC[0, 0])

        # Use planar distance and bearing, consistent with the lab's model.
        dx, dy = self.landmark - p_WC[:2]
        d_true = math.hypot(dx, dy)
        if d_true <= 0.0:
            return
        theta_true = math.atan2(dy, dx) - camera_yaw
        d_error = measurement.vector.x - d_true
        theta_error = measurement.vector.y - theta_true
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        # Publish the matched measurement alongside its signed errors.
        error = Vector3Stamped()
        error.header = measurement.header
        error.vector.x = d_error
        error.vector.y = theta_error
        error.vector.z = 0.0
        self.pub_measurement.publish(measurement)
        self.pub_error.publish(error)


def main():
    rclpy.init()
    landmark_measurement_error = LandmarkMeasurementError()
    landmark_measurement_error.spin()
    landmark_measurement_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
