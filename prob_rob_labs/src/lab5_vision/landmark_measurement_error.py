"""Assignments 3--4: publish camera measurement errors and model variances."""

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

from lab5_vision.landmark_measurement_variance import landmark_measurement_variance


def rotation_matrix(q):
    """Normalize a ROS quaternion and convert it to a 3x3 rotation matrix.

    Args:
        q: Quaternion with x, y, z, and w components.

    Returns:
        NumPy rotation matrix, or None if the quaternion norm is invalid.
    """
    # ROS stores quaternion components in x, y, z, w order.
    values = numpy.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = numpy.linalg.norm(values)

    # Reject quaternions that cannot be normalized.
    if not numpy.isfinite(norm) or norm == 0.0:
        return None
    
    # Normalize before applying the quaternion-to-rotation formula.
    x, y, z, w = values / norm
    return numpy.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class LandmarkMeasurementError(Node):
    """Publish matched measurements, signed errors, and model variances.

    All outputs use Vector3Stamped as a scalar-pair encoding:
    measurement.vector = [distance (m), bearing (rad), 0];
    error.vector = [signed distance error (m), signed bearing error (rad), 0].
    variance.vector = [distance variance (m^2), bearing variance (rad^2), 0].
    They share the source measurement header and are not Cartesian vectors.
    """

    def __init__(self):
        super().__init__('landmark_measurement_error')
        self.log = self.get_logger()

        # Use the same landmark color as the Assignment 2 measurement node.
        self.landmark_color = self.declare_parameter(
            'landmark_color', 'cyan').value

        # Known landmark centers in world coordinates, from landmarks.world.
        landmarks = {
            'red': (8.5, -5.0),
            'green': (8.5, 5.0),
            'yellow': (-11.5, 5.0),
            'magenta': (-11.5, -5.0),
            'cyan': (0.0, 0.0),
        }
        # Reject colors that have no known center in this world.
        if self.landmark_color not in landmarks:
            raise ValueError('landmark_color must match a supplied landmark color.')
        self.landmark = numpy.array(landmarks[self.landmark_color])

        # Ground truth reports the robot origin, represented by base_footprint.
        self.robot_frame = 'base_footprint'

        # The RGB camera frame uses forward, left, and up axes.
        self.camera_frame = 'camera_rgb_frame'

        # Cache the camera mounting pose expressed in body coordinates.
        self.R_BC = None
        self.t_BC = None

        # Receive TF messages so the fixed camera mounting pose can be queried.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to robot ground truth and this color's camera measurements.
        prefix = f'/vision_{self.landmark_color}'
        self.sub_truth = message_filters.Subscriber(
            self, PoseStamped, '/tb3/ground_truth/pose',
            qos_profile=qos_profile_sensor_data)
        self.sub_measurement = message_filters.Subscriber(
            self, Vector3Stamped, prefix + '/measurement',
            qos_profile=qos_profile_sensor_data)
        
        # Pair messages whose acquisition timestamps differ by at most 10 ms.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_truth, self.sub_measurement], queue_size=200, slop=0.01)
        self.sync.registerCallback(self.handle_pair)

        # Publish paired measurements, signed errors, and predicted variances.
        self.pub_measurement = self.create_publisher(
            Vector3Stamped, prefix + '/evaluation/measurement', 1)
        self.pub_error = self.create_publisher(
            Vector3Stamped, prefix + '/evaluation/error', 1)
        self.pub_variance = self.create_publisher(
            Vector3Stamped, prefix + '/evaluation/variance', 1)
        self.log.info(
            f'Comparing {self.landmark_color} measurements with world ground truth.')

    def spin(self):
        rclpy.spin(self)

    def handle_pair(self, truth, measurement):
        """Publish errors and model variances for a synchronized observation.

        Args:
            truth: Robot ground-truth PoseStamped expressed in the world frame.
            measurement: Stamped distance (m) and bearing (rad) from Assignment 2.
        """
        # Check that both messages use the frames expected by this calculation.
        if (truth.header.frame_id.lstrip('/') != 'world'
                or measurement.header.frame_id.lstrip('/') != self.camera_frame):
            self.log.warning(
                'Expected world ground truth and camera_rgb_frame measurements.',
                throttle_duration_sec=5.0)
            return
        
        # Reject invalid scalar measurements before computing errors or variances.
        if (measurement.vector.x <= 0.0
                or not all(math.isfinite(value) for value in (
                    measurement.vector.x, measurement.vector.y))):
            return

        # Look up the fixed camera-to-body transform once, then reuse it.
        if self.R_BC is None:
            try:
                mount = self.tf_buffer.lookup_transform(
                    self.robot_frame, self.camera_frame, Time()).transform
            except TransformException:
                self.log.warning(
                    'Waiting for the base_footprint to camera_rgb_frame transform.',
                    throttle_duration_sec=5.0)
                return
            
            # Express the camera mounting rotation and translation in the body.
            R_BC = rotation_matrix(mount.rotation)
            t_BC = numpy.array([
                mount.translation.x, mount.translation.y, mount.translation.z])
            
            # Cache only a finite mounting pose with a valid rotation.
            if R_BC is None or not numpy.isfinite(t_BC).all():
                return
            self.R_BC, self.t_BC = R_BC, t_BC

        # Express the robot body's orientation and position in world coordinates.
        R_WB = rotation_matrix(truth.pose.orientation)
        p_WB = numpy.array([
            truth.pose.position.x, truth.pose.position.y, truth.pose.position.z])
        
        # Skip ground-truth poses that cannot define a valid camera pose.
        if R_WB is None or not numpy.isfinite(p_WB).all():
            return
        
        # Rotate the camera offset into the world and add the robot position.
        p_WC = p_WB + R_WB @ self.t_BC

        # Compose the mounting rotation with the robot's world orientation.
        R_WC = R_WB @ self.R_BC

        # Read camera heading from its forward axis in the world XY plane.
        camera_yaw = math.atan2(R_WC[1, 0], R_WC[0, 0])

        # Calculate the horizontal displacement from the camera to the landmark.
        dx, dy = self.landmark - p_WC[:2]

        # Use planar range, consistent with the Assignment 2 measurement model.
        d_true = math.hypot(dx, dy)

        # Bearing is undefined if camera and landmark share a planar position.
        if d_true <= 0.0:
            return
        
        # Express the landmark direction relative to the camera heading.
        theta_true = math.atan2(dy, dx) - camera_yaw

        # Signed errors are measured values minus their ground-truth values.
        d_error = measurement.vector.x - d_true
        theta_error = measurement.vector.y - theta_true

        # Wrap bearing error onto the principal interval around zero.
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        # Keep the source measurement header on every paired output.
        error = Vector3Stamped()
        error.header = measurement.header

        # Encode distance error in x and bearing error in y; z is unused.
        error.vector.x = d_error
        error.vector.y = theta_error
        error.vector.z = 0.0

        # Republish only the measurement used for this error calculation.
        self.pub_measurement.publish(measurement)
        self.pub_error.publish(error)

        # Evaluate the fitted Assignment 4 model using the measured distance.
        d_variance, theta_variance = landmark_measurement_variance(
            measurement.vector.x)
        
        # Encode distance variance in m^2 and bearing variance in rad^2.
        variance = Vector3Stamped()
        variance.header = measurement.header
        variance.vector.x = d_variance
        variance.vector.y = theta_variance
        variance.vector.z = 0.0
        self.pub_variance.publish(variance)


def main():
    rclpy.init()
    landmark_measurement_error = LandmarkMeasurementError()
    landmark_measurement_error.spin()
    landmark_measurement_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
