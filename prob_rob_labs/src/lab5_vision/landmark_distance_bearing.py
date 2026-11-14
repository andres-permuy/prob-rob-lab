"""Assignment 2: convert landmark corner pixels into distance and bearing."""

import math

import numpy
import rclpy
from geometry_msgs.msg import Vector3Stamped
from prob_rob_msgs.msg import Point2DArrayStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from lab5_vision.landmark_measurement import landmark_pixel_geometry


class LandmarkDistanceBearing(Node):
    """Publish distance (m) and left-positive bearing (rad) for one color.

    Vector3Stamped encodes scalar values, not a Cartesian vector:
    vector.x = distance, vector.y = bearing, and vector.z = 0 (unused).
    The header preserves the source image timestamp and camera frame.
    """

    def __init__(self):
        super().__init__('landmark_distance_bearing')
        self.log = self.get_logger()

        # Read the known landmark height, selected color, and image-edge margin.
        self.landmark_height = self.declare_parameter(
            'landmark_height', 0.5).value
        self.landmark_color = self.declare_parameter(
            'landmark_color', 'cyan').value
        self.edge_margin_px = self.declare_parameter(
            'edge_margin_px', 5.0).value

        # Reject invalid startup parameters before processing camera messages.
        if (not math.isfinite(self.landmark_height)
                or self.landmark_height <= 0.0):
            raise ValueError('landmark_height must be finite and positive.')
        if self.landmark_color not in ('red', 'green', 'yellow', 'magenta', 'cyan'):
            raise ValueError('landmark_color must match a supplied landmark color.')
        if not math.isfinite(self.edge_margin_px) or self.edge_margin_px < 0.0:
            raise ValueError('edge_margin_px must be finite and nonnegative.')

        # Wait for valid calibration before accepting feature measurements.
        self.camera = None

        # Select the detector and measurement topics for this landmark color.

        topic_prefix = f'/vision_{self.landmark_color}'

        # Sensor-data QoS accepts messages from the existing camera pipeline.
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.handle_camera_info,
            qos_profile_sensor_data)
        self.sub_features = self.create_subscription(
            Point2DArrayStamped, topic_prefix + '/corners', self.handle_features,
            qos_profile_sensor_data)
        
        # Publish the scalar distance/bearing pair in a stamped ROS message.
        self.pub_measurement = self.create_publisher(
            Vector3Stamped, topic_prefix + '/measurement', 1)
        self.log.info(
            f'Using {self.landmark_color}, height={self.landmark_height} m; '
            f'publishing distance/bearing on {topic_prefix}/measurement.')

    def spin(self):
        """Process camera and feature callbacks until the node is stopped."""
        rclpy.spin(self)

    def handle_camera_info(self, msg):
        """Cache valid camera calibration and full image dimensions.

        Args:
            msg: CameraInfo for the full, unbinned simulation image.
        """
        # Extract focal lengths and horizontal principal point from CameraInfo.p.
        # The projection matrix is flattened in 3x4 row-major order.
        fx, fy, cx = float(msg.p[0]), float(msg.p[5]), float(msg.p[2])

        # Disable measurements if calibration or image dimensions are invalid.
        if (not all(math.isfinite(value) for value in (fx, fy, cx))
                or fx <= 0.0 or fy <= 0.0
                or msg.width <= 0 or msg.height <= 0):
            self.camera = None
            return
        
        # Keep the calibration values needed by the feature callback.
        self.camera = (fx, fy, cx, msg.width, msg.height)

    def handle_features(self, msg):
        """Publish distance and bearing for a valid landmark detection.

        Args:
            msg: Landmark feature points with the source image header.
        """
        # A distance estimate requires the camera focal lengths.
        if self.camera is None:
            return

        # Convert the detected corners into an array of image coordinates.
        points = numpy.array([[point.x, point.y] for point in msg.points])

        # Assignment 1 gives the pixel height and horizontal center.
        geometry = landmark_pixel_geometry(points)

        # Skip empty, nonfinite, or degenerate feature geometry.
        if geometry is None:
            return
        
        # Unpack the feature geometry and cached camera calibration.
        delta_y_p, x_p = geometry
        fx, fy, cx, width, height = self.camera

        # Find the detection bounds to check for image-edge clipping.
        x_min, y_min = points.min(axis=0)
        x_max, y_max = points.max(axis=0)
        margin = self.edge_margin_px

        # Reject views that touch the configured image-edge margin.
        if (x_min <= margin or x_max >= width - 1 - margin
                or y_min <= margin or y_max >= height - 1 - margin):
            return

        # Compare height and width after normalizing by the focal lengths.
        shape_ratio = (delta_y_p / fy) / ((x_max - x_min) / fx)

        # The expected ratio uses the handout's cylinder diameter of 0.2 m.
        expected_ratio = self.landmark_height / 0.2

        # These heuristic bounds reject thin strips and incomplete side views.
        if not 0.75 * expected_ratio <= shape_ratio <= 1.5 * expected_ratio:
            return

        # Image x increases to the right, so cx - x_p gives left-positive bearing.
        theta = math.atan((cx - x_p) / fx)

        # Convert pixel height to planar range using the known cylinder height.
        d = self.landmark_height * fy / (delta_y_p * math.cos(theta))

        # Suppress nonfinite or nonpositive range estimates.
        if not math.isfinite(d) or d <= 0.0:
            return

        # Preserve the detection timestamp and camera frame for synchronization.
        measurement = Vector3Stamped()
        measurement.header = msg.header

        # Encode distance and bearing as scalar fields; z is unused.
        measurement.vector.x = d
        measurement.vector.y = theta
        measurement.vector.z = 0.0
        self.pub_measurement.publish(measurement)


def main():
    rclpy.init()
    landmark_distance_bearing = LandmarkDistanceBearing()
    landmark_distance_bearing.spin()
    landmark_distance_bearing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
