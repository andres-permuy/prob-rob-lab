"""Assignment 2: publish landmark distance and bearing from camera features."""

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
    """Calculate distance (m) and left-positive bearing (rad) for one color.

    Vector3Stamped encodes scalar values, not a Cartesian vector:
    vector.x = distance, vector.y = bearing, and vector.z = 0 (unused).
    Header preserves the source image's timestamp and camera frame.
    """

    def __init__(self):
        super().__init__('landmark_distance_bearing')
        self.log = self.get_logger()

        self.landmark_height = self.declare_parameter(
            'landmark_height', 0.5).value
        self.landmark_color = self.declare_parameter(
            'landmark_color', 'cyan').value
        self.edge_margin_px = self.declare_parameter(
            'edge_margin_px', 5.0).value

        # Parameters are read at startup, reject invalid configurations.
        if (not math.isfinite(self.landmark_height)
                or self.landmark_height <= 0.0):
            raise ValueError('landmark_height must be finite and positive.')
        if self.landmark_color not in ('red', 'green', 'yellow', 'magenta', 'cyan'):
            raise ValueError('landmark_color must match a supplied landmark color.')
        if not math.isfinite(self.edge_margin_px) or self.edge_margin_px < 0.0:
            raise ValueError('edge_margin_px must be finite and nonnegative.')

        self.camera = None
        topic_prefix = f'/vision_{self.landmark_color}'
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.handle_camera_info,
            qos_profile_sensor_data)
        self.sub_features = self.create_subscription(
            Point2DArrayStamped, topic_prefix + '/corners', self.handle_features,
            qos_profile_sensor_data)
        self.pub_measurement = self.create_publisher(
            Vector3Stamped, topic_prefix + '/measurement', 1)
        self.log.info(
            f'Using {self.landmark_color}, height={self.landmark_height} m; '
            f'publishing distance/bearing on {topic_prefix}/measurement.')

    def spin(self):
        rclpy.spin(self)

    def handle_camera_info(self, msg):
        """Cache valid intrinsics and full simulation image dimensions.

        Args:
            msg: CameraInfo for the full, unbinned simulation image.
        """
        # CameraInfo.p is a flattened 3x4 row-major projection matrix.
        fx, fy, cx = float(msg.p[0]), float(msg.p[5]), float(msg.p[2])
        if (not all(math.isfinite(value) for value in (fx, fy, cx))
                or fx <= 0.0 or fy <= 0.0
                or msg.width <= 0 or msg.height <= 0):
            self.camera = None
            return
        self.camera = (fx, fy, cx, msg.width, msg.height)

    def handle_features(self, msg):
        """Calculate and publish one measurement from a valid detection.

        Args:
            msg: Landmark feature points with the source image header.
        """
        if self.camera is None:
            return

        # Reuse Assignment 1 without copying or changing its implementation.
        points = numpy.array([[point.x, point.y] for point in msg.points])
        geometry = landmark_pixel_geometry(points)
        if geometry is None:
            return
        delta_y_p, x_p = geometry
        fx, fy, cx, width, height = self.camera

        # Bounds near an image edge may indicate a partially clipped landmark.
        x_min, y_min = points.min(axis=0)
        x_max, y_max = points.max(axis=0)
        margin = self.edge_margin_px
        if (x_min <= margin or x_max >= width - 1 - margin
                or y_min <= margin or y_max >= height - 1 - margin):
            return

        # Missing edges can leave only a thin strip inside the image bounds.
        # Normalize by focal lengths; the handout's cylinder diameter is 0.2 m.
        shape_ratio = (delta_y_p / fy) / ((x_max - x_min) / fx)
        expected_ratio = self.landmark_height / 0.2
        # Conservative initial bounds, checked offline against vision_test_01.
        if not 0.75 * expected_ratio <= shape_ratio <= 1.5 * expected_ratio:
            return

        theta = math.atan((cx - x_p) / fx)
        d = self.landmark_height * fy / (delta_y_p * math.cos(theta))
        if not math.isfinite(d) or d <= 0.0:
            return

        # Publish only a fresh valid detection, preserve its acquisition time.
        measurement = Vector3Stamped()
        measurement.header = msg.header
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
