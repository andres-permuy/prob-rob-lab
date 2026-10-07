"""Assignments 3 and 4: open the door, traverse it, stop, and close it."""

import rclpy
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

from door_controller.sequence import PARAMETERS, Sequence


class DoorController(Node):
    def __init__(self):
        super().__init__('door_controller')
        values = {}
        for name, (default, description) in PARAMETERS.items():
            self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True))
            values[name] = self.get_parameter(name).get_parameter_value().double_value
        self.sequence = Sequence(values)
        self.velocity_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.torque_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 10)
        self.ready = False
        self.previous_stage = None
        self.aborted = False
        self.timer = self.create_timer(0.05, self.heartbeat)
        self.get_logger().info(
            f'Waiting for simulation clock and command subscribers. '
            f'Forward speed: {values["forward_speed"]:.3f} m/s; '
            f'drive duration: {values["drive_duration"]:.1f} s.')

    def publish_commands(self, speed=0.0, torque=0.0):
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = 'base_footprint'
        velocity.twist.linear.x = speed
        self.velocity_pub.publish(velocity)
        self.torque_pub.publish(Float64(data=torque))

    def heartbeat(self):
        if self.aborted:
            self.publish_commands()
            return
        if not self.ready:
            self.publish_commands()
            if (self.velocity_pub.get_subscription_count() == 0
                    or self.torque_pub.get_subscription_count() == 0):
                return
            self.ready = True

        try:
            stage = self.sequence.tick(self.get_clock().now().nanoseconds * 1e-9)
        except ValueError as error:
            self.aborted = True
            self.publish_commands()
            self.get_logger().error(str(error))
            return
        self.publish_commands(stage.speed, stage.torque)
        if stage.name != self.previous_stage:
            self.get_logger().info(
                f'{stage.name}: speed={stage.speed:.3f} m/s, '
                f'torque={stage.torque:.1f} N m')
            self.previous_stage = stage.name
            if stage.name == 'DONE':
                self.get_logger().info(
                    'Timed sequence complete. Verify the robot and door visually; '
                    'this controller has no sensor feedback. Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DoorController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_commands()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
