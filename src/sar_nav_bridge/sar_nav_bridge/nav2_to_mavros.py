#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


class Nav2ToMavrosBridge(Node):
    """
    Bridges Nav2 /cmd_vel to MAVROS velocity setpoints.
    """

    def __init__(self):
        super().__init__('nav2_to_mavros_bridge')

        self.latch_duration = 0.5
        self.publish_rate = 20.0

        self.last_cmd_time = None
        self.last_cmd = Twist()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        self.mavros_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            qos
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Nav2 → MAVROS bridge active')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.time()

    def timer_callback(self):
        now = time.time()
        out = Twist()

        if self.last_cmd_time and (now - self.last_cmd_time) < self.latch_duration:
            out.linear.x = self.last_cmd.linear.x
            out.linear.y = self.last_cmd.linear.y
            out.angular.z = self.last_cmd.angular.z
        else:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0

        out.linear.z = 0.0
        self.mavros_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ToMavrosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mavros_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
