#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class Nav2PathToSetpoint(Node):
    def __init__(self):
        super().__init__('nav2_path_to_setpoint')

        # Parameters
        self.declare_parameter('mission_altitude', 2.0)
        self.altitude = self.get_parameter('mission_altitude').value

        # Subscriber: Nav2 global plan
        self.sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publisher: PX4 position setpoints
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            return

        # Take the next waypoint (not the final one)
        p0 = msg.poses[0].pose.position
        p1 = msg.poses[1].pose.position

        # Compute yaw from path tangent
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        yaw = math.atan2(dy, dx)

        # Build setpoint
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map' #change to odom for REAL Drone

        sp.pose.position.x = p1.x
        sp.pose.position.y = p1.y
        sp.pose.position.z = self.altitude

        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(sp)


def main():
    rclpy.init()
    node = Nav2PathToSetpoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

