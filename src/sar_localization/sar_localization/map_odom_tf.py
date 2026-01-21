#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MapOdomTF(Node):
    def __init__(self):
        super().__init__('map_odom_tf')

        self.declare_parameter('rate', 20.0)
        self.declare_parameter('use_sim_time', True)

        rate = self.get_parameter('rate').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.publish_tf)

        self.get_logger().info('Publishing time-valid identity TF: map -> odom')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MapOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

