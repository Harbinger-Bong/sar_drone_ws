#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import time
from collections import defaultdict

class TopicRateMonitor(Node):

    def __init__(self):
        super().__init__('topic_rate_monitor')

        self.topics = {
            '/imu/data': qos_profile_sensor_data,
            '/scan': qos_profile_sensor_data,
            '/rtabmap/odom': qos_profile_sensor_data,
            '/odometry/filtered': qos_profile_sensor_data,
        }

        self.counts = defaultdict(int)
        self.start_time = time.time()

        for topic, qos in self.topics.items():
            self.create_subscription(
                type(None),
                topic,
                lambda msg, t=topic: self.cb(t),
                qos
            )

        self.timer = self.create_timer(5.0, self.report)

    def cb(self, topic):
        self.counts[topic] += 1

    def report(self):
        now = time.time()
        dt = now - self.start_time
        self.get_logger().info('--- Topic Rates ---')
        for topic, count in self.counts.items():
            rate = count / dt if dt > 0 else 0.0
            self.get_logger().info(f'{topic}: {rate:.2f} Hz')
        self.counts.clear()
        self.start_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
