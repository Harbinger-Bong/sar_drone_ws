#!/usr/bin/env python3
"""
SAR Drone System Health Monitor
Monitors critical topics, TF, and node status
Publishes diagnostics and warnings
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import time
from collections import defaultdict


class SystemHealthMonitor(Node):
    """Monitor ROS 2 system health and publish diagnostics"""

    def __init__(self):
        super().__init__('system_health_monitor')
        
        # Parameters
        self.declare_parameter('check_frequency', 2.0)
        self.declare_parameter('topic_timeout', 2.0)
        
        self.check_freq = self.get_parameter('check_frequency').value
        self.topic_timeout = self.get_parameter('topic_timeout').value
        
        # Topic tracking
        self.last_received = {}
        self.message_counts = defaultdict(int)
        self.rate_calc_start = time.time()
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers for monitoring
        self.topics_to_monitor = {
            '/scan': (LaserScan, sensor_qos, 'LiDAR'),
            '/camera/image_raw': (Image, sensor_qos, 'Camera RGB'),
            '/camera/depth_image': (Image, sensor_qos, 'Camera Depth'),
            '/camera/camera_info': (CameraInfo, sensor_qos, 'Camera Info'),
            '/imu/data': (Imu, sensor_qos, 'IMU'),
            '/odom': (Odometry, sensor_qos, 'Gazebo Odom'),
            '/rtabmap/odom': (Odometry, reliable_qos, 'RTAB-Map Odom'),
            '/odometry/filtered': (Odometry, reliable_qos, 'EKF Filtered Odom'),
        }
        
        # Create subscribers
        self.subscribers = {}
        for topic, (msg_type, qos, name) in self.topics_to_monitor.items():
            self.subscribers[topic] = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.topic_callback(t, msg),
                qos
            )
            self.last_received[topic] = None
        
        # Diagnostics publisher
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Timer for periodic checks
        self.timer = self.create_timer(
            1.0 / self.check_freq,
            self.check_system_health
        )
        
        self.get_logger().info('System Health Monitor started')
    
    def topic_callback(self, topic_name, msg):
        """Track when topics are received"""
        self.last_received[topic_name] = self.get_clock().now()
        self.message_counts[topic_name] += 1
    
    def check_system_health(self):
        """Check all monitored systems and publish diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        current_time = self.get_clock().now()
        elapsed = time.time() - self.rate_calc_start
        
        # Check each monitored topic
        for topic, (_, _, name) in self.topics_to_monitor.items():
            status = DiagnosticStatus()
            status.name = f"SAR Drone: {name}"
            status.hardware_id = topic
            
            last_rx = self.last_received.get(topic)
            
            if last_rx is None:
                # Never received
                status.level = DiagnosticStatus.ERROR
                status.message = "No data received"
                status.values.append(
                    KeyValue(key="Status", value="OFFLINE")
                )
            else:
                # Check timeout
                age = (current_time - last_rx).nanoseconds / 1e9
                
                if age > self.topic_timeout:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Stale data ({age:.1f}s old)"
                    status.values.append(
                        KeyValue(key="Status", value="STALE")
                    )
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = "Active"
                    status.values.append(
                        KeyValue(key="Status", value="OK")
                    )
                
                # Calculate rate
                msg_count = self.message_counts[topic]
                rate = msg_count / elapsed if elapsed > 0 else 0.0
                
                status.values.append(
                    KeyValue(key="Rate (Hz)", value=f"{rate:.2f}")
                )
                status.values.append(
                    KeyValue(key="Age (s)", value=f"{age:.2f}")
                )
                status.values.append(
                    KeyValue(key="Total Messages", value=str(msg_count))
                )
            
            diag_array.status.append(status)
        
        # Overall system status
        overall = DiagnosticStatus()
        overall.name = "SAR Drone: Overall System"
        overall.hardware_id = "system"
        
        error_count = sum(1 for s in diag_array.status if s.level == DiagnosticStatus.ERROR)
        warn_count = sum(1 for s in diag_array.status if s.level == DiagnosticStatus.WARN)
        
        if error_count > 0:
            overall.level = DiagnosticStatus.ERROR
            overall.message = f"{error_count} critical errors"
        elif warn_count > 0:
            overall.level = DiagnosticStatus.WARN
            overall.message = f"{warn_count} warnings"
        else:
            overall.level = DiagnosticStatus.OK
            overall.message = "All systems operational"
        
        overall.values.append(KeyValue(key="Errors", value=str(error_count)))
        overall.values.append(KeyValue(key="Warnings", value=str(warn_count)))
        overall.values.append(KeyValue(key="Uptime (s)", value=f"{elapsed:.1f}"))
        
        diag_array.status.insert(0, overall)
        
        # Publish diagnostics
        self.diag_pub.publish(diag_array)
        
        # Log warnings
        if error_count > 0:
            self.get_logger().warn(
                f"System health: {error_count} errors, {warn_count} warnings",
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = SystemHealthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
