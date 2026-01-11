#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SimpleSquareMission(Node):
    """
    Simple square pattern mission for testing.
    Publishes a slow square pattern as /cmd_vel (for sim testing).
    """
    
    def __init__(self):
        super().__init__('simple_square_mission')
        
        # Parameters
        self.side_length = 2.0  # meters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.side_duration = self.side_length / self.linear_speed  # seconds per side
        self.turn_duration = math.pi / 2 / self.angular_speed  # seconds per 90-degree turn
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Mission state
        self.mission_start_time = None
        self.current_phase = 0  # 0-3: sides, 4-7: turns
        
        # Timer to control mission
        self.mission_timer = self.create_timer(0.1, self.mission_callback)
        
        self.get_logger().info('SimpleSquareMission started')
        self.get_logger().info(f'Side length: {self.side_length}m, Speed: {self.linear_speed}m/s')
    
    def mission_callback(self):
        """Execute the square mission pattern."""
        if self.mission_start_time is None:
            self.mission_start_time = time.time()
        
        current_time = time.time()
        elapsed = current_time - self.mission_start_time
        
        # Calculate which phase we're in
        phase_duration = self.side_duration + self.turn_duration
        cycle_time = elapsed % (phase_duration * 4)  # 4 sides + 4 turns
        
        cmd = Twist()
        
        # Determine current phase
        if cycle_time < self.side_duration:
            # Moving forward
            cmd.linear.x = self.linear_speed
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
        elif cycle_time < self.side_duration + self.turn_duration:
            # Turning
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = self.angular_speed
        else:
            # Between cycles, continue pattern
            # This handles the transition between sides
            remaining = cycle_time - (self.side_duration + self.turn_duration)
            if remaining < self.side_duration:
                cmd.linear.x = self.linear_speed
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = self.angular_speed
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Log progress every 5 seconds
        if int(elapsed) % 5 == 0 and elapsed > 0:
            self.get_logger().info(f'Mission running: {elapsed:.1f}s elapsed')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleSquareMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop command
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.get_logger().info('Mission stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

