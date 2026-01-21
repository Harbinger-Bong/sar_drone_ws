#!/usr/bin/env python3
"""
Drone Velocity Controller Node
Converts /cmd_vel to appropriate drone commands for simulation or real hardware
"""
import rclpy
from rclpy.node import Node
import math
import subprocess
import threading

class DroneVelocityController(Node):
    def __init__(self):
        super().__init__('drone_velocity_controller')
        
        # Declare parameters
        self.declare_parameter('mode', 'simulation')
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('hover_altitude', 0.5)
        self.declare_parameter('integration_rate', 10.0)
        
        # Get parameters
        self.mode = self.get_parameter('mode').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.max_yaw = self.get_parameter('max_yaw_rate').value
        self.hover_alt = self.get_parameter('hover_altitude').value
        rate = self.get_parameter('integration_rate').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.z = self.hover_alt
        self.yaw = 0.0
        
        self.current_cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        
        # Lock for thread-safe service calls
        self.pose_lock = threading.Lock()
        
        # TF broadcaster for RViz visualization
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers for debugging/visualization
        self.odom_pub = self.create_publisher(
            Odometry, '/drone/controller/odom', 10)
        
        # Integration timer
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.update_state)
        
        # Watchdog timer
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)
        
        self.get_logger().info(f'Drone Velocity Controller started in {self.mode} mode')
        self.get_logger().info(f'Max velocity: {self.max_vel} m/s, Update rate: {rate} Hz')
    
    def cmd_vel_callback(self, msg):
        """Receive velocity commands from Nav2 or teleop"""
        msg.linear.x = max(-self.max_vel, min(self.max_vel, msg.linear.x))
        msg.linear.y = max(-self.max_vel, min(self.max_vel, msg.linear.y))
        msg.linear.z = max(-self.max_vel, min(self.max_vel, msg.linear.z))
        msg.angular.z = max(-self.max_yaw, min(self.max_yaw, msg.angular.z))
        
        self.current_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        
        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(
                f'Cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, w={msg.angular.z:.2f}',
                throttle_duration_sec=2.0
            )
    
    def watchdog_check(self):
        """Stop drone if no cmd_vel received"""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            if abs(self.current_cmd_vel.linear.x) > 0.01 or \
               abs(self.current_cmd_vel.linear.y) > 0.01 or \
               abs(self.current_cmd_vel.angular.z) > 0.01:
                self.get_logger().warn('No cmd_vel for >1s, stopping', throttle_duration_sec=5.0)
                self.current_cmd_vel = Twist()
    
    def update_state(self):
        """Integrate velocities and update Gazebo"""
        # Transform velocity from body frame to world frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        vx_world = (self.current_cmd_vel.linear.x * cos_yaw - 
                    self.current_cmd_vel.linear.y * sin_yaw)
        vy_world = (self.current_cmd_vel.linear.x * sin_yaw + 
                    self.current_cmd_vel.linear.y * cos_yaw)
        
        # Integrate position
        self.x += vx_world * self.dt
        self.y += vy_world * self.dt
        self.z += self.current_cmd_vel.linear.z * self.dt
        self.yaw += self.current_cmd_vel.angular.z * self.dt
        
        # Clamp altitude
        self.z = max(0.1, min(10.0, self.z))
        
        # Normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Update Gazebo
        if self.mode == 'simulation':
            self.set_gazebo_pose()
        
        # Publish TF and odometry
        self.publish_transforms()
        self.publish_odom()
    
    def set_gazebo_pose(self):
        """Set drone pose in Gazebo using ign service"""
        with self.pose_lock:
            try:
                qz = math.sin(self.yaw / 2.0)
                qw = math.cos(self.yaw / 2.0)
                
                cmd = [
                    'ign', 'service', '-s', '/world/sar_world/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '100',
                    '--req', 
                    f'name: "sar_drone", '
                    f'position: {{x: {self.x:.3f}, y: {self.y:.3f}, z: {self.z:.3f}}}, '
                    f'orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}'
                ]
                
                # Run in background to avoid blocking
                subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception as e:
                self.get_logger().error(f'Failed to set Gazebo pose: {e}', throttle_duration_sec=5.0)
    
    '''def publish_transforms(self):
        """Publish TF transforms for RViz visualization"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
    
    def publish_odom(self):
        """Publish odometry for debugging"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        odom.twist.twist = self.current_cmd_vel'''

def main(args=None):
    rclpy.init(args=args)
    node = DroneVelocityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

