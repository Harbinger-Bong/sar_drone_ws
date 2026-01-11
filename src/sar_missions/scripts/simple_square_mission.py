#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math
import time


class SimpleSquareMission(Node):
    """
    Simple square-pattern mission using Nav2 goals.
    Mission layer publishes navigation intent ONLY.
    """

    def __init__(self):
        super().__init__('simple_square_mission')

        # Square parameters
        self.side_length = 2.0  # meters
        self.altitude = 2.0     # fixed flight altitude (handled downstream)

        # Action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.square_goals = []
        self.current_goal_index = 0

        # Timer to wait for Nav2 and then start
        self.timer = self.create_timer(1.0, self.start_mission)

        self.get_logger().info('SimpleSquareMission initialized')

    def start_mission(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('Waiting for Nav2 action server...')
            return

        self.timer.cancel()
        self.build_square()
        self.send_next_goal()

    def build_square(self):
        """
        Build square waypoints in the map frame.
        Assumes starting pose is (0, 0).
        """
        x, y = 0.0, 0.0
        yaw = 0.0

        self.square_goals = []

        for _ in range(4):
            x += self.side_length * math.cos(yaw)
            y += self.side_length * math.sin(yaw)
            self.square_goals.append(self.make_goal(x, y, yaw))
            yaw += math.pi / 2.0

        self.get_logger().info('Square waypoints generated')

    def make_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0  # altitude handled by bridge / PX4

        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        return goal

    def send_next_goal(self):
        if self.current_goal_index >= len(self.square_goals):
            self.get_logger().info('Square mission completed')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.square_goals[self.current_goal_index]

        self.get_logger().info(
            f'Sending goal {self.current_goal_index + 1}/4'
        )

        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info('Goal reached')
        self.current_goal_index += 1
        time.sleep(1.0)  # small pause between legs
        self.send_next_goal()

    def feedback_callback(self, feedback):
        pass  # intentionally minimal


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSquareMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

