#!/usr/bin/python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 8.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted')
        # Cancel goal after 2 seconds
        self.create_timer(60.0, self.cancel_goal)

    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled')
        else:
            self.get_logger().info('Goal was not cancelled')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateClient()
    node.send_goal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
