#!/usr/bin/python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger,SetBool
from robot_interfaces.srv import Target2Go

class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        
        # Default position
        self.target_x = 8.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_orientation_w = 1.0
        
        # Services
        self.set_position_srv = self.create_service(
            Target2Go,
            'set_navigation_position',
            self.set_position_callback
        )
        
        self.start_navigation_srv = self.create_service(
            Trigger,
            'start_navigation',
            self.start_navigation_callback
        )
        
        self.cancel_navigation_srv = self.create_service(
            SetBool,
            'face_matched',
            self.cancel_navigation_callback
        )
        
        self.get_logger().info('Navigation services ready:')
        self.get_logger().info('  - /set_navigation_position (geometry_msgs/srv/SetPose)')
        self.get_logger().info('  - /start_navigation (std_srvs/srv/Trigger)')
        self.get_logger().info('  - /cancel_navigation (std_srvs/srv/Trigger)')

    def set_position_callback(self, request, response):
        """Service callback to set target position"""
        try:
            self.target_x = request.target.x
            self.target_y = request.target.y
            self.target_z = request.target.z
            self.target_orientation_w = request.target.w
            
            response.success = True
            response.message = f"Position set to x={self.target_x:.2f}, y={self.target_y:.2f}, z={self.target_z:.2f}, w={self.target_orientation_w:.2f}"
            
            self.get_logger().info(f"Target position updated: x={self.target_x}, y={self.target_y}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error setting position: {str(e)}"
            self.get_logger().error(f"Failed to set position: {e}")
        
        return response

    def start_navigation_callback(self, request, response):
        """Service callback to start navigation to set position"""
        try:
            # Check if there's already an active goal
            if self._goal_handle and not self._goal_handle.is_cancel_requested:
                response.success = False
                response.message = "Navigation already in progress"
                return response
            
            self.send_goal()
            response.success = True
            response.message = f"Navigation started to position x={self.target_x:.2f}, y={self.target_y:.2f}"
            
        except Exception as e:
            response.success = False
            response.message = f"Error starting navigation: {str(e)}"
            self.get_logger().error(f"Failed to start navigation: {e}")
        return response

    def cancel_navigation_callback(self, request, response):
        """Service callback to cancel current navigation"""
        try:
            if self._goal_handle:
                self.cancel_goal()
                response.success = True
                response.message = "Navigation cancellation requested"
            else:
                response.success = False
                response.message = "No active navigation to cancel"
                
        except Exception as e:
            response.success = False
            response.message = f"Error cancelling navigation: {str(e)}"
            self.get_logger().error(f"Failed to cancel navigation: {e}")
        
        return response

    def send_goal(self):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigate to pose server not available')
            return False
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = self.target_z
        goal_msg.pose.pose.orientation.w = self.target_orientation_w

        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'Sending goal: x={self.target_x}, y={self.target_y}')
        return True

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted - Navigation started')
        
        # Get result
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed')
        self._goal_handle = None

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
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = NavigateClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()