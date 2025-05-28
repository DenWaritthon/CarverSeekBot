#!/usr/bin/python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class FaceMatchingNavigationNode(Node):
    def __init__(self):
        super().__init__('face_matching_navigation_node')
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        
        # Service server for face matching
        self._face_match_service = self.create_service(
            SetBool, 
            '/face_matched', 
            self.face_matching_callback
        )
        
        self.get_logger().info('Face matching navigation node started')
        self.get_logger().info('Service available at: /face_matching_cancel')

    def send_goal_to_position(self, x, y, yaw=0.0):
        """Send robot to specified position"""
        self._nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simple case - only rotation around z-axis)
        import math
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f'Sending robot to position: x={x}, y={y}, yaw={yaw}')

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected')
            return

        self.get_logger().info('Navigation goal accepted - robot is moving')
        
        # Get result when navigation completes
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result().result
        self.get_logger().info('Navigation completed')
        self._goal_handle = None

    def face_matching_callback(self, request, response):
        """Service callback for face matching"""
        if request.data:  # If face matching is true
            if self._goal_handle:
                self.get_logger().info('Face match detected! Cancelling navigation...')
                
                # Cancel the current navigation goal
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)
                
                response.success = True
                response.message = "Navigation cancelled due to face match"
            else:
                self.get_logger().info('Face match detected but no active navigation goal')
                response.success = False
                response.message = "No active navigation to cancel"
        else:
            self.get_logger().info('Face matching service called with false - no action taken')
            response.success = True
            response.message = "No action taken - face match was false"
        return response

    def cancel_done_callback(self, future):
        """Handle navigation cancellation result"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Navigation successfully cancelled due to face match')
        else:
            self.get_logger().warn('Failed to cancel navigation')
        
        self._goal_handle = None

    def navigate_to_face_matching_position(self, x=-4.0, y=-1.0, yaw=0.0):
        """Convenience method to start navigation to face matching position"""
        self.send_goal_to_position(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = FaceMatchingNavigationNode()
    
    # Example: Send robot to face matching position
    # You can modify these coordinates as needed
    node.navigate_to_face_matching_position(x=8.0, y=0.0, yaw=0.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()