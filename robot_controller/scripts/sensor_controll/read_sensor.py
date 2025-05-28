#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__('sensor_monitor_node')
        
        # Create subscribers for all sensor topics
        self.limit_fl_sub = self.create_subscription(
            Bool, '/sensor/limit_f_l', self.limit_fl_callback, 10)
        self.limit_fr_sub = self.create_subscription(
            Bool, '/sensor/limit_f_r', self.limit_fr_callback, 10)
        self.limit_bl_sub = self.create_subscription(
            Bool, '/sensor/limit_b_l', self.limit_bl_callback, 10)
        self.limit_br_sub = self.create_subscription(
            Bool, '/sensor/limit_b_r', self.limit_br_callback, 10)
        self.emer_sub = self.create_subscription(
            Bool, '/sensor/emer', self.emer_callback, 10)
        
        # Create cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State of rising edge
        self.prev_limit_br = False
        self.prev_limit_bl = False
        self.prev_limit_fr = False
        self.prev_limit_fl = False
        self.prev_emer_triger = False
        
        self.get_logger().info('Sensor monitor node started')
        self.last_command_time = time.time()
        self.command_cooldown = 0.5  # 500ms cooldown between commands
    
    def send_cmd_vel(self, linear_x, sensor_name):
        # Check if we should throttle commands
        current_time = time.time()
        if current_time - self.last_command_time < self.command_cooldown:
            return
            
        self.get_logger().warn(f'{sensor_name} triggered! Sending cmd_vel: linear_x={linear_x}')
        
        # Create and send Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist_msg)
        self.last_command_time = current_time
    
    def limit_fl_callback(self, msg):
        if msg.data and not self.prev_limit_fl:
            # Front limit triggered - go back
            self.send_cmd_vel(-0.5, 'Front Left Limit Sensor')
        self.prev_limit_fl = msg.data
    
    def limit_fr_callback(self, msg):
        if msg.data and not self.prev_limit_fr:
            # Front limit triggered - go back
            self.send_cmd_vel(-0.5, 'Front Right Limit Sensor')
        self.prev_limit_fr = msg.data

    def limit_bl_callback(self, msg):
        if msg.data and not self.prev_limit_bl:
            # Back limit triggered - go forward
            self.send_cmd_vel(0.5, 'Back Left Limit Sensor')
        self.prev_limit_bl = msg.data
    
    def limit_br_callback(self, msg):
        if msg.data and not self.prev_limit_br:
            # Back limit triggered - go forward
            self.send_cmd_vel(0.5, 'Back Right Limit Sensor')
        self.prev_limit_br = msg.data
    
    def emer_callback(self, msg):
        if msg.data and not self.prev_emer_triger:
            # Emergency - stop
            self.send_cmd_vel(0.0, 'Emergency Sensor')
        self.prev_emer_triger = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()