#!/usr/bin/env python3
# filepath: /home/carver/CarverCAB_ws/src/robot_controller/scripts/read_sensor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
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
        
        # Create service client
        self.motor_client = self.create_client(SetBool, '/command/motor')
        
        # Wait for service to be available
        while not self.motor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /command/motor not available, waiting...')
        
        self.get_logger().info('Sensor monitor node started')
        self.last_service_call_time = time.time()
        self.service_cooldown = 0.5  # 500ms cooldown between service calls
    
    def call_motor_service(self, sensor_name):
        # Check if we should throttle service calls
        current_time = time.time()
        if current_time - self.last_service_call_time < self.service_cooldown:
            return
            
        self.get_logger().warn(f'{sensor_name} triggered! Stopping motors')
        
        # Create request
        request = SetBool.Request()
        request.data = False
        
        # Call service
        future = self.motor_client.call_async(request)
        future.add_done_callback(self.service_callback)
        self.last_service_call_time = current_time
        
    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully stopped motors')
            else:
                self.get_logger().error(f'Failed to stop motors: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def limit_fl_callback(self, msg):
        if msg.data:
            self.call_motor_service('Front Left Limit Sensor')
    
    def limit_fr_callback(self, msg):
        if msg.data:
            self.call_motor_service('Front Right Limit Sensor')
    
    def limit_bl_callback(self, msg):
        if msg.data:
            self.call_motor_service('Back Left Limit Sensor')
    
    def limit_br_callback(self, msg):
        if msg.data:
            self.call_motor_service('Back Right Limit Sensor')
    
    def emer_callback(self, msg):
        if msg.data:
            self.call_motor_service('Emergency Sensor')

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()