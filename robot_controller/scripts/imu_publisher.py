#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile ,ReliabilityPolicy
from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import os
import yaml
import numpy as np


class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')

        # Load calibration parameter
        self.declare_parameter('file','imu_calibration.yaml')
        pkg_name = 'robot_controller'

        imu_calib_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_calib_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        imu_calib_path = os.path.join(ws_path ,'src', pkg_name , 'config', file)

        with open(imu_calib_path, 'r') as file:
            self.calibration_value = yaml.safe_load(file)

        # publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10) 

        # Subscription
        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,depth = 10)
        self.create_subscription(Imu,'/M5C_IMU_publisher', self.imu_callback, qos_profile)

        # Variables
        self.imu_acc = [0.0, 0.0, 0.0]
        self.imu_gyro = [0.0, 0.0, 0.0]
        self.imu_orientation = [0.0, 0.0, 0.0, 1.0]

        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]

        self.get_logger().info(f'Imu Publisher Node Start!!!')

    def imu_offset(self):
        self.gyro[0] = self.imu_gyro[0] - self.calibration_value['gyro offset'][0]
        self.gyro[1] = self.imu_gyro[1] - self.calibration_value['gyro offset'][1]
        self.gyro[2] = self.imu_gyro[2] - self.calibration_value['gyro offset'][2]

        self.acc[0] = self.imu_acc[0] - self.calibration_value['acc offset'][0]
        self.acc[1] = self.imu_acc[1] - self.calibration_value['acc offset'][1]
        self.acc[2] = self.imu_acc[2] - self.calibration_value['acc offset'][2]
    
    def imu_callback(self, msg :Imu):
        self.imu_acc[0] = msg.linear_acceleration.x
        self.imu_acc[1] = msg.linear_acceleration.y  
        self.imu_acc[2] = msg.linear_acceleration.z  

        self.imu_gyro[0] = msg.angular_velocity.x
        self.imu_gyro[1] = msg.angular_velocity.y
        self.imu_gyro[2] = msg.angular_velocity.z

        self.imu_offset()

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        imu.angular_velocity.x = self.gyro[0]
        imu.angular_velocity.y = self.gyro[1]
        imu.angular_velocity.z = self.gyro[2]
        # imu.angular_velocity_covariance = np.array(self.calibration_value['gyro covarience']).flatten().tolist()

        imu.linear_acceleration.x = self.acc[0]
        imu.linear_acceleration.y = self.acc[1]
        imu.linear_acceleration.z = self.acc[2]
        # imu.linear_acceleration_covariance = np.array(self.calibration_value['acc covarience']).flatten().tolist()

        # Covariance
        vel_cov = 0.5
        acc_cov = 0.5
        imu.angular_velocity_covariance = [vel_cov, 0.0, 0.0,
                                           0.0, vel_cov, 0.0,
                                           0.0, 0.0, vel_cov]
        imu.linear_acceleration_covariance = [acc_cov, 0.0, 0.0,
                                              0.0, acc_cov, 0.0,
                                              0.0, 0.0, acc_cov]
        
        # Publish imu
        self.imu_publisher.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()