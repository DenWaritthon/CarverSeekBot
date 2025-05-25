#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations

from PyRoboteq import RoboteqHandler
from PyRoboteq import roboteq_commands as cmds

import time
import math
from math import sin, cos, pi

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        self.controller = RoboteqHandler()
        # Set MotorDriver Connectt in com port /dev/tty{your_port} (Commonly /dev/ttyACM0) for linux.
        self.connected = self.controller.connect("/dev/diffdrive") 

        # Pub Topic
        self.pub_odom = self.create_publisher(Odometry,'/wheel/odom',10)

        # Sup Topic
        self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)

        # Variables
        # Mobile variables
        self.wheel_separation = 0.35
        self.wheel_radius = 0.075
        self.get_logger().error

        # Controll variables
        self.k_gain = 20

        # Odom variables

        self.get_logger().info("Diff drive controller started")
        
    def cmd_vel_callback(self,msg:Twist):
        # Calculate Inverse Kinematics
        cmd = self.compute(msg.linear.x,msg.angular.z)
        v_r = cmd[0] * self.k_gain
        v_l = cmd[1] * self.k_gain
        self.get_logger().info(f'Right wheel velocity : {v_r} , Left wheel velocity : {v_l}')

        # Send command to set wheel velocity
        self.controller.send_command(cmds.DUAL_DRIVE, v_r, v_l)
        
    def compute(self,v,w):
        # compute Inverse Kinematics
        left_wheel_velocity = (v/self.wheel_radius) - ((self.wheel_separation/(2*self.wheel_radius))*w)
        right_wheel_velocity = (v/self.wheel_radius) + ((self.wheel_separation/(2*self.wheel_radius))*w)
        return [right_wheel_velocity,left_wheel_velocity]
    
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
