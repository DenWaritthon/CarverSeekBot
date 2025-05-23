#!/usr/bin/python3

import rclpy
from rclpy.node import Node
##from std_msgs.msg import String
from std_msgs.msg import Float32, String
# from robotiq_ft_sensor.msg import ft_sensor
from geometry_msgs.msg import WrenchStamped, Wrench
import socket
from time import gmtime, strftime
# import ConfigParser
import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import yaml
import math
from math import sin, cos, pi
import numpy as np
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from PyRoboteq import RoboteqHandler
from PyRoboteq import roboteq_commands as cmds

class AJNController(Node):
    def __init__(self):
        super().__init__('AJN_controller')
        self.controller = RoboteqHandler()
        # self.connected = self.controller.connect("/dev/ttyDiffDrive") # Insert your COM port (for windows) or /dev/tty{your_port} (Commonly /dev/ttyACM0) for linux.
        self.connected = self.controller.connect("/dev/ttyUSB0") 

        # self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub_odom = self.create_publisher(Odometry,'/wheel/odom',10)
        
        # self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cmd_vel = Twist()
        self.period = 0.1
        
        self.counter = 0
        self.dt = 0.1

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.k_gain = 20
        self.wheel = np.array([0.0,0.0])
        self.odom_broadcaster = TransformBroadcaster(self)

        # self.timer = self.create_timer(self.period,self.timer_callback)
        # self.timer = self.create_timer(0.02,self.timer_roboteq_callback)
        self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        # load yaml file
        # with open(sys.argv[1]) as f:
        #     model_parameter = yaml.load(f, Loader=yaml.loader.SafeLoader)
        self.wheel_separation = 0.35#float(model_parameter['wheel_separation'])  #0.45
        self.wheel_radius = 0.075#float(model_parameter['wheel_radius'])          #0.08
        self.data = {}
        self.data["wheel_l"] = 0
        self.data["wheel_r"] = 0
        self.t = time.time()

        self.get_logger().info("AJN diff drive controller started")
        
    
    def cmd_vel_callback(self,msg:Twist):
        # cmd = Float64MultiArray()
        # print(str(msg.linear.x) + " " + str(msg.angular.z))
        cmd = self.compute(msg.linear.x,msg.angular.z)
        # self.counter = 0
        # print(cmd)
        # self.command_publisher.publish(cmd)
        v_l = cmd[0] * self.k_gain
        v_r = cmd[1] * self.k_gain
        print("v_l: " + str(v_l) + " v_r: " + str(v_r))
        self.controller.send_command(cmds.DUAL_DRIVE, v_l, v_r)
        bl = self.controller.read_value(cmds.READ_BL_MOTOR_RPM, 0)
        print("BL: " + str(bl))
        # self.controller.send_command(cmds.)
        
        # self.controller.send_command(cmds.DUAL_DRIVE, 0, 10)
        # self.controller.send_command(cmds.EM_STOP, 1)
        
    def compute(self,v,w):
        # compute Inverse Kinematics
        left_wheel_velocity = (v/self.wheel_radius) - ((self.wheel_separation/(2*self.wheel_radius))*w)
        right_wheel_velocity = (v/self.wheel_radius) + ((self.wheel_separation/(2*self.wheel_radius))*w)
        return [right_wheel_velocity,left_wheel_velocity]
    
def main(args=None):
    rclpy.init(args=args)
    node = AJNController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
