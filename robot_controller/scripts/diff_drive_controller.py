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
        self.connected = self.controller.connect("/dev/ttyUSB2") 

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
        self.wheel = np.array([0.0,0.0])
        self.odom_broadcaster = TransformBroadcaster(self)

        # self.timer = self.create_timer(self.period,self.timer_callback)
        self.timer = self.create_timer(0.02,self.timer_roboteq_callback)
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
    
    def timer_roboteq_callback(self):
        try:
            bl = self.controller.read_value(cmds.READ_BL_MOTOR_RPM, 0) # Read value 1 of battery amps
            spd_1 = bl.split("=")
            spd_2 = spd_1[1].split(":")
            dt = time.time() - self.t
            self.t = time.time()
            self.data["speed_l"] = -float(spd_2[0]) *  2 * math.pi / 2400
            self.data["speed_r"] = -float(spd_2[1]) *  2 * math.pi / 2400
            self.odometry_compute(dt)
        except:
            print("error")
    
    def cmd_vel_callback(self,msg:Twist):
        # cmd = Float64MultiArray()
        print(str(msg.linear.x) + " " + str(msg.angular.z))
        cmd = self.compute(msg.linear.x,msg.angular.z)
        # self.counter = 0
        # print(cmd)
        # self.command_publisher.publish(cmd)
        v_l = cmd[0] * 80
        v_r = cmd[1] * 80
        self.controller.send_command(cmds.DUAL_DRIVE, v_l, v_r)
        
        # self.controller.send_command(cmds.DUAL_DRIVE, 0, 10)
        # self.controller.send_command(cmds.EM_STOP, 1)
        
    def compute(self,v,w):
        # compute Inverse Kinematics
        left_wheel_velocity = (v/self.wheel_radius) - ((self.wheel_separation/(2*self.wheel_radius))*w)
        right_wheel_velocity = (v/self.wheel_radius) + ((self.wheel_separation/(2*self.wheel_radius))*w)
        return [left_wheel_velocity,right_wheel_velocity]
    
    def integrate(self, r, b, dt):
        self.vx = (self.data["speed_l"]+self.data["speed_r"])*r/2
        self.vy = 0.0
        self.vth = (self.data["speed_r"]-self.data["speed_l"])*r/b * 1.0
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        # print(self.data)
        print(self.vth)
        # print(str(self.vx) + " " + str(self.vth) + " " + str(self.x) + " " + str(self.y) + " " + str(self.th * 180 / 3.14))

    def odometry_compute(self, dt):
        r = self.wheel_radius
        b = self.wheel_separation
        # print(dt)
        self.integrate(r,b,dt)
        # current_time = self.get_clock().now()
        # odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        # self.odom_broadcaster.sendTransform((self.x, self.y, 0.),odom_quat,current_time,"base_footprint","odom")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.odom_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.child_frame_id = "base_footprint"
        # odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # p_cov = np.array([0.00001]*36).reshape(6,6)

		# position covariance
		# p_cov[0:2,0:2] = self.P[0:2,0:2]
		# orientation covariance for Yaw
		# x and Yaw
		# p_cov[5,0] = p_cov[0,5] = self.P[2,0]
		# y and Yaw
		# p_cov[5,1] = p_cov[1,5] = self.P[2,1]
		# Yaw and Yaw
		# p_cov[5,5] = self.P[2,2]
        pose_cov = 0.01
        twist_cov = 0.01
        odom.pose.covariance = [ pose_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, pose_cov, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, pose_cov, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, pose_cov, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, pose_cov, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, pose_cov]
        odom.twist.covariance = [ twist_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, twist_cov, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, twist_cov, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, twist_cov, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, twist_cov, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, twist_cov]
        # print(odom)
        # publish the message
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = AJNController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
