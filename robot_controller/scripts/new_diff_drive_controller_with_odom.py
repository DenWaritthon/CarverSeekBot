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

        # Timer loop
        self.timer = self.create_timer(0.02,self.timer_roboteq_callback)

        # Odom broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # Variables
        # Mobile variables
        self.wheel_separation = 0.35
        self.wheel_radius = 0.075

        # Controll variables
        self.k_gain = 20

        # Odom variables
        self.data = {}
        self.data["wheel_l"] = 0
        self.data["wheel_r"] = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.

        self.t = time.time()

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
    
    def timer_roboteq_callback(self):
        try:
            bl = self.controller.read_value(cmds.READ_BL_MOTOR_RPM, 0) # Read value 1 of battery amps
            spd_1 = bl.split("=")
            spd_2 = spd_1[1].split(":")
            dt = time.time() - self.t
            self.t = time.time()
            self.data["speed_r"] = -float(spd_2[0]) *  2 * math.pi / 2400
            self.data["speed_l"] = -float(spd_2[1]) *  2 * math.pi / 2400
            self.odometry_compute(dt)
        except:
            self.get_logger().error(f'Read wheel speed error!')
    
    def odometry_compute(self, dt):
        # Find x, y, th, v_x, v_y, V_th
        self.integrate(self.wheel_radius,self.wheel_separation,dt)

        # Set Odom Traransform Header
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'map'

        # Set Odom Traransform Data
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.odom_broadcaster.sendTransform(t)
        
        # Set Odom Header
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "map"

        # Set Odom Position Data
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set Odom Velocity Data
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Set Odom Covariance Data
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

        # publish odom
        self.pub_odom.publish(odom)

    def integrate(self, r, b, dt):
        # Calculate Vx , Vy, Vth (Vy = 0.0)
        self.vx = (self.data["speed_l"]+self.data["speed_r"])*r/2   
        self.vth = (self.data["speed_r"]-self.data["speed_l"])*r/b * 1.0

        # Calculate Position x, y, th
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        self.x += delta_x

        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        self.y += delta_y

        delta_th = self.vth * dt
        self.th += delta_th
    
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
