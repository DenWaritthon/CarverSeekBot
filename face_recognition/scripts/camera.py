#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # Publish at 10Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Use default camera

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            # Convert the frame to ROS Image message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(ros_image)
                self.get_logger().info('Publishing image')
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
        else:
            self.get_logger().error('Failed to capture image')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
