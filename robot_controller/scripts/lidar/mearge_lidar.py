#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import message_filters

class DualScanMerger(Node):
    def __init__(self):
        super().__init__('dual_scan_merger')
        self.get_logger().info('Dual Scan Merger node started')
        
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Subscribers for front and back scans
        self.sub_front = message_filters.Subscriber(self, LaserScan, '/f_scan_filtered')
        self.sub_back = message_filters.Subscriber(self, LaserScan, '/b_scan_filtered')

        # Approximate time synchronizer to merge scans
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_front, self.sub_back],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.merge_callback)
        
        self.get_logger().info('Subscribers and publisher initialized')

    def merge_callback(self, front_scan, back_scan):
        merged_scan = LaserScan()
        # Copy header from front scan
        merged_scan.header.stamp = front_scan.header.stamp
        merged_scan.header.frame_id = 'base_footprint'
        # Merge ranges and intensities
        merged_scan.angle_min = front_scan.angle_min
        merged_scan.angle_max = back_scan.angle_max
        merged_scan.angle_increment = front_scan.angle_increment
        merged_scan.range_min = min(front_scan.range_min, back_scan.range_min)
        merged_scan.range_max = max(front_scan.range_max, back_scan.range_max)
        merged_scan.ranges = front_scan.ranges + back_scan.ranges
        self.pub.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = DualScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
