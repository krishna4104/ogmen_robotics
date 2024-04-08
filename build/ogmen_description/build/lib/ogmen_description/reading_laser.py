import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan/unfiltered',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/filtered_scan', 10)

    def scan_callback(self, msg):
        print("Received unfiltered scan message")
        print("Number of ranges:", len(msg.ranges))
        print("Header:", msg.header)
        print("Angle increment:", msg.angle_increment)
        print("Time increment:", msg.time_increment)
        print("Scan time:", msg.scan_time)
        print("Range min:", msg.range_min)
        print("Range max:", msg.range_max)

        filtered_ranges = []
        filtered_angles = []

        # Define range of angles to keep
        min_angle = -1.0472
        max_angle = 1.0472  # 120 degrees in radians

        # Calculate corresponding indices for min_angle and max_angle
        min_index = int(min_angle / msg.angle_increment)
        max_index = int(max_angle / msg.angle_increment)

        # Filter ranges and angles
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if min_angle <= angle <= max_angle:
                filtered_ranges.append(distance)
                filtered_angles.append(angle)

        print("Filtered ranges:", filtered_ranges)
        print("Filtered angles:", filtered_angles)

        # Update LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = float(min_angle)
        filtered_scan.angle_max = float(max_angle)
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = []  # Clear intensities

        # Publish filtered scan
        print("Publishing filtered scan message")
        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    scan_filter = ScanFilter()
    rclpy.spin(scan_filter)
    scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
