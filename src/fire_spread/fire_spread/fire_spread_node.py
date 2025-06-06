#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FireScanPublisher(Node):
    def __init__(self):
        super().__init__('fire_scan_publisher')
        # Publish at 5 Hz on /fire_scan
        self.pub = self.create_publisher(LaserScan, '/fire_scan', 10)
        timer_period = 0.2  # seconds = 5 Hz
        self.timer = self.create_timer(timer_period, self.publish_fire_scan)

        # Configure a 180° scan from -90° to +90°, 1 m away
        self.angle_min = -math.pi / 2.0
        self.angle_max = math.pi / 2.0
        self.angle_increment = math.radians(1.0)  # 1° increments (180 points)
        self.range_min = 0.0
        self.range_max = 5.0  # 5 m sensor range

    def publish_fire_scan(self):
        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'base_scan'  # or 'base_link', whichever your costmap expects
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = 0.2
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)

        # Build a “fire wall” between -30° and +30° at 1.0 m, everything else is “far away”
        n_points = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        ranges = [float(self.range_max)] * n_points

        for i in range(n_points):
            angle = self.angle_min + i * self.angle_increment
            # If angle between -30° and +30°, mark as obstacle at 1.0 m
            if abs(angle) <= math.radians(30.0):
                ranges[i] = 1.0

        msg.ranges = ranges
        msg.intensities = [0.0] * n_points

        self.pub.publish(msg)
        self.get_logger().debug(f'Published fire_scan with {n_points} points')

def main(args=None):
    rclpy.init(args=args)
    node = FireScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
