#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np
from sklearn.cluster import DBSCAN
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time
from std_msgs.msg import Header
def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class LidarBottleDetector(Node):
    """
    Detect roughly bottle-sized clusters in a LaserScan using DBSCAN,
    filter by diameter, shape, and distance, then publish triggers.
    """
    def __init__(self):
        super().__init__('lidar_bottle_detector')
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.create_timer(1.0, self._pub_heartbeat)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb_scan,
            10)
        self.pub = self.create_publisher(PointStamped, '/fire_triggers', 10)

        self.declare_parameter('eps',       0.05)
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('min_d',     0.05)
        self.declare_parameter('max_d',     0.30)
        self.declare_parameter('max_shape_ratio', 3.0)
        self.declare_parameter('min_dist',  0.2)
        self.declare_parameter('max_dist',  2.0)

        self.eps           = self.get_parameter('eps').get_parameter_value().double_value
        self.min_samples   = self.get_parameter('min_samples').get_parameter_value().integer_value
        self.min_d         = self.get_parameter('min_d').get_parameter_value().double_value
        self.max_d         = self.get_parameter('max_d').get_parameter_value().double_value
        self.max_shape     = self.get_parameter('max_shape_ratio').get_parameter_value().double_value
        self.min_dist      = self.get_parameter('min_dist').get_parameter_value().double_value
        self.max_dist      = self.get_parameter('max_dist').get_parameter_value().double_value

        self.get_logger().info(
            f"Detector init eps={self.eps}, min_samples={self.min_samples}, d_range=[{self.min_d},{self.max_d}], shape<={self.max_shape}, dist_range=[{self.min_dist},{self.max_dist}]"
        )
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detected = False
  
    def _pub_heartbeat(self):
      hdr = Header()
      hdr.stamp = self.get_clock().now().to_msg()
      hdr.frame_id = self.get_name()    # e.g. "navigator" or "bridge_node"
      self.hb_pub.publish(hdr)

    def cb_scan(self, scan: LaserScan):
        if self.detected:
            return
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        pts = []
        for r, a in zip(scan.ranges, angles):
            if np.isfinite(r) and scan.range_min < r < scan.range_max:
                x = r * math.cos(a)
                y = r * math.sin(a)
                pts.append([x, y])
        if len(pts) < self.min_samples:
            return
        pts = np.array(pts)

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(pts)
        labels = clustering.labels_

        for lab in set(labels):
            if lab < 0:
                continue  
            cluster = pts[labels == lab]
            dists = np.linalg.norm(cluster[:, None, :] - cluster[None, :, :], axis=2)
            diameter = float(np.max(dists))
            if not (self.min_d <= diameter <= self.max_d):
                continue

            cov = np.cov(cluster, rowvar=False)
            eigs = np.linalg.eigvals(cov)
            ratio = float(max(eigs) / min(eigs)) if np.min(eigs) > 0 else np.inf
            if ratio > self.max_shape:
                continue

            cx, cy = cluster.mean(axis=0)
            dist = math.hypot(cx, cy)
            if not (self.min_dist <= dist <= self.max_dist):
                continue
            trigger = PointStamped()
            trigger.header.stamp = scan.header.stamp            
            trigger.header.frame_id = 'base_link'
            trigger.point.x = float(cx)
            trigger.point.y = float(cy)
            trigger.point.z = 0.0

            try:
                t = self.tf_buffer.lookup_transform(
                    'map',           
                    'base_link',     
                    Time(),             
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                trigger = do_transform_point(trigger, t)
               
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                
            if abs(cx - 2.0) < 0.15:
                return 
            self.pub.publish(trigger)
            self.get_logger().info("First bottle detected, shutting down detector.")
            self.detected = True
            self.destroy_subscription(self.sub)

        def _pub_heartbeat(self):
            hdr = Header()
            hdr.stamp = self.get_clock().now().to_msg()
            hdr.frame_id = self.get_name()    # e.g. "navigator" or "bridge_node"
            self.hb_pub.publish(hdr)
def main(args=None):
    rclpy.init(args=args)
    node = LidarBottleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
