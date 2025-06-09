#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Header

def quaternion_to_yaw(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)

class SegmentedNavigator(Node):
    def __init__(self):
        super().__init__('segmented_navigator')
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_timer(1.0, self._pub_heartbeat)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.waypoints = [
            (1.0, -0.8),   # WP1
            (3.0, -0.8),    # WP2
            (2.5,  3.0),    # WP3
            (2.5,  0.0),    # WP4
        ]

        # Sequence: move, turn, move, turn, move, turn, move
        self.steps = [
            ('move', 0),
            ('turn',  math.pi/2),
            ('move', 1),
            ('turn',  math.pi/2),
            ('move', 2),
            ('turn',  math.pi),
            ('move', 3),           # ← now actually go to WP4
        ]
        self.current_step = 0

        self.x = self.y = self.yaw = None
        self.target_yaw = None

        # Faster gains
        self.k_lin = 0.5
        self.k_ang = 2.0
        self.max_lin = 0.5
        self.max_ang = 2.0
        self.dist_thresh = 0.05
        self.yaw_thresh  = math.radians(2.0)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SegmentedNavigator ready. Waiting for odom...")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)

    def control_loop(self):
        if self.x is None:
            return

        if self.current_step >= len(self.steps):
            self.get_logger().info("✅ All steps done — shutting down navigator.")
            # stop the robot
            self.cmd_pub.publish(Twist())
            # shutdown so OnProcessExit can fire
            rclpy.shutdown()
            raise SystemExit(0)

        kind, val = self.steps[self.current_step]
        twist = Twist()

        if kind == 'move':
            gx, gy = self.waypoints[val]
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist < self.dist_thresh:
                self.get_logger().info(f"Reached WP{val+1} at ({gx:.2f},{gy:.2f})")
                self.current_step += 1
                return
            # small angular correction while moving
            target_yaw = math.atan2(dy, dx)
            err = math.atan2(math.sin(target_yaw - self.yaw),
                             math.cos(target_yaw - self.yaw))
            twist.linear.x  = min(self.k_lin * dist, self.max_lin)
            twist.angular.z = max(-self.max_ang,
                                  min(self.k_ang * err, self.max_ang))

        else:  # a 'turn'
            angle = val
            if self.target_yaw is None:
                # compute absolute target yaw
                self.target_yaw = math.atan2(
                    math.sin(self.yaw + angle),
                    math.cos(self.yaw + angle)
                )
                deg = math.degrees(angle)
                self.get_logger().info(f"🔄 Turning {deg:+.0f}° → target yaw {math.degrees(self.target_yaw):.1f}°")

            err = math.atan2(math.sin(self.target_yaw - self.yaw),
                             math.cos(self.target_yaw - self.yaw))
            if abs(err) < self.yaw_thresh:
                self.get_logger().info("Turn complete")
                self.target_yaw = None
                self.current_step += 1
                return
            twist.angular.z = max(-self.max_ang,
                                  min(self.k_ang * err, self.max_ang))
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

    def _pub_heartbeat(self):
      hdr = Header()
      hdr.stamp = self.get_clock().now().to_msg()
      hdr.frame_id = self.get_name()    # e.g. "navigator" or "bridge_node"
      self.hb_pub.publish(hdr)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentedNavigator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        # ensure robot is stopped
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
