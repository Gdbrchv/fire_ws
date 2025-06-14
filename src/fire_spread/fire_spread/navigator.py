
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))

class SegmentedNavigator(Node):
    def __init__(self):
        super().__init__('segmented_navigator')
        self.declare_parameter('x_min',  0.0)
        self.declare_parameter('x_max',  3.61 - 0.25)
        self.declare_parameter('y_min', -1.0)
        self.declare_parameter('y_max',  3.61 - 0.25)
        self.declare_parameter('wall_x_center',   2.0)
        self.declare_parameter('wall_thickness',  0.35)
        self.declare_parameter('wall_y_min',     -0.61)
        self.declare_parameter('wall_y_max',      2.5)

        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        wall_xc = self.get_parameter('wall_x_center').value
        th = self.get_parameter('wall_thickness').value
        wall_x_min = wall_xc - th / 2.0
        wall_x_max = wall_xc + th / 2.0
        wall_y_min = self.get_parameter('wall_y_min').value
        wall_y_max = self.get_parameter('wall_y_max').value

        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_timer(1.0, self._pub_heartbeat)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)

        self.waypoints = [
            (2.5,  0.0),
            (2.5, -0.8),
            (1.0, -0.8),
            (1.0,  3.0),
            (2.5,  3.0),
            (2.5,  0.0),
        ]

        self.steps = [
            ('move',  1),
            ('turn', -math.pi/2),
            ('move',  2),
            ('turn', -math.pi/2),
            ('move',  3),
            ('turn', -math.pi/2),
            ('move',  4),
            ('turn', -math.pi/2),
            ('move',  5),
        ]
        self.current_step = 0

        self.current_step = 0
        self.x = self.y = self.yaw = None
        self.target_yaw = None

        self.k_lin = 0.5
        self.k_ang = 2.0
        self.max_lin = 0.5
        self.max_ang = 2.0
        self.dist_thresh = 0.05
        self.yaw_thresh = math.radians(2.0)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("SegmentedNavigator ready. Waiting for odom...")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)

    def control_loop(self):
        if self.x is None:
            return

        if self.current_step >= len(self.steps):
            self.get_logger().info("All steps done — shutting down navigator.")
            self.cmd_pub.publish(Twist())
            self.destroy_node()
            return

        kind, val = self.steps[self.current_step]
        twist = Twist()

        if kind == 'move':
            gx, gy = self.waypoints[val]
            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)
            if dist < self.dist_thresh:
                self.get_logger().info(f"Reached WP{val} at ({gx:.2f},{gy:.2f})")
                self.current_step += 1
                return
            target_yaw = math.atan2(dy, dx)
            err = math.atan2(math.sin(target_yaw - self.yaw),
                             math.cos(target_yaw - self.yaw))
            twist.linear.x  = min(self.k_lin * dist, self.max_lin)
            twist.angular.z = max(-self.max_ang,
                                  min(self.k_ang * err, self.max_ang))
        else:
            angle = val
            if self.target_yaw is None:
                self.target_yaw = math.atan2(
                    math.sin(self.yaw + angle),
                    math.cos(self.yaw + angle)
                )
                self.get_logger().info(
                    f"Turning to target yaw {math.degrees(self.target_yaw):.1f}°"
                )
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
        hdr.frame_id = self.get_name()
        self.hb_pub.publish(hdr)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentedNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
