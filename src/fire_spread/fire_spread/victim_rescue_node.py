#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Header
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus

class VictimRescueNode(Node):
    def __init__(self):
        super().__init__('victim_rescue_node')
        # Heartbeat
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.create_timer(1.0, self._pub_hb)
        # Nav2 clients
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_local = self.create_client(Empty, '/local_costmap/clear_entirely')
        self.clear_global = self.create_client(Empty, '/global_costmap/clear_entirely')
        self._connect_timer = self.create_timer(1.0, self._try_connect)
        # Params
        self.declare_parameter('goal_timeout_s', 60.0)
        self.goal_timeout = self.get_parameter('goal_timeout_s').value
        # Waypoints
        self._wps = [{'x':2.0,'y':3.0,'yaw':0.0},{'x':2.5,'y':-0.5,'yaw':0.0}]
        self._cur = 0
        self._returning = False
        # State
        self._pillars = []
        self._plan = None
        self._out = None
        self._start_time = None
        # Subscriptions
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(PointStamped, 'twin/pillars', self._on_pillar, 10)

    def _pub_hb(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.get_name()
        self.hb_pub.publish(h)

    def _on_plan(self, msg: Path):
        if not self._out and not self._returning:
            self._out = list(msg.poses)
        self._plan = msg.poses

    def _on_pillar(self, msg: PointStamped):
        self._pillars.insert(0, (msg.point.x, msg.point.y))

    def _try_connect(self):
        if not self.nav_cli.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose server...')
            return
        self._connect_timer.cancel()
        self._send()

    def _predict(self, poses):
        for p in poses:
            x, y = p.pose.position.x, p.pose.position.y
            for px, py in self._pillars[:20]:
                if math.hypot(x-px, y-py) < (0.035 + 0.10):
                    return True
        return False

    def _send(self):
        # Completed all waypoints
        if self._cur >= len(self._wps):
            return self._handle_done()
        wp = self._wps[self._cur]
        # Check block
        if self._plan and self._predict(self._plan):
            self.get_logger().warn('Path blocked; clearing costmaps to replan')
            if self.clear_local.service_is_ready(): self.clear_local.call_async(Empty.Request())
            if self.clear_global.service_is_ready(): self.clear_global.call_async(Empty.Request())
            # retry after clear
            self.create_timer(1.0, lambda: self._send(), once=True)
            return
        # Send goal
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(wp)
        self._start_time = self.get_clock().now()
        self.get_logger().info(f'Sending waypoint #{self._cur+1}: ({wp["x"]:.2f},{wp["y"]:.2f})')
        fut = self.nav_cli.send_goal_async(goal, feedback_callback=self._fb)
        fut.add_done_callback(self._on_resp)

    def _make_pose(self, wp):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = wp['x']; p.pose.position.y = wp['y']
        qz = math.sin(wp['yaw']/2.0); qw = math.cos(wp['yaw']/2.0)
        p.pose.orientation.z = qz; p.pose.orientation.w = qw
        return p

    def _fb(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f'distance_remaining: {d:.2f} m')
        if (self.get_clock().now() - self._start_time).nanoseconds * 1e-9 > self.goal_timeout:
            self.get_logger().warn('Goal timed out; skipping')
            self._cur += 1
            return self._send()

    def _on_resp(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected; shutting down')
            rclpy.shutdown(); return
        handle.get_result_async().add_done_callback(self._on_res)

    def _on_res(self, fut):
        res = fut.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint #{self._cur+1} reached')
            self._cur += 1
            self._send()
        else:
            self.get_logger().error(f'Goal failed with status {res.status}; shutting down')
            rclpy.shutdown()

    def _handle_done(self):
        if not self._returning and self._out:
            self.get_logger().info('Outbound complete; evaluating return path')
            return_path = list(reversed(self._out))
            if self._predict(return_path):
                self.get_logger().warn('Return path blocked; clearing costmaps')
                if self.clear_local.service_is_ready(): self.clear_local.call_async(Empty.Request())
                if self.clear_global.service_is_ready(): self.clear_global.call_async(Empty.Request())
            else:
                self.get_logger().info('Return path clear')
            # set goal back to start
            start = self._out[0].pose.position
            self._wps = [{'x': start.x, 'y': start.y, 'yaw': 0.0}]
            self._cur = 0
            self._returning = True
            return self._send()
        self.get_logger().info('All missions done; shutting down')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VictimRescueNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
