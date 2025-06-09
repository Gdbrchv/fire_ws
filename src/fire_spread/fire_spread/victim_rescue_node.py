#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class VictimRescueNode(Node):
    def __init__(self):
        super().__init__('victim_rescue_node')

        # 1) Tell RCLPy to use /clock
        self.declare_parameter('use_sim_time', True)

        # 2) Heartbeat (optional)
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.create_timer(1.0, self._pub_heartbeat)

        # 3) Non‐blocking timer to check for Nav2
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._connect_timer = self.create_timer(1.0, self._try_send_goal)

        # 4) Rescue waypoints
        self._waypoints = [
            {'x': 2.0, 'y': 3.5, 'yaw': 0.0},
            {'x': 2.5, 'y': 0.0, 'yaw': 0.0},
        ]
        self._current = 0

    def _pub_heartbeat(self):
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.get_name()
        self.hb_pub.publish(hdr)

    def _try_send_goal(self):
        # Try to wait for server once, up to 10s
        self._connect_timer.cancel()
        self.get_logger().info('🔗 Waiting (up to 10s) for navigate_to_pose server…')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Action server never appeared—giving up.')
            rclpy.shutdown()
            return
        self.get_logger().info('✅ Action server is up—sending first goal.')
        self._send_next_goal()

    def _create_pose(self, wp):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = wp['x']
        p.pose.position.y = wp['y']
        qz = math.sin(wp['yaw']/2.0)
        qw = math.cos(wp['yaw']/2.0)
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _send_next_goal(self):
        if self._current >= len(self._waypoints):
            self.get_logger().info('🎉 All rescue goals complete—shutting down.')
            rclpy.shutdown()
            return

        wp = self._waypoints[self._current]
        goal = NavigateToPose.Goal()
        goal.pose = self._create_pose(wp)

        self.get_logger().info(f"→ Sending goal #{self._current+1}: ({wp['x']},{wp['y']})")
        fut = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback)
        fut.add_done_callback(self._on_response)

    def _on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('❌ Goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('▶️ Goal accepted, waiting for result…')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._on_result)

    def _feedback(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f"    distance_remaining: {d:.2f} m")

    def _on_result(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"✅ Reached waypoint #{self._current+1}")
            self._current += 1
            self._send_next_goal()
        else:
            self.get_logger().error(f"❌ Failed at waypoint #{self._current+1}, status={status}")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VictimRescueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
