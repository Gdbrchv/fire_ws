#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class VictimRescueNode(Node):
    def __init__(self):
        super().__init__('victim_rescue_node')
        # Create an ActionClient for the NavigateToPose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define the sequence of waypoints (x, y, yaw)
        self._waypoints = [
            {'x': 2.0, 'y':  3.5, 'yaw': 0.0},  # Go to victim
            {'x': 2.5, 'y': 0.0, 'yaw': 0.0}   # Return to drop-off
        ]
        self._current_index = 0

        # Wait for the action server and send the first goal
        self._action_client.wait_for_server()
        self._send_next_goal()

    def _create_pose(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = 0.0
        # Convert yaw to quaternion
        qz = math.sin(wp['yaw'] / 2.0)
        qw = math.cos(wp['yaw'] / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_next_goal(self):
        if self._current_index >= len(self._waypoints):
            self.get_logger().info('All waypoints reached. Shutting down.')
            rclpy.shutdown()
            return

        wp = self._waypoints[self._current_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(wp)

        self.get_logger().info(
            f"Sending goal #{self._current_index + 1}: x={wp['x']} y={wp['y']}"
        )
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} m"
        )

    def _get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"Reached waypoint #{self._current_index + 1}!"
            )
            self._current_index += 1
            self._send_next_goal()
        else:
            self.get_logger().error(
                f"Failed to reach waypoint #{self._current_index + 1}, status={status}"
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VictimRescueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
