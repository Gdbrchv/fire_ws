#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from gazebo_msgs.srv import SpawnEntity

class CircularClusterSpawner(Node):
    """Spawns pillars in an expanding circular cluster bounded by a rectangular region."""
    def __init__(self):
        super().__init__('circular_cluster_spawner')

        # Growth and timing parameters
        self.declare_parameter('spread_rate',    0.1)    # meters per second radial growth
        self.declare_parameter('angle_step_deg', 20.0)   # degrees between pillars on each ring
        self.declare_parameter('spawn_interval', 1.0)    # seconds between rings
        self.declare_parameter('fill_duration',  15.0)   # seconds total

        # Boundary of allowable region
        self.declare_parameter('region_x_min', 2.0)
        self.declare_parameter('region_x_max', 3.4)
        self.declare_parameter('region_y_min', 0.0)
        self.declare_parameter('region_y_max', 3.5)

        p = self.get_parameter
        self.spread_rate    = p('spread_rate').value
        self.angle_step     = math.radians(p('angle_step_deg').value)
        self.spawn_interval = p('spawn_interval').value
        self.fill_duration  = p('fill_duration').value
        self.x_min = p('region_x_min').value
        self.x_max = p('region_x_max').value
        self.y_min = p('region_y_min').value
        self.y_max = p('region_y_max').value

        # Gazebo spawn service
        self.spawn_cli = self.create_client(SpawnEntity, 'spawn_entity')
        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('spawn_entity service unavailable; shutting down.')
            rclpy.shutdown()
            return

        # Publisher for pillar positions
        self.pillar_pub = self.create_publisher(PointStamped, 'twin/pillars', 10)

        # SDF template for a pillar
        self.sdf = '''<sdf version='1.6'>
  <model name='fire_pillar_{i}'>
    <static>true</static>
    <link name='link'>
      <collision name='col'>
        <geometry><cylinder>
          <radius>0.035</radius>
          <length>2.5</length>
        </cylinder></geometry>
      </collision>
      <visual name='vis'>
        <geometry><cylinder>
          <radius>0.035</radius>
          <length>2.5</length>
        </cylinder></geometry>
      </visual>
    </link>
  </model>
</sdf>'''

        # Internal state
        self.center_x = None
        self.center_y = None
        self.center_frame = None
        self.start_time = None
        self.spawned_rings = set()
        self.next_idx = 0

        # Subscribe to trigger topic
        self.create_subscription(
            PointStamped,
            'fire_triggers',
            self._on_trigger,
            10
        )

        # Timer to grow cluster
        self.create_timer(self.spawn_interval, self._on_timer)
        self.get_logger().info('CircularClusterSpawner ready; awaiting /fire_triggers.')

    def _on_trigger(self, msg: PointStamped):
        self.center_x = msg.point.x
        self.center_y = msg.point.y
        self.center_frame = msg.header.frame_id
        self.start_time = self.get_clock().now()
        self.spawned_rings.clear()
        self.next_idx = 0
        self.get_logger().info(f'Trigger at ({self.center_x:.2f}, {self.center_y:.2f}); starting cluster growth.')

    def _on_timer(self):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.fill_duration:
            self.get_logger().info('Cluster growth complete; stopping.')
            self.start_time = None
            return

        ring = int(math.floor(elapsed / self.spawn_interval))
        if ring in self.spawned_rings:
            return
        self.spawned_rings.add(ring)
        radius = self.spread_rate * elapsed
        self._spawn_ring(radius)

    def _spawn_ring(self, radius: float):
        angle = 0.0
        while angle < 2 * math.pi:
            x = self.center_x + radius * math.cos(angle)
            y = self.center_y + radius * math.sin(angle)
            # Enforce rectangular boundary
            if self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max:
                self._spawn_pillar(x, y)
            angle += self.angle_step

    def _spawn_pillar(self, x: float, y: float):
        req = SpawnEntity.Request()
        req.name = f'pillar_{self.next_idx}'
        req.xml = self.sdf.format(i=self.next_idx)
        req.reference_frame = self.center_frame
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 1.25
        req.initial_pose.orientation.w = 1.0
        self.spawn_cli.call_async(req)

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = self.center_frame
        pt.point.x = x
        pt.point.y = y
        self.pillar_pub.publish(pt)

        self.next_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = CircularClusterSpawner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
