#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from gazebo_msgs.srv import SpawnEntity
import random, math
from std_msgs.msg import Header

class PillarPlume:
    """Tracks state for one plume of pillars."""
    def __init__(self, center_x, center_y, frame_id, params, spawn_fn):
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.create_timer(1.0, self._pub_heartbeat)
        self.cx = center_x
        self.cy = center_y
        self.frame = frame_id
        self.params = params
        self.spawn_fn = spawn_fn

        self.eligible = [{'x': center_x, 'y': center_y, 'children': 0}]
        self.step = 3.0 * params['pillar_radius']
        self.survival = params['max_children_per_parent']
        self.done = False
        

    def tick(self):
        """Spawn a batch; retire plume when no eligible parents remain."""
        if not self.eligible:
            self.done = True
            return

        count = random.randint(1, self.params['max_per_tick'])
        for _ in range(count):
            if not self.eligible:
                break
            self._spawn_one()
    def _pub_heartbeat(self):
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.get_name()    # e.g. "navigator" or "bridge_node"
        self.hb_pub.publish(hdr)
    def _spawn_one(self):
        parent = random.choice(self.eligible)
        px, py = parent['x'], parent['y']

        # outward‐biased angle
        dx, dy = px - self.cx, py - self.cy
        base = math.atan2(dy, dx) if (dx or dy) else random.uniform(0, 2*math.pi)
        angle = base + random.uniform(-math.pi/3, math.pi/3)
        dist  = random.uniform(self.step, 3*self.step)

        x = px + math.cos(angle)*dist
        y = py + math.sin(angle)*dist

        # clamp to bounds
        x = min(max(x, self.params['x_min']), self.params['x_max'])
        y = min(max(y, self.params['y_min']), self.params['y_max'])
        wx0 = self.params['wall_x_min']
        wx1 = self.params['wall_x_max']
        wy0 = self.params['wall_y_min']
        wy1 = self.params['wall_y_max']
        if wx0 < x < wx1 and wy0 < y < wy1:
            return
        self.spawn_fn(x, y, self.frame)

        parent['children'] += 1
        if parent['children'] >= self.survival:
            self.eligible.remove(parent)
        self.eligible.append({'x': x, 'y': y, 'children': 0})

class ImprovedFireSpawner(Node):
    def __init__(self):
        super().__init__('improved_fire_spawner')
        self.hb_pub = self.create_publisher(Header, 'sync/heartbeat', 10)
        self.create_timer(1.0, self._pub_heartbeat)
        
        self.declare_parameter('pillar_radius',           0.025)
        self.declare_parameter('spawn_interval',          0.75)
        self.declare_parameter('max_per_tick',            3)
        self.declare_parameter('max_children_per_parent', 3)
        self.declare_parameter('x_min',  0.0)
        self.declare_parameter('x_max',  3.61 - 0.25)
        self.declare_parameter('y_min', -1.0)
        self.declare_parameter('y_max',  3.61 - 0.25)
        self.declare_parameter('wall_x_center',   2.0)
        self.declare_parameter('wall_thickness',  0.35)
        self.declare_parameter('wall_y_min',     -0.61)
        self.declare_parameter('wall_y_max',      2.5)
        p  = self.get_parameter
        wc = p('wall_x_center').value
        wt = p('wall_thickness').value

        self.params = {
            # core
            'pillar_radius':           p('pillar_radius').value,
            'spawn_interval':          p('spawn_interval').value,
            'max_per_tick':            p('max_per_tick').value,
            'max_children_per_parent': p('max_children_per_parent').value,
            # corridor
            'x_min':                   p('x_min').value,
            'x_max':                   p('x_max').value,
            'y_min':                   p('y_min').value,
            'y_max':                   p('y_max').value,
            # wall
            'wall_x_min':              wc - wt/2.0,
            'wall_x_max':              wc + wt/2.0,
            'wall_y_min':              p('wall_y_min').value,
            'wall_y_max':              p('wall_y_max').value,
        }

        self.create_subscription(
            PointStamped,
            '/fire_triggers',
            self._on_trigger,
            10
        )

    
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("spawn_entity service not available, shutting down.")
            rclpy.shutdown()
            return

        self.sdf = """
<sdf version='1.6'>
  <model name='fire_pillar_{i}'>
    <static>true</static>
    <link name='link'>
      <collision name='col'>
        <geometry><cylinder>
          <radius>{r_p:.3f}</radius>
          <length>{length:.3f}</length>
        </cylinder></geometry>
      </collision>
      <visual name='vis'>
        <geometry><cylinder>
          <radius>{r_p:.3f}</radius>
          <length>{length:.3f}</length>
        </cylinder></geometry>
        <material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

        self.next_idx = 0
        self.active_plumes = []
        self.timer = self.create_timer(
            self.params['spawn_interval'],
            self._on_timer)

        self.get_logger().info("ImprovedFireSpawner ready.")

    def _pub_heartbeat(self):
      hdr = Header()
      hdr.stamp = self.get_clock().now().to_msg()
      hdr.frame_id = self.get_name()    # e.g. "navigator" or "bridge_node"
      self.hb_pub.publish(hdr)
    def _spawn_entity(self, x, y, frame):
        idx = self.next_idx; self.next_idx += 1
        req = SpawnEntity.Request()
        req.name            = f'pillar_{idx}'
        req.xml             = self.sdf.format(i=idx,
                                               r_p=self.params['pillar_radius'],
                                               length=2.5)
        req.robot_namespace = ''
        req.reference_frame = frame
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 1.25
        req.initial_pose.orientation.w = 1.0

        fut = self.cli.call_async(req)
        fut.add_done_callback(
            lambda f, i=idx, xx=x, yy=y: self._on_spawn_response(f, i, xx, yy))

    def _on_spawn_response(self, fut, idx, x, y):
        try:
            res = fut.result()
            if res.success:
                self.get_logger().info(f"Spawned pillar_{idx} at ({x:.2f},{y:.2f})")
            else:
                self.get_logger().error(
                    f"pillar_{idx} failed: {res.status_message}")
        except Exception as e:
            self.get_logger().error(f"pillar_{idx} exception: {e}")

    def _on_trigger(self, msg: PointStamped):
        self.get_logger().info(
            f"Trigger at {msg.header.frame_id} ({msg.point.x:.2f},{msg.point.y:.2f})")

        # immediate spawn of the detection point
        self._spawn_entity(msg.point.x, msg.point.y, msg.header.frame_id)

        # start a new plume
        plume = PillarPlume(
            msg.point.x, msg.point.y,
            msg.header.frame_id, self.params,
            self._spawn_entity)
        self.active_plumes.append(plume)

    def _on_timer(self):
        # advance each plume; remove finished ones
        for plume in self.active_plumes[:]:
            plume.tick()
            if plume.done:
                self.active_plumes.remove(plume)

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedFireSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
