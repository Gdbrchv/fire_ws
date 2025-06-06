#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import math
import random

class SlowFireSpawner(Node):
    def __init__(self):
        super().__init__('slow_fire_spawner')

        # ───────────────────────────────
        # 1) PARAMETERS YOU CAN TUNE
        # ───────────────────────────────
        self.declare_parameter('pillar_radius',      0.05)    # [m]
        self.declare_parameter('num_pillars',        50)      # total to spawn
        self.declare_parameter('max_ring_radius',    1.0)     # [m]
        self.declare_parameter('min_height',         0.2)     # [m]
        self.declare_parameter('max_height',         1.0)     # [m]
        self.declare_parameter('center_x',           0.0)     # [m]
        self.declare_parameter('center_y',           0.0)     # [m]
        self.declare_parameter('shift_first_to',    [0.5, 0.0])# optional
        self.declare_parameter('spawn_interval',     0.5)     # [s] time between pillars

        # Retrieve them
        self.r_p         = self.get_parameter('pillar_radius').get_parameter_value().double_value
        self.N           = self.get_parameter('num_pillars').get_parameter_value().integer_value
        self.R_max       = self.get_parameter('max_ring_radius').get_parameter_value().double_value
        self.h_min       = self.get_parameter('min_height').get_parameter_value().double_value
        self.h_max       = self.get_parameter('max_height').get_parameter_value().double_value
        self.cx          = self.get_parameter('center_x').get_parameter_value().double_value
        self.cy          = self.get_parameter('center_y').get_parameter_value().double_value
        self.shift       = self.get_parameter('shift_first_to').get_parameter_value().double_array_value
        self.spawn_interval = self.get_parameter('spawn_interval').get_parameter_value().double_value

        # Precompute all pillar data (x, y, z, height, color) in a list
        self.pillars = []
        for i in range(self.N):
            theta = random.uniform(0.0, 2.0 * math.pi)
            rho = random.random()
            r_i = self.R_max * math.sqrt(rho)
            x_i = self.cx + r_i * math.cos(theta)
            y_i = self.cy + r_i * math.sin(theta)
            t_frac = r_i / self.R_max
            h_i = self.h_max - t_frac * (self.h_max - self.h_min)

            alpha = random.random()
            r_col = 0.8 + 0.2 * alpha
            g_col = 0.1 + 0.8 * alpha
            b_col = 0.1

            self.pillars.append({
                'x': x_i, 'y': y_i, 'h': h_i,
                'r': r_col, 'g': g_col, 'b': b_col
            })

        # Compute shift (dx, dy) based on pillar 0 if requested
        self.dx = 0.0
        self.dy = 0.0
        if self.N > 0 and len(self.shift) == 2:
            x0_target, y0_target = self.shift
            actual_x0 = self.pillars[0]['x']
            actual_y0 = self.pillars[0]['y']
            self.dx = x0_target - actual_x0
            self.dy = y0_target - actual_y0
            self.get_logger().info(
                f"Pillar 0 was at ({actual_x0:.3f},{actual_y0:.3f}); "
                f"shifting all by (dx,dy)=({self.dx:.3f},{self.dy:.3f})"
            )

        # Prepare the /spawn_entity client
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Connected to /spawn_entity')

        # Index of next pillar to spawn
        self.next_index = 0

        # Build a timer that fires every spawn_interval seconds
        timer_period = float(self.spawn_interval)
        self.spawn_timer = self.create_timer(timer_period, self.spawn_one_pillar)

        # Pre‐build the static part of the SDF template
        self.sdf_template = """
        <sdf version='1.6'>
          <model name='fire_pillar_{idx}'>
            <static>true</static>
            <link name='link'>
              <collision name='collision'>
                <geometry>
                  <cylinder>
                    <radius>{radius:.3f}</radius>
                    <length>{height:.3f}</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <cylinder>
                    <radius>{radius:.3f}</radius>
                    <length>{height:.3f}</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
                  <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        self.get_logger().info(
            f"Will spawn {self.N} pillars, one every {self.spawn_interval:.2f}s."
        )

    def spawn_one_pillar(self):
        # If we've already spawned all N, stop the timer and return
        if self.next_index >= self.N:
            self.get_logger().info("All pillars spawned; cancelling timer.")
            self.spawn_timer.cancel()
            return

        i = self.next_index
        data = self.pillars[i]

        x_spawn = data['x'] + self.dx
        y_spawn = data['y'] + self.dy
        z_spawn = data['h'] / 2.0  # sit on ground

        # Fill in this pillar’s SDF
        xml = self.sdf_template.format(
            idx=i,
            radius=self.r_p,
            height=data['h'],
            r=data['r'], g=data['g'], b=data['b']
        )

        req = SpawnEntity.Request()
        req.name = f'fire_pillar_{i}'
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose.position.x = float(x_spawn)
        req.initial_pose.position.y = float(y_spawn)
        req.initial_pose.position.z = float(z_spawn)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f"Spawned pillar {i:02d} at ({x_spawn:.3f}, {y_spawn:.3f}), "
                f"height={data['h']:.3f}, color=({data['r']:.2f},{data['g']:.2f},{data['b']:.2f})"
            )
        else:
            self.get_logger().error(f"Failed to spawn pillar {i:02d}.")

        self.next_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = SlowFireSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
