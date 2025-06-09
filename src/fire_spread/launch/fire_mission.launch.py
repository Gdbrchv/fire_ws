#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1) TurtleBot3 in Gazebo
    tb3 = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'turtlebot3_gazebo',
            'turtlebot3_world.launch.py',
            f"world:={os.path.expanduser('~/fire_ws/fire_rescue.world')}"
        ], output='screen'
    )

    # 2) Nav2 with SLAM
    nav2 = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'turtlebot3_navigation2',
            'navigation2.launch.py', 'slam:=True', 'use_sim_time:=True',
            "params_file:=/home/abdul/fire_ws/src/starter.yaml"
        ], output='screen'
    )

    # 3) Navigator immediately
    navigator = Node(
        package='fire_spread', executable='navigator',
        name='navigator', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4–6) Delay these three by 60 s
    spawner = Node(
        package='fire_spread', executable='spawn_pillar_fire',
        name='spawn_pillar_fire', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    detector = Node(
        package='fire_spread', executable='detector',
        name='lidar_bottle_detector', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    rescue = Node(
        package='fire_spread', executable='victim_rescue',
        name='victim_rescue', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    delayed = TimerAction(
        period=60.0,
        actions=[spawner, detector, rescue],
    )

    return LaunchDescription([tb3, nav2, navigator, delayed])
