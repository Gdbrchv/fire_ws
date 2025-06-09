#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1) TurtleBot3 in Gazebo
    tb3_gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'turtlebot3_gazebo',
            'turtlebot3_world.launch.py',
            f"world:={os.path.expanduser('~/fire_ws/fire_rescue.world')}"
        ],
        output='screen'
    )

    # 2) Nav2 with SLAM
    nav2 = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'turtlebot3_navigation2',
            'navigation2.launch.py',
            'slam:=True',
            'use_sim_time:=True',
            "params_file:=/home/abdul/fire_ws/src/starter.yaml"
        ],
        output='screen'
    )

    # 3) Your navigator
    navigator = Node(
        package='fire_spread',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4) Pillar spawner
    spawner = Node(
        package='fire_spread',
        executable='spawn_pillar_fire',
        name='spawn_pillar_fire',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 5) Detector
    detector = Node(
        package='fire_spread',
        executable='detector',
        name='lidar_bottle_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 6) Victim rescue
    victim_rescue = Node(
        package='fire_spread',
        executable='victim_rescue',
        name='victim_rescue',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Launch spawner, detector, and victim_rescue 5 seconds after startup
    delayed_start = TimerAction(
        period=60.0,
        actions=[spawner, detector, victim_rescue],
    )

    return LaunchDescription([
        tb3_gazebo,
        nav2,
        navigator,
        delayed_start,
    ])
