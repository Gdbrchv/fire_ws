#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare('fire_spread').find('fire_spread')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': '/home/abdul/fire_ws/fire_rescue.world'
        }.items()
    )
    reset_world = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/reset_world',
                'std_srvs/srv/Empty', '{}'
            ],
            output='screen'
        )
    reset_timer = TimerAction(
        period=6.0,     
        actions=[reset_world]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': os.path.join(pkg_share, 'src', 'starter.yaml')
        }.items()
    )

    navigator = Node(
        package='fire_spread', executable='navigator',
        name='navigator', output='screen',
        parameters=[{'use_sim_time': True}],
    )

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

    
    return LaunchDescription([
        gazebo_launch,
        reset_timer,
        nav2_launch,
        navigator,
        delayed,
    ])
