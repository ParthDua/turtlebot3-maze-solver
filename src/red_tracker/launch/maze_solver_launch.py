#!/usr/bin/env python3
"""
maze_solver_launch.py — Phase 2: Red object tracker + Frontier Exploration = Maze Solver
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    tb3_cart_dir   = get_package_share_directory("turtlebot3_cartographer")
    nav2_dir       = get_package_share_directory("nav2_bringup")
    red_tracker_dir = get_package_share_directory("red_tracker")

    cartographer_config_dir = os.path.join(tb3_cart_dir, "config")

    # Generic Nav2 params — matches installed Nav2 version (plugin names correct)
    nav2_params = os.path.join(nav2_dir, "params", "nav2_params.yaml")

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    
    spawn_red_box = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", "red_box",
                    "-file", os.path.join(red_tracker_dir, "models", "red_box", "model.sdf"),
                    "-x", "2.0", "-y", "0.5", "-z", "0.0",
                ],
                output="screen",
            )
        ],
    )

    # 2. Cartographer SLAM nodes (direct)
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", "turtlebot3_lds_2d.lua",
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    # 3. Nav2 — generic params, delayed 5s
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "params_file": nav2_params,
                }.items(),
            )
        ],
    )

    # 4. RViz, delayed 4s
    rviz_config = os.path.join(tb3_cart_dir, "rviz", "tb3_cartographer.rviz")
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # 5. Maze Solver, delayed 15s (give Nav2 more time to fully activate)
    explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="red_tracker",
                executable="maze_solver",
                name="maze_solver",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        spawn_red_box,
        cartographer_node,
        occupancy_grid_node,
        nav2,
        rviz,
        explorer,
    ])
