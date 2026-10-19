#!/usr/bin/env python3
"""
maze2_solver_launch.py — Maze Solver on a CUSTOM generated maze world.

Single command:  ros2 launch red_tracker maze2_solver_launch.py

Before running, generate the maze:
    cd <your workspace>   # the folder containing src/ and install/
    python3 maze_generator.py easy        # creates maze_easy.world

To switch difficulty: regenerate with medium/hard and change WORLD_FILE +
RED_BOX_X/Y below to the position the generator printed.
"""

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ── EDIT THESE when changing maze difficulty ──────────────────────────────────
WORLD_FILE = "maze_easy.world"     # generated file in workspace root (not in src/red_tracker/launch/)
RED_BOX_X  = "4.60"                # far-corner position the generator printed
RED_BOX_Y  = "4.60"
# ──────────────────────────────────────────────────────────────────────────────


def generate_launch_description():

    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    tb3_cart_dir   = get_package_share_directory("turtlebot3_cartographer")
    nav2_dir       = get_package_share_directory("nav2_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    red_tracker_dir = get_package_share_directory("red_tracker")

    cartographer_config_dir = os.path.join(tb3_cart_dir, "config")
    # Workspace root = the folder that holds install/  (<ws>/install/red_tracker -> <ws>)
    ws_root = os.path.dirname(os.path.dirname(get_package_prefix("red_tracker")))
    nav2_params = os.path.join(ws_root, "maze_nav2_params.yaml")

    world_path = os.path.join(ws_root, WORLD_FILE)
    robot_sdf  = os.path.join(tb3_gazebo_dir, "models",
                              "turtlebot3_waffle_pi", "model.sdf")
    red_box_sdf = os.path.join(red_tracker_dir, "models", "red_box", "model.sdf")

    # ── 1. Gazebo server with the maze world ──────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "true",
        }.items(),
    )

    # ── 2. Gazebo client (GUI) ────────────────────────────────────────────────
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        ),
    )

    # ── 3. Robot state publisher (needed for TF) ──────────────────────────────
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ── 4. Spawn the robot at the maze start (0,0) ────────────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "waffle_pi",
                    "-file", robot_sdf,
                    "-x", "0.0", "-y", "0.0", "-z", "0.01",
                ],
                output="screen",
            )
        ],
    )

    # ── 5. Red box in the far corner ──────────────────────────────────────────
    spawn_red_box = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity", "red_box",
                    "-file", red_box_sdf,
                    "-x", RED_BOX_X, "-y", RED_BOX_Y, "-z", "0.0",
                ],
                output="screen",
            )
        ],
    )

    # ── 6. Cartographer SLAM ──────────────────────────────────────────────────
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

    # ── 7. Nav2 (delayed 7s — robot must be spawned first) ────────────────────
    nav2 = TimerAction(
        period=7.0,
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

    # ── 8. RViz (delayed 5s) ──────────────────────────────────────────────────
    rviz_config = os.path.join(tb3_cart_dir, "rviz", "tb3_cartographer.rviz")
    rviz = TimerAction(
        period=5.0,
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

    # ── 9. Maze Solver (delayed 17s — bigger maze, more startup time) ─────────
    maze_solver = TimerAction(
        period=17.0,
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
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        spawn_red_box,
        cartographer_node,
        occupancy_grid_node,
        nav2,
        rviz,
        maze_solver,
    ])
