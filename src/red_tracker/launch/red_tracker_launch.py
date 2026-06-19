#!/usr/bin/env python3
"""
red_tracker_launch.py  (v2)
─────────────────────────────────────────────────────────────────────────────
Fixes vs v1:
  • Removed duplicate RViz node — cartographer.launch.py already starts one.
  • Increased node startup delay to 10 s (was 8 s) to give Gazebo more time.
  • Added GAZEBO_MODEL_PATH env var so red_box SDF is always found.
  • Red box delay increased to 6 s (Gazebo world must be fully loaded first).

Usage:
  ros2 launch red_tracker red_tracker_launch.py

Optional args:
  red_box_x:=<float>   (default 1.5)
  red_box_y:=<float>   (default 0.5)
─────────────────────────────────────────────────────────────────────────────
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               ExecuteProcess,
                                               IncludeLaunchDescription,
                                               SetEnvironmentVariable,
                                               TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node


def generate_launch_description():

    tb3_gazebo_dir  = get_package_share_directory("turtlebot3_gazebo")
    tb3_cart_dir    = get_package_share_directory("turtlebot3_cartographer")
    red_tracker_dir = get_package_share_directory("red_tracker")

    # ── args ──────────────────────────────────────────────────────────────────
    arg_box_x = DeclareLaunchArgument("red_box_x", default_value="1.5")
    arg_box_y = DeclareLaunchArgument("red_box_y", default_value="0.5")

    red_box_x = LaunchConfiguration("red_box_x")
    red_box_y = LaunchConfiguration("red_box_y")

    # ── make sure Gazebo can find the red_box model ───────────────────────────
    model_path = os.path.join(red_tracker_dir, "models")
    set_model_path = SetEnvironmentVariable(
        name  = "GAZEBO_MODEL_PATH",
        value = model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
    )

    # ── 1. Gazebo world ───────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ── 2. Cartographer SLAM + RViz (tb3 package handles RViz internally) ─────
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_cart_dir, "launch", "cartographer.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ── 3. Red box spawn (t=6 s — world must be fully loaded) ─────────────────
    red_box_model = os.path.join(red_tracker_dir, "models", "red_box", "model.sdf")

    spawn_red_box = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "gazebo_ros", "spawn_entity.py",
                    "-entity",  "red_box",
                    "-file",    red_box_model,
                    "-x",       red_box_x,
                    "-y",       red_box_y,
                    "-z",       "0.0",
                ],
                output="screen",
            )
        ],
    )

    # ── 4. Decision Node (t=10 s — sensors must be fully publishing) ──────────
    decision_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package    = "red_tracker",
                executable = "decision_node",
                name       = "decision_node",
                output     = "screen",
                parameters = [{"use_sim_time": True}],
            )
        ],
    )

    # ── 5. Safety Filter (t=10 s — same time as decision node) ───────────────
    safety_filter = TimerAction(
        period=10.0,
        actions=[
            Node(
                package    = "red_tracker",
                executable = "safety_filter",
                name       = "safety_filter",
                output     = "screen",
                parameters = [{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription([
        set_model_path,
        arg_box_x,
        arg_box_y,
        gazebo,
        cartographer,
        spawn_red_box,
        decision_node,
        safety_filter,
    ])
