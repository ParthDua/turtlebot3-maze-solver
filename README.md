# TurtleBot3 Autonomous Maze Solver

An autonomous robot that explores an unknown maze, builds a map in real time, finds a red box, and navigates to stop just in front of it — all from a single launch command.

Built with ROS 2 Humble, Nav2, Cartographer SLAM, and OpenCV, simulated in Gazebo Classic.

## What it does

Starting with **no prior map**, the robot:

1. **Explores** the maze using frontier-based exploration — it finds the boundaries between explored and unexplored space and navigates to them via Nav2.
2. **Maps** the environment in real time using Cartographer SLAM.
3. **Watches** for a red box using an OpenCV HSV color detector on the camera feed.
4. **Approaches** the box when detected: it computes the box's position by fusing camera bearing with LiDAR range (cross-checked against a pixel-height distance estimate), navigates to it with Nav2 — routing around walls when the direct path is blocked — and performs a final visual-servo approach to stop just short of the box.

The whole mission runs autonomously and typically completes in 1–3 minutes depending on maze size.

## Architecture

The system is a state machine (`maze_solver.py`) with four states:

| State | Behavior |
|-------|----------|
| `EXPLORE` | Frontier-based exploration via Nav2; camera continuously checked for the red box |
| `APPROACH_COARSE` | Nav2 drives toward the box; routes *around* walls when the straight path is blocked |
| `APPROACH_FINE` | Direct velocity control for the final clear approach, with LiDAR-based stopping |
| `STOPPED` | Mission complete — robot holds position |

Key design decisions:

- **Nav2 handles all navigation**, including the approach to the box, so obstacle avoidance applies throughout. A short visual-servo phase handles only the final precise docking.
- **Clear-line-of-sight gating**: the robot only switches to the direct (fine) approach when LiDAR confirms nothing is between it and the box, preventing it from driving into walls.
- **Robust completion**: exploration is only declared finished when zero frontier cells remain anywhere in the map — not by timers or heuristics that could fire during momentary pauses.
- **Stuck recovery**: if the robot stops making progress, it abandons the current goal and performs a recovery maneuver, so it can never deadlock permanently.
- **Gradient costmap tuning** (in `maze_nav2_params.yaml`) lets the robot prefer corridor centers while still fitting through tight gaps, without per-map manual tuning.

## Tech stack

- **ROS 2 Humble**
- **Nav2** — path planning, costmaps, obstacle avoidance
- **Cartographer** — 2D SLAM
- **OpenCV** — red box detection (HSV color segmentation)
- **Gazebo Classic 11** — simulation
- **TurtleBot3 Waffle Pi** — robot platform (differential drive, 360° LiDAR, RGB camera)

## Package layout

```
src/red_tracker/
├── red_tracker/
│   ├── maze_solver.py          # main mission node (explore + detect + approach)
│   ├── frontier_explorer.py    # standalone frontier exploration (no box)
│   └── ...                      # earlier reactive-control nodes (kept for reference)
├── launch/
│   ├── maze_solver_launch.py    # full mission on the default TurtleBot3 world
│   ├── maze2_solver_launch.py   # full mission on a custom generated maze
│   └── exploration_launch.py    # exploration only
├── models/
│   └── red_box/                 # the red box target model
├── setup.py
└── package.xml

maze_generator.py                # procedurally generates solvable grid mazes
maze_nav2_params.yaml            # tuned Nav2 parameters for maze navigation
```

## Running it

Requires ROS 2 Humble with TurtleBot3 packages, Nav2, Cartographer, and Gazebo Classic installed, plus `export TURTLEBOT3_MODEL=waffle_pi`.

**On the default TurtleBot3 world:**

```bash
cd ~/ros2_ws
colcon build --packages-select red_tracker
source install/setup.bash
ros2 launch red_tracker maze_solver_launch.py
```

**On a custom generated maze:**

```bash
cd ~/ros2_ws
python3 maze_generator.py easy        # or medium / hard
colcon build --packages-select red_tracker
source install/setup.bash
ros2 launch red_tracker maze2_solver_launch.py
```

The maze generator prints the suggested red-box position for each maze; update the constants at the top of `maze2_solver_launch.py` when changing difficulty.

## Status

Active development. Core mission (explore → map → detect → approach → stop) works end to end on both the default world and custom generated mazes. Ongoing work focuses on robustness of the final approach and dynamic-obstacle handling.

## Author

Parth — robotics and AI.
