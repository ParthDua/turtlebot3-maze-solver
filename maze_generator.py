#!/usr/bin/env python3
"""
maze_generator.py — generates a Gazebo Classic .world file with a grid maze.

Usage:
    python3 maze_generator.py easy
    python3 maze_generator.py medium
    python3 maze_generator.py hard
    python3 maze_generator.py medium --seed 42        # reproducible
    python3 maze_generator.py medium --out my_maze.world
"""

import random
import sys
import argparse

CORRIDOR  = 1.0    # m — corridor width (fits TB3 + Nav2 inflation)
WALL_TH   = 0.15   # m — wall thickness
WALL_H    = 0.5    # m — wall height (LiDAR sees it)

DIFFICULTY = {"easy": 5, "medium": 8, "hard": 12}


def carve_maze(n, seed=None):
    """Recursive backtracker. Returns set of OPEN walls between cells."""
    if seed is not None:
        random.seed(seed)
    visited = set()
    open_walls = set()
    stack = [(0, 0)]
    visited.add((0, 0))
    while stack:
        r, c = stack[-1]
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < n and (nr, nc) not in visited:
                neighbors.append((nr, nc))
        if neighbors:
            nxt = random.choice(neighbors)
            open_walls.add(frozenset({(r, c), nxt}))
            visited.add(nxt)
            stack.append(nxt)
        else:
            stack.pop()
    return open_walls


def cell_center(r, c):
    step = CORRIDOR + WALL_TH
    return c * step, r * step


def build_wall_segments(n, open_walls):
    """Convert maze into wall boxes: (x_center, y_center, size_x, size_y)."""
    walls = []
    step = CORRIDOR + WALL_TH
    half = CORRIDOR / 2.0
    x_min, x_max = -half, (n - 1) * step + half
    y_min, y_max = -half, (n - 1) * step + half
    cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
    width, height = x_max - x_min, y_max - y_min

    # perimeter
    walls.append((cx, y_min - WALL_TH/2, width + 2*WALL_TH, WALL_TH))
    walls.append((cx, y_max + WALL_TH/2, width + 2*WALL_TH, WALL_TH))
    walls.append((x_min - WALL_TH/2, cy, WALL_TH, height + 2*WALL_TH))
    walls.append((x_max + WALL_TH/2, cy, WALL_TH, height + 2*WALL_TH))

    # interior walls where there's no open passage
    for r in range(n):
        for c in range(n):
            if c + 1 < n and frozenset({(r, c), (r, c+1)}) not in open_walls:
                x0, y0 = cell_center(r, c)
                walls.append((x0 + step/2, y0, WALL_TH, CORRIDOR + WALL_TH))
            if r + 1 < n and frozenset({(r, c), (r+1, c)}) not in open_walls:
                x0, y0 = cell_center(r, c)
                walls.append((x0, y0 + step/2, CORRIDOR + WALL_TH, WALL_TH))
    return walls


def write_world(walls, n, path):
    wall_models = []
    for i, (x, y, sx, sy) in enumerate(walls):
        wall_models.append(f"""
    <model name="wall_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

    gx, gy = cell_center(n - 1, n - 1)

    world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
{''.join(wall_models)}
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
"""
    with open(path, "w") as f:
        f.write(world)
    return gx, gy


def main():
    parser = argparse.ArgumentParser(description="Generate a Gazebo grid maze world")
    parser.add_argument("difficulty", choices=["easy", "medium", "hard"])
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--out", type=str, default=None)
    args = parser.parse_args()

    n = DIFFICULTY[args.difficulty]
    out = args.out or f"maze_{args.difficulty}.world"
    open_walls = carve_maze(n, args.seed)
    walls = build_wall_segments(n, open_walls)
    gx, gy = write_world(walls, n, out)

    print(f"✅ Generated {args.difficulty} maze ({n}x{n})")
    print(f"   File: {out}")
    print(f"   Walls: {len(walls)} segments")
    print(f"   Robot start: spawn at (0, 0)")
    print(f"   Suggested red box position: ({gx:.2f}, {gy:.2f})  [far corner]")


if __name__ == "__main__":
    main()
