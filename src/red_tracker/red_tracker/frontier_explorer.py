#!/usr/bin/env python3
"""
frontier_explorer.py — Phase 1: autonomous frontier-based exploration
─────────────────────────────────────────────────────────────────────────────
Explores an unknown map by repeatedly navigating to the nearest frontier
(the boundary between known-free and unknown space), using Nav2 for all
path planning and obstacle avoidance.

No camera, no red box yet — pure exploration + mapping validation.

Flow:
  1. Wait for /map and TF (map->base_link)
  2. Detect frontier cells, cluster them, filter noise
  3. Pick nearest viable cluster centroid
  4. Send as NavigateToPose goal (map frame)
  5. On arrival/failure → recompute
  6. No frontiers left → mapping complete, stop
─────────────────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

import numpy as np
from collections import deque
import math

# ── tunable parameters ────────────────────────────────────────────────────────

MIN_FRONTIER_SIZE = 8
PLAN_PERIOD       = 2.0
GOAL_TIMEOUT      = 60.0
ROBOT_FRAME       = "base_link"
MAP_FRAME         = "map"

# Guards against premature "exploration done":
MIN_KNOWN_CELLS   = 200   # need at least this many known cells before trusting "no frontiers"
EMPTY_CONFIRMATIONS = 3   # must see "no frontiers" this many times in a row to stop

NEIGHBORS_8 = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

# ─────────────────────────────────────────────────────────────────────────────


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__("frontier_explorer")

        self.map_msg = None

        self.nav_active   = False
        self.goal_handle  = None
        self.goal_start_t = None
        self.pending_goal = None

        self.failed_frontiers = []
        self.explore_done     = False
        self.empty_count      = 0       # consecutive "no frontier" observations
        self.goals_sent       = 0

        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(PLAN_PERIOD, self.plan_loop)

        self.get_logger().info("🗺️  Frontier Explorer started — waiting for map + Nav2...")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def map_cb(self, msg):
        self.map_msg = msg

    # ── robot position via TF ─────────────────────────────────────────────────

    def get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    # ── coordinate conversion ─────────────────────────────────────────────────

    def cell_to_world(self, col, row, info):
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    # ── frontier detection ────────────────────────────────────────────────────
        
    def find_frontier_cells(self, grid, w, h):
        """
        Frontier = a free/traversable cell adjacent to an UNKNOWN (-1) cell.
        Cartographer uses 0-100 probabilities: low = free, high = wall, -1 = unknown.
        Treat anything below FREE_THRESHOLD as traversable floor.
        """
        FREE_THRESHOLD = 50   # cells with value < 50 are considered free/traversable

        unknown   = (grid == -1)
        free      = (grid >= 0) & (grid < FREE_THRESHOLD)

        # cells adjacent to at least one unknown cell
        unknown_adj = np.zeros((h, w), dtype=bool)
        unknown_adj[1:, :]  |= unknown[:-1, :]
        unknown_adj[:-1, :] |= unknown[1:, :]
        unknown_adj[:, 1:]  |= unknown[:, :-1]
        unknown_adj[:, :-1] |= unknown[:, 1:]

        return free & unknown_adj

    def cluster_frontiers(self, frontier_mask, w, h):
        visited = np.zeros((h, w), dtype=bool)
        clusters = []
        rows, cols = np.where(frontier_mask)
        for r0, c0 in zip(rows, cols):
            if visited[r0, c0]:
                continue
            cluster = []
            q = deque([(r0, c0)])
            visited[r0, c0] = True
            while q:
                r, c = q.popleft()
                cluster.append((r, c))
                for dr, dc in NEIGHBORS_8:
                    nr, nc = r + dr, c + dc
                    if (0 <= nr < h and 0 <= nc < w
                            and frontier_mask[nr, nc] and not visited[nr, nc]):
                        visited[nr, nc] = True
                        q.append((nr, nc))
            clusters.append(cluster)
        return clusters

    def cluster_centroid(self, cluster, info):
        rows = [c[0] for c in cluster]
        cols = [c[1] for c in cluster]
        return self.cell_to_world(sum(cols)/len(cols), sum(rows)/len(rows), info)

    # ── main planning loop ────────────────────────────────────────────────────

    def plan_loop(self):
        if self.explore_done:
            return
        if self.nav_active:
            self.check_goal_timeout()
            return
        if self.map_msg is None:
            self.get_logger().info("⏳ Waiting for /map ...")
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().info("⏳ Waiting for TF (map→base_link) ...")
            return

        info = self.map_msg.info
        w, h = info.width, info.height
        grid = np.array(self.map_msg.data, dtype=np.int8).reshape((h, w))

        # ── guard: is the map substantial yet? ────────────────────────────────
        known_cells = int(np.count_nonzero(grid != -1))
        if known_cells < MIN_KNOWN_CELLS:
            self.get_logger().info(
                f"⏳ Map still sparse ({known_cells} known cells) — waiting...")
            return

        frontier_mask = self.find_frontier_cells(grid, w, h)
        clusters = self.cluster_frontiers(frontier_mask, w, h)
        clusters = [c for c in clusters if len(c) >= MIN_FRONTIER_SIZE]

        # ── completion check with confirmation ────────────────────────────────
        if not clusters:
            self.empty_count += 1
            self.get_logger().info(
                f"🔍 No frontiers this scan ({self.empty_count}/{EMPTY_CONFIRMATIONS}) "
                f"| known cells: {known_cells}")
            if self.empty_count >= EMPTY_CONFIRMATIONS:
                self.get_logger().info("✅ MAP COMPLETE — exploration finished.")
                self.explore_done = True
            return
        else:
            self.empty_count = 0   # reset; we found frontiers

        # ── pick nearest viable cluster ───────────────────────────────────────
        rx, ry = robot_xy
        best = None
        best_dist = float("inf")
        for cluster in clusters:
            cx, cy = self.cluster_centroid(cluster, info)
            if self.is_failed(cx, cy):
                continue
            dist = math.hypot(cx - rx, cy - ry)
            if dist < best_dist:
                best_dist = dist
                best = (cx, cy, len(cluster))

        if best is None:
            self.get_logger().warn("⚠️ All frontiers in failed list — clearing and retrying")
            self.failed_frontiers.clear()
            return

        cx, cy, size = best
        self.get_logger().info(
            f"🎯 {len(clusters)} frontiers | nearest ({cx:.2f},{cy:.2f}) "
            f"dist={best_dist:.2f}m size={size}")
        self.send_goal(cx, cy)

    # ── failed-frontier tracking ──────────────────────────────────────────────

    def is_failed(self, x, y, tol=0.5):
        for fx, fy in self.failed_frontiers:
            if math.hypot(x - fx, y - fy) < tol:
                return True
        return False

    # ── Nav2 goal handling ────────────────────────────────────────────────────

    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ Nav2 action server not available!")
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self.nav_active   = True
        self.goal_start_t = self.get_clock().now()
        self.pending_goal = (x, y)
        self.goals_sent  += 1

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("⚠️ Goal rejected by Nav2")
            self.nav_active = False
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("✅ Reached frontier")
        else:
            self.get_logger().warn(f"⚠️ Goal ended status={status} — marking failed")
            if self.pending_goal is not None:
                self.failed_frontiers.append(self.pending_goal)
        self.nav_active  = False
        self.goal_handle = None

    def check_goal_timeout(self):
        if self.goal_start_t is None:
            return
        elapsed = (self.get_clock().now() - self.goal_start_t).nanoseconds / 1e9
        if elapsed > GOAL_TIMEOUT:
            self.get_logger().warn(f"⏰ Goal timeout ({elapsed:.0f}s) — cancelling")
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            if self.pending_goal is not None:
                self.failed_frontiers.append(self.pending_goal)
            self.nav_active  = False
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
