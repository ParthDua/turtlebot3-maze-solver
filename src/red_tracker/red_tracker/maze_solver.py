#!/usr/bin/env python3
"""
maze_solver.py — Frontier exploration + red box detection + precise approach.
Keeps frontier_explorer.py untouched; this is the combined mission node.

States:
  EXPLORE         frontier-based exploration via Nav2; camera watched for red
  APPROACH_COARSE Nav2 drives toward the box; paths AROUND walls if one blocks
  APPROACH_FINE   direct /cmd_vel visual servo for the last few CLEAR metres
  STOPPED         success; hold position

Robustness features:
  - FINE is entered ONLY when the LiDAR confirms a clear straight path to the
    box (no wall in the way). If a wall blocks the direct line, the robot stays
    in COARSE and lets Nav2 path around the obstacle.
  - When blocked, the Nav2 goal is placed at the box's ESTIMATED TRUE position
    (cross-checked with pixel-height ranging, which is not fooled by an
    intervening wall) so the planner routes around the corner toward the box.
  - A stuck-detector guarantees the robot can never loop forever: if it stops
    moving while pursuing a goal, the goal is abandoned + a recovery nudge runs.
  - Exploration only declares "done" when zero frontier cells remain anywhere.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import math

# ── perception ────────────────────────────────────────────────────────────────
RED_LOWER1 = np.array([0,   120,  70])
RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  70])
RED_UPPER2 = np.array([180, 255, 255])

DETECT_MIN_AREA   = 500      # px² to count as a real detection
CONFIRM_FRAMES    = 5        # consecutive detections before diverting to box
LOST_FRAMES       = 25       # consecutive misses during approach → give up, re-explore

# ── camera geometry ───────────────────────────────────────────────────────────
IMG_WIDTH   = 640
IMG_HEIGHT  = 480
CAMERA_FOV  = 1.085595       # rad (horizontal)
CAMERA_VFOV = 2 * math.atan(math.tan(CAMERA_FOV / 2) * IMG_HEIGHT / IMG_WIDTH)  # ~0.85 rad
BOX_HEIGHT_M = 0.5           # real red box height (m) — used for pixel-size ranging

# ── LiDAR ─────────────────────────────────────────────────────────────────────
LIDAR_N     = 360            # rays; index 0 = front
RANGE_WINDOW = 3             # average over ±this many rays around target bearing

# ── approach geometry ─────────────────────────────────────────────────────────
BACKOFF       = 0.50         # m — coarse goal placed this far in front of box
FINE_TRIGGER_AREA = 25000    # px² — box this big in frame → FINE (backup trigger)
FINE_TRIGGER_RANGE = BACKOFF + 0.30   # m — box within this range → consider FINE
CLEAR_PATH_TOL = 0.10        # m — LiDAR-ahead may be at most this much shorter than
                             #     the box distance for the path to count as "clear"
FINE_STOP_DIST = 0.2         # m — front LiDAR closer than this → stop (just short)
FINE_LINEAR   = 0.15         # m/s creep speed
FINE_ANGULAR_GAIN = 0.004    # rotate-to-center gain

# ── exploration / frontier ────────────────────────────────────────────────────
MIN_FRONTIER_SIZE   = 3      # catch small frontiers at ends of narrow corridors
FREE_THRESHOLD      = 50
MIN_KNOWN_CELLS     = 200
EMPTY_CONFIRMATIONS = 8
GOAL_TIMEOUT        = 30.0
PLAN_PERIOD         = 2.0

# ── stuck detection / recovery ────────────────────────────────────────────────
STUCK_DIST_EPS    = 0.08     # m — moved less than this between checks = "not moving"
STUCK_TIME_LIMIT  = 12.0     # s — stationary this long during a goal → recovery
RECOVERY_BACK_T   = 1.2      # s — duration of the recovery back-up nudge
RECOVERY_BACK_V   = -0.08    # m/s — gentle reverse during recovery

ROBOT_FRAME = "base_link"
MAP_FRAME   = "map"
NEIGHBORS_8 = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

MAP_SAVE_PATH = "explored_map"

# ─────────────────────────────────────────────────────────────────────────────


class MazeSolver(Node):

    def __init__(self):
        super().__init__("maze_solver")

        self.bridge = CvBridge()
        self.scan   = None
        self.frame  = None

        self.state = "EXPLORE"

        # detection confirmation counters
        self.red_streak  = 0
        self.miss_streak = 0

        # exploration / nav bookkeeping
        self.map_msg      = None
        self.nav_active   = False
        self.goal_handle  = None
        self.goal_start_t = None
        self.pending_goal = None
        self.failed_frontiers = []
        self.empty_count  = 0

        # approach bookkeeping
        self.coarse_goal_sent = False
        self.last_coarse_goal = None    # remember last coarse goal to avoid re-spamming

        # stuck-detection bookkeeping
        self.last_pose_xy   = None
        self.last_move_t    = None
        self.recovering     = False
        self.recovery_end_t = None

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, "/map",              self.map_cb,   10)
        self.create_subscription(Image,         "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(LaserScan,     "/scan",             self.scan_cb,  10)

        # ── TF + Nav2 + direct cmd_vel ────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client  = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub     = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── timers ────────────────────────────────────────────────────────────
        self.create_timer(PLAN_PERIOD, self.plan_loop)    # exploration + coarse goals
        self.create_timer(0.1,         self.fast_loop)    # detection + fine approach
        self.create_timer(1.0,         self.stuck_loop)   # stuck detection + recovery

        self.get_logger().info("🤖 Maze Solver started — EXPLORE")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def map_cb(self, msg):   self.map_msg = msg
    def scan_cb(self, msg):  self.scan = msg
    def image_cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge: {e}")

    # ── perception ────────────────────────────────────────────────────────────

    def detect_red(self):
        """
        Return (area, cx_pixel, box_h_pixels, touches_edge) or (0, None, 0, False).
        box_h_pixels: bounding-box height (for pixel-size distance estimation).
        touches_edge: True if the blob touches top/bottom frame edge (clipped →
                      pixel-size estimate unreliable, treat with caution).
        """
        if self.frame is None or self.frame.shape[0] == 0:
            return 0, None, 0, False
        hsv  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, RED_LOWER1, RED_UPPER1) |
                cv2.inRange(hsv, RED_LOWER2, RED_UPPER2))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, None, 0, False
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < DETECT_MIN_AREA:
            return 0, None, 0, False
        x, y, w, h = cv2.boundingRect(largest)
        touches_edge = (y <= 1) or (y + h >= IMG_HEIGHT - 1)
        return area, x + w // 2, h, touches_edge

    def pixel_to_bearing(self, cx):
        """Pixel column → bearing in radians (0=straight ahead, +left)."""
        norm = (cx - IMG_WIDTH / 2) / (IMG_WIDTH / 2)   # [-1, 1]
        return -norm * (CAMERA_FOV / 2)                  # +ve = left

    def distance_from_pixel_height(self, box_h_pixels):
        """
        Estimate distance to the 0.5m box from its pixel height.
        A box of real height H at distance d subtends angle ~ H/d; the pixel
        height is IMG_HEIGHT * (subtended_angle / VFOV) for small angles, giving
        d = BOX_HEIGHT_M * IMG_HEIGHT / (box_h_pixels * 2*tan(VFOV/2)).
        NOT fooled by an intervening wall (camera sees the box over/past it),
        but IS unreliable if the box is clipped by the frame edge.
        """
        if box_h_pixels <= 0:
            return None
        return (BOX_HEIGHT_M * IMG_HEIGHT) / (box_h_pixels * 2 * math.tan(CAMERA_VFOV / 2))

    def range_at_bearing(self, bearing):
        """LiDAR range at given bearing (rad). index 0 = front, CCW positive."""
        if self.scan is None:
            return None
        n = len(self.scan.ranges)
        idx = int(round((bearing / (2 * math.pi)) * n)) % n
        vals = []
        for d in range(-RANGE_WINDOW, RANGE_WINDOW + 1):
            i = (idx + d) % n
            r = self.scan.ranges[i]
            if r == r and r != float('inf') and r > 0.01:
                vals.append(r)
        return min(vals) if vals else None

    # ── TF / coordinate helpers ───────────────────────────────────────────────

    def get_robot_pose(self):
        """Return (x, y, yaw) in map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return x, y, math.atan2(siny, cosy)
        except TransformException:
            return None

    def box_point_in_map(self, bearing, rng):
        """Project box (bearing, range from robot) into map-frame (x, y, rx, ry)."""
        pose = self.get_robot_pose()
        if pose is None:
            return None
        rx, ry, ryaw = pose
        world_angle = ryaw + bearing
        bx = rx + rng * math.cos(world_angle)
        by = ry + rng * math.sin(world_angle)
        return bx, by, rx, ry

    def estimate_box_distance(self, bearing, box_h_pixels, touches_edge):
        """
        Best estimate of the TRUE distance to the box, combining LiDAR (accurate
        when path is clear) and pixel-height (robust to intervening walls).
        Returns (best_distance, lidar_ahead, lidar_at_bearing).
        """
        lidar_ahead = self.range_at_bearing(0.0)
        lidar_bear  = self.range_at_bearing(bearing)
        pix_dist    = None if touches_edge else self.distance_from_pixel_height(box_h_pixels)

        # If pixel estimate exists and is FARTHER than the LiDAR-at-bearing by a
        # clear margin, the LiDAR is hitting a wall in front of the box → trust
        # the pixel estimate for the box's true distance.
        if pix_dist is not None and lidar_bear is not None and pix_dist > lidar_bear + 0.20:
            best = pix_dist
        elif lidar_bear is not None:
            best = lidar_bear
        elif pix_dist is not None:
            best = pix_dist
        else:
            best = None
        return best, lidar_ahead, lidar_bear

    # ── fast loop: detection + fine approach ──────────────────────────────────

    def fast_loop(self):
        # Recovery nudge in progress → drive it and skip everything else.
        if self.recovering:
            now = self.get_clock().now()
            if self.recovery_end_t is not None and now.nanoseconds < self.recovery_end_t:
                t = Twist()
                t.linear.x = RECOVERY_BACK_V
                self.cmd_pub.publish(t)
                return
            self.cmd_pub.publish(Twist())
            self.recovering = False
            self.recovery_end_t = None
            self.last_move_t = self.get_clock().now()
            return

        area, cx, box_h, touches_edge = self.detect_red()
        red_visible = area >= DETECT_MIN_AREA

        # ── EXPLORE: watch for confirmed red, then divert ─────────────────────
        if self.state == "EXPLORE":
            if red_visible:
                self.red_streak += 1
                if self.red_streak >= CONFIRM_FRAMES:
                    self.get_logger().info("🔴 Red box confirmed — diverting to approach")
                    self.cancel_nav()
                    self.state = "APPROACH_COARSE"
                    self.coarse_goal_sent = False
                    self.last_coarse_goal = None
                    self.miss_streak = 0
            else:
                self.red_streak = 0
            return

        # ── APPROACH_COARSE: only go FINE when the straight path is CLEAR ──────
        if self.state == "APPROACH_COARSE":
            if red_visible:
                self.miss_streak = 0
                bearing = self.pixel_to_bearing(cx)
                box_dist, lidar_ahead, lidar_bear = self.estimate_box_distance(
                    bearing, box_h, touches_edge)

                if box_dist is None or lidar_ahead is None:
                    return   # not enough info this frame; wait

                # Clear-path test: nothing ahead is closer than the box (within
                # tolerance). If LiDAR-ahead is much shorter than the box, a wall
                # blocks the straight line → stay in COARSE, let Nav2 route around.
                path_clear = lidar_ahead >= box_dist - CLEAR_PATH_TOL
                close_enough = box_dist < FINE_TRIGGER_RANGE

                if close_enough and path_clear:
                    self.get_logger().info(
                        f"🔬 Clear path — FINE (box={box_dist:.2f}, ahead={lidar_ahead:.2f})")
                    self.cancel_nav()
                    self.state = "APPROACH_FINE"
                # else: blocked or still far → plan_loop keeps driving Nav2.
                # The stuck-detector guarantees we can't loop here forever.
            else:
                self.miss_streak += 1
                if self.miss_streak >= LOST_FRAMES:
                    self.get_logger().warn("⚠️ Lost box during coarse — back to EXPLORE")
                    self.cancel_nav()
                    self.state = "EXPLORE"
                    self.red_streak = 0
            return

        # ── APPROACH_FINE: direct visual servo, LiDAR-primary stop ────────────
        if self.state == "APPROACH_FINE":
            front = self.range_at_bearing(0.0)
            if front is not None and front < FINE_STOP_DIST:
                self.get_logger().info(f"🎯 RED BOX REACHED ({front:.2f}m) — mission complete")
                self.cmd_pub.publish(Twist())
                self.state = "STOPPED"
                return

            if not red_visible:
                self.miss_streak += 1
                self.cmd_pub.publish(Twist())
                if self.miss_streak >= LOST_FRAMES:
                    self.get_logger().warn("⚠️ Lost box during fine — back to EXPLORE")
                    self.state = "EXPLORE"
                    self.red_streak = 0
                return
            self.miss_streak = 0

            # Safety: if FINE somehow finds a wall closer than the box appears,
            # bail back to COARSE so Nav2 can re-route (defends against entering
            # FINE on a transient clear reading).
            bearing = self.pixel_to_bearing(cx)
            box_dist, lidar_ahead, _ = self.estimate_box_distance(bearing, box_h, touches_edge)
            if (box_dist is not None and lidar_ahead is not None
                    and lidar_ahead < box_dist - CLEAR_PATH_TOL - 0.05
                    and front is not None and front > FINE_STOP_DIST):
                self.get_logger().warn(
                    f"🧱 Wall appeared in FINE (ahead={lidar_ahead:.2f}, box={box_dist:.2f}) "
                    f"— back to COARSE to re-route")
                self.cmd_pub.publish(Twist())
                self.state = "APPROACH_COARSE"
                self.coarse_goal_sent = False
                self.last_coarse_goal = None
                return

            # creep forward while centering the box
            err = cx - IMG_WIDTH / 2
            twist = Twist()
            twist.linear.x  = FINE_LINEAR
            twist.angular.z = -FINE_ANGULAR_GAIN * err
            self.cmd_pub.publish(twist)
            return

        # ── STOPPED: hold ─────────────────────────────────────────────────────
        if self.state == "STOPPED":
            self.cmd_pub.publish(Twist())
            return

    # ── stuck loop: detect a frozen robot and break the deadlock ──────────────

    def stuck_loop(self):
        if self.recovering:
            return
        if self.state not in ("EXPLORE", "APPROACH_COARSE"):
            return
        if not self.nav_active:
            self.last_move_t = self.get_clock().now()
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        x, y, _ = pose
        now = self.get_clock().now()

        if self.last_pose_xy is None:
            self.last_pose_xy = (x, y)
            self.last_move_t = now
            return

        moved = math.hypot(x - self.last_pose_xy[0], y - self.last_pose_xy[1])
        if moved > STUCK_DIST_EPS:
            self.last_pose_xy = (x, y)
            self.last_move_t = now
            return

        if self.last_move_t is None:
            self.last_move_t = now
            return
        stationary_for = (now.nanoseconds - self.last_move_t.nanoseconds) / 1e9
        if stationary_for >= STUCK_TIME_LIMIT:
            self.get_logger().warn(
                f"🧊 Stuck for {stationary_for:.0f}s — abandoning goal, recovering")
            if self.pending_goal is not None:
                self.failed_frontiers.append(self.pending_goal)
                if len(self.failed_frontiers) > 30:
                    self.failed_frontiers = self.failed_frontiers[-30:]
            self.cancel_nav()
            if self.state == "APPROACH_COARSE":
                self.coarse_goal_sent = False
                self.last_coarse_goal = None
            self.recovering = True
            self.recovery_end_t = now.nanoseconds + int(RECOVERY_BACK_T * 1e9)
            self.last_pose_xy = (x, y)
            self.last_move_t = now

    # ── slow loop: exploration planning + coarse approach goal ────────────────

    def plan_loop(self):
        if self.state == "STOPPED":
            return
        if self.recovering:
            return

        # ── coarse approach: drive Nav2 toward the box, routing around walls ──
        if self.state == "APPROACH_COARSE":
            if self.nav_active and self.coarse_goal_sent:
                self.check_goal_timeout()
                return
            area, cx, box_h, touches_edge = self.detect_red()
            if area < DETECT_MIN_AREA:
                return
            bearing = self.pixel_to_bearing(cx)
            box_dist, lidar_ahead, lidar_bear = self.estimate_box_distance(
                bearing, box_h, touches_edge)
            if box_dist is None:
                self.get_logger().info("⏳ No box range yet...")
                return

            path_clear = (lidar_ahead is not None and
                          lidar_ahead >= box_dist - CLEAR_PATH_TOL)

            # If close AND path clear, fast_loop will switch to FINE; just wait.
            if box_dist < FINE_TRIGGER_RANGE and path_clear:
                return

            # Compute the box's TRUE map position using the best distance estimate.
            proj = self.box_point_in_map(bearing, box_dist)
            if proj is None:
                return
            bx, by, rx, ry = proj

            if path_clear:
                # Clear but still far: aim for a point BACKOFF in front of box.
                dx, dy = bx - rx, by - ry
                dist = math.hypot(dx, dy)
                if dist < 1e-3:
                    return
                gx = bx - (dx / dist) * BACKOFF
                gy = by - (dy / dist) * BACKOFF
                tag = "clear/approach"
            else:
                # BLOCKED: a wall is between us and the box. Send Nav2 to the BOX
                # ITSELF so the planner routes AROUND the wall corner. Nav2's
                # costmap inflation keeps the robot off the box; we don't need a
                # backoff here because the planner won't drive through the wall.
                gx, gy = bx, by
                tag = "blocked/route-around"

            # Avoid re-spamming an almost-identical goal (cuts log noise + churn).
            if (self.last_coarse_goal is not None and
                    math.hypot(gx - self.last_coarse_goal[0],
                               gy - self.last_coarse_goal[1]) < 0.15 and
                    self.nav_active):
                return

            self.get_logger().info(
                f"🎯 Box~({bx:.2f},{by:.2f}) d={box_dist:.2f}m [{tag}] → goal ({gx:.2f},{gy:.2f})")
            self.send_goal(gx, gy)
            self.coarse_goal_sent = True
            self.last_coarse_goal = (gx, gy)
            return

        # ── EXPLORE: frontier planning ────────────────────────────────────────
        if self.state != "EXPLORE":
            return
        if self.nav_active:
            self.check_goal_timeout()
            return
        if self.map_msg is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return

        info = self.map_msg.info
        w, h = info.width, info.height
        grid = np.array(self.map_msg.data, dtype=np.int8).reshape((h, w))

        known = int(np.count_nonzero(grid != -1))
        if known < MIN_KNOWN_CELLS:
            return

        clusters = self.find_clusters(grid, w, h)
        if not clusters:
            raw_mask = self.frontier_mask_cache if hasattr(self, "frontier_mask_cache") else None
            raw_count = int(np.count_nonzero(raw_mask)) if raw_mask is not None else 0

            if raw_count > 0:
                self.empty_count = 0
                rx, ry, _ = pose
                ys, xs = np.where(raw_mask)
                best_d, best = float("inf"), None
                for fy, fx in zip(ys, xs):
                    wx = info.origin.position.x + (fx + 0.5) * info.resolution
                    wy = info.origin.position.y + (fy + 0.5) * info.resolution
                    if self.is_failed(wx, wy):
                        continue
                    d = math.hypot(wx - rx, wy - ry)
                    if d < best_d:
                        best_d, best = d, (wx, wy)
                if best is not None:
                    self.get_logger().info(
                        f"🔬 Only small frontiers ({raw_count} cells) — "
                        f"going to nearest at {best[0]:.2f},{best[1]:.2f}")
                    self.send_goal(*best)
                else:
                    self.empty_count += 1
                    self.get_logger().info(
                        f"🔍 Small frontiers all unreachable "
                        f"({self.empty_count}/{EMPTY_CONFIRMATIONS})")
                    if self.empty_count >= EMPTY_CONFIRMATIONS:
                        self.get_logger().info("✅ All remaining frontiers unreachable — done.")
                        self.state = "STOPPED"
                return

            self.empty_count += 1
            self.get_logger().info(
                f"🔍 Zero frontiers ({self.empty_count}/{EMPTY_CONFIRMATIONS}) known={known}")
            if self.empty_count >= EMPTY_CONFIRMATIONS:
                self.get_logger().info("✅ Map genuinely complete — every frontier explored.")
                self.state = "STOPPED"
            return
        self.empty_count = 0

        rx, ry, _ = pose
        best, best_d = None, float("inf")
        for cl in clusters:
            cx_w, cy_w = self.cluster_centroid(cl, info)
            if self.is_failed(cx_w, cy_w):
                continue
            d = math.hypot(cx_w - rx, cy_w - ry)
            if d < best_d:
                best_d, best = d, (cx_w, cy_w)
        if best is None:
            self.get_logger().info("♻️ All frontiers were failed — clearing failed list, retrying")
            self.failed_frontiers.clear()
            return
        self.get_logger().info(f"🗺️ {len(clusters)} frontiers | goal {best[0]:.2f},{best[1]:.2f}")
        self.send_goal(*best)

    # ── frontier helpers ──────────────────────────────────────────────────────

    def find_clusters(self, grid, w, h):
        unknown = (grid == -1)
        free    = (grid >= 0) & (grid < FREE_THRESHOLD)
        adj = np.zeros((h, w), dtype=bool)
        adj[1:, :]  |= unknown[:-1, :]
        adj[:-1, :] |= unknown[1:, :]
        adj[:, 1:]  |= unknown[:, :-1]
        adj[:, :-1] |= unknown[:, 1:]
        mask = free & adj
        self.frontier_mask_cache = mask
        visited = np.zeros((h, w), dtype=bool)
        clusters = []
        rs, cs = np.where(mask)
        for r0, c0 in zip(rs, cs):
            if visited[r0, c0]:
                continue
            cl = []
            q = deque([(r0, c0)])
            visited[r0, c0] = True
            while q:
                r, c = q.popleft()
                cl.append((r, c))
                for dr, dc in NEIGHBORS_8:
                    nr, nc = r+dr, c+dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))
            if len(cl) >= MIN_FRONTIER_SIZE:
                clusters.append(cl)
        return clusters

    def cluster_centroid(self, cl, info):
        rows = [c[0] for c in cl]; cols = [c[1] for c in cl]
        mc = sum(cols)/len(cols); mr = sum(rows)/len(rows)
        x = info.origin.position.x + (mc + 0.5) * info.resolution
        y = info.origin.position.y + (mr + 0.5) * info.resolution
        return x, y

    def is_failed(self, x, y, tol=0.5):
        return any(math.hypot(x-fx, y-fy) < tol for fx, fy in self.failed_frontiers)

    # ── Nav2 plumbing ─────────────────────────────────────────────────────────

    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ Nav2 server unavailable")
            return
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal.pose = pose
        self.nav_active = True
        self.goal_start_t = self.get_clock().now()
        self.pending_goal = (x, y)
        self.last_pose_xy = None
        self.last_move_t = self.get_clock().now()
        f = self.nav_client.send_goal_async(goal)
        f.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("⚠️ Goal rejected")
            self.nav_active = False
            return
        self.goal_handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("✅ Reached goal")
            if self.state == "APPROACH_COARSE":
                self.coarse_goal_sent = False
        else:
            if self.state == "EXPLORE" and self.pending_goal:
                self.failed_frontiers.append(self.pending_goal)
            elif self.state == "APPROACH_COARSE":
                self.coarse_goal_sent = False
        self.nav_active  = False
        self.goal_handle = None

    def check_goal_timeout(self):
        if self.goal_start_t is None:
            return
        el = (self.get_clock().now() - self.goal_start_t).nanoseconds / 1e9
        if el > GOAL_TIMEOUT:
            self.get_logger().warn(f"⏰ Goal timeout {el:.0f}s — cancelling")
            self.cancel_nav()
            if self.state == "EXPLORE" and self.pending_goal:
                self.failed_frontiers.append(self.pending_goal)
            if self.state == "APPROACH_COARSE":
                self.coarse_goal_sent = False
                self.last_coarse_goal = None

    def cancel_nav(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.nav_active  = False
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.cmd_pub.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
