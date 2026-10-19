#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess, os, math
from ament_index_python.packages import get_package_prefix

# ── constants ─────────────────────────────────────────────────────────────────

RED_LOWER1 = np.array([0,   120,  70])
RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  70])
RED_UPPER2 = np.array([180, 255, 255])

DETECT_MIN_AREA    = 800
APPROACH_STOP_AREA = 110000   # raised → robot gets closer before stopping

ALIGN_TOLERANCE  = 0.08
OBSTACLE_DIST    = 0.50

LINEAR_SPEED     = 0.12
ANGULAR_SEARCH   = 0.40
ANGULAR_ALIGN    = 0.30
ANGULAR_P_GAIN   = 0.003

SPIN_TICKS       = 45         # full-ish rotation when searching blind
ADVANCE_TICKS    = 30
AVOID_TICKS      = 15
WARMUP_TICKS     = 30
FRONT_HALF       = 20

STALL_LIMIT      = 25         # ticks with no area growth → ORBIT

# ORBIT (obstacle circling with box-bearing memory)
ORBIT_STEP_TICKS = 18         # forward burst length while circling
ORBIT_CHECK_TICKS= 14         # how long to pause-and-look back toward box
MAX_ORBIT_COUNT  = 8          # safety cap before full reset

# Maps go in <workspace>/maps (gitignored). Workspace = folder holding install/.
_WS_ROOT = os.path.dirname(os.path.dirname(get_package_prefix("red_tracker")))
MAP_SAVE_PATH = os.path.join(_WS_ROOT, "maps", "explored_map")

# ─────────────────────────────────────────────────────────────────────────────


def yaw_from_quat(q):
    """Extract yaw (z-rotation) from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    """Smallest signed difference a-b wrapped to [-pi, pi]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class DecisionNode(Node):

    def __init__(self):
        super().__init__("decision_node")

        self.bridge = CvBridge()
        self.scan   = None
        self.frame  = None
        self.yaw    = 0.0
        self.have_odom = False

        self.state         = "WARMUP"
        self.explore_phase = "SPIN"
        self.tick          = 0
        self.warmup_tick   = 0
        self.map_saved     = False

        # obstacle avoidance in EXPLORE
        self.avoiding   = False
        self.avoid_tick = 0
        self.avoid_dir  = 0.0

        # stall detection in APPROACH
        self.last_area    = 0.0
        self.stall_ticks  = 0

        # ── BOX MEMORY ──────────────────────────────────────────────
        self.box_ever_seen     = False
        self.last_box_yaw      = 0.0    # odom yaw when box last seen
        self.last_box_error    = 0.0    # normalized x error when last seen

        # ORBIT state
        self.orbit_phase = "MOVE"       # MOVE → LOOK
        self.orbit_tick  = 0
        self.orbit_dir   = 0.0
        self.orbit_count = 0

        self.create_subscription(Image,     "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(LaserScan, "/scan",             self.scan_cb,  10)
        self.create_subscription(Odometry,  "/odom",             self.odom_cb,  10)

        self.vel_pub   = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.state_pub = self.create_publisher(Bool,  "/red_found",   10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🤖 Decision Node (v5, box-memory) — warming up...")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def scan_cb(self, msg):  self.scan = msg

    def odom_cb(self, msg):
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def image_cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge: {e}")

    # ── LiDAR helpers — index 0 = FRONT ──────────────────────────────────────

    def _clean(self, indices):
        if self.scan is None:
            return []
        r = self.scan.ranges
        n = len(r)
        return [r[i] for i in indices
                if 0 <= i < n and r[i] == r[i]
                and r[i] != float('inf') and r[i] > 0.01]

    def obstacle_ahead(self) -> bool:
        n = len(self.scan.ranges) if self.scan else 0
        if n == 0:
            return False
        idx  = list(range(n - FRONT_HALF, n)) + list(range(0, FRONT_HALF + 1))
        vals = self._clean(idx)
        return bool(vals) and min(vals) < OBSTACLE_DIST

    def side_openness(self):
        """Return (left_avg, right_avg) clear distances."""
        if self.scan is None:
            return 0.0, 0.0
        n = len(self.scan.ranges)
        def avg(idx):
            v = self._clean(idx)
            return sum(v)/len(v) if v else 0.0
        left  = avg(range(max(0, n//6),   min(n, n//3)))
        right = avg(range(max(0, n*2//3), min(n, n*5//6)))
        return left, right

    def best_turn_dir(self) -> float:
        left, right = self.side_openness()
        return ANGULAR_ALIGN if left >= right else -ANGULAR_ALIGN

    # ── perception ────────────────────────────────────────────────────────────

    def detect_red(self):
        if self.frame is None or self.frame.shape[0] == 0:
            return 0, 0.0
        hsv  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = (cv2.inRange(hsv, RED_LOWER1, RED_UPPER1) |
                cv2.inRange(hsv, RED_LOWER2, RED_UPPER2))
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, 0.0
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < DETECT_MIN_AREA:
            return 0, 0.0
        x, y, w, h = cv2.boundingRect(largest)
        cx         = x + w // 2
        width      = self.frame.shape[1]
        return area, (cx - width // 2) / (width // 2)

    def remember_box(self, norm_error):
        """Record current odom yaw + error as the box's last-seen bearing."""
        self.box_ever_seen  = True
        self.last_box_yaw   = self.yaw
        self.last_box_error = norm_error

    # ── misc ──────────────────────────────────────────────────────────────────

    def save_map(self):
        if self.map_saved:
            return
        self.map_saved = True
        os.makedirs(os.path.dirname(MAP_SAVE_PATH), exist_ok=True)
        self.get_logger().info(f"💾 Saving map → {MAP_SAVE_PATH}")
        try:
            subprocess.Popen([
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", MAP_SAVE_PATH, "--ros-args", "-p", "use_sim_time:=true"
            ])
        except Exception as e:
            self.get_logger().error(f"Map save failed: {e}")

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def transition(self, new_state: str):
        if self.state != new_state:
            self.get_logger().info(f"🔄  {self.state} → {new_state}")
        self.state = new_state
        self.tick  = 0

    # ── main loop ─────────────────────────────────────────────────────────────

    def control_loop(self):
        self.tick += 1

        found_msg      = Bool()
        found_msg.data = (self.state == "STOPPED")
        self.state_pub.publish(found_msg)

        # WARMUP
        if self.state == "WARMUP":
            self.publish(0.0, 0.0)
            self.warmup_tick += 1
            if self.warmup_tick >= WARMUP_TICKS:
                self.get_logger().info("✅ Warmup done — EXPLORE")
                self.transition("EXPLORE")
            return

        area, norm_error = self.detect_red()
        red_visible      = area >= DETECT_MIN_AREA

        if red_visible:
            self.remember_box(norm_error)   # always update memory when seen

        # STOPPED
        if self.state == "STOPPED":
            self.publish(0.0, 0.0)
            self.save_map()
            return

        # ───────────────────────── EXPLORE ────────────────────────────────────
        if self.state == "EXPLORE":
            if red_visible:
                self.avoiding    = False
                self.last_area   = area
                self.stall_ticks = 0
                self.orbit_count = 0
                self.transition("ALIGN")
                return

            # obstacle avoidance
            if self.avoiding:
                self.avoid_tick += 1
                self.publish(0.0, self.avoid_dir)
                if self.avoid_tick >= AVOID_TICKS:
                    self.avoiding   = False
                    self.avoid_tick = 0
                return
            if self.obstacle_ahead():
                self.avoiding   = True
                self.avoid_tick = 0
                self.avoid_dir  = self.best_turn_dir()
                self.publish(0.0, self.avoid_dir)
                return

            # ── DIRECTED SEARCH: if we've seen the box, bias toward its bearing ──
            if self.box_ever_seen and self.have_odom:
                err_to_box = angle_diff(self.last_box_yaw, self.yaw)
                if abs(err_to_box) > 0.15:
                    # rotate back toward last-known box heading
                    self.publish(0.0, ANGULAR_ALIGN if err_to_box > 0 else -ANGULAR_ALIGN)
                    return
                else:
                    # facing roughly the right way → advance to get a new viewpoint
                    self.publish(LINEAR_SPEED, 0.0)
                    return

            # ── BLIND SEARCH (never seen box): spin then advance ──
            if self.explore_phase == "SPIN":
                self.publish(0.0, ANGULAR_SEARCH)
                if self.tick >= SPIN_TICKS:
                    self.explore_phase = "ADVANCE"
                    self.tick = 0
            else:
                self.publish(LINEAR_SPEED, 0.0)
                if self.tick >= ADVANCE_TICKS:
                    self.explore_phase = "SPIN"
                    self.tick = 0

        # ───────────────────────── ALIGN ──────────────────────────────────────
        elif self.state == "ALIGN":
            if not red_visible:
                self.transition("EXPLORE")
                return
            if abs(norm_error) <= ALIGN_TOLERANCE:
                self.last_area   = area
                self.stall_ticks = 0
                self.transition("APPROACH")
                return
            angular = -ANGULAR_ALIGN if norm_error > 0 else ANGULAR_ALIGN
            self.publish(0.0, angular)

        # ───────────────────────── APPROACH ───────────────────────────────────
        elif self.state == "APPROACH":
            if not red_visible:
                # lost the box mid-approach — likely an obstacle cut in front.
                # Don't blind-spin; go orbit toward its last bearing.
                self.start_orbit()
                return
            if area >= APPROACH_STOP_AREA:
                self.orbit_count = 0
                self.transition("STOPPED")
                return
            if abs(norm_error) > ALIGN_TOLERANCE * 2:
                self.transition("ALIGN")
                return

            # stall detection
            if area > self.last_area * 1.05:
                self.last_area   = area
                self.stall_ticks = 0
            else:
                self.stall_ticks += 1

            if self.stall_ticks >= STALL_LIMIT:
                self.start_orbit()
                return

            half_w = self.frame.shape[1] // 2 if self.frame is not None else 320
            self.publish(LINEAR_SPEED, -ANGULAR_P_GAIN * norm_error * half_w)

        # ───────────────────────── ORBIT ──────────────────────────────────────
        # Circle the blocking obstacle while periodically looking back toward
        # the box's remembered bearing. Replaces the old blind DETOUR.
        elif self.state == "ORBIT":
            self.orbit_tick += 1

            if self.orbit_phase == "MOVE":
                # Move sideways around the obstacle, hugging the open side
                if self.obstacle_ahead():
                    # still blocked ahead → keep turning away
                    self.publish(0.0, self.orbit_dir)
                else:
                    # edge forward along the open side, with slight curve toward box
                    self.publish(LINEAR_SPEED, self.orbit_dir * 0.4)
                if self.orbit_tick >= ORBIT_STEP_TICKS:
                    self.orbit_phase = "LOOK"
                    self.orbit_tick  = 0

            elif self.orbit_phase == "LOOK":
                # Pause forward motion, rotate toward remembered box bearing & look
                if self.box_ever_seen and self.have_odom:
                    err = angle_diff(self.last_box_yaw, self.yaw)
                    self.publish(0.0, ANGULAR_ALIGN if err > 0 else -ANGULAR_ALIGN)
                else:
                    self.publish(0.0, self.orbit_dir)

                if self.orbit_tick >= ORBIT_CHECK_TICKS:
                    self.orbit_phase = "MOVE"
                    self.orbit_tick  = 0
                    self.orbit_count += 1
                    if self.orbit_count >= MAX_ORBIT_COUNT:
                        self.get_logger().warn("🔁 Orbit cap — full reset to EXPLORE")
                        self.orbit_count   = 0
                        self.box_ever_seen = False  # forget stale bearing
                        self.explore_phase = "SPIN"
                        self.transition("EXPLORE")

    def start_orbit(self):
        """Enter ORBIT mode toward the more open side."""
        left, right = self.side_openness()
        self.orbit_dir   = ANGULAR_ALIGN if left >= right else -ANGULAR_ALIGN
        self.orbit_phase = "MOVE"
        self.orbit_tick  = 0
        self.stall_ticks = 0
        self.last_area   = 0.0
        self.get_logger().warn(f"🚧 Stuck — ORBIT (dir={'L' if self.orbit_dir>0 else 'R'})")
        self.transition("ORBIT")


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()
