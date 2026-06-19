#!/usr/bin/env python3
"""
safety_filter.py  (v3 — correct LiDAR front index)
─────────────────────────────────────────────────────────────────────────────
KEY FIX: TurtleBot3 /scan index 0 = FRONT (not size//2).
  Front cone wraps: indices [size-HALF : size] + [0 : HALF+1]
  Left  side:  indices [60 : 120]   (90° zone)
  Right side:  indices [240 : 300]  (270° zone)
─────────────────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

EMERGENCY_DIST = 0.30   # m — hard stop
CAUTION_DIST   = 0.50   # m — begin scaling speed

STOP_TICKS   = 5        # 0.5 s
BACK_TICKS   = 12       # 1.2 s
ROTATE_TICKS = 25       # 2.5 s

BACK_SPEED   = -0.07
ROTATE_SPEED =  0.50

FRONT_HALF   = 20       # rays each side of index-0 for front cone
WARMUP_TICKS = 30       # 3 s startup grace period


class SafetyFilter(Node):

    def __init__(self):
        super().__init__("safety_filter")

        self.scan            = None
        self.last_cmd        = Twist()
        self.override_active = False
        self.override_state  = "STOP"
        self.tick            = 0
        self.turn_dir        = ROTATE_SPEED
        self.warmup_remaining = WARMUP_TICKS

        self.create_subscription(LaserScan, "/scan",        self.scan_cb,    10)
        self.create_subscription(Twist,     "/cmd_vel_raw", self.cmd_raw_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🛡️  Safety Filter started — warming up (3 s)...")

    def scan_cb(self, msg):
        self.scan = msg

    def cmd_raw_cb(self, msg):
        self.last_cmd = msg

    # ── LiDAR helpers — index 0 = FRONT ──────────────────────────────────────

    def _clean(self, indices):
        """Return valid (non-nan, non-inf, non-zero) range values for given indices."""
        if self.scan is None:
            return []
        r = self.scan.ranges
        n = len(r)
        return [r[i] for i in indices
                if 0 <= i < n
                and not (r[i] != r[i])   # nan check
                and r[i] != float('inf')
                and r[i] > 0.01]

    def front_min(self) -> float:
        if self.scan is None:
            return float("inf")
        n = len(self.scan.ranges)
        # Wrap around index 0
        idx = list(range(n - FRONT_HALF, n)) + list(range(0, FRONT_HALF + 1))
        vals = self._clean(idx)
        return min(vals) if vals else float("inf")

    def best_turn_dir(self) -> float:
        if self.scan is None:
            return ROTATE_SPEED
        n = len(self.scan.ranges)
        # Left = 60°-120°, Right = 240°-300° (for 360-ray scan)
        left_idx  = range(max(0, n//6),     min(n, n//3))
        right_idx = range(max(0, n*2//3),   min(n, n*5//6))

        def avg(idx):
            v = self._clean(idx)
            return sum(v)/len(v) if v else 0.0

        left_avg  = avg(left_idx)
        right_avg = avg(right_idx)

        chosen = "LEFT" if left_avg >= right_avg else "RIGHT"
        self.get_logger().info(
            f"  turn → {chosen}  (L:{left_avg:.2f} R:{right_avg:.2f})")
        return ROTATE_SPEED if left_avg >= right_avg else -ROTATE_SPEED

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    # ── control loop ─────────────────────────────────────────────────────────

    def control_loop(self):

        # Warmup: pass through, safety inactive
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            if self.warmup_remaining == 0:
                self.get_logger().info("🛡️  Safety Filter ACTIVE")
            self.pub.publish(self.last_cmd)
            return

        dist = self.front_min()

        # Trigger override
        if dist < EMERGENCY_DIST and not self.override_active:
            self.override_active = True
            self.override_state  = "STOP"
            self.tick            = 0
            self.turn_dir        = self.best_turn_dir()
            self.get_logger().warn(f"🚨 SAFETY — obstacle {dist:.2f} m ahead")

        # Escape manoeuvre
        if self.override_active:
            self.tick += 1
            if self.override_state == "STOP":
                self.publish(0.0, 0.0)
                if self.tick >= STOP_TICKS:
                    self.override_state = "BACK"
                    self.tick = 0
            elif self.override_state == "BACK":
                self.publish(BACK_SPEED, 0.0)
                if self.tick >= BACK_TICKS:
                    self.override_state = "ROTATE"
                    self.tick = 0
            elif self.override_state == "ROTATE":
                self.publish(0.0, self.turn_dir)
                if self.tick >= ROTATE_TICKS:
                    self.override_active = False
                    self.tick = 0
                    self.get_logger().info("✅ Safety manoeuvre complete")
            return

        # Normal pass-through with caution scaling
        cmd = Twist()
        cmd.linear.x  = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z
        if dist < CAUTION_DIST and cmd.linear.x > 0:
            scale        = max(0.0, (dist - EMERGENCY_DIST) / (CAUTION_DIST - EMERGENCY_DIST))
            cmd.linear.x = cmd.linear.x * scale
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publish(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()
