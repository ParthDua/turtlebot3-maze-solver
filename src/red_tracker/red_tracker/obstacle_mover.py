#!/usr/bin/env python3
"""
obstacle_mover.py — drives the moving_obstacle back and forth.
Publishes a Twist on /obstacle/cmd_vel that reverses direction periodically,
making the box patrol a straight line to cross the robot's path.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SPEED        = 0.20   # m/s patrol speed
LEG_SECONDS  = 5.0    # seconds before reversing direction


class ObstacleMover(Node):
    def __init__(self):
        super().__init__("obstacle_mover")
        self.pub = self.create_publisher(Twist, "/obstacle/cmd_vel", 10)
        self.direction = 1.0
        self.elapsed   = 0.0
        self.dt        = 0.1
        self.create_timer(self.dt, self.loop)
        self.get_logger().info("🔵 Obstacle mover started — patrolling")

    def loop(self):
        self.elapsed += self.dt
        if self.elapsed >= LEG_SECONDS:
            self.direction *= -1.0
            self.elapsed = 0.0
            self.get_logger().info(f"🔵 Reversing → {'+' if self.direction>0 else '-'}")
        msg = Twist()
        msg.linear.x = SPEED * self.direction
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # stop the obstacle on exit
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
