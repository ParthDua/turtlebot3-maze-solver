import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np


class RedTracker(Node):
    def __init__(self):
        super().__init__('red_tracker')

        self.bridge = CvBridge()
        self.scan = None

        # 🔴 REQUIRED for search logic
        self.search_state = 0
        self.search_timer = 0

        # Camera subscriber
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # LIDAR subscriber
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Velocity publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Red Tracker Node Started")

    # ✅ NOW INSIDE CLASS
    def scan_callback(self, msg):
        self.scan = msg

    # ✅ NOW INSIDE CLASS
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -------------------------------
        # 🔴 RED DETECTION
        # -------------------------------
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_red1, upper_red1) + \
               cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        # -------------------------------
        # 🔴 OBSTACLE DETECTION
        # -------------------------------
        obstacle_ahead = False
        turn_direction = 0.0

        if self.scan:
            ranges = self.scan.ranges
            size = len(ranges)

            front = ranges[size//2 - 10 : size//2 + 10]
            left  = ranges[size//2 + 30 : size//2 + 60]
            right = ranges[size//2 - 60 : size//2 - 30]

            def clean(data):
                return [r for r in data if not np.isnan(r) and not np.isinf(r)]

            front = clean(front)
            left = clean(left)
            right = clean(right)

            if front and min(front) < 0.5:
                obstacle_ahead = True

                if left and right:
                    if min(left) > min(right):
                        turn_direction = 0.5
                    else:
                        turn_direction = -0.5
                else:
                    turn_direction = 0.5

        # -------------------------------
        # 🔴 TARGET TRACKING
        # -------------------------------
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                width = frame.shape[1]
                error = cx - width // 2

                if area > 250000:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Reached target ✅")

                else:
                    if obstacle_ahead:
                        twist.linear.x = 0.0
                        twist.angular.z = turn_direction
                        self.get_logger().info("Obstacle ahead 🚧")
                    else:
                        twist.linear.x = 0.1
                        twist.angular.z = -0.002 * error

        else:
            # -------------------------------
            # 🔴 SEARCH BEHAVIOR
            # -------------------------------
            self.search_timer += 1

            if obstacle_ahead:
                twist.linear.x = 0.0
                twist.angular.z = turn_direction
            else:
                if self.search_state == 0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5

                    if self.search_timer > 30:
                        self.search_state = 1
                        self.search_timer = 0
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0

                    if self.search_timer > 20:
                        self.search_state = 0
                        self.search_timer = 0

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RedTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop robot on exit
    stop_msg = Twist()
    node.pub.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()
