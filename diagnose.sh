#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# diagnose.sh — run this when things go wrong
# Shows exactly what nodes are alive and who is publishing to /cmd_vel
# ─────────────────────────────────────────────────────────────────────────────

echo ""
echo "══════════════════════════════════════════════"
echo "  ACTIVE ROS2 NODES"
echo "══════════════════════════════════════════════"
ros2 node list

echo ""
echo "══════════════════════════════════════════════"
echo "  WHO IS PUBLISHING /cmd_vel  (the critical one)"
echo "══════════════════════════════════════════════"
ros2 topic info /cmd_vel --verbose

echo ""
echo "══════════════════════════════════════════════"
echo "  WHO IS PUBLISHING /cmd_vel_raw"
echo "══════════════════════════════════════════════"
ros2 topic info /cmd_vel_raw --verbose

echo ""
echo "══════════════════════════════════════════════"
echo "  CURRENT /cmd_vel VALUES  (5 messages)"
echo "══════════════════════════════════════════════"
timeout 3 ros2 topic echo /cmd_vel --once

echo ""
echo "══════════════════════════════════════════════"
echo "  CURRENT /scan RANGE MIN/MAX"
echo "══════════════════════════════════════════════"
timeout 3 ros2 topic echo /scan --once | grep -E "range_min|range_max|ranges" | head -5

echo ""
echo "══════════════════════════════════════════════"
echo "  DECISION NODE STATE  (check /red_found)"
echo "══════════════════════════════════════════════"
timeout 2 ros2 topic echo /red_found --once

echo ""
echo "══════════════════════════════════════════════"
echo "  ACTIVE TOPICS"
echo "══════════════════════════════════════════════"
ros2 topic list | grep -E "cmd_vel|scan|camera|red_found|map"

echo ""
echo "✅ Diagnosis complete."
echo "If you see multiple publishers on /cmd_vel → old nodes are still running."
echo "Fix: pkill -9 -f ros2 && pkill -9 -f gazebo && pkill -9 -f rviz"
